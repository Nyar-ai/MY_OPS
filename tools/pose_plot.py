#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MY_OPS 全局位姿模块 - 上位机离线分析工具

用途
----
1) 解析串口抓包工具保存的原始字节流，校验 CRC 并统计质量；
2) 画位姿/速度曲线，用于验收与标定；
3) 无需 pyserial、无需硬件：--demo 会用与固件完全相同的帧格式
   生成一段示例数据再自解析，用于回归自检。

固件侧帧格式（与 Core/Src/ops_frame.c 严格一致，上行帧固定 26 字节）
    上行：[0]=0xAA [1]=0x55 [2]=type [3]=seq [4..23]=payload [24..25]=CRC16
          CRC-16/MODBUS，范围 [2..23]，低字节在前
    POSE  payload：x_mm i32 | y_mm i32 | yaw_0.01deg i32 |
                   vx i16 | vy i16 | w i16 | status u16
    INFO  payload：fw u16 | report_hz u16 | period_ms u16 | mode u8 | cal u8 |
                   frames u32 | gyro_err u16 | uart_err u16 | pulse_err u16 | scale i16
    DEBUG payload：cnt_a i32 | cnt_b i32 | angle_cdeg i32 | dps_cdeg i16 |
                   age_ms u16 | loop_us u16 | bias_x1000 i16
    下行：[0]=0x5A [1]=0xA5 [2]=cmd [3]=param [4]=seq [5..6]=CRC16 [7]=0xA5
          CRC 范围 [2..4]

用法
----
    python tools/pose_plot.py capture.bin                 # 解析并画图
    python tools/pose_plot.py capture.bin --csv out.csv   # 另存 CSV
    python tools/pose_plot.py --demo                      # 生成并自解析示例
"""

import argparse
import math
import struct
import sys

# 统一按 UTF-8 输出，避免在 GBK 控制台里中文乱码
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

FRAME_LEN = 26
CMD_LEN = 8
UP_H0, UP_H1 = 0xAA, 0x55
DN_H0, DN_H1, DN_TAIL = 0x5A, 0xA5, 0xA5

MSG_POSE, MSG_INFO, MSG_DEBUG = 0x01, 0x02, 0x03
ACK_FLAG = 0x80

ST_BITS = [
    (0x0001, "VALID"),
    (0x0002, "CALIBRATED"),
    (0x0004, "STATIC"),
    (0x0008, "GYRO_OK"),
    (0x0010, "OVERRUN"),
    (0x0020, "GYRO_RESTORED"),
    (0x0040, "GEOM_DIAG"),
]


def crc16_modbus(data: bytes) -> int:
    """CRC-16/MODBUS，与固件 Ops_CRC16_Modbus / 官方 CY-Z 参考实现逐位一致。"""
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
            crc &= 0xFFFF
    return crc


def decode_status(st: int) -> str:
    names = [name for bit, name in ST_BITS if st & bit]
    rest = st & ~sum(bit for bit, _ in ST_BITS)
    if rest:
        names.append("0x%04X" % rest)
    return "|".join(names) if names else "-"


def parse_pose(payload: bytes) -> dict:
    a, b, c = struct.unpack_from("<iii", payload, 0)
    vx, vy, w = struct.unpack_from("<hhh", payload, 12)
    st = struct.unpack_from("<H", payload, 18)[0]
    return {
        "x": a / 1000.0,
        "y": b / 1000.0,
        "yaw": c / 100.0,
        "vx": vx / 1000.0,
        "vy": vy / 1000.0,
        "w": w / 100.0,
        "status": st,
    }


def parse_info(payload: bytes) -> dict:
    fw, hz, period = struct.unpack_from("<HHH", payload, 0)
    mode, cal = payload[6], payload[7]
    frames = struct.unpack_from("<I", payload, 8)[0]
    gerr, uerr, perr = struct.unpack_from("<HHH", payload, 12)
    scale = struct.unpack_from("<h", payload, 18)[0]
    return {
        "fw": "0x%04X" % fw,
        "report_hz": hz,
        "gyro_period_ms": period,
        "cyz_mode": mode,
        "calibrated": cal,
        "gyro_frames": frames,
        "gyro_err": gerr,
        "uart_err": uerr,
        "pulse_err": perr,
        "scale": scale / 1000.0 if scale else 0.0,
    }


def parse_debug(payload: bytes) -> dict:
    ca, cb, angle = struct.unpack_from("<iii", payload, 0)
    dps, age, loop, bias = struct.unpack_from("<hHHh", payload, 12)
    return {
        "cnt_a": ca,
        "cnt_b": cb,
        "gyro_angle": angle / 100.0,
        "gyro_dps": dps / 100.0,
        "gyro_age_ms": age,
        "loop_us": loop,
        "bias_dps": bias / 1000.0,
    }


def parse_stream(raw: bytes):
    """逐字节扫描，遇 0xAA 0x55 同步；返回 (poses, infos, debugs, stats)。"""
    poses, infos, debugs = [], [], []
    i = 0
    n = len(raw)
    crc_err = 0
    unknown = 0

    while i < n:
        if raw[i] != UP_H0 or (i + 1 >= n) or raw[i + 1] != UP_H1:
            i += 1
            continue
        if i + FRAME_LEN > n:
            break
        frame = raw[i:i + FRAME_LEN]
        want = crc16_modbus(frame[2:24])
        got = frame[24] | (frame[25] << 8)
        if want != got:
            crc_err += 1
            i += 1
            continue
        msg_type, seq, payload = frame[2], frame[3], frame[4:24]
        if msg_type == MSG_POSE:
            rec = parse_pose(payload)
            rec["seq"] = seq
            poses.append(rec)
        elif msg_type == MSG_INFO:
            rec = parse_info(payload)
            rec["seq"] = seq
            infos.append(rec)
        elif msg_type == MSG_DEBUG:
            rec = parse_debug(payload)
            rec["seq"] = seq
            debugs.append(rec)
        else:
            unknown += 1
        i += FRAME_LEN

    stats = {
        "bytes": n,
        "crc_err": crc_err,
        "unknown": unknown,
        "poses": len(poses),
        "infos": len(infos),
        "debugs": len(debugs),
    }
    return poses, infos, debugs, stats


def build_pose_frame(seq: int, x: float, y: float, yaw: float,
                     vx: float, vy: float, w: float, status: int) -> bytes:
    """与固件 Ops_Frame_BuildPose 完全一致地打包（用于 --demo 交叉自检）。"""
    payload = struct.pack("<iii", round(x * 1000), round(y * 1000), round(yaw * 100))
    payload += struct.pack("<hhh",
                           max(-32768, min(32767, round(vx * 1000))),
                           max(-32768, min(32767, round(vy * 1000))),
                           max(-32768, min(32767, round(w * 100))))
    payload += struct.pack("<H", status)
    body = bytes([MSG_POSE, seq]) + payload
    return bytes([UP_H0, UP_H1]) + body + struct.pack("<H", crc16_modbus(body))


def make_demo(report_hz: int = 100) -> bytes:
    """生成 2s 圆弧示例（与固件 test_ops.c 的导出内容等价），末尾附一帧 INFO。"""
    out = bytearray()
    n = 2 * report_hz
    dt = 1.0 / report_hz
    for i in range(n):
        t = i * dt
        x = 0.5 * math.sin(1.5708 * t)
        y = 0.5 * (1.0 - math.cos(1.5708 * t))
        yaw = 90.0 * t
        vx = 0.5 * 1.5708 * math.cos(1.5708 * t)
        vy = 0.5 * 1.5708 * math.sin(1.5708 * t)
        out += build_pose_frame(i & 0xFF, x, y, yaw, vx, vy, 90.0, 0x0001 | 0x0008)

    info = struct.pack("<HHHBB", 0x0100, 100, 20, 1, 1)
    info += struct.pack("<I", 12345)
    info += struct.pack("<HHH", 0, 0, 0)
    info += struct.pack("<h", 1000)
    body = bytes([MSG_INFO, 1]) + info
    out += bytes([UP_H0, UP_H1]) + body + struct.pack("<H", crc16_modbus(body))
    return bytes(out)


def report(poses, infos, debugs, stats) -> int:
    print("=== MY_OPS 位姿抓包分析 ===")
    print("字节数 %d | POSE %d | INFO %d | DEBUG %d | CRC 错 %d | 未知类型 %d"
          % (stats["bytes"], stats["poses"], stats["infos"], stats["debugs"],
             stats["crc_err"], stats["unknown"]))
    if not poses:
        print("未解析到位姿帧。请确认：波特率 115200、帧长 26、CRC-16/MODBUS。")
        return 1

    last = poses[-1]
    static_n = len([p for p in poses if p["status"] & 0x0004])
    print("最终位姿: x=%.4f m  y=%.4f m  yaw=%.3f deg"
          % (last["x"], last["y"], last["yaw"]))
    print("最大位移: |x|max=%.4f m  |y|max=%.4f m"
          % (max(abs(p["x"]) for p in poses), max(abs(p["y"]) for p in poses)))
    print("速度区间: vx=[%.3f, %.3f] vy=[%.3f, %.3f] w=[%.1f, %.1f] deg/s"
          % (min(p["vx"] for p in poses), max(p["vx"] for p in poses),
             min(p["vy"] for p in poses), max(p["vy"] for p in poses),
             min(p["w"] for p in poses), max(p["w"] for p in poses)))
    print("静止帧占比: %.1f%%（STATIC 位）" % (100.0 * static_n / len(poses)))

    seen = sorted({p["status"] for p in poses})
    print("出现过的 status: " + ", ".join("0x%04X(%s)" % (b, decode_status(b)) for b in seen))

    if debugs:
        d = debugs[-1]
        mx = max(x["loop_us"] for x in debugs)
        print("最近 DEBUG: gyro_dps=%.2f age=%dms loop=%dus bias=%.4f dps cnt=%d,%d"
              % (d["gyro_dps"], d["gyro_age_ms"], d["loop_us"], d["bias_dps"],
                 d["cnt_a"], d["cnt_b"]))
        print("1ms 任务耗时峰值: %d us（占 1ms 的 %.1f%%）" % (mx, mx / 10.0))
    if infos:
        i0 = infos[-1]
        print("最近 INFO: fw=%s report=%dHz gyro周期=%dms mode=%d cal=%d scale=%.4f "
              "gyro_err=%d uart_err=%d"
              % (i0["fw"], i0["report_hz"], i0["gyro_period_ms"], i0["cyz_mode"],
                 i0["calibrated"], i0["scale"], i0["gyro_err"], i0["uart_err"]))
    return 0


def dump_csv(path, poses):
    with open(path, "w", encoding="utf-8") as f:
        f.write("seq,x_m,y_m,yaw_deg,vx_mps,vy_mps,w_dps,status\n")
        for p in poses:
            f.write("%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,0x%04X\n"
                    % (p["seq"], p["x"], p["y"], p["yaw"], p["vx"], p["vy"],
                       p["w"], p["status"]))
    print("已写出 CSV: %s" % path)


def plot(poses, png) -> bool:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        print("未安装 matplotlib（%s），跳过绘图；可用 --csv 导出后自行画图。" % exc)
        return False

    t = [i * 0.01 for i in range(len(poses))]
    x = [p["x"] for p in poses]
    y = [p["y"] for p in poses]
    yaw = [p["yaw"] for p in poses]
    vx = [p["vx"] for p in poses]
    vy = [p["vy"] for p in poses]

    fig, ax = plt.subplots(3, 1, figsize=(9, 10))
    ax[0].plot(x, y, "-o", ms=2, label="trajectory")
    ax[0].plot(x[0], y[0], "go", label="start")
    ax[0].plot(x[-1], y[-1], "rs", label="end")
    ax[0].set_title("MY_OPS trajectory (world frame: x forward, y left)")
    ax[0].set_xlabel("x [m]")
    ax[0].set_ylabel("y [m]")
    ax[0].grid(True)
    ax[0].legend()
    ax[0].axis("equal")

    ax[1].plot(t, yaw)
    ax[1].set_title("yaw [deg] (unwrapped)")
    ax[1].set_xlabel("t [s]")
    ax[1].set_ylabel("yaw [deg]")
    ax[1].grid(True)

    ax[2].plot(t, vx, label="vx (body)")
    ax[2].plot(t, vy, label="vy (body)")
    ax[2].set_title("body velocity [m/s]")
    ax[2].set_xlabel("t [s]")
    ax[2].set_ylabel("v [m/s]")
    ax[2].legend()
    ax[2].grid(True)

    fig.tight_layout()
    fig.savefig(png, dpi=130)
    print("已写出图像: %s" % png)
    return True


def main() -> int:
    ap = argparse.ArgumentParser(description="MY_OPS pose capture analyzer")
    ap.add_argument("capture", nargs="?", help="二进制抓包文件（原始字节流）")
    ap.add_argument("--demo", action="store_true",
                    help="生成示例抓包并自解析（无需硬件）")
    ap.add_argument("--csv", help="导出 CSV 路径")
    ap.add_argument("--png", default="pose_plot.png", help="输出图像路径")
    args = ap.parse_args()

    if args.demo:
        raw = make_demo()
        if not args.capture:
            args.capture = "capture_demo.bin"
        with open(args.capture, "wb") as f:
            f.write(raw)
        print("已生成示例抓包: %s (%d 字节)" % (args.capture, len(raw)))
    elif not args.capture:
        ap.print_help()
        return 2
    else:
        with open(args.capture, "rb") as f:
            raw = f.read()

    poses, infos, debugs, stats = parse_stream(raw)
    rc = report(poses, infos, debugs, stats)
    if args.csv and poses:
        dump_csv(args.csv, poses)
    if poses:
        plot(poses, args.png)
    return rc


if __name__ == "__main__":
    sys.exit(main())


