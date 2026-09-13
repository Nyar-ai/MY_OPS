/**
  ******************************************************************************
  * @file    test_ops.c
  * @brief   PC 端单元测试：覆盖 CRC/协议解析/帧打包/几何/融合/ZUPT/零偏自愈/
  *          采样层读-差-存。用 gcc 直接编译运行，不需要任何硬件。
  * @note    运行：test\run_tests.bat
  ******************************************************************************
  */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#include "ops_config.h"
#include "ops_types.h"
#include "ops_math.h"
#include "ops_crc.h"
#include "ops_geom.h"
#include "ops_frame.h"
#include "ops_fusion.h"
#include "ops_sample.h"
#include "ops_hw.h"
#include "ops_cyz_proto.h"

static int g_pass = 0;
static int g_fail = 0;
static const char *g_case = "";

#define CHECK(cond, ...) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; printf("  [FAIL] %s:%d: ", g_case, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
} while (0)

static void check_near(float got, float want, float tol, int line)
{
    if (fabsf(got - want) <= tol) {
        g_pass++;
    } else {
        g_fail++;
        printf("  [FAIL] %s:%d: got %.6f want %.6f (tol %.6f)\n", g_case, line, got, want, tol);
    }
}
#define CHECK_NEAR(got, want, tol) check_near((got), (want), (tol), __LINE__)

/* ===================== ops_hw 桩件（供 ops_sample.c 链接） ===================== */
static uint16_t g_cnt[2];
static uint8_t  g_dir[2];

uint16_t Ops_Hw_ReadCounter(uint8_t idx)
{
    return g_cnt[idx & 1u];
}

int8_t Ops_Hw_PulseSign(uint8_t idx)
{
    return Ops_SignFromDir(idx, g_dir[idx & 1u]);
}

/* ===================== 官方文档样例帧 ===================== */
/* AA 55 01 00 | 00 00 80 3F (=1.0f) | 00 00 00 00 (=0.0f) | 2A 07 | 55 AA */
static const uint8_t DOC_FRAME[16] = {
    0xAA, 0x55, 0x01, 0x00, 0x00, 0x00, 0x80, 0x3F,
    0x00, 0x00, 0x00, 0x00, 0x2A, 0x07, 0x55, 0xAA
};

/* ===================== T1：CRC ===================== */
static void test_crc(void)
{
    uint32_t crc;

    g_case = "T1-CRC";

    /* 官方给出的样例帧：CRC 范围 [2..11]，期望 0x072A */
    crc = Ops_CRC16_Modbus(&DOC_FRAME[2], 10u);
    CHECK(crc == 0x072Au, "doc frame CRC = 0x%04X, want 0x072A", (unsigned)crc);

    /* MODBUS 标准校验值：CRC16("123456789") = 0x4B37 */
    crc = Ops_CRC16_Modbus((const uint8_t *)"123456789", 9u);
    CHECK(crc == 0x4B37u, "check value = 0x%04X, want 0x4B37", (unsigned)crc);

    /* 空数据：初值 */
    crc = Ops_CRC16_Modbus(0, 0u);
    CHECK(crc == 0xFFFFu, "empty CRC = 0x%04X, want 0xFFFF", (unsigned)crc);
}

/* ===================== T2：CY-Z 协议层 ===================== */
static void test_cyz_proto(void)
{
    ops_cyz_telemetry_t t;
    ops_cyz_parser_t    parser;
    ops_cyz_frame_t     frame;
    ops_cyz_ack_t       ack;
    uint8_t             bad[16];
    uint8_t             cmd[8];
    uint8_t             i;
    uint8_t             got;
    int8_t              r;

    g_case = "T2-CYZ协议";

    /* 2.1 解析官方样例帧：seq=1, angle=1.0, gyro=0.0 */
    r = Ops_Cyz_ParseTelemetry(DOC_FRAME, 16u, &t);
    CHECK(r == 0, "parse telemetry r=%d", (int)r);
    CHECK(t.sequence == 1u, "seq=%u", (unsigned)t.sequence);
    CHECK_NEAR(t.angle_deg, 1.0f, 1e-6f);
    CHECK_NEAR(t.gyro_dps, 0.0f, 1e-6f);

    /* 2.2 各种损坏情况 */
    memcpy(bad, DOC_FRAME, 16u);
    bad[12] ^= 0xFFu;
    CHECK(Ops_Cyz_ParseTelemetry(bad, 16u, &t) == OPS_CYZ_ERR_CRC, "bad crc not detected");
    memcpy(bad, DOC_FRAME, 16u);
    bad[15] = 0x00u;
    CHECK(Ops_Cyz_ParseTelemetry(bad, 16u, &t) == OPS_CYZ_ERR_TAIL, "bad tail not detected");
    memcpy(bad, DOC_FRAME, 16u);
    bad[0] = 0x00u;
    CHECK(Ops_Cyz_ParseTelemetry(bad, 16u, &t) == OPS_CYZ_ERR_HEADER, "bad header not detected");
    CHECK(Ops_Cyz_ParseTelemetry(bad, 15u, &t) == OPS_CYZ_ERR_LEN, "bad length not detected");

    /* 2.3 流解析：前面有垃圾字节也能同步 */
    Ops_CyzParser_Init(&parser);
    got = 0u;
    for (i = 0u; i < 3u; ++i) {
        (void)Ops_CyzParser_Input(&parser, (uint8_t)(0x11u + i), &frame);
    }
    for (i = 0u; i < 16u; ++i) {
        if (Ops_CyzParser_Input(&parser, DOC_FRAME[i], &frame) == 1) {
            got = 1u;
        }
    }
    CHECK(got == 1u, "parser missed telemetry frame");
    CHECK(frame.type == OPS_CYZ_FRAME_TELEMETRY, "frame type=%d", (int)frame.type);
    CHECK_NEAR(frame.data.telemetry.angle_deg, 1.0f, 1e-6f);

    /* 2.4 未启用的 boot 帧头（C5）必须被直接忽略，且灌入大量字节也不越界 */
    Ops_CyzParser_Init(&parser);
    r = Ops_CyzParser_Input(&parser, 0xC5u, &frame);
    CHECK(r == 0, "C5 should be ignored, r=%d", (int)r);
    r = Ops_CyzParser_Input(&parser, 0x5Au, &frame);
    CHECK(r == 0, "stray 5A should be ignored, r=%d", (int)r);
    CHECK(parser.position == 0u, "stray bytes must not enter the buffer, pos=%u",
          (unsigned)parser.position);
    for (i = 0u; i < 40u; ++i) {                 /* 灌 40 字节也不得越界 */
        (void)Ops_CyzParser_Input(&parser, 0x00u, &frame);
    }
    CHECK(parser.position <= OPS_CYZ_PROTO_BUF_SIZE, "parser overflow! pos=%u",
          (unsigned)parser.position);

    /* 2.5 之后仍能正常解析一帧 */
    got = 0u;
    for (i = 0u; i < 16u; ++i) {
        if (Ops_CyzParser_Input(&parser, DOC_FRAME[i], &frame) == 1) {
            got = 1u;
        }
    }
    CHECK(got == 1u, "parser did not recover after resync");

    /* 2.6 ACK 帧：A5 5B Cmd Result Seq CRC(2) 5B */
    {
        uint8_t  a[8];
        uint16_t c;

        a[0] = 0xA5u; a[1] = 0x5Bu; a[2] = 0x01u; a[3] = 0x00u; a[4] = 0x37u;
        a[7] = 0x5Bu;
        c = Ops_CRC16_Modbus(&a[2], 3u);
        a[5] = (uint8_t)(c & 0xFFu);
        a[6] = (uint8_t)(c >> 8);

        CHECK(Ops_Cyz_ParseAck(a, 8u, &ack) == 0, "ack parse failed");
        CHECK((ack.cmd == 0x01u) && (ack.result == 0x00u) && (ack.seq == 0x37u),
              "ack fields wrong: %u %u %u", (unsigned)ack.cmd, (unsigned)ack.result,
              (unsigned)ack.seq);
    }

    /* 2.7 命令帧打包：A5 5A Cmd Param Seq CRC(2) 5A，CRC 范围 [2..4] */
    CHECK(Ops_Cyz_PackCommand(cmd, 0x01u, 0x02u, 0x05u) == 8u, "pack len");
    CHECK((cmd[0] == 0xA5u) && (cmd[1] == 0x5Au) && (cmd[2] == 0x01u) &&
          (cmd[3] == 0x02u) && (cmd[4] == 0x05u) && (cmd[7] == 0x5Au), "pack header/tail");
    {
        uint16_t c = Ops_CRC16_Modbus(&cmd[2], 3u);
        uint16_t g = (uint16_t)((uint16_t)cmd[5] | ((uint16_t)cmd[6] << 8));

        CHECK(c == g, "pack CRC 0x%04X != 0x%04X", (unsigned)g, (unsigned)c);
    }
}

/* ===================== 小端读取（测试用） ===================== */
static uint16_t rd_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int32_t rd_i32(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

/* ===================== T3：我方上行/下行帧 ===================== */
static void test_frames(void)
{
    uint8_t    buf[32];
    uint8_t    c[8];
    uint8_t    i;
    ops_pose_t p;
    ops_cmd_t  cmd;
    uint16_t   len;
    uint16_t   crc;
    ops_info_t info;
    ops_debug_t dbg;

    g_case = "T3-我方帧";

    /* 3.1 位姿帧 */
    memset(&p, 0, sizeof(p));
    p.x_m = 1.234f;
    p.y_m = -2.5f;
    p.yaw_deg = 90.0f;
    p.vx_mps = 0.5f;
    p.vy_mps = -0.25f;
    p.w_dps = 45.0f;
    p.status = OPS_ST_STATIC;
    p.valid = 1u;

    len = Ops_Frame_BuildPose(buf, 7u, &p);
    CHECK(len == OPS_POSE_FRAME_LEN, "pose len=%u", (unsigned)len);
    CHECK((buf[0] == 0xAAu) && (buf[1] == 0x55u) &&
          (buf[2] == OPS_MSG_POSE) && (buf[3] == 7u), "pose header");
    CHECK(rd_i32(&buf[4]) == 1234, "x_mm=%d", (int)rd_i32(&buf[4]));
    CHECK(rd_i32(&buf[8]) == -2500, "y_mm=%d", (int)rd_i32(&buf[8]));
    CHECK(rd_i32(&buf[12]) == 9000, "yaw_cdeg=%d", (int)rd_i32(&buf[12]));
    CHECK((int16_t)rd_u16(&buf[16]) == 500, "vx=%d", (int)(int16_t)rd_u16(&buf[16]));
    CHECK((int16_t)rd_u16(&buf[18]) == -250, "vy=%d", (int)(int16_t)rd_u16(&buf[18]));
    CHECK((int16_t)rd_u16(&buf[20]) == 4500, "w=%d", (int)(int16_t)rd_u16(&buf[20]));
    CHECK(rd_u16(&buf[22]) == (uint16_t)(OPS_ST_STATIC | OPS_ST_VALID),
          "status=0x%04X", (unsigned)rd_u16(&buf[22]));
    crc = Ops_CRC16_Modbus(&buf[2], 22u);
    CHECK(rd_u16(&buf[24]) == crc, "pose crc 0x%04X != 0x%04X",
          (unsigned)rd_u16(&buf[24]), (unsigned)crc);

    /* 3.2 INFO 帧 */
    memset(&info, 0, sizeof(info));
    info.fw_version = OPS_FW_VERSION;
    info.report_hz = 100u;
    info.gyro_period_ms = 20u;
    info.cyz_mode = 1u;
    info.calibrated = 1u;
    info.gyro_frames = 12345u;
    info.scale_x1000 = 1002;
    len = Ops_Frame_BuildInfo(buf, 1u, &info);
    CHECK(len == OPS_POSE_FRAME_LEN, "info len=%u", (unsigned)len);
    CHECK(buf[2] == OPS_MSG_INFO, "info type");
    CHECK(rd_u16(&buf[4]) == OPS_FW_VERSION, "fw");
    CHECK(rd_u16(&buf[6]) == 100u, "report_hz");
    CHECK(rd_u16(&buf[8]) == 20u, "period_ms");
    CHECK(buf[10] == 1u, "cyz_mode");
    CHECK(buf[11] == 1u, "calibrated");
    CHECK(rd_i32(&buf[12]) == 12345, "gyro_frames");
    CHECK((int16_t)rd_u16(&buf[22]) == 1002, "scale");
    crc = Ops_CRC16_Modbus(&buf[2], 22u);
    CHECK(rd_u16(&buf[24]) == crc, "info crc");

    /* 3.3 DEBUG 帧 */
    memset(&dbg, 0, sizeof(dbg));
    dbg.cnt_a = -40000;
    dbg.cnt_b = 40000;
    dbg.gyro_angle_cdeg = 123456;
    dbg.gyro_dps_cdeg = -1234;
    dbg.gyro_age_ms = 21u;
    dbg.loop_us = 87u;
    dbg.bias_dps_x1000 = -356;
    len = Ops_Frame_BuildDebug(buf, 2u, &dbg);
    CHECK(len == OPS_POSE_FRAME_LEN, "debug len");
    CHECK(buf[2] == OPS_MSG_DEBUG, "debug type");
    CHECK(rd_i32(&buf[4]) == -40000, "cnt_a");
    CHECK(rd_i32(&buf[8]) == 40000, "cnt_b");
    CHECK(rd_i32(&buf[12]) == 123456, "angle_cdeg");
    CHECK((int16_t)rd_u16(&buf[16]) == -1234, "dps_cdeg");
    CHECK(rd_u16(&buf[18]) == 21u, "age");
    CHECK(rd_u16(&buf[20]) == 87u, "loop_us");
    CHECK((int16_t)rd_u16(&buf[22]) == -356, "bias");

    /* 3.4 ACK 帧 */
    len = Ops_Frame_BuildAck(buf, 3u, OPS_CMD_ZERO, OPS_ACK_OK);
    CHECK(len == OPS_ACK_FRAME_LEN, "ack len");
    CHECK((buf[0] == 0x5Au) && (buf[1] == 0xA5u) &&
          (buf[2] == (uint8_t)(OPS_MSG_ACK_FLAG | OPS_CMD_ZERO)) &&
          (buf[3] == OPS_ACK_OK) && (buf[4] == 3u) && (buf[7] == 0xA5u), "ack fields");
    crc = Ops_CRC16_Modbus(&buf[2], 3u);
    CHECK(rd_u16(&buf[5]) == crc, "ack crc");

    /* 3.5 下行命令帧解析 */
    Ops_Frame_Init();
    c[0] = 0x5Au; c[1] = 0xA5u; c[2] = OPS_CMD_SET_REPORT_HZ;
    c[3] = 100u; c[4] = 9u; c[7] = 0xA5u;
    crc = Ops_CRC16_Modbus(&c[2], 3u);
    c[5] = (uint8_t)(crc & 0xFFu);
    c[6] = (uint8_t)(crc >> 8);

    for (i = 0u; i < 8u; ++i) {
        Ops_Frame_RxByte(c[i]);
    }
    CHECK(Ops_Frame_PopCommand(&cmd) == 1u, "command not parsed");
    CHECK((cmd.cmd == OPS_CMD_SET_REPORT_HZ) && (cmd.param == 100u) && (cmd.seq == 9u),
          "command fields %u %u %u", (unsigned)cmd.cmd, (unsigned)cmd.param, (unsigned)cmd.seq);
    CHECK(Ops_Frame_PopCommand(&cmd) == 0u, "queue should be empty");

    /* 3.6 坏 CRC 不入队 */
    c[3] = 5u;
    for (i = 0u; i < 8u; ++i) {
        Ops_Frame_RxByte(c[i]);
    }
    CHECK(Ops_Frame_HasCommand() == 0u, "bad CRC enqueued");
    CHECK(Ops_Frame_RxErrorCount() > 0u, "RX error not counted");

    /* 3.7 前缀 5A 5A 后仍可同步 */
    Ops_Frame_Init();
    Ops_Frame_RxByte(0x5Au);
    Ops_Frame_RxByte(0x5Au);
    c[3] = 100u;                       /* 恢复正确内容 */
    crc = Ops_CRC16_Modbus(&c[2], 3u);
    c[5] = (uint8_t)(crc & 0xFFu);
    c[6] = (uint8_t)(crc >> 8);
    for (i = 1u; i < 8u; ++i) {
        Ops_Frame_RxByte(c[i]);
    }
    CHECK(Ops_Frame_PopCommand(&cmd) == 1u, "no resync after 5A 5A prefix");
    CHECK(cmd.cmd == OPS_CMD_SET_REPORT_HZ, "resync cmd wrong");
}

/* ===================== T4：几何解算 ===================== */
static void test_geom(void)
{
    ops_geom_delta_t d;
    const float      mp = OPS_M_PER_PULSE;
    float            x;
    float            y;
    int32_t          k;

    g_case = "T4-几何";

    /* 4.1 纯直行：轮B(沿 x) 1000 脉冲 → 只有 x 位移 */
    Ops_Geom_Delta(0, 1000, 0.0f, 0.0f, &d);
    CHECK_NEAR(d.dx_body_m, 1000.0f * mp, 1e-6f);
    CHECK_NEAR(d.dy_body_m, 0.0f, 1e-6f);
    CHECK_NEAR(d.dx_world_m, 1000.0f * mp, 1e-6f);
    CHECK_NEAR(d.dy_world_m, 0.0f, 1e-6f);

    /* 4.2 原地旋转（双轮无脉冲）：位移必须恒为 0（Q28.a 的直接推论） */
    Ops_Geom_Delta(0, 0, 0.3f, 1.5708f, &d);
    CHECK_NEAR(d.dx_world_m, 0.0f, 1e-9f);
    CHECK_NEAR(d.dy_world_m, 0.0f, 1e-9f);

    /* 4.3 反向脉冲 */
    Ops_Geom_Delta(0, -1000, 0.0f, 0.0f, &d);
    CHECK_NEAR(d.dx_world_m, -1000.0f * mp, 1e-6f);

    /* 4.4 机体系→世界系：航向 90° 时 +y(轮A) 映射到 -x */
    Ops_Geom_Delta(1000, 0, OPS_PI_F * 0.5f, 0.0f, &d);
    CHECK_NEAR(d.dx_world_m, -1000.0f * mp, 1e-5f);
    CHECK_NEAR(d.dy_world_m, 0.0f, 1e-5f);

    /* 4.5 圆弧收敛性：90 步 × (6 脉冲 + 1°) 的位移应为弧长/θ 的弦
     *     |d| = 90*6*mp = 0.09608 m，θ = 90° ⇒ 期望 (0.06117, 0.06117) */
    x = 0.0f;
    y = 0.0f;
    {
        float yaw_rad = 0.0f;

        for (k = 0; k < 90; ++k) {
            Ops_Geom_Delta(0, 6, yaw_rad, (float)OPS_DEG2RAD, &d);
            x += d.dx_world_m;
            y += d.dy_world_m;
            yaw_rad += (float)OPS_DEG2RAD;       /* 航向逐级推进 */
        }
    }
    CHECK_NEAR(x, 0.061172f, 1e-4f);
    CHECK_NEAR(y, 0.061172f, 1e-4f);
}

/* ===================== T5：采样层（读-差-存 / 链路超时 / 重定基） ===================== */
static void test_sample(void)
{
    ops_sample_snapshot_t s;
    int32_t               ca;
    int32_t               cb;
    uint32_t              i;
    uint32_t              t;
    float                 ang;

    g_case = "T5-采样层";

    g_cnt[0] = 0x1234u;
    g_cnt[1] = 0x5678u;
    g_dir[0] = 1u;
    g_dir[1] = 1u;
    Ops_Sample_Init();

    /* 5.1 正向增量（方向脚 = 1） */
    g_cnt[0] = 0x1244u;                 /* +16 */
    g_cnt[1] = 0x5688u;                 /* +16 */
    Ops_Sample_Take(&s, 1000u, 1u);
    CHECK(s.dpulses[0] == (16 * (int32_t)OPS_DIR_SIGN_A), "A=%d", (int)s.dpulses[0]);
    CHECK(s.dpulses[1] == (16 * (int32_t)OPS_DIR_SIGN_B), "B=%d", (int)s.dpulses[1]);

    /* 5.2 方向脚取反 → 符号取反 */
    g_dir[0] = 0u;
    g_cnt[0] = 0x1245u;                 /* +1 */
    Ops_Sample_Take(&s, 1001u, 1u);
    CHECK(s.dpulses[0] == (-1 * (int32_t)OPS_DIR_SIGN_A), "dir inversion A=%d", (int)s.dpulses[0]);
    g_dir[0] = 1u;

    /* 5.3 正向 16 位回绕：0xFFF8 → 0x0008 = +16 */
    g_cnt[0] = 0xFFF8u;
    g_cnt[1] = 0x0000u;
    Ops_Sample_Take(&s, 1002u, 1u);
    g_cnt[0] = 0x0008u;
    g_cnt[1] = 0x0010u;
    Ops_Sample_Take(&s, 1003u, 1u);
    CHECK(s.dpulses[0] == (16 * (int32_t)OPS_DIR_SIGN_A), "wrap+ A=%d", (int)s.dpulses[0]);
    CHECK(s.dpulses[1] == (16 * (int32_t)OPS_DIR_SIGN_B), "wrap+ B=%d", (int)s.dpulses[1]);

    /* 5.4 反向回绕：0x0004 → 0xFFFE 即 -6；0x0004 → 0xFFFC 即 -8 */
    g_cnt[0] = 0x0004u;
    g_cnt[1] = 0x0004u;
    Ops_Sample_Take(&s, 1004u, 1u);
    g_cnt[0] = 0xFFFCu;
    g_cnt[1] = 0xFFFEu;
    Ops_Sample_Take(&s, 1005u, 1u);
    CHECK(s.dpulses[0] == (-8 * (int32_t)OPS_DIR_SIGN_A), "wrap- A=%d", (int)s.dpulses[0]);
    CHECK(s.dpulses[1] == (-6 * (int32_t)OPS_DIR_SIGN_B), "wrap- B=%d", (int)s.dpulses[1]);

    /* 5.5 长间隔（模拟 Flash 擦写导致的 CPU 停摆）：脉冲仍然无损 */
    g_cnt[0] += 500u;
    g_cnt[1] += 400u;
    Ops_Sample_Take(&s, 1045u, 40u);
    CHECK(s.dpulses[0] == (500 * (int32_t)OPS_DIR_SIGN_A), "stall A=%d", (int)s.dpulses[0]);
    CHECK(s.dpulses[1] == (400 * (int32_t)OPS_DIR_SIGN_B), "stall B=%d", (int)s.dpulses[1]);

    /* 5.6 累计计数只读 */
    ca = 0;
    cb = 0;
    Ops_Sample_GetCounters(&ca, &cb);
    CHECK(s.dpulses[0] != 0, "sanity");

    /* 5.7 链路超时自适应：50Hz(20ms) → 阈值 60ms */
    Ops_Sample_Init();
    t = 1000u;
    for (i = 0u; i < 40u; ++i) {
        t += 20u;
        Ops_Sample_OnGyroTelemetry((float)i, 10.0f, t);
    }
    CHECK(Ops_Sample_GetPeriodMs() == 20u, "period=%u", (unsigned)Ops_Sample_GetPeriodMs());

    Ops_Sample_Take(&s, t + 10u, 10u);
    CHECK(s.gyro_ok == 1u, "gyro_ok should be 1 at age 10ms");
    Ops_Sample_Take(&s, t + 30u, 20u);
    CHECK(s.gyro_ok == 1u, "gyro_ok should be 1 at age 30ms");
    Ops_Sample_Take(&s, t + 40u, 10u);
    CHECK(s.gyro_ok == 1u, "gyro_ok should be 1 at age 40ms");
    Ops_Sample_Take(&s, t + 100u, 60u);
    CHECK(s.gyro_ok == 0u, "gyro_ok should be 0 at age 100ms");

    /* 5.8 200Hz(5ms) → 阈值被下限钳到 25ms */
    Ops_Sample_Init();
    t = 5000u;
    for (i = 0u; i < 200u; ++i) {
        t += 5u;
        Ops_Sample_OnGyroTelemetry((float)i, 1.0f, t);
    }
    CHECK(Ops_Sample_GetPeriodMs() == 5u, "period=%u", (unsigned)Ops_Sample_GetPeriodMs());
    Ops_Sample_Take(&s, t + 20u, 20u);
    CHECK(s.gyro_ok == 1u, "gyro_ok should be 1 at age 20ms");
    Ops_Sample_Take(&s, t + 30u, 10u);
    CHECK(s.gyro_ok == 0u, "gyro_ok should be 0 at age 30ms");

    /* 5.9 模块侧重定基检测：3000° → 0.5° 必须判为 rebase（非真实运动） */
    Ops_Sample_Init();
    Ops_Sample_OnGyroTelemetry(3000.0f, 0.0f, 2000u);
    Ops_Sample_OnGyroTelemetry(3000.2f, 0.0f, 2020u);
    Ops_Sample_Take(&s, 2030u, 10u);
    CHECK(s.rebase == 0u, "unexpected rebase");
    Ops_Sample_OnGyroTelemetry(0.5f, 0.0f, 2040u);
    Ops_Sample_Take(&s, 2050u, 10u);
    CHECK(s.rebase == 1u, "rebase not detected");
    Ops_Sample_Take(&s, 2060u, 10u);
    CHECK(s.rebase == 0u, "rebase flag not cleared");

    /* 5.10 丢帧恢复：先超时，再收到帧 → 视为恢复（rebase） */
    Ops_Sample_Init();
    Ops_Sample_OnGyroTelemetry(10.0f, 0.0f, 3000u);
    Ops_Sample_Take(&s, 3200u, 200u);                 /* 200ms 无帧 → 超时 */
    CHECK(s.gyro_ok == 0u, "should be timed out");
    Ops_Sample_OnGyroTelemetry(10.5f, 0.0f, 3210u);
    Ops_Sample_Take(&s, 3220u, 10u);
    CHECK(s.rebase == 1u, "loss recovery not treated as rebase");

    /* 5.11 统计只读接口 */
    (void)Ops_Sample_GetLastAngle(&ang);
    CHECK(Ops_Sample_GetCyzMode() == 0u, "cyz mode default");
    Ops_Sample_SetCyzMode(1u);
    CHECK(Ops_Sample_GetCyzMode() == 1u, "cyz mode set");
}

/* ===================== 融合层仿真辅助 ===================== */
static uint32_t g_now;
static float    g_ang;          /* 模块上报角（模拟真实积分） */

static void sim_reset(void)
{
    g_now = 1000u;
    g_ang = 0.0f;
    Ops_Fusion_Init();
}

static void sim_step(int32_t dpa, int32_t dpb, float dps,
                     uint8_t frame_new, uint8_t rebase, uint8_t gyro_ok)
{
    ops_sample_snapshot_t s;

    memset(&s, 0, sizeof(s));
    s.dpulses[0] = dpa;
    s.dpulses[1] = dpb;
    s.gyro_dps = dps;
    s.gyro_angle_deg = g_ang;
    s.tick_ms = 1u;
    s.gyro_ok = gyro_ok;
    s.gyro_frame_new = frame_new;
    s.rebase = rebase;

    g_now += 1u;
    Ops_Fusion_Step(&s, g_now);
}

/* ===================== T6：ZUPT 与静止冻结 ===================== */
static void test_zupt(void)
{
    uint32_t          i;
    const ops_pose_t *p;

    g_case = "T6-ZUPT";
    sim_reset();

    /* 6.1 静止 400ms（角速度 0、无脉冲、每 20ms 一帧）→ 应置 STATIC */
    for (i = 0u; i < 400u; ++i) {
        sim_step(0, 0, 0.0f, (uint8_t)((i % 20u) == 0u), 0u, 1u);
    }
    p = Ops_Fusion_GetPose();
    CHECK((p->status & OPS_ST_STATIC) != 0u, "STATIC not set, status=0x%04X", (unsigned)p->status);
    CHECK(Ops_Fusion_IsStatic() == 1u, "IsStatic=0");
    CHECK((p->status & OPS_ST_GYRO_OK) != 0u, "GYRO_OK not set");

    /* 6.2 静止期间的亚阈值脉冲（每 25ms 1 个 = 4 个/100ms < 8）必须被丢弃，
     *     这正是抑制计数器量化噪声随机游走的关键 */
    for (i = 0u; i < 300u; ++i) {
        sim_step(0, ((i % 25u) == 0u) ? 1 : 0, 0.0f, 0u, 0u, 1u);
    }
    p = Ops_Fusion_GetPose();
    CHECK_NEAR(p->x_m, 0.0f, 1e-9f);
    CHECK_NEAR(p->y_m, 0.0f, 1e-9f);
    CHECK(Ops_Fusion_IsStatic() == 1u, "static lost on noise-level pulses");

    /* 6.3 真实运动（5 脉冲/ms ≈ 0.89 m/s）→ static 必须在一个窗口内解除并开始累加 */
    for (i = 0u; i < 300u; ++i) {
        sim_step(0, 5, 0.0f, 0u, 0u, 1u);
    }
    p = Ops_Fusion_GetPose();
    CHECK(Ops_Fusion_IsStatic() == 0u, "static not cleared on real motion");
    CHECK(p->x_m > 0.10f, "x should grow, x=%.4f", (double)p->x_m);
    CHECK(p->x_m < 0.30f, "x too large, x=%.4f", (double)p->x_m);

    /* 6.4 上报窗口速度 */
    {
        ops_pose_t rep;

        Ops_Fusion_TakeReport(&rep);            /* 清掉历史累加 */
        for (i = 0u; i < 100u; ++i) {
            sim_step(0, 5, 0.0f, 0u, 0u, 1u);
        }
        Ops_Fusion_TakeReport(&rep);
        CHECK(rep.vx_mps > 0.2f, "vx=%.3f", (double)rep.vx_mps);
        CHECK_NEAR(rep.vy_mps, 0.0f, 1e-6f);
    }
}

/* ===================== T7：航向（速率积分 + 每帧慢校正） ===================== */
static void test_yaw(void)
{
    uint32_t          i;
    const ops_pose_t *p;
    float             y0;
    float             x0;

    g_case = "T7-航向";

    /* 7.1 90°/s 转 1s：积分应等于真值 */
    sim_reset();
    for (i = 0u; i < 1000u; ++i) {
        uint8_t fn = (uint8_t)((i % 20u) == 0u);

        if (fn != 0u) {
            g_ang += 90.0f * 0.020f;
        }
        sim_step(0, 0, 90.0f, fn, 0u, 1u);
    }
    p = Ops_Fusion_GetPose();
    CHECK_NEAR(p->yaw_deg, 90.0f, 0.3f);

    /* 7.2 陀螺比例误差 1%（上报 89.1，真值 90）：慢校正把偏差压到 1.5° 内 */
    sim_reset();
    for (i = 0u; i < 1000u; ++i) {
        uint8_t fn = (uint8_t)((i % 20u) == 0u);

        if (fn != 0u) {
            g_ang += 90.0f * 0.020f;
        }
        sim_step(0, 0, 89.1f, fn, 0u, 1u);
    }
    p = Ops_Fusion_GetPose();
    CHECK(fabsf(p->yaw_deg - 90.0f) < 1.5f, "yaw=%.3f (1%% scale error)", (double)p->yaw_deg);

    /* 7.3 原地旋转 2s：航向累积但位置必须恒为 0 */
    sim_reset();
    for (i = 0u; i < 2000u; ++i) {
        uint8_t fn = (uint8_t)((i % 20u) == 0u);

        if (fn != 0u) {
            g_ang += 45.0f * 0.020f;
        }
        sim_step(0, 0, 45.0f, fn, 0u, 1u);
    }
    p = Ops_Fusion_GetPose();
    CHECK_NEAR(p->x_m, 0.0f, 1e-6f);
    CHECK_NEAR(p->y_m, 0.0f, 1e-6f);
    CHECK_NEAR(p->yaw_deg, 90.0f, 0.5f);

    /* 7.4 陀螺丢帧：航向冻结、位置仍由轮子推进；恢复后按重定基处理且航向连续 */
    sim_reset();
    for (i = 0u; i < 300u; ++i) {
        uint8_t fn = (uint8_t)((i % 20u) == 0u);

        if (fn != 0u) {
            g_ang += 1.8f;
        }
        sim_step(0, 0, 90.0f, fn, 0u, 1u);
    }
    y0 = Ops_Fusion_GetPose()->yaw_deg;
    x0 = Ops_Fusion_GetPose()->x_m;

    for (i = 0u; i < 500u; ++i) {               /* 500ms 无有效帧 */
        sim_step(0, 5, 90.0f, 0u, 0u, 0u);
    }
    p = Ops_Fusion_GetPose();
    CHECK_NEAR(p->yaw_deg, y0, 1e-4f);          /* 航向必须完全冻结 */
    CHECK((p->status & OPS_ST_GYRO_OK) == 0u, "GYRO_OK should clear");
    CHECK(p->x_m > (x0 + 0.05f), "position should still advance, x=%.4f", (double)p->x_m);

    g_ang += 45.0f;                             /* 丢帧期间模块继续积分 */
    y0 = Ops_Fusion_GetPose()->yaw_deg;
    sim_step(0, 0, 90.0f, 1u, 1u, 1u);          /* 恢复帧（rebase=1） */
    p = Ops_Fusion_GetPose();
    CHECK((p->status & OPS_ST_GYRO_RESTORED) != 0u, "RESTORED not set");
    /* 重定基不得造成航向跳变：只允许本拍自身的速率积分 0.09°，
     * 而模块侧 45° 的“角度跳变”必须被完全屏蔽 */
    CHECK(fabsf(p->yaw_deg - y0) < 0.2f, "rebase caused a yaw jump: %.4f",
          (double)(p->yaw_deg - y0));
    CHECK((p->status & OPS_ST_GYRO_OK) != 0u, "GYRO_OK should restore");
}

/* ===================== T8：零偏自愈与参数接口 ===================== */
static void test_bias(void)
{
    uint32_t          i;
    const ops_pose_t *p;
    float             b;

    g_case = "T8-零偏自愈";

    /* 8.1 残余零偏 0.4°/s、静止 6s：必须收敛到 0.4±0.05，
     *     且这段时间航向必须被 ZUPT 完全冻结（这就是 ≤1°/min 漂移的关键） */
    sim_reset();
    for (i = 0u; i < 6000u; ++i) {
        uint8_t fn = (uint8_t)((i % 20u) == 0u);

        if (fn != 0u) {
            g_ang += 0.4f * 0.020f;             /* 模块角按同样速率漂 */
        }
        sim_step(0, 0, 0.4f, fn, 0u, 1u);
    }
    CHECK(fabsf(Ops_Fusion_GetBiasDps() - 0.4f) < 0.05f, "bias=%.4f",
          (double)Ops_Fusion_GetBiasDps());
    CHECK(Ops_Fusion_IsCalibrated() == 1u, "calibrated not set after convergence");
    p = Ops_Fusion_GetPose();
    /* ZUPT 需要 200~300ms 才生效，这期间残余零偏会带来一次性、非累积的
     * ≤0.12° 航向偏移（0.4°/s × 0.3s）；此后航向被完全冻结 */
    CHECK(fabsf(p->yaw_deg) < 0.2f, "static yaw drift too large: %.4f", (double)p->yaw_deg);
    CHECK_NEAR(p->x_m, 0.0f, 1e-9f);

    /* 8.2 落盘节流 */
    b = 0.0f;
    CHECK(Ops_Fusion_NeedBiasSave(g_now, &b) == 1u, "bias save not requested");
    CHECK_NEAR(b, 0.4f, 0.05f);
    Ops_Fusion_MarkBiasSaved(g_now);
    CHECK(Ops_Fusion_NeedBiasSave(g_now, &b) == 0u, "save not throttled after MarkBiasSaved");
    CHECK(Ops_Fusion_NeedBiasSave(g_now + 60000u, &b) == 0u, "no new bias -> no save");

    /* 8.3 从 Flash 载入零偏不得置 calibrated（Q30.b） */
    Ops_Fusion_Init();
    Ops_Fusion_LoadBias(0.3f);
    CHECK(Ops_Fusion_IsCalibrated() == 0u, "LoadBias must NOT set calibrated");
    CHECK_NEAR(Ops_Fusion_GetBiasDps(), 0.3f, 1e-6f);

    /* 8.4 SetBiasDps 会置 calibrated，且拒绝超过上限的输入 */
    Ops_Fusion_SetBiasDps(-0.2f);
    CHECK(Ops_Fusion_IsCalibrated() == 1u, "SetBiasDps should set calibrated");
    CHECK_NEAR(Ops_Fusion_GetBiasDps(), -0.2f, 1e-6f);
    Ops_Fusion_SetBiasDps(50.0f);
    CHECK_NEAR(Ops_Fusion_GetBiasDps(), -0.2f, 1e-6f);   /* 超限被拒 */

    /* 8.5 ZERO：世界原点与航向归零，但不影响零偏/标定 */
    Ops_Fusion_ZeroPose();
    p = Ops_Fusion_GetPose();
    CHECK_NEAR(p->x_m, 0.0f, 1e-9f);
    CHECK_NEAR(p->y_m, 0.0f, 1e-9f);
    CHECK_NEAR(p->yaw_deg, 0.0f, 1e-9f);
    CHECK(Ops_Fusion_IsCalibrated() == 1u, "ZeroPose must not clear calibrated");

    /* 8.6 REINIT：清积分器但保留零偏与标定 */
    Ops_Fusion_Reset();
    p = Ops_Fusion_GetPose();
    CHECK_NEAR(p->yaw_deg, 0.0f, 1e-9f);
    CHECK(Ops_Fusion_IsCalibrated() == 1u, "Reset must keep calibrated");
    CHECK_NEAR(Ops_Fusion_GetBiasDps(), -0.2f, 1e-6f);
}

/* ===================== 抓包导出（供上位机工具跨语言校验） ===================== */
static void test_capture_dump(void)
{
    FILE      *f;
    uint8_t    buf[OPS_POSE_FRAME_LEN];
    ops_pose_t p;
    int32_t    i;
    int32_t    n = 0;

    f = fopen("capture_demo.bin", "wb");
    if (f == 0) {
        printf("[warn] cannot write capture_demo.bin\n");
        return;
    }

    memset(&p, 0, sizeof(p));
    for (i = 0; i < 200; ++i) {            /* 2s @100Hz 的一段圆弧 */
        float t = (float)i * 0.01f;

        p.x_m = 0.5f * sinf(1.5708f * t);
        p.y_m = 0.5f * (1.0f - cosf(1.5708f * t));
        p.yaw_deg = 90.0f * t;
        p.vx_mps = 0.5f * 1.5708f * cosf(1.5708f * t);
        p.vy_mps = 0.5f * 1.5708f * sinf(1.5708f * t);
        p.w_dps = 90.0f;
        p.status = OPS_ST_GYRO_OK;
        p.valid = 1u;

        if (Ops_Frame_BuildPose(buf, (uint8_t)i, &p) != 0u) {
            if (fwrite(buf, 1u, (size_t)OPS_POSE_FRAME_LEN, f) == (size_t)OPS_POSE_FRAME_LEN) {
                n++;
            }
        }
    }
    fclose(f);
    printf("[info] capture_demo.bin: %d frames written (for tools/pose_plot.py)\n", (int)n);
}

/* ===================== main ===================== */
int main(void)
{
    printf("=== MY_OPS 全局位姿模块 单元测试 ===\n");

    test_crc();
    test_cyz_proto();
    test_frames();
    test_geom();
    test_sample();
    test_zupt();
    test_yaw();
    test_bias();
    test_capture_dump();

    printf("--- 通过 %d 项，失败 %d 项 ---\n", g_pass, g_fail);
    if (g_fail == 0) {
        printf("ALL TESTS PASSED\n");
        return 0;
    }
    printf("SOME TESTS FAILED\n");
    return 1;
}





