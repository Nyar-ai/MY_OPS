# MY_OPS 全局位姿模块 - PC 端单元测试（可选：需要 make，Windows 上用 run_tests.bat 亦可）
CC      ?= gcc
CFLAGS  := -std=c99 -Wall -Wextra -Wno-unused-parameter -O1 -I ../Core/Inc
SRC     := test_ops.c \
           ../Core/Src/ops_crc.c \
           ../Core/Src/ops_geom.c \
           ../Core/Src/ops_cyz_proto.c \
           ../Core/Src/ops_frame.c \
           ../Core/Src/ops_fusion.c \
           ../Core/Src/ops_sample.c

test_ops.exe: $(SRC)
	$(CC) $(CFLAGS) $(SRC) -o $@

test: test_ops.exe
	./test_ops.exe

clean:
	-rm -f test_ops.exe

.PHONY: test clean
