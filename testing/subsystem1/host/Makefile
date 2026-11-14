# Host unit tests for subsystem 1's pure modules (comms_proto, hmi_button),
# and the laptop codec (test_codec.py) against the same golden frames.
# Run on a PC:   make -C testing/subsystem1/host
# Needs a host C compiler (gcc or clang).  Not part of the firmware build.

SRC_DIR := ../../../app_program/subsystem1
CC      ?= cc
CFLAGS  := -std=c11 -Wall -Wextra -Werror -O1 -g -I$(SRC_DIR) -I$(SRC_DIR)/..
SAN     := -fsanitize=address,undefined -fno-omit-frame-pointer

# Rebuild whenever any app_program header changes: the tuning values
# (motion_cfg.h, line_cfg.h, ...) are #defines compiled into the tests.
HDRS    := $(wildcard ../../../app_program/*.h ../../../app_program/*/*.h)
test_s1 test_s1_bench: $(HDRS)

test_s1: test_s1.c golden.h $(SRC_DIR)/comms_proto.c $(SRC_DIR)/hmi_button.c
	$(CC) $(CFLAGS) $(SAN) -o $@ $(filter %.c,$^)

# Same tests with bench mode on (app_config.h APP_BENCH=1)
test_s1_bench: test_s1.c golden.h $(SRC_DIR)/comms_proto.c $(SRC_DIR)/hmi_button.c
	$(CC) $(CFLAGS) -DAPP_BENCH=1 $(SAN) -o $@ $(filter %.c,$^)

run: test_s1 test_s1_bench
	./test_s1
	./test_s1_bench
	python3 test_codec.py

.DEFAULT_GOAL := run
.PHONY: run clean
clean:
	rm -f test_s1 test_s1_bench
