# Host unit tests for subsystem 1's pure modules (comms_proto, hmi_button).
# Run on a PC:   make -C testing/subsystem1/host
# Needs a host C compiler (gcc or clang).  Not part of the firmware build.

SRC_DIR := ../../../app_program/subsystem1
CC      ?= cc
CFLAGS  := -std=c11 -Wall -Wextra -Werror -O1 -g -I$(SRC_DIR)
SAN     := -fsanitize=address,undefined -fno-omit-frame-pointer

test_s1: test_s1.c $(SRC_DIR)/comms_proto.c $(SRC_DIR)/hmi_button.c
	$(CC) $(CFLAGS) $(SAN) -o $@ $^

# Same tests with bench mode on (app_config.h APP_BENCH=1)
test_s1_bench: test_s1.c $(SRC_DIR)/comms_proto.c $(SRC_DIR)/hmi_button.c
	$(CC) $(CFLAGS) -DAPP_BENCH=1 $(SAN) -o $@ $^

run: test_s1 test_s1_bench
	./test_s1
	./test_s1_bench

.DEFAULT_GOAL := run
.PHONY: run clean
clean:
	rm -f test_s1 test_s1_bench
