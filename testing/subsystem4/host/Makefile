# Host unit tests for subsystem 4's pure modules (imu_core, safety_core).
# Run on a PC:   make -C testing/subsystem4/host
SRC_DIR := ../../../app_program
CC      ?= cc
CFLAGS  := -std=c11 -Wall -Wextra -Werror -O1 -g -D_DEFAULT_SOURCE -I$(SRC_DIR)/subsystem4 -I$(SRC_DIR)
SAN     := -fsanitize=address,undefined -fno-omit-frame-pointer

# Rebuild whenever any app_program header changes: the tuning values
# (motion_cfg.h, line_cfg.h, ...) are #defines compiled into the tests.
HDRS    := $(wildcard ../../../app_program/*.h ../../../app_program/*/*.h)
test_s4: $(HDRS)
SRCS    := $(SRC_DIR)/subsystem4/imu_core.c $(SRC_DIR)/subsystem4/safety_core.c $(SRC_DIR)/app_math.c

test_s4: test_s4.c $(SRCS)
	$(CC) $(CFLAGS) $(SAN) -o $@ test_s4.c $(SRCS) -lm

run: test_s4
	./test_s4

.DEFAULT_GOAL := run
.PHONY: run clean
clean:
	rm -f test_s4
