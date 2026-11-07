# Host unit tests for subsystem 2's pure controller (motion_core) against a
# simple motor model.  Run on a PC:   make -C testing/subsystem2/host
SRC_DIR := ../../../app_program
CC      ?= cc
CFLAGS  := -std=c11 -Wall -Wextra -Werror -O1 -g -I$(SRC_DIR)/subsystem2 -I$(SRC_DIR)
SAN     := -fsanitize=address,undefined -fno-omit-frame-pointer

# Rebuild whenever any app_program header changes: the tuning values
# (motion_cfg.h, line_cfg.h, ...) are #defines compiled into the tests.
HDRS    := $(wildcard ../../../app_program/*.h ../../../app_program/*/*.h)
test_s2: $(HDRS)

test_s2: test_s2.c $(SRC_DIR)/subsystem2/motion_core.c $(SRC_DIR)/subsystem2/motion_api.c $(SRC_DIR)/app_math.c
	$(CC) $(CFLAGS) $(SAN) -o $@ $(filter %.c,$^)

run: test_s2
	./test_s2

.DEFAULT_GOAL := run
.PHONY: run clean
clean:
	rm -f test_s2
