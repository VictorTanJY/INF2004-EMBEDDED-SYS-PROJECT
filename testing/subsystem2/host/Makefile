# Host unit tests for subsystem 2's pure controller (motion_core) against a
# simple motor model.  Run on a PC:   make -C testing/subsystem2/host
SRC_DIR := ../../../app_program
CC      ?= cc
CFLAGS  := -std=c11 -Wall -Wextra -Werror -O1 -g -I$(SRC_DIR)/subsystem2 -I$(SRC_DIR)
SAN     := -fsanitize=address,undefined -fno-omit-frame-pointer

test_s2: test_s2.c $(SRC_DIR)/subsystem2/motion_core.c $(SRC_DIR)/subsystem2/motion_api.c $(SRC_DIR)/app_math.c
	$(CC) $(CFLAGS) $(SAN) -o $@ $^

run: test_s2
	./test_s2

.DEFAULT_GOAL := run
.PHONY: run clean
clean:
	rm -f test_s2
