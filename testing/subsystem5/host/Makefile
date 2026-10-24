# Host unit tests for subsystem 5's pure modules (mission_core, obstacle_core).
# Run on a PC:   make -C testing/subsystem5/host
SRC_DIR := ../../../app_program
CC      ?= cc
CFLAGS  := -std=c11 -Wall -Wextra -Werror -O1 -g -D_DEFAULT_SOURCE -I$(SRC_DIR)/subsystem5 -I$(SRC_DIR)
SAN     := -fsanitize=address,undefined -fno-omit-frame-pointer
SRCS    := $(SRC_DIR)/subsystem5/mission_core.c $(SRC_DIR)/subsystem5/obstacle_core.c $(SRC_DIR)/app_math.c

test_s5: test_s5.c $(SRCS)
	$(CC) $(CFLAGS) $(SAN) -o $@ test_s5.c $(SRCS) -lm

test_s5_bench: test_s5.c $(SRCS)
	$(CC) $(CFLAGS) -DAPP_BENCH=1 $(SAN) -o $@ test_s5.c $(SRCS) -lm

run: test_s5 test_s5_bench
	./test_s5
	./test_s5_bench

.DEFAULT_GOAL := run
.PHONY: run clean
clean:
	rm -f test_s5 test_s5_bench
