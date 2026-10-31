# Host unit tests for the shared pure modules (app_math).
# Run on a PC:   make -C testing/shared/host
SRC_DIR := ../../../app_program
CC      ?= cc
CFLAGS  := -std=c11 -Wall -Wextra -Werror -O1 -g -D_DEFAULT_SOURCE -I$(SRC_DIR)
SAN     := -fsanitize=address,undefined -fno-omit-frame-pointer

test_shared: test_math.c $(SRC_DIR)/app_math.c
	$(CC) $(CFLAGS) $(SAN) -o $@ $^ -lm

run: test_shared
	./test_shared

.DEFAULT_GOAL := run
.PHONY: run clean
clean:
	rm -f test_shared
