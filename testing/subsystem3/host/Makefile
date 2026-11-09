# Host unit tests for subsystem 3's pure modules (code39, line_barcode,
# line_track), including the instructor's barcode sheets (vectors_slides.h,
# made by make_vectors.py from BARCODE.pptx).
# Run on a PC:   make -C testing/subsystem3/host
SRC_DIR := ../../../app_program
CC      ?= cc
CFLAGS  := -std=c11 -Wall -Wextra -Werror -O1 -g -I$(SRC_DIR)/subsystem3 -I$(SRC_DIR)
SAN     := -fsanitize=address,undefined -fno-omit-frame-pointer

# Rebuild whenever any app_program header changes: the tuning values
# (motion_cfg.h, line_cfg.h, ...) are #defines compiled into the tests.
HDRS    := $(wildcard ../../../app_program/*.h ../../../app_program/*/*.h)
test_s3: $(HDRS)
SRCS    := $(SRC_DIR)/subsystem3/code39.c $(SRC_DIR)/subsystem3/line_barcode.c \
           $(SRC_DIR)/subsystem3/line_track.c

test_s3: test_s3.c vectors_slides.h $(SRCS)
	$(CC) $(CFLAGS) $(SAN) -o $@ test_s3.c $(SRCS) -lm

run: test_s3
	./test_s3

.DEFAULT_GOAL := run
.PHONY: run clean
clean:
	rm -f test_s3
