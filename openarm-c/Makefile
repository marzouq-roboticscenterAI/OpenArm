# OpenArm-C: CAN control + gamepad + HTTP dashboard, all in C.
CC      ?= cc
CFLAGS  ?= -std=c11 -O2 -Wall -Wextra -Wpedantic
LDFLAGS ?= -lpthread -lm

OBJ = damiao.o socketcan.o gamepad.o calib.o ranger.o ds2c.o control.o httpd.o log.o main.o
BIN = openarm

all: $(BIN)

$(BIN): $(OBJ)
	$(CC) $(CFLAGS) -o $@ $(OBJ) $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJ) $(BIN)

.PHONY: all clean
