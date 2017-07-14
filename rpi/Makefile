CC = gcc
LD = gcc

CFLAGS = -Wall -g
LDFLAGS = -lmosquitto

PROG = bbq_thermo
SRC = main.c
OBJS = $(SRC:%.c=%.o)

all: $(PROG)

$(PROG): $(OBJS) 
	$(LD) $(LDFLAGS) $(OBJS) -o $(PROG)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

.PHONY: clean
clean:
	rm -f $(OBJS) $(PROG)
