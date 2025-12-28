CC = m68k-amigaos-gcc
CFLAGS = -m68040 -O3
LDFLAGS = -noixemul

TARGET1 = ftupdate
TARGET2 = fttool

all: $(TARGET1) $(TARGET2)

$(TARGET1): ftupdate.o rga_host.o
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

$(TARGET2): fttool.o rga_host.o
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f *.o $(TARGET1) $(TARGET2)

.PHONY: all clean
