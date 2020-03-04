TARGET=hw10feng

SOURCES=import_registers.c \
        enable_pwm_clock.c \
        LSM9DS1.c \
        transact_SPI.c \
        wait_period.c \
        time_difference.c \
	wait_key.c \
        hw10feng.c

OBJECTS=$(patsubst %.c,%.o,$(SOURCES))

all: $(OBJECTS)
	g++ raspicam_wrapper.cpp -c -o raspicam_wrapper.o
	g++ $(OBJECTS) raspicam_wrapper.o -lpthread -lm -o $(TARGET) -lraspicam

clean:
	rm -f $(OBJECTS) $(TARGET)

%.o:%.c
	gcc -c $< -o $@
