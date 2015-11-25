ARMCC=arm-none-linux-gnueabi-gcc
CC=gcc
CFLAGS=-Wall -g -I../../include 
LDFLAGS=-Wall -g 
LIBS=-L../../algorithm/osp -lOSP


OBJS=OSPDaemon_queue.o OSPDaemon.o OSPDaemon_iio.o OSPDaemon_config.o \
		OSPDaemon_input.o OSPDaemon_inputreader.o	\
		OSPDaemon_filecsv.o 	\
		OSPDaemon_pm.o OSPDaemon_driver.o

OSPDaemon: $(OBJS) 
	$(CC) $(LDFLAGS) $(OBJS) -o OSPDaemon $(LIBS)

clean:
	rm -f *.o OSPDaemon
