obj-m += otarover.o
otarover-objs := otarover_blue_drv.o otarover_blue_io.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
