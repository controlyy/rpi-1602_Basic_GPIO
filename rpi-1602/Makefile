
KDIR	:= /home/wilson/Downloads/linux
INCLUDES=-I. -I/home/wilson/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian/lib/gcc/arm-linux-gnueabihf/4.8.3/include
obj-m				:= gpio_module.o
#gpio_module-objs	:= gpio_module.o bcm2835.o

all:
	make -C $(KDIR) M=$(PWD) modules ARCH=arm CROSS_COMPILE=/home/wilson/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian/bin/arm-linux-gnueabihf-


clean:
	rm -f *.ko *.o *.mod.o *.mod.c *.symvers  modul* 


