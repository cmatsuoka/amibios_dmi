ifndef KERNEL
KERNEL = /lib/modules/$(shell uname -r)/build
endif

obj-m += amibios_dmi.o
amibios_dmi-objs := amibios_smi.o amibios_sysfs.o

all:
	make -C $(KERNEL) SUBDIRS=$(PWD) modules

clean:
	make -C $(KERNEL) SUBDIRS=$(PWD) clean

