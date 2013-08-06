ifndef KERNEL
KERNEL = /lib/modules/$(shell uname -r)/build
endif

obj-m += amibios_dmi.o
amibios_dmi-objs := amibios_smi.o amibios_sysfs.o

all: modules

modules clean:
	make -C $(KERNEL) SUBDIRS=$(PWD) $@

