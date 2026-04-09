# Имя твоего файла (если файл называется rk3588_gpio.c)
obj-m += rk3588_gpio.o

# Путь к заголовкам текущего ядра
KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean