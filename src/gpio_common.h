#ifndef GPIO_COMMON_H
#define GPIO_COMMON_H

#ifdef __KERNEL__
#include <linux/ioctl.h>
#else
#include <sys/ioctl.h>
#endif

/* Magic Number для нашего драйвера */
#define GPIO_IOC_MAGIC 'g'

/* Формальные границы HAL/UAPI */
#define GPIO_MAX_BANKS      5
#define GPIO_PINS_PER_BANK  32

/* Формальные значения направления и уровня */
#define GPIO_DIR_INPUT      0
#define GPIO_DIR_OUTPUT     1
#define GPIO_LEVEL_LOW      0
#define GPIO_LEVEL_HIGH     1


/**
 * @struct rk3588_gpio_args
 * @brief Структура для передачи аргументов через ioctl
 *
 * Используется для структурированного интерфейса ioctl команд
 * (более безопасный способ передачи аргументов по сравнению с raw int).
 */
struct rk3588_gpio_args {
    int pin;    /* 0-31 */
    int value;  /* Для данных: 0/1. Для направления: 0-вход, 1-выход */
};

/* Коды команд IOCTL */
#define GPIO_IOC_SET_DIRECTION _IOW(GPIO_IOC_MAGIC, 0x24, struct rk3588_gpio_args)
#define GPIO_IOC_WRITE_BIT     _IOW(GPIO_IOC_MAGIC, 0x25, struct rk3588_gpio_args)
#define GPIO_IOC_READ_BIT      _IOR(GPIO_IOC_MAGIC, 0x23, int)

#endif /* GPIO_COMMON_H */