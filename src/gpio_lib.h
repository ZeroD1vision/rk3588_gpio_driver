/* GPIO_LIB_H */
#ifndef GPIO_LIB_H
#define GPIO_LIB_H

#include "gpio_common.h"

/* Прототипы функций для таблицы операций плат */
typedef int (*gpio_ops_set_dir_t)(int fd, int pin, int is_output);
typedef int (*gpio_ops_write_t)(int fd, int pin, int value);
typedef int (*gpio_ops_read_t)(int fd, int pin);

typedef struct {
    const char *model_name;
    const char *cpu_lookup;    /* Строка для поиска в /proc/cpuinfo */
    const char *dev_prefix;
    int bank_count;
    
    /* Таблица операций (HAL) */
    gpio_ops_set_dir_t set_direction;
    gpio_ops_write_t write;
    gpio_ops_read_t read;
} BoardConfig;

// Публичный интерфейс
const BoardConfig* gpio_init(void);
int gpio_set_direction(int bank, int pin, int mode);
int gpio_write(int bank, int pin, int value);
int gpio_read(int bank, int pin);
void gpio_close(void);

#endif /* GPIO_LIB_H */