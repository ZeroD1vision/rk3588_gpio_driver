/* GPIO_LIB_H */
#ifndef GPIO_LIB_H
#define GPIO_LIB_H

#include "gpio_common.h"

typedef struct {
    const char *model_name;
    const char *dev_prefix;
    int bank_count;
} BoardConfig;

// Публичный интерфейс
const BoardConfig* gpio_init(void);
int gpio_set_direction(int bank, int pin, int mode);
int gpio_write(int bank, int pin, int value);
int gpio_read(int bank, int pin);
void gpio_close(void);

#endif /* GPIO_LIB_H */