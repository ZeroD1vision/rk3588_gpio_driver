#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include "gpio_lib.h"

/* ============================================================================
 * НИЗКОУРОВНЕВЫЕ РЕАЛИЗАЦИИ ДЛЯ BANANA PI (RK3588)
 * ============================================================================ */
static int rk3588_set_direction(int fd, int pin, int is_output) {
    struct gpio_ioctl_args args = { .pin = pin, .value = is_output };
    return ioctl(fd, GPIO_IOC_SET_DIRECTION, &args);
}

static int rk3588_write(int fd, int pin, int value) {
    struct gpio_ioctl_args args = { .pin = pin, .value = value };
    return ioctl(fd, GPIO_IOC_WRITE_BIT, &args);
}

static int rk3588_read(int fd, int pin) {
    return ioctl(fd, GPIO_IOC_READ_BIT, pin);
}

/* ============================================================================
 * НИЗКОУРОВНЕВЫЕ РЕАЛИЗАЦИИ ДЛЯ BEAGLEBONE BLACK (AM335x)
 * (Пример адаптации: если у BBB драйвер принимает сырой int или другую логику)
 * ============================================================================ */
static int bbb_set_direction(int fd, int pin, int is_output) {
    /* Какая то другая логика */
    (void)fd; (void)pin; (void)is_output;
    return -1; /* Not implemented */
}

static int bbb_write(int fd, int pin, int value) {
    /* Какая то другая логика */
    (void)fd; (void)pin; (void)value;
    return -1; /* Not implemented */
}

static int bbb_read(int fd, int pin) {
    /* Какая то другая логика */
    (void)fd; (void)pin;
    return -1; /* Not implemented */
}

/* ============================================================================
 * БАЗА ДАННЫХ ПОДДЕРЖИВАЕМЫХ ПЛАТ
 * ============================================================================ */
static const BoardConfig supported_boards[] = {
    {
        .model_name = "Banana Pi BPI-M7 (RK3588)",
        .cpu_lookup = "RK3588",
        .dev_prefix = "/dev/gpio",
        .bank_count = 5,
        .set_direction = rk3588_set_direction,
        .write = rk3588_write,
        .read = rk3588_read
    },
    {
        .model_name = "BeagleBone Black (AM335x)",
        .cpu_lookup = "AM335X", // Характерная строка в /proc/cpuinfo или Hardware поле
        .dev_prefix = "/dev/bbb_gpio",
        .bank_count = 4,
        .set_direction = bbb_set_direction,
        .write = bbb_write,
        .read = bbb_read
    },
    { NULL, NULL, NULL, 0, NULL, NULL, NULL }
};

static int *bank_fds = NULL;
static const BoardConfig *current_board = NULL;

static int get_fd(int bank);

static int gpio_validate_bank(int bank)
{
    if (!current_board || !bank_fds)
        return -1;
    if (bank < 0 || bank >= current_board->bank_count)
        return -1;
    return 0;
}

static int gpio_validate_pin(int pin)
{
    if (pin < 0 || pin >= GPIO_PINS_PER_BANK)
        return -1;
    return 0;
}

static int gpio_do_ioctl_struct(int bank, unsigned long cmd, int pin, int value)
{
    if (gpio_validate_pin(pin) < 0) return -1;

    int fd = get_fd(bank);
    if (fd < 0) return -1;

    struct gpio_ioctl_args args = { .pin = pin, .value = value };
    return ioctl(fd, cmd, &args);
}

static int get_fd(int bank) {
    if (gpio_validate_bank(bank) < 0)
        return -1;
    if (bank_fds[bank] != -1)
        return bank_fds[bank];

    char path[32];
    snprintf(path, sizeof(path), "%s%d", current_board->dev_prefix, bank);
    bank_fds[bank] = open(path, O_RDWR);
    return bank_fds[bank];
}

/**
 * @brief Автоопределение платы через /proc/cpuinfo
 */
const BoardConfig* gpio_init(void) {
    if (current_board)
        return current_board;

    FILE *f = fopen("/proc/cpuinfo", "r");
    if (!f) return NULL;

    char line[256];
    int i;

    while (fgets(line, sizeof(line), f)) {
        for (i = 0; supported_boards[i].model_name; i++) {
            if (strstr(line, supported_boards[i].cpu_lookup)) {
                current_board = &supported_boards[i];
                bank_fds = malloc(sizeof(int) * current_board->bank_count);
                if (!bank_fds) {
                    fclose(f);
                    current_board = NULL;
                    return NULL;
                }
                for (int j = 0; j < current_board->bank_count; j++)
                    bank_fds[j] = -1;
                fclose(f);
                return current_board;
            }
        }
    }
    fclose(f);
    return NULL;
}

/* ============================================================================
 * ПУБЛИЧНЫЙ АБСТРАКТНЫЙ ИНТЕРФЕЙС (HAL)
 * ============================================================================ */

int gpio_set_direction(int bank, int pin, int is_output) {
    if (gpio_validate_pin(pin) < 0) return -1;
    int fd = get_fd(bank);
    if (fd < 0) return -1;

    // Вызов операции через указатель в конфигурации текущей платы
    return current_board->set_direction(fd, pin, is_output);
}

int gpio_write(int bank, int pin, int value) {
    if (gpio_validate_pin(pin) < 0) return -1;
    int fd = get_fd(bank);
    if (fd < 0) return -1;

    return current_board->write(fd, pin, value);
}

int gpio_read(int bank, int pin) {
    if (gpio_validate_pin(pin) < 0) return -1;
    int fd = get_fd(bank);
    if (fd < 0) return -1;

    return current_board->read(fd, pin);
}

void gpio_close(void) {
    if (!bank_fds || !current_board) return;
    
    for (int i = 0; i < current_board->bank_count; i++) {
        if (bank_fds[i] != -1) {
            close(bank_fds[i]);
        }
    }
    free(bank_fds);
    bank_fds = NULL;
    current_board = NULL;
}