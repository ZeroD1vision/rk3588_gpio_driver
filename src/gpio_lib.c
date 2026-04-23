#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include "gpio_lib.h"

/* База данных поддерживаемых плат */
static const BoardConfig supported_boards[] = {
    {"RK3588", "/dev/gpio", 5},   /* Rockchip RK3588 (BananaPi BPI-M7)*/
    /* Еще что-нибудь */
    {NULL, NULL, 0},
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
    int fd;
    struct rk3588_gpio_args args = { .pin = pin, .value = value };

    if (gpio_validate_pin(pin) < 0)
        return -1;

    fd = get_fd(bank);
    if (fd < 0)
        return -1;

    return ioctl(fd, cmd, &args);
}

/**
 * @brief Автоопределение платы через /proc/cpuinfo
 */
const BoardConfig* gpio_init(void) {
    FILE *f = fopen("/proc/cpuinfo", "r");
    char line[256];
    int i;

    if (current_board)
        return current_board;

    if (!f) return NULL;

    while (fgets(line, sizeof(line), f)) {
        for (i = 0; supported_boards[i].model_name; i++) {
            if (strstr(line, supported_boards[i].model_name)) {
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

int gpio_set_direction(int bank, int pin, int is_output) {
    return gpio_do_ioctl_struct(bank, GPIO_IOC_SET_DIRECTION, pin, is_output);
}

int gpio_write(int bank, int pin, int value) {
    return gpio_do_ioctl_struct(bank, GPIO_IOC_WRITE_BIT, pin, value);
}

int gpio_read(int bank, int pin) {
    int fd = get_fd(bank);

    if (fd < 0)
        return -1;
    if (gpio_validate_pin(pin) < 0)
        return -1;

    /* Для чтения передаем номер пина напрямую как значение */
    return ioctl(fd, GPIO_IOC_READ_BIT, pin);
}

void gpio_close(void) {
    if (!bank_fds || !current_board)
        return;
    for (int i = 0; i < current_board->bank_count; i++) {
        if (bank_fds[i] != -1)
            close(bank_fds[i]);
    }
    free(bank_fds);
    bank_fds = NULL;
    current_board = NULL;
}