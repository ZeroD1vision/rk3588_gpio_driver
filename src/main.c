#include "gpio_lib.h"
#include <stdio.h>

int main() {
    const BoardConfig *board = gpio_init();
    if (!board) {
        fprintf(stderr, "Error: Unknown board!\n");
        return 1;
    }

    printf("Detected board: %s with %d banks\n", board->model_name, board->bank_count);

    // Работаем абстрактно
    gpio_set_direction(1, 15, 1); // Bank 1, Pin 15, Output
    gpio_write(1, 15, 1);   // Set High

    gpio_close();

    return 0;
}