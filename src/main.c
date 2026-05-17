#include "gpio_lib.h"
#include <stdio.h>

/* ============================================================================
 * ПРИМЕР ИСПОЛЬЗОВАНИЯ HAL
 * ============================================================================ */

 int main() {
    const BoardConfig *board = gpio_init();
    if (!board) {
        fprintf(stderr, "Error: Unknown board or /proc/cpuinfo unreadable!\n");
        return 1;
    }

    printf("Successfully initialized HAL\n");
    printf("Detected board : %s\n", board->model_name);
    printf("Available banks: %d\n", board->bank_count);

    // Логика работы прозрачна и не меняется от смены платформы
    if (gpio_set_direction(1, 15, GPIO_DIR_OUTPUT) < 0) {
        fprintf(stderr, "Failed to set direction\n");
        gpio_close();
        return 1;
    }
    
    gpio_write(1, 15, GPIO_LEVEL_HIGH);
    printf("Bank 1, Pin 15 set to HIGH\n");

    gpio_close();
    return 0;
}