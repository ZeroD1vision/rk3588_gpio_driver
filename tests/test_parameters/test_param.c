#include <linux/module.h>
#include <linux/init.h>
#include <linux/moduleparam.h>

static int mode = 0; // Значение по умолчанию
module_param(mode, int, S_IRUGO | S_IWUSR); // S_IWUSR разрешает менять через sysfs

static int __init test_init(void) {
    if (mode == 1) {
        printk(KERN_INFO "Mode: Агрессивный (Turbo)\n");
    } else {
        printk(KERN_INFO "Mode: Энергосберегающий\n");
    }
    return 0;
}

static void __exit test_exit(void) {
    printk(KERN_INFO "Модуль выгружен\n");
}

module_init(test_init);
module_exit(test_exit);
MODULE_LICENSE("GPL");