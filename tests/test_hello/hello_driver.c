#include <linux/module.h>
#include <linux/kernel.h>
MODULE_LICENSE("GPL");

int init_module(void) {
    printk(KERN_INFO "Hello from Manjaro to Ubuntu!\n");
    return 0;
}

void cleanup_module(void) {
    printk(KERN_INFO "Goodbye, kernel world!\n");
}
