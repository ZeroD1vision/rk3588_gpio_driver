#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>         // kzalloc
#include <linux/io.h>           // ioremap
#include <linux/fs.h>           // alloc_chrdev_region
#include <linux/cdev.h>
#include <linux/device.h>

#define GPIO_BANKS_COUNT 5

/* Базовые физические адреса из TRM (Chapter 1.1 Address Mapping) */
static const unsigned long rk3588_gpio_phys_bases[] = {
    0xFD8A0000, /* GPIO0 */
    0xFEC20000, /* GPIO1 */
    0xFEC30000, /* GPIO2 */
    0xFEC40000, /* GPIO3 */
    0xFEC50000  /* GPIO4 */
};

typedef struct {
    unsigned int dr_l;          /* 0x0000: Данные (Low 0-15) - для Ф2 */
    unsigned int dr_h;          /* 0x0004: Данные (High 16-31) - для Ф2 */
    unsigned int ddr_l;         /* 0x0008: Направление (Low) - для Ф1 */
    unsigned int ddr_h;         /* 0x000C: Направление (High) - для Ф1 */

    /* Пропускаем прерывания (0x0010 - 0x0044), они нам пока не нужны */
    unsigned int reserved1[14];

    unsigned int dbclk_div_con; /* 0x0048 */
    unsigned int reserved2;     /* 0x004C (пропуск) */
    unsigned int int_status;    /* 0x0050 */
    unsigned int reserved3;     /* 0x0054 (пропуск) */
    unsigned int int_rawstatus; /* 0x0058 */
    unsigned int reserved4[1];  /* 0x005C (пропуск до 0x60) */

    unsigned int port_eoi_l;    /* 0x0060 */
    unsigned int port_eoi_h;    /* 0x0064 */
    unsigned int reserved5[2];  /* 0x0068 - 0x006C */

    unsigned int ext_port;      /* 0x0070: Чтение состояния - для Ф3 */

    /* Остальное (Version ID и т.д.) пока не описавывем*/
} rk3588_gpio_regs_t;

typedef struct
{
    void __iomem *base;      /* Виртуальный адрес после ioremap */
    int id;                  /* Номер банка (0, 1, 2, 3, 4) */
    dev_t dev_num;           /* Мажорный + Минорный номер */
    struct cdev cdev;        /* Символьное устройство */
    struct device *device;   /* Указатель на созданное устройство в /dev */
} rk3588_gpio_bank_t;

static const struct file_operations gpio_fops = {
    .owner = THIS_MODULE,
    /* пока пусто */
};

/* Глобальный массив указателей на наши бабки */
static rk3588_gpio_bank_t *g_banks[GPIO_BANKS_COUNT];
static int g_major;           /* Место для выделенного Major номер */
static struct class *g_class; /* Общий класс для всех 5 устройств (потом udev сам пробежится и растыкает) */

static int __init rk3588_gpio_init(void) {
    int err, i;
    dev_t dev_base;

    err = alloc_chrdev_region(&dev_base, 0, GPIO_BANKS_COUNT, "rk3588_gpio");
    if (err < 0) {
        printk(KERN_ERR "Failed to allocate char device region\n");
        goto fail_chrdev_region;
    }

    g_major = MAJOR(dev_base);

    g_class = class_create(THIS_MODULE, "rk3588_gpio_class");
    if (IS_ERR(g_class)) {
        err = PTR_ERR(g_class);
        printk(KERN_ERR "Failed to create class\n");
        goto fail_class_create;
    }

    for(int i = 0; i < GPIO_BANKS_COUNT; i++) {
        g_banks[i] = kzalloc(sizeof(rk3588_gpio_bank_t), GFP_KERNEL);
        if (!g_banks[i]) {
            err = -ENOMEM;
            goto err_cleanup_banks;
        }

        g_banks[i]->id = i;
        g_banks[i]->dev_num = MKDEV(g_major, i);

        /* Мапим регистры */
        g_banks[i]->base = ioremap(rk3588_gpio_phys_bases[i], sizeof(rk3588_gpio_regs_t));
        if (!g_banks[i]->base) {
            err = -EIO;
            goto err_free_bank_mem;
        }

        /* Инит cdev */
        cdev_init(&g_banks[i]->cdev, &gpio_fops);
        err = cdev_add(&g_banks[i]->cdev, g_banks[i]->dev_num, 1);
        if (err) goto err_unmap;
        
        /* Создаем файл /dev/gpioX */
        g_banks[i]->device = device_create(g_class, NULL, g_banks[i]->dev_num, NULL, "gpio%d", i);
        if (IS_ERR(g_banks[i]->device)) {
            err = PTR_ERR(g_banks[i]->device);
            goto err_cdev_del;
        }
    }

    printk("RK3588 GPIO driver initialized successfully\n");
    return 0;

    /* --- ЦЕПОЧКА ОБРАБОТКИ ОШИБОК --- */

err_cdev_del:
    cdev_del(&g_banks[i]->cdev);
err_unmap:
    iounmap(g_banks[i]->base);
err_free_bank_mem:
    kfree(g_banks[i]);
    g_banks[i] = NULL;
err_cleanup_banks:
    /* Эта метка чистит все ПРЕДЫДУЩИЕ успешно созданные банки */
    while (--i >= 0) {
        device_destroy(g_class, g_banks[i]->dev_num);
        cdev_del(&g_banks[i]->cdev);
        iounmap(g_banks[i]->base);
        kfree(g_banks[i]);
        g_banks[i] = NULL;
    }
    class_destroy(g_class);
fail_chrdev_region:
    unregister_chrdev_region(MKDEV(g_major, 0), GPIO_BANKS_COUNT);
    return err;
}

static void __exit rk3588_gpio_exit(void) {
    int i;

    for (i = 0; i < GPIO_BANKS_COUNT; i++) {
        if (g_banks[i]) {
            /* 1. Убираем файл из /dev */
            if (g_banks[i]->device)
                device_destroy(g_class, g_banks[i]->dev_num);

            /* 2. Удаляем символьное устройство из системы */
            cdev_del(&g_banks[i]->cdev);

            /* 3. Разрываем связь с регистрами */
            if (g_banks[i]->base)
                iounmap(g_banks[i]->base);

            /* 4. Освобождаем память структуры */
            kfree(g_banks[i]);
            g_banks[i] = NULL;
        }
    }

    /* 5. Уничтожаем класс */
    if (g_class)
        class_destroy(g_class);

    /* 6. Возвращаем диапазон мажор/минор */
    unregister_chrdev_region(MKDEV(g_major, 0), GPIO_BANKS_COUNT);

    printk(KERN_INFO "RK3588 GPIO driver unloaded\n");
}

module_init(rk3588_gpio_init);
module_exit(rk3588_gpio_exit);

/* Примерная запланированная схема */
/**
 * =================================================================================
 * ПРИМЕРНАЯ ЗАПЛАНИРОВАННАЯ СХЕМА ДРАЙВЕРА
 * =================================================================================
 * ---------------------------------------------------------------------------------
 * * 1. ПОДГОТОВКА ДАННЫХ (ВЕРХУШКА ФАЙЛА) (уже описано выше)
 * ---------------------------------------------------------------------------------
 * - СТРУКТУРА rk3588_gpio_bank:
 * Опишем поля (base, cdev, bank_num, is_open для Ф6).
 * * - МАССИВ АДРЕСОВ:
 * Константный список из 5 физических баз.
 * * - МАССИВ УКАЗАТЕЛЕЙ:
 * static struct rk3588_gpio_bank *banks[5]
 * ---------------------------------------------------------------------------------
 * * 2. ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ИНИЦИАЛИЗАЦИИ
 * ---------------------------------------------------------------------------------
 * - bank_setup_hardware(bank, phys_addr):
 * Только ioremap.
 * * - bank_setup_cdev(bank, index):
 * cdev_init + cdev_add.
 * ---------------------------------------------------------------------------------
 * * 3. ГЛАВНЫЙ ЦИКЛ (MODULE_INIT)
 * ---------------------------------------------------------------------------------
 * 1. Вызовем alloc_chrdev_region (возьмем Major и 5 Minor-ов).
 * 2. Создадим class_create.
 * 3. Цикл на 5 итераций:
 * - kzalloc (выделим память под структуру).
 * - Вызовем сосбтвенные функции подготовки (hardware, cdev).
 * - device_create (создадим файл в /dev/).
 * ---------------------------------------------------------------------------------
 * * 4. ПОЛЬЗОВАТЕЛЬСКИЕ ФУНКЦИИ (FOPS)
 * ---------------------------------------------------------------------------------
 * - gpio_open:
 * Самый важный момент. Сюда придет запрос, и мы должеы из inode вытащить 
 * указатель на нужный банк (через container_of если разрешат) и проверить, не занят ли 
 * он (is_open).
 * * - gpio_release:
 * Сбрасываем флаг is_open.
 * * - gpio_ioctl:
 * Тут будет вся магия чтения/записи в регистры через base + оффсет.
 
 * * ---------------------------------------------------------------------------------
 * ЛОГИКА ЖИЗНЕННОГО ЦИКЛА (примерно)
 * ---------------------------------------------------------------------------------
 * module_init:
 * - alloc_chrdev_region -> g_major
 * - class_create -> g_class
 * - for (i = 0; i < GPIO_BANKS_COUNT; i++) {
 * - kzalloc для g_banks[i]
 * - bank_setup_hardware(g_banks[i], rk3588_gpio_phys_bases[i])
 * - bank_setup_cdev(g_banks[i], i)
 * - device_create(g_class, NULL, g_banks[i]->dev_num, NULL, "gpio%d", i)
 * }
 * * module_exit:
 * - for (i = 0; i < GPIO_BANKS_COUNT; i++) {
 * - device_destroy(g_class, g_banks[i]->dev_num)
 * - cdev_del(&g_banks[i]->cdev)
 * - kfree(g_banks[i])
 * }
 * - class_destroy(g_class)
 * - unregister_chrdev_region(MKDEV(g_major, 0), GPIO_BANKS_COUNT)
 * =================================================================================
 */



MODULE_LICENSE("GPL");
MODULE_AUTHOR("Artemy Koshkin");
MODULE_DESCRIPTION("Rockchip RK3588 GPIO Driver");