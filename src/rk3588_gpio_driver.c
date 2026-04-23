/**
 * @file rk3588_gpio_driver.c
 * @brief Драйвер GPIO для Rockchip RK3588
 *
 * Реализует символьное устройство для управления 5 GPIO банками (GPIO0-GPIO4)
 * процессора RK3588. Каждый банк содержит до 32 пинов.
 *
 * Архитектура:
 * - GPIO0-GPIO4: по одному символьному устройству /dev/gpio[0-4]
 * - Управление через ioctl команды (установка направления, чтение/запись)
 * - Использование атомарных операций для защиты от конкуренции при открытии устройств
 *
 * @author Artemy Narziev
 * @date 2026-20-04
 * @license GPL
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>         // kzalloc
#include <linux/io.h>           // ioremap
#include <linux/fs.h>           // alloc_chrdev_region
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>    // copy_from_user
#include <linux/atomic.h>     // atomic_t, atomic_set и atomic_cmpxchg

#include "gpio_common.h"


/* ============================================================================
 * МАКРОСЫ И КОНСТАНТЫ
 * ============================================================================ */

/** @brief Количество GPIO банков */
#define GPIO_BANKS_COUNT GPIO_MAX_BANKS

// Взяли все свободное, основываясь на https://www.kernel.org/doc/Documentation/userspace-api/ioctl/ioctl-number.rst
#define RK3588_GPIO_MAGIC GPIO_IOC_MAGIC

/** @brief Установить направление GPIO (сырая версия) */
#define RK3588_GPIO_SET_DIRECTION _IOW(RK3588_GPIO_MAGIC, 0x21, int)

/** @brief Записать бит GPIO (сырая версия) */
#define RK3588_GPIO_WRITE_BIT     _IOW(RK3588_GPIO_MAGIC, 0x22, int)

/* UAPI ioctl (структурированный интерфейс) */
/** @brief Установить направление GPIO (структурированная версия) */
#define RK3588_GPIO_SET_DIRECTION_STRUCT GPIO_IOC_SET_DIRECTION
/** @brief Записать бит GPIO (структурированная версия) */
#define RK3588_GPIO_WRITE_BIT_STRUCT     GPIO_IOC_WRITE_BIT
/** @brief Прочитать бит GPIO */
#define RK3588_GPIO_READ_BIT             GPIO_IOC_READ_BIT

// /** @brief Прочитать бит GPIO */
// #define RK3588_GPIO_READ_BIT      _IOR(RK3588_GPIO_MAGIC, 0x23, int)

// /** @brief Установить направление GPIO (структурированная версия) */
// #define RK3588_GPIO_SET_DIRECTION_STRUCT _IOW(RK3588_GPIO_MAGIC, 0x24, struct rk3588_gpio_args)

// /** @brief Записать бит GPIO (структурированная версия) */
// #define RK3588_GPIO_WRITE_BIT_STRUCT     _IOW(RK3588_GPIO_MAGIC, 0x25, struct rk3588_gpio_args)

/** @brief Для совместимости с сырым int */
#define RK3588_GPIO_SET_DIRECTION_RAW    RK3588_GPIO_SET_DIRECTION
#define RK3588_GPIO_WRITE_BIT_RAW        RK3588_GPIO_WRITE_BIT

/** @brief Количество бит в регистре GPIO */
#define RK_GPIO_BITS_PER_REG 16

/** @brief Макрос для вычисления маски записи */
#define RK_GPIO_WRITE_MASK(bit) (1U << ((bit) + 16))

/* ============================================================================
 * АППАРАТНЫЕ КОНСТАНТЫ
 * ============================================================================ */

/**
 * @brief Физические адреса базовых регистров GPIO
 *
 * Адреса банков GPIO на процессоре RK3588. Каждый банк имеет свой базовый адрес, 
 * от которого идут регистры управления и состояния.
 * Согласно TRM RK3588 ((Chapter 1, Section 1.1,  Address Mapping)):
 * https://www.scs.stanford.edu/~zyedidia/docs/rockchip/rk3588_part1.pdf (страница 16-20)
 */
static const unsigned long rk3588_gpio_phys_bases[] = {
    0xFD8A0000, /* GPIO0 */
    0xFEC20000, /* GPIO1 */
    0xFEC30000, /* GPIO2 */
    0xFEC40000, /* GPIO3 */
    0xFEC50000  /* GPIO4 */
};

/* ============================================================================
 * СТРУКТУРЫ ДАННЫХ
 * ============================================================================ */

/**
 * @struct rk3588_gpio_regs_t
 * @brief Структура хранения всех регистров GPIO контроллера RK3588
 *
 * Описывает расположение регистров управления и состояния GPIO в памяти.
 * Согласно TRM RK3588 (Chapter 20 GPIO, Section 20.4.1 Registers Summary):
 * https://www.scs.stanford.edu/~zyedidia/docs/rockchip/rk3588_part1.pdf (страница 1470)
 */
typedef struct {
    unsigned int dr_l;          /* 0x0000: Данные (Low 0-15) */
    unsigned int dr_h;          /* 0x0004: Данные (High 16-31) */
    unsigned int ddr_l;         /* 0x0008: Направление (Low) */
    unsigned int ddr_h;         /* 0x000C: Направление (High) */

    /** @brief Зарезервировано (0x0010-0x0044): регистры прерываний */
    unsigned int reserved1[14];

    unsigned int dbclk_div_con; /* 0x0048 Debounce Clock Division */
    unsigned int reserved2;     /* 0x004C (пропуск) */
    unsigned int int_status;    /* 0x0050 Interrupt Status */
    unsigned int reserved3;     /* 0x0054 (пропуск) */
    unsigned int int_rawstatus; /* 0x0058 Interrupt Raw Status */
    unsigned int reserved4[1];  /* 0x005C (пропуск до 0x60) */

    unsigned int port_eoi_l;    /* 0x0060 Port EOI Low: очистка флагов прерывания 0-15 */
    unsigned int port_eoi_h;    /* 0x0064 Port EOI High: очистка флагов прерывания 16-31 */
    unsigned int reserved5[2];  /* 0x0068 - 0x006C */

    /** @brief External Port (0x0070): регистр чтения состояния пинов */
    unsigned int ext_port;

    // Остальное пока не описываем
} rk3588_gpio_regs_t;

/**
 * @struct rk3588_gpio_bank_t
 * @brief Управляющая структура для банка GPIO
 *
 * Хранит информацию о конкретном GPIO банке, включая виртуальный адрес
 * регистров, информацию о символьном устройстве и состояние банка.
 */
typedef struct {
    /** @brief Виртуальный адрес регистров (после ioremap) */
    void __iomem *base;

    /** @brief Идентификатор банка (0-4) */
    int id;

    /** @brief Номер устройства (мажорный + минорный номер) */
    dev_t dev_num;

    /** @brief Структура символьного устройства */
    struct cdev cdev;

    /** @brief Указатель на устройство в sysfs (/dev/gpio*) */
    struct device *device;

    /** @brief Атомарный флаг занятости (0=свободен, 1=занят) */
    atomic_t is_open;
} rk3588_gpio_bank_t;


/* ============================================================================
 * ПРОТОТИПЫ ФУНКЦИЙ
 * ============================================================================ */

static int rk3588_gpio_open(struct inode *inode, struct file *filp);
static int rk3588_gpio_release(struct inode *inode, struct file *filp);
static long rk3588_gpio_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static int rk3588_setup_bank(int i);
static int rk3588_validate_pin(int pin);

/* ============================================================================
 * FILE_OPERATIONS
 * ============================================================================ */

/**
 * @brief Структура операций символьного устройства
 *
 * Определяет функции для файловых операций GPIO устройств.
 * Используется общая структура для всех 5 GPIO банков.
 */
static const struct file_operations gpio_fops = {
    .owner = THIS_MODULE,
    .open = rk3588_gpio_open,
    .unlocked_ioctl = rk3588_gpio_ioctl, // Используем unlocked_ioctl для современных ядер
    .release = rk3588_gpio_release,
    /* пока пусто */
};

/* ============================================================================
 * ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ
 * ============================================================================ */

/** @brief Массив указателей на структуры GPIO банков */
static rk3588_gpio_bank_t *g_banks[GPIO_BANKS_COUNT];

/** @brief Выделенный мажорный номер устройства */
static int g_major;

/** @brief Класс символьных устройств для GPIO (потом udev сам пробежится и растыкает) */
static struct class *g_class;


/* ============================================================================
 * ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ПАРСИНГА АРГУМЕНТОВ
 * ============================================================================ */

/**
 * @brief Распаковка аргументов из структуры (безопасный способ)
 *
 * Цивилизованный способ передачи параметров через ioctl с использованием
 * структуры вместо прямого кодирования в целое число.
 *
 * @param[in] arg      Указатель на структуру из пользовательского пространства
 * @param[out] pin     Указатель на переменную для номера пина
 * @param[out] val     Указатель на переменную для значения
 *
 * @return 0 в случае успеха, -EFAULT если копирование из user space не удалось
 */
static int unpack_struct(unsigned long arg, int *pin, int *val)
{
    struct rk3588_gpio_args karg;

    if (copy_from_user(&karg, (void __user *)arg, sizeof(struct rk3588_gpio_args)))
        return -EFAULT;

    *pin = karg.pin;
    *val = karg.value;

    return 0;
}

/**
 * @brief Распаковка аргументов из целого числа (упрощенный способ)
 *
 * Дилетантский способ - кодирует пин в младшие 5 бит, значение в 6-й бит.
 * Используется для совместимости, но менее безопасен чем структурированный интерфейс.
 *
 * Формат: bits[5:0] = pin (0-31), bit[6] = value (0 или 1)
 *
 * @param[in] arg      Закодированное значение из ioctl
 * @param[out] pin     Указатель на переменную для номера пина
 * @param[out] val     Указатель на переменную для значения
 */
static void unpack_raw(unsigned long arg, int *pin, int *val)
{
    int data = (int)arg;

    *pin = data & 0x1F;
    *val = !!(data & 0x20);
}

static int rk3588_validate_pin(int pin)
{
    if (pin < 0 || pin >= GPIO_PINS_PER_BANK)
        return -EINVAL;

    return 0;
}

/* ============================================================================
 * АППАРАТНЫЕ ФУНКЦИИ УПРАВЛЕНИЯ GPIO
 * ============================================================================ */

/**
 * @brief Получить адрес регистра данных или направления
 *
 * Вычисляет адрес нужного регистра (Low или High) в зависимости от номера пина.
 * GPIO содержит до 32 линий, разделенные на два 16-битных регистра.
 *
 * @param[in] bank      Указатель на структуру банка GPIO
 * @param[in] pin       Номер пина (0-31)
 * @param[in] is_data   true для регистра данных (DR), false для направления (DDR)
 *
 * @return Виртуальный адрес нужного регистра
 */
static void __iomem *rk3588_get_reg_addr(rk3588_gpio_bank_t *bank, unsigned int pin, bool is_data)
{
    rk3588_gpio_regs_t __iomem *regs = (rk3588_gpio_regs_t __iomem *)bank->base;

    if (is_data) {
        /* Регистры данных: нижняя половина для пинов 0-15, верхняя для 16-31 */
        return (pin < RK_GPIO_BITS_PER_REG) ? &regs->dr_l : &regs->dr_h;
    }

    /* Регистры направления: нижняя половина для пинов 0-15, верхняя для 16-31 */
    return (pin < RK_GPIO_BITS_PER_REG) ? &regs->ddr_l : &regs->ddr_h;
}

/**
 * @brief Установить или сбросить бит GPIO в аппаратуре
 *
 * Выполняет операцию Read-Modify-Write для установки/сброса бита с учетом
 * особенностей контроллера RK3588 (использование write-mask битов).
 *
 * @param[in] bank      Указатель на структуру банка GPIO
 * @param[in] pin       Номер пина (0-31)
 * @param[in] value     Значение: true (1) для установки, false (0) для сброса
 * @param[in] is_data   true для записи данных, false для установки направления
 *
 * @return 0 в случае успеха
 */
static int rk3588_hw_gpio_set_bit(rk3588_gpio_bank_t *bank, unsigned int pin,
                                   bool value, bool is_data)
{
    void __iomem *reg_addr;
    unsigned int bit_offset;
    u32 write_mask;
    u32 current_val;

    /* 1. Получаем адрес нужного регистра (L или H) */
    reg_addr = rk3588_get_reg_addr(bank, pin, is_data);

    /* 2. Вычисляем смещение бита внутри 16-битного регистра */
    bit_offset = pin % RK_GPIO_BITS_PER_REG;

    /* 3. Готовим маску записи (биты [31:16] содержат маску) */
    write_mask = RK_GPIO_WRITE_MASK(bit_offset);

    /* 4. Читаем актуальное значение регистра */
    current_val = ioread32(reg_addr);

    /* 5. Устанавливаем или сбрасываем бит значения */
    if (value) {
        /* Сложение: устанавливаем бит в 1 */
        current_val |= (1U << bit_offset);
    } else {
        /* Умножение: сбрасываем бит в 0 */
        current_val &= ~(1U << bit_offset);
    }

    /* 6. Добавляем маску записи (RMW-safe write) */
    current_val |= write_mask;

    /* 7. Записываем обратно в регистр */
    iowrite32(current_val, reg_addr);

    return 0;
}

/**
 * @brief Прочитать состояние бита GPIO из аппаратуры
 *
 * @param[in] bank      Указатель на структуру банка GPIO
 * @param[in] pin       Номер пина (0-31)
 *
 * @return true если бит установлен (логическая 1), false если сброшен (логический 0)
 */
static bool rk3588_hw_gpio_get_bit(rk3588_gpio_bank_t *bank, unsigned int pin)
{
    rk3588_gpio_regs_t __iomem *regs = (rk3588_gpio_regs_t __iomem *)bank->base;
    u32 current_val;

    /* Читаем регистр состояния (EXT_PORT) */
    current_val = ioread32(&regs->ext_port);

    /* Проверяем, установлен ли бит для данного пина */
    return !!(current_val & (1U << pin));
}

/* ============================================================================
 * FILE_OPERATIONS ОБРАБОТЧИКИ
 * ============================================================================ */

/**
 * @brief Открыть GPIO устройство
 *
 * Проверяет, что устройство не занято другим процессом, используя атомарную
 * операцию compare-and-swap. Это обеспечивает эксклюзивный доступ к банку GPIO.
 *
 * @param[in] inode     Указатель на inode файла
 * @param[in] filp      Указатель на структуру открытого файла
 *
 * @return 0 в случае успеха, -EBUSY если устройство уже открыто
 */
static int rk3588_gpio_open(struct inode *inode, struct file *filp)
{
    rk3588_gpio_bank_t *bank;

    /* 1. Получаем указатель на структуру банка из inode через container_of */
    bank = container_of(inode->i_cdev, struct rk3588_gpio_bank_t, cdev);

    /* 2. Проверяем и устанавливаем флаг занятости (атомарно) */
    if (atomic_cmpxchg(&bank->is_open, 0, 1))
        return -EBUSY;

    /* 3. Сохраняем указатель на банк в private_data для других операций */
    filp->private_data = bank;

    return 0;
}

/**
 * @brief Закрыть GPIO устройство
 *
 * Освобождает банк GPIO и делает его доступным для открытия другими процессами.
 *
 * @param[in] inode     Указатель на inode файла
 * @param[in] filp      Указатель на структуру открытого файла
 *
 * @return 0
 */
static int rk3588_gpio_release(struct inode *inode, struct file *filp)
{
    rk3588_gpio_bank_t *bank = filp->private_data;

    /* Освобождаем банк, сбрасывая флаг занятости */
    atomic_set(&bank->is_open, 0);

    return 0;
}

/**
 * @brief Обработчик ioctl команд GPIO
 *
 * Поддерживает две группы команд:
 * - Управление направлением (вход/выход)
 * - Чтение/запись логических уровней
 *
 * Каждая команда имеет две версии: структурированную (безопасную) и
 * сырую (быструю, используется для совместимости).
 *
 * @param[in] filp      Указатель на открытый файл
 * @param[in] cmd       Код команды ioctl
 * @param[in] arg       Аргумент команды (структура или int)
 *
 * @return 0 при успехе, отрицательное значение ошибки в противном случае
 */
static long rk3588_gpio_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    rk3588_gpio_bank_t *bank = filp->private_data;
    int pin, val;

    switch (cmd) {
    /* --- УСТАНОВКА НАПРАВЛЕНИЯ GPIO --- */
    case RK3588_GPIO_SET_DIRECTION_STRUCT:
        if (unpack_struct(arg, &pin, &val))
            return -EFAULT;
        if (rk3588_validate_pin(pin))
            return -EINVAL;
        rk3588_hw_gpio_set_bit(bank, pin, val, false);
        break;

    case RK3588_GPIO_SET_DIRECTION_RAW:
        /* Для сырых команд проверка копирования не нужна */
        unpack_raw(arg, &pin, &val);
        if (rk3588_validate_pin(pin))
            return -EINVAL;
        rk3588_hw_gpio_set_bit(bank, pin, val, false);
        break;

    /* --- ЗАПИСЬ ДАННЫХ GPIO --- */
    case RK3588_GPIO_WRITE_BIT_STRUCT:
        if (unpack_struct(arg, &pin, &val))
            return -EFAULT;
        if (rk3588_validate_pin(pin))
            return -EINVAL;
        rk3588_hw_gpio_set_bit(bank, pin, val, true);
        break;

    case RK3588_GPIO_WRITE_BIT_RAW:
        unpack_raw(arg, &pin, &val);
        if (rk3588_validate_pin(pin))
            return -EINVAL;
        rk3588_hw_gpio_set_bit(bank, pin, val, true);
        break;

    /* --- ЧТЕНИЕ ДАННЫХ GPIO --- */
    case RK3588_GPIO_READ_BIT:
        /* Для чтения нужен только номер пина */
        pin = (int)arg & 0x1F;
        if (rk3588_validate_pin(pin))
            return -EINVAL;
        return rk3588_hw_gpio_get_bit(bank, pin);

    default:
        return -EINVAL;
    }

    return 0;
}


/* ============================================================================
 * MODULE INITIALIZATION AND CLEANUP
 * ============================================================================ */

/**
 * @brief Инициализировать банк GPIO
 *
 * Выполняет все необходимые шаги для инициализации одного GPIO банка:
 * - Выделение памяти для управляющей структуры
 * - Отображение аппаратных регистров в виртуальную память
 * - Регистрация символьного устройства
 * - Создание файла в /dev
 *
 * При ошибке на любом этапе все ранее выделенные ресурсы освобождаются.
 *
 * @param[in] i Индекс банка (0-4)
 *
 * @return 0 в случае успеха, отрицательный код ошибки в противном случае
 */
static int rk3588_setup_bank(int i)
{
    int err;

    /* 1. Выделяем память для управляющей структуры */
    g_banks[i] = kzalloc(sizeof(rk3588_gpio_bank_t), GFP_KERNEL);
    if (!g_banks[i]) {
        err = -ENOMEM;
        return err;
    }

    /* 2. Инициализируем поля структуры */
    atomic_set(&g_banks[i]->is_open, 0);
    g_banks[i]->id = i;
    g_banks[i]->dev_num = MKDEV(g_major, i);

    /* 3. Отображаем регистры GPIO в виртуальную память */
    g_banks[i]->base = ioremap(rk3588_gpio_phys_bases[i], sizeof(rk3588_gpio_regs_t));
    if (!g_banks[i]->base) {
        err = -EIO;
        goto err_free_mem;
    }

    /* 4. Инициализируем символьное устройство */
    cdev_init(&g_banks[i]->cdev, &gpio_fops);
    err = cdev_add(&g_banks[i]->cdev, g_banks[i]->dev_num, 1);
    if (err)
        goto err_unmap;

    /* 5. Создаем файл устройства в /dev */
    g_banks[i]->device = device_create(g_class, NULL, g_banks[i]->dev_num, NULL, "gpio%d", i);
    if (IS_ERR(g_banks[i]->device)) {
        err = PTR_ERR(g_banks[i]->device);
        goto err_cdev_del;
    }

    return 0;

/* --- ЦЕПОЧКА ОБРАБОТКИ ОШИБОК --- */

err_cdev_del:
    cdev_del(&g_banks[i]->cdev);

err_unmap:
    iounmap(g_banks[i]->base);

err_free_mem:
    kfree(g_banks[i]);
    g_banks[i] = NULL;

    return err;
}

/**
 * @brief Инициализация модуля драйвера GPIO
 *
 * Выполняет инициализацию всех пяти GPIO банков:
 * 1. Выделяет диапазон мажорного номера устройства
 * 2. Создает класс устройств в sysfs
 * 3. Инициализирует каждый GPIO банк
 * 4. При ошибке откатывает все изменения
 *
 * @return 0 при успешной инициализации, отрицательный код ошибки в противном случае
 */
static int __init rk3588_gpio_init(void)
{
    int err, i;
    dev_t dev_base;

    /* 1. Выделяем диапазон мажор/минор для 5 устройств */
    err = alloc_chrdev_region(&dev_base, 0, GPIO_BANKS_COUNT, "rk3588_gpio");
    if (err < 0) {
        pr_err("Failed to allocate char device region\n");
        return err;
    }

    g_major = MAJOR(dev_base);

    /* 2. Создаем класс для наших устройств */
    g_class = class_create(THIS_MODULE, "rk3588_gpio_class");
    if (IS_ERR(g_class)) {
        err = PTR_ERR(g_class);
        pr_err("Failed to create class\n");
        goto fail_unregister_chrdev_region;
    }

    /* 3. Основной цикл инициализации для каждого банка */
    for (i = 0; i < GPIO_BANKS_COUNT; i++) {
        err = rk3588_setup_bank(i);
        if (err) {
            pr_err("Failed to setup GPIO bank %d\n", i);
            goto err_cleanup_loop;
        }
    }

    pr_info("RK3588 GPIO driver initialized successfully\n");
    return 0;

/* --- ЦЕПОЧКА ОБРАБОТКИ ОШИБОК --- */

err_cleanup_loop:
    while (--i >= 0) {
        device_destroy(g_class, g_banks[i]->dev_num);
        cdev_del(&g_banks[i]->cdev);
        iounmap(g_banks[i]->base);
        kfree(g_banks[i]);
        g_banks[i] = NULL;
    }
    class_destroy(g_class);

fail_unregister_chrdev_region:
    unregister_chrdev_region(MKDEV(g_major, 0), GPIO_BANKS_COUNT);

    return err;
}

/**
 * @brief Очистка модуля драйвера GPIO
 *
 * Выполняет полную очистку всех выделенных ресурсов в обратном порядке:
 * 1. Удаляет файлы устройств из /dev
 * 2. Удаляет символьные устройства
 * 3. Отменяет отображение регистров
 * 4. Освобождает память
 * 5. Разрушает класс устройств
 * 6. Возвращает мажорный номер системе
 */
static void __exit rk3588_gpio_exit(void)
{
    int i;

    for (i = 0; i < GPIO_BANKS_COUNT; i++) {
        if (g_banks[i]) {
            /* 1. Удаляем файл из /dev */
            if (g_banks[i]->device)
                device_destroy(g_class, g_banks[i]->dev_num);

            /* 2. Удаляем символьное устройство из системы */
            cdev_del(&g_banks[i]->cdev);

            /* 3. Отменяем отображение регистров */
            if (g_banks[i]->base)
                iounmap(g_banks[i]->base);

            /* 4. Освобождаем память структуры */
            kfree(g_banks[i]);
            g_banks[i] = NULL;
        }
    }

    /* 5. Разрушаем класс устройств */
    if (g_class)
        class_destroy(g_class);

    /* 6. Возвращаем диапазон мажор/минор */
    unregister_chrdev_region(MKDEV(g_major, 0), GPIO_BANKS_COUNT);

    pr_info("RK3588 GPIO driver unloaded\n");
}

/* ============================================================================
 * MODULE METADATA
 * ============================================================================ */

module_init(rk3588_gpio_init);
module_exit(rk3588_gpio_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Artemy Narziev");
MODULE_DESCRIPTION("Rockchip RK3588 GPIO Driver");
MODULE_VERSION("1.0");