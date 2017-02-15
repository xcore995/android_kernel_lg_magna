#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/switch.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/wakelock.h>

#include <mach/eint.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>

#ifndef TPD_NO_GPIO
#include "cust_eint.h"
#include "cust_gpio_usage.h"
#include "cust_eint_md1.h"
#endif

/* SMART COVER Support */
#define SMARTCOVER_POUCH_CLOSED		1
#define SMARTCOVER_POUCH_OPENED		0

#ifdef CONFIG_STYLUS_PEN_DETECTION
#define STYLUS_PEN_IN   1
#define STYLUS_PEN_OUT  0
#endif

#define HALL_IC_DEV_NAME "bu52033nvx"

struct mt6xxx_cradle {
	struct switch_dev sdev;
	struct device *dev;
	struct wake_lock wake_lock;
	int pouch;
	spinlock_t lock;
	int state;
#ifdef CONFIG_STYLUS_PEN_DETECTION
	struct switch_dev pen_sdev;
	int pen;
	int pen_state;
#endif
};

static struct delayed_work pouch_work;
#ifdef CONFIG_STYLUS_PEN_DETECTION
static struct delayed_work pen_work;
#endif

static struct workqueue_struct *cradle_wq;
static struct mt6xxx_cradle *cradle;

int Touch_Quick_Cover_Closed = 0;

static void boot_cradle_det_func(void)
{
	int state;
	int gpio_status;
#ifdef CONFIG_STYLUS_PEN_DETECTION
	int pen_state;
#endif

	// cradle->pouch is set means that the pouch is closed.
	gpio_status = mt_get_gpio_in(GPIO_HALL_1_PIN);
	cradle->pouch = !gpio_status;

	printk("%s : boot pouch ==> %d\n", __func__, cradle->pouch);

	if(cradle->pouch == 1)
		state = SMARTCOVER_POUCH_CLOSED;
	else
		state = SMARTCOVER_POUCH_OPENED;

	printk("%s : [Cradle] boot cradle state is %d\n", __func__, state);

	cradle->state = state;
	wake_lock_timeout(&cradle->wake_lock, msecs_to_jiffies(3000));
	switch_set_state(&cradle->sdev, cradle->state);

#ifdef CONFIG_STYLUS_PEN_DETECTION
	gpio_status = mt_get_gpio_in(GPIO_HALL_2_PIN);
	cradle->pen = !gpio_status;

	printk("%s : boot pen ==> %d\n", __func__, cradle->pen);

	if(cradle->pen == 1)
		pen_state = STYLUS_PEN_IN;
	else
		pen_state = STYLUS_PEN_OUT;

	printk("%s : [Cradle] boot cradle pen_state is %d\n", __func__, pen_state);

	cradle->pen_state = pen_state;
	wake_lock_timeout(&cradle->wake_lock, msecs_to_jiffies(3000));
	switch_set_state(&cradle->pen_sdev, cradle->pen_state);
#endif
}

static void mt6xxx_pouch_work_func(struct work_struct *work)
{
	int state = 0;
	int gpio_status;
	unsigned long polarity;
	u32 pull;

	spin_lock_irq(&cradle->lock);

	gpio_status = mt_get_gpio_in(GPIO_HALL_1_PIN);
	cradle->pouch = !gpio_status;

	printk("%s : pouch ==> %d\n", __func__, cradle->pouch);

	if (cradle->pouch == 1)
		state = SMARTCOVER_POUCH_CLOSED;
	else if (cradle->pouch == 0)
		state = SMARTCOVER_POUCH_OPENED;
	if(!gpio_status)//Cover Closed
		Touch_Quick_Cover_Closed = 1;
	else
		Touch_Quick_Cover_Closed = 0;

	if (cradle->state != state) {
		cradle->state = state;
		if (gpio_status == 1) {
			polarity = CUST_EINT_POLARITY_LOW;
			pull = GPIO_PULL_UP;
		}
		else {
			polarity = CUST_EINT_POLARITY_HIGH;
			pull = GPIO_PULL_DOWN;
		}

		mt_set_gpio_pull_select(GPIO_HALL_1_PIN, pull);
		mt_eint_set_polarity(CUST_EINT_HALL_1_NUM, polarity);

		mt_eint_unmask(CUST_EINT_HALL_1_NUM);
		spin_unlock_irq(&cradle->lock);

		wake_lock_timeout(&cradle->wake_lock, msecs_to_jiffies(3000));
		switch_set_state(&cradle->sdev, cradle->state);

		printk("%s : [Cradle] pouch value is %d\n", __func__ , state);
	}
	else {
		printk("%s : [Cradle] pouch value is %d (no change)\n", __func__ , state);
		mt_eint_unmask(CUST_EINT_HALL_1_NUM);
		spin_unlock_irq(&cradle->lock);
	}

}

static void mt6xxx_pouch_irq_handler(void)
{
int gpio_status;
	printk("pouch irq!!!!\n");
	mt_eint_mask(CUST_EINT_HALL_1_NUM);
	gpio_status = mt_get_gpio_in(GPIO_HALL_1_PIN);
	if(!gpio_status)//Cover Closed
		Touch_Quick_Cover_Closed = 1;
	else
		Touch_Quick_Cover_Closed = 0;
	queue_delayed_work(cradle_wq, &pouch_work, msecs_to_jiffies(200));
}

static void smart_cover_gpio_set(void)
{

    printk("[hall_ic]gpio_set start\n");

	int gpio_status;
	unsigned int flag;
	u32 pull;

	gpio_status = mt_get_gpio_in(GPIO_HALL_1_PIN);
	cradle->pouch = !gpio_status;

	if(cradle->pouch == 0) {
		flag = EINTF_TRIGGER_FALLING;
		pull = GPIO_PULL_UP;
	}
	else {
		flag = EINTF_TRIGGER_RISING;
		pull = GPIO_PULL_DOWN;
	}

	/* initialize irq of gpio_hall */
	mt_set_gpio_mode(GPIO_HALL_1_PIN, GPIO_HALL_1_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_HALL_1_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_HALL_1_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_HALL_1_PIN, pull);

	mt_eint_set_hw_debounce(CUST_EINT_HALL_1_NUM, CUST_EINT_HALL_1_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_HALL_1_NUM, flag, mt6xxx_pouch_irq_handler, 0);

	mt_eint_unmask(CUST_EINT_HALL_1_NUM);
}

static ssize_t cradle_pouch_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int len;

	len = snprintf(buf, PAGE_SIZE, "pouch : %d -> %s\n", cradle->pouch, cradle->pouch == 0 ? "open" : "close");

	return len;
}
#if 1 /* block_warning_message */
static ssize_t cradle_pouch_store(struct device *dev,struct device_attribute *attr, char *buf)
{
	return 0;
}
#endif

static ssize_t cradle_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int len;

	len = snprintf(buf, PAGE_SIZE, "cradle state : %d -> %s\n", cradle->state, cradle->state == 1 ? "open" : "close");

	return len;
}
#if 1 /* block_warning_message */
static ssize_t cradle_state_store(struct device *dev,struct device_attribute *attr, char *buf)
{
	return 0;
}
#endif

static struct device_attribute cradle_state_attr = __ATTR(state, S_IRUGO | S_IWUSR, cradle_state_show, cradle_state_store);
static struct device_attribute cradle_pouch_attr   = __ATTR(pouch, S_IRUGO | S_IWUSR, cradle_pouch_show, cradle_pouch_store);

#ifdef CONFIG_STYLUS_PEN_DETECTION
static void mt6xxx_pen_work_func(struct work_struct *work)
{
	int pen_state = 0;
	int gpio_status;
	unsigned long polarity;
	u32 pull;

	spin_lock_irq(&cradle->lock);

	gpio_status = mt_get_gpio_in(GPIO_HALL_2_PIN);
	cradle->pen = !gpio_status;

	printk("%s : pen ==> %d\n", __func__, cradle->pen);

	if (cradle->pen == 1)
		pen_state = STYLUS_PEN_IN;
	else if (cradle->pen == 0)
		pen_state = STYLUS_PEN_OUT;

	if (cradle->pen_state != pen_state) {
		cradle->pen_state = pen_state;
		if (gpio_status == 1) {
			polarity = CUST_EINT_POLARITY_LOW;
			pull = GPIO_PULL_UP;
		}
		else {
			polarity = CUST_EINT_POLARITY_HIGH;
			pull = GPIO_PULL_DOWN;
		}
		mt_set_gpio_pull_select(GPIO_HALL_2_PIN, pull);
		mt_eint_set_polarity(CUST_EINT_HALL_2_NUM, polarity);

		mt_eint_unmask(CUST_EINT_HALL_2_NUM);
		spin_unlock_irq(&cradle->lock);
	
		wake_lock_timeout(&cradle->wake_lock, msecs_to_jiffies(3000));
		switch_set_state(&cradle->pen_sdev, cradle->pen_state);
		printk("%s : [Cradle] pen value is %d\n", __func__ , pen_state);
	}
	else {
				mt_eint_unmask(CUST_EINT_HALL_2_NUM);
				spin_unlock_irq(&cradle->lock);
				printk("%s : [Cradle] pen value is %d (no change)\n", __func__ , pen_state);
	}


}

static void mt6xxx_pen_irq_handler(void)
{
	printk("pen irq!!!!\n");
	mt_eint_mask(CUST_EINT_HALL_2_NUM);
	queue_delayed_work(cradle_wq, &pen_work, msecs_to_jiffies(200));
}

static void stylus_pen_gpio_set(void)
{
	int gpio_status;
	unsigned int flag;
	u32 pull;

	gpio_status = mt_get_gpio_in(GPIO_HALL_2_PIN);
	cradle->pen = !gpio_status;

	if(cradle->pen == 0) {
		flag = EINTF_TRIGGER_FALLING;
		pull = GPIO_PULL_UP;
	}
	else {
		flag = EINTF_TRIGGER_RISING;
		pull = GPIO_PULL_DOWN;
	}

	/* initialize irq of gpio_hall */
	mt_set_gpio_mode(GPIO_HALL_2_PIN, GPIO_HALL_2_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_HALL_2_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_HALL_2_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_HALL_2_PIN, pull);

	mt_eint_set_hw_debounce(CUST_EINT_HALL_2_NUM, CUST_EINT_HALL_2_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_HALL_2_NUM, flag, mt6xxx_pen_irq_handler, 0);

	mt_eint_unmask(CUST_EINT_HALL_2_NUM);
}

static ssize_t cradle_pen_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int len;

	len = snprintf(buf, PAGE_SIZE, "pen : %d -> %s\n", cradle->pen, cradle->pen == 0 ? "out" : "in");

	return len;
}


static ssize_t cradle_pen_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int len;

	len = snprintf(buf, PAGE_SIZE, "pen state : %d -> %s\n", cradle->pen_state, cradle->pen_state == 1 ? "out" : "in");

	return len;
}

static struct device_attribute cradle_pen_state_attr = __ATTR(pen_state, S_IRUGO | S_IWUSR, cradle_pen_state_show, NULL);
static struct device_attribute cradle_pen_attr   = __ATTR(pen, S_IRUGO | S_IWUSR, cradle_pen_show, NULL);
#endif

static ssize_t cradle_print_name(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(sdev)) {
		case 0:
			return sprintf(buf, "UNDOCKED\n");
		case 2:
			return sprintf(buf, "CARKET\n");
	}
	return -EINVAL;
}

//static int __devinit mt6xxx_cradle_probe(struct platform_device *pdev)
static int __init mt6xxx_cradle_probe(struct platform_device *pdev)
{

	int ret;

	cradle = kzalloc(sizeof(*cradle), GFP_KERNEL);
	if (!cradle)
		return -ENOMEM;

	cradle->sdev.name = "smartcover";
	cradle->sdev.print_name = cradle_print_name;

	spin_lock_init(&cradle->lock);

	ret = switch_dev_register(&cradle->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	wake_lock_init(&cradle->wake_lock, WAKE_LOCK_SUSPEND, "hall_ic_wakeups");

	INIT_DELAYED_WORK(&pouch_work, mt6xxx_pouch_work_func);

	smart_cover_gpio_set();

#ifdef CONFIG_STYLUS_PEN_DETECTION
	cradle->pen_sdev.name = "styluspen";
	cradle->pen_sdev.print_name = cradle_print_name;

	ret = switch_dev_register(&cradle->pen_sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	INIT_DELAYED_WORK(&pen_work, mt6xxx_pen_work_func);

	stylus_pen_gpio_set();
#endif

	printk("%s : init cradle\n", __func__);

	printk("%s :boot_cradle_det_func START\n",__func__);
	boot_cradle_det_func();

	ret = device_create_file(&pdev->dev, &cradle_state_attr);
	if (ret)
		goto err_request_irq;

	ret = device_create_file(&pdev->dev, &cradle_pouch_attr);
	if (ret)
		goto err_request_irq;

#ifdef CONFIG_STYLUS_PEN_DETECTION
	ret = device_create_file(&pdev->dev, &cradle_pen_state_attr);
	if (ret)
		goto err_request_irq;

	ret = device_create_file(&pdev->dev, &cradle_pen_attr);
	if (ret)
		goto err_request_irq;
#endif


    printk("[hall_ic]probe done\n");

	return 0;

err_request_irq:
err_switch_dev_register:
	switch_dev_unregister(&cradle->sdev);
#ifdef CONFIG_STYLUS_PEN_DETECTION
	switch_dev_unregister(&cradle->pen_sdev);
#endif
	kfree(cradle);
	return ret;
}

//static int __devexit mt6xxx_cradle_remove(struct platform_device *pdev)
static int __exit mt6xxx_cradle_remove(struct platform_device *pdev)
{
	cancel_delayed_work_sync(&pouch_work);
	switch_dev_unregister(&cradle->sdev);
#ifdef CONFIG_STYLUS_PEN_DETECTION
	cancel_delayed_work_sync(&pen_work);
	switch_dev_unregister(&cradle->pen_sdev);
#endif
	platform_set_drvdata(pdev, NULL);
	kfree(cradle);

	return 0;
}

static struct platform_driver mt6xxx_cradle_driver = {
	.probe  = mt6xxx_cradle_probe,
	.remove = mt6xxx_cradle_remove,
	.driver	= {
		.name	= HALL_IC_DEV_NAME,
		.owner	= THIS_MODULE,
	},
};


static int __init mt6xxx_cradle_init(void)
{
	cradle_wq = create_singlethread_workqueue("cradle_wq");

	if (!cradle_wq) {
		printk(KERN_ERR "fail to create workqueue\n");
		return -ENOMEM;
	}

	return platform_driver_register(&mt6xxx_cradle_driver);
}

static void __exit mt6xxx_cradle_exit(void)
{
	if (cradle_wq)
		destroy_workqueue(cradle_wq);

	platform_driver_unregister(&mt6xxx_cradle_driver);
}

module_init(mt6xxx_cradle_init);
module_exit(mt6xxx_cradle_exit);

MODULE_AUTHOR("LG Electronics Inc.");
MODULE_DESCRIPTION("BU52033NVX HALL IC Driver for MTK platform");
MODULE_LICENSE("GPL");
