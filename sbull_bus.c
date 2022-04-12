#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/errno.h>
#include <linux/types.h>
#include "include/sbull_bus.h"

MODULE_LICENSE("Dual BSD/GPL");

static char user_cmd_array[20] = {0};

/*
 * Instance of sbull bus device - parent
 */
static void sbull_bus_release(struct device *dev)
{
	printk("sbull_bus release\n");
}

struct device sbull_bus = {
	.init_name = "sbull-0",
	.release = sbull_bus_release,
};

static int sbull_device_match(struct device *dev, struct device_driver *driver)
{
	return !strncmp(dev->init_name, driver->name, strlen(driver->name));
}

struct bus_type sbull_bus_type = {
	.name = "sbull",
	.match = sbull_device_match,
	//.probe = sbull_device_probe,
	//.remove = sbull_device_remove,
	//.shutdown = sbull_device_shutdown
};
EXPORT_SYMBOL(sbull_bus_type);

/*
 * device register & unregister functions
 */
int register_sbull_device(struct sbull_bus_device *sbull)
{
	sbull->dev.parent = &sbull_bus;
	sbull->dev.bus = &sbull_bus_type;

	return device_register(&sbull->dev);
}
EXPORT_SYMBOL(register_sbull_device);

void unregister_sbull_device(struct sbull_bus_device *sbull)
{
	device_unregister(&sbull->dev);
}
EXPORT_SYMBOL(unregister_sbull_device);

/*
 * driver attributes
 */
static ssize_t user_cmd_store(struct device_driver *driver, const char *buf, size_t count)
{
	sprintf(user_cmd_array, "%s\n", buf);
	//return copy_from_user(user_cmd_array, buf, );
	if (strlen(buf) > 0) {
		printk("user command received\n");
		if (!strncmp(buf, "add_device", 10)) {
			
		}
	}
	return strlen(buf);
}

static ssize_t user_cmd_show(struct device_driver *driver, char *buf)
{
	sprintf(buf, "%s\n", user_cmd_array);
	//return copy_to_user(buf, user_cmd_array, );
	return strlen(buf);
}

static DRIVER_ATTR_RW(user_cmd);

/*
 * driver register & unregister functions
 */
void register_sbull_driver(struct sbull_bus_driver *driver)
{
	int ret;

	driver->driver.bus = &sbull_bus_type;
	ret = driver_register(&driver->driver);
	/*
	 * creating files for driver attributes
	 */
	ret = driver_create_file(&driver->driver, &driver_attr_user_cmd);
}
EXPORT_SYMBOL(register_sbull_driver);

void unregister_sbull_driver(struct sbull_bus_driver *driver)
{
	driver_unregister(&driver->driver);
}
EXPORT_SYMBOL(unregister_sbull_driver);

/*
 * module init & exit functions
 */
static int __init sbull_bus_init(void)
{
	int ret;

	printk(">>> sbull_bus module has been loaded\n");
	ret = bus_register(&sbull_bus_type); // register in /sys/bus
	if (ret)
		return ret;
	ret = device_register(&sbull_bus);
	if (ret)
		printk("unable to register sbull-0\n");	
	return 0;
}

static void __exit sbull_bus_exit(void)
{	
	device_unregister(&sbull_bus);
	bus_unregister(&sbull_bus_type);

	printk(">>> sbull_bus module has been unloaded\n");
}

module_init(sbull_bus_init);
module_exit(sbull_bus_exit);
