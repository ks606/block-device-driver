#ifndef _SBULL_H
#define _SBULL_H

extern struct bus_type sbull_bus_type;

struct sbull_bus_driver {
	struct module *owner;
	struct device_driver driver;
	int cmd_came_flag;
};

#define to_sbull_bus_driver(drv) container_of(drv, struct sbull_bus_driver, driver);

/*
 * type for devices plugged into the sbull bus
 */
struct sbull_bus_device {
	unsigned char *name;
	struct module *owner;
	//struct sbull_bus_driver *driver;
	struct device dev;
};

#define to_sbull_bus_device(dvc) container_of(dvc, struct sbull_bus_device, device);

/*
 * internal structure of block device
 */
struct sbull_dev {
	int size;
	u8 *data;
	short users;
	spinlock_t lock;
	struct blk_mq_tag_set tag_set;
	short media_change;
	struct request_queue *queue;
	struct gendisk *gd;
	struct timer_list timer;

	char *name;
	struct sbull_bus_device sdev;
};

#endif /* _SBULL_H* /