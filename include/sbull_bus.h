extern struct bus_type sbull_bus_type;

struct sbull_bus_driver {
	struct module *owner;
	struct device_driver driver;
	int cmd_came_flag;
};

/*
 * type for devices plugged into the sbull bus
 */
struct sbull_bus_device {
	unsigned char *name;
	struct module *owner;
	//struct sbull_bus_driver *driver;
	struct device dev;
};

#define to_sbull_bus_driver(drv) container_of(drv, struct sbull_bus_driver, driver);
#define to_sbull_bus_device(dvc) container_of(dvc, struct sbull_bus_device, device);

extern void register_sbull_driver(struct sbull_bus_driver *);
extern void unregister_sbull_driver(struct sbull_bus_driver *);
extern int register_sbull_device(struct sbull_bus_device *);
extern void unregister_sbull_device(struct sbull_bus_device *);

