#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/blk-mq.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/hdreg.h>
#include <linux/kdev_t.h>
#include <linux/vmalloc.h>
#include <linux/genhd.h>
#include <linux/blkdev.h>
#include <linux/buffer_head.h>
#include <linux/bio.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/wait.h>

#include "include/sbull.h"

MODULE_LICENSE("Dual BSD/GPL");

#define KERNEL_SECTOR_SIZE	512
#define INVALIDATE_DELAY	30*HZ
#define SBULL_MINORS		16
#define MINOR_SHIFT		4

#define USER_CMD_BUF		20

/*
 * macro tranforms desired size of device into number of sectors
 */
#define SECTORS_FROM_DEVSIZE(SIZE)	((SIZE)/(KERNEL_SECTOR_SIZE))	

static int sbull_major = 0;
module_param(sbull_major, int, 0);
static const int devsize = 100 * 1024 * 1024;
static int nr_sectors = SECTORS_FROM_DEVSIZE(devsize);
module_param(nr_sectors, int, 0);
static int hardsect_size = 512;
module_param(hardsect_size, int, 0);
static int nr_devices = 1;
module_param(nr_devices, int, 0);
static int creation_mode = 1;
module_param(creation_mode, int, 0);

static char user_cmd_array[USER_CMD_BUF] = {0};
static char user_cmd_atoibuf[6] = {0};

enum {
	WAIT_EXIT_CLEANUP = 1,
	WAIT_EXIT_USERCMD = 2,
};

enum {
	RM_SIMPLE  = 0,	/* simple request function */
	RM_FULL    = 1, /* full request function */
	RM_NOQUEUE = 2,	
};

/* variation of user commands */
typedef enum {
	NO_UCMD,
	UCMD1,
	UCMD2,
} user_command;

static int request_mode = RM_FULL;

static struct sbull_dev *Devices = NULL;

/*
 * multithread stuff for awaiting of user commands
 */
static struct task_struct *wait_thread;

DECLARE_WAIT_QUEUE_HEAD(wait_queue_usr);

int wait_queue_flag = 0;

/* BUS section */
/*
 * Instance of sbull bus device - parent
 */
static void sbull_bus_release(struct device *dev)
{
	printk(KERN_INFO "sbull_bus release\n");
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

/*
 * device register & unregister functions
 */
int register_sbull_device(struct sbull_bus_device *sbull)
{
	sbull->dev.parent = &sbull_bus;
	sbull->dev.bus = &sbull_bus_type;

	return device_register(&sbull->dev);
}

void unregister_sbull_device(struct sbull_bus_device *sbull)
{
	device_unregister(&sbull->dev);
}

/*
 * driver attributes
 */
static ssize_t user_cmd_store(struct device_driver *driver, const char *buf, size_t count)
{
	int cmd_len;
	int i;
	long result = 0;
	user_command cmd_flag = NO_UCMD;

	sprintf(user_cmd_array, "%s\n", buf);
	//return copy_from_user(user_cmd_array, buf, );
	if (strlen(buf) > 0) {
		printk(KERN_INFO ">>> user command received\n");
		cmd_len = 12;
		if (!strncmp(buf, "set_devsize=", cmd_len)) {
			cmd_flag = UCMD1;
			printk(KERN_INFO ">>> user command has been identified -- it's <set_devsize>!\n");
			for (i = 0; i < 6; i++) {
				user_cmd_atoibuf[i] = buf[cmd_len + i];
			}

			kstrtol(user_cmd_atoibuf, 0, &result);
			if ((result > 0) && (result < 500000)) {
				nr_sectors = result;
				printk(KERN_INFO ">>> Block device size set in %ld sectors\n", result);
			}
		} 

		cmd_len = 10;
		if (!strncmp(buf, "add_device", cmd_len)) {
			cmd_flag = UCMD2;
			printk(KERN_INFO ">>> user command has been identified -- it's <add_device>!\n");
			wait_queue_flag = WAIT_EXIT_USERCMD;
			wake_up_interruptible(&wait_queue_usr);
		}

		if (cmd_flag == NO_UCMD)
			printk(KERN_INFO ">>> user command has been not identified -- please try again!\n");
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


void unregister_sbull_driver(struct sbull_bus_driver *driver)
{
	driver_unregister(&driver->driver);
}

/*
 * device model stuff
 */
static struct sbull_bus_driver sbull_driver = {
	.driver = {
		.name = "sbulld"
	}
};

/* Block device section */
static inline struct request_queue *blk_generic_alloc_queue(make_request_fn make_request, int node_id)
{
	struct request_queue *q = blk_alloc_queue(GFP_KERNEL);
	
	if (q != NULL)
		blk_queue_make_request(q, make_request);

	return (q);
}

static void sbull_transfer(struct sbull_dev *dev, unsigned long sector, 
		unsigned long nsect, char *buffer, int write)
{
	unsigned long offset = sector*KERNEL_SECTOR_SIZE;
	unsigned long nbytes = nsect*KERNEL_SECTOR_SIZE;

	if ((offset + nbytes) > dev->size) {
		printk (KERN_INFO "Beyond-end write (%ld %ld)\n", offset, nbytes);
		return;
	}

	if (write)
		memcpy(dev->data + offset, buffer, nbytes);
	else
		memcpy(buffer, dev->data + offset, nbytes);
}

/*
 * simple request function
 */
static blk_status_t sbull_request(struct blk_mq_hw_ctx *hctx, const struct blk_mq_queue_data *bd)
{
	struct request *req = bd->rq;
	struct sbull_dev *dev = req->rq_disk->private_data;
	struct bio_vec bvec;
	struct req_iterator iter;
	sector_t pos_sector = blk_rq_pos(req);
	void *buffer;
	blk_status_t ret;

	blk_mq_start_request (req);
	if (blk_rq_is_passthrough(req)) {
		printk (KERN_INFO "Skip non-fs request\n");
		ret = BLK_STS_IOERR;  //-EIO
			goto done;
	}

	rq_for_each_segment(bvec, req, iter)
	{
		size_t num_sector = blk_rq_cur_sectors(req);
		buffer = page_address(bvec.bv_page) + bvec.bv_offset;
		sbull_transfer(dev, pos_sector, num_sector, buffer, rq_data_dir(req) == WRITE);
		pos_sector += num_sector;
	}

	ret = BLK_STS_OK;
done:
	blk_mq_end_request (req, ret);

	return ret;
}

/*
 *  transfer single bio 
 */ 
static int sbull_xfer_bio(struct sbull_dev *dev, struct bio *bio)
{
	struct bio_vec bvec;
	struct bvec_iter iter;
	sector_t sector = bio->bi_iter.bi_sector;

	bio_for_each_segment(bvec, bio, iter) {
		char *buffer = kmap_atomic(bvec.bv_page) + bvec.bv_offset;
		sbull_transfer(dev, sector,(bio_cur_bytes(bio)/KERNEL_SECTOR_SIZE), buffer, bio_data_dir(bio) == WRITE);
		sector += (bio_cur_bytes(bio) / KERNEL_SECTOR_SIZE);
		kunmap_atomic(buffer);
	}

	return 0;
}
/*
 * full bio request
 */
static int sbull_xfer_request(struct sbull_dev *dev, struct request *req)
{
	struct bio *bio;
	int nsect = 0;

	__rq_for_each_bio(bio, req) {
		sbull_xfer_bio(dev, bio);
		nsect += bio->bi_iter.bi_size/KERNEL_SECTOR_SIZE;
	}

	return nsect;
}

/*
 * full request function
 */ 
static blk_status_t sbull_full_request(struct blk_mq_hw_ctx *hctx, const struct blk_mq_queue_data *bd)
{
	struct request *req = bd->rq;
	int sectors_xferred;
	struct sbull_dev *dev = req->q->queuedata;
	blk_status_t ret;

	blk_mq_start_request(req);
	if(blk_rq_is_passthrough(req)) {
		printk(KERN_INFO "skip non-fs request\n");
		ret = BLK_STS_IOERR;
		goto done;
	}

	sectors_xferred = sbull_xfer_request(dev, req);
	ret = BLK_STS_OK;
done:
	blk_mq_end_request(req, ret);

	return ret;
}

/*
 * simplest request function (RM_NOQUEUE)
 */ 
static blk_qc_t sbull_make_request(struct request_queue *q, struct bio *bio)
{
	struct sbull_dev *dev = bio->bi_disk->private_data;
	int status;

	status = sbull_xfer_bio(dev, bio);
	bio->bi_status = status;
	bio_endio(bio);

	return BLK_QC_T_NONE;
}

static int sbull_open(struct block_device *bdev, fmode_t mode)
{
	struct sbull_dev *dev = bdev->bd_disk->private_data;

	printk(KERN_INFO "disk %s has been opened\n", dev->gd->disk_name);

	del_timer_sync(&dev->timer);
	spin_lock(&dev->lock);
	if (!dev->users)
		check_disk_change(bdev);

	dev->users++;
	spin_unlock(&dev->lock);

	return 0;
}

static void sbull_release(struct gendisk *gd, fmode_t mode)
{
	struct sbull_dev *dev = gd->private_data;

	spin_lock(&dev->lock);
	dev->users--;
	if (!dev->users) {
		dev->timer.expires = jiffies + INVALIDATE_DELAY;
		add_timer(&dev->timer);
	}

	spin_unlock(&dev->lock);
	printk(KERN_INFO "disk %s has been released\n", dev->gd->disk_name);
}

int sbull_media_changed(struct gendisk *gd)
{
	struct sbull_dev *dev = gd->private_data;

	return dev->media_change;
}

/*
static unsigned int sbull_check_events(struct gendisk *gd, unsigned int clearing)
{
	printk("sbull check ev\n");
	return 0;
}
*/
int sbull_revalidate_disk(struct gendisk *gd)
{
	struct sbull_dev *dev = gd->private_data;

	if (dev->media_change) {
		dev->media_change = 0;
		memset(dev->data, 0, dev->size);
	}

	printk(KERN_INFO "disk %s has been revalidated\n", dev->gd->disk_name);

	return 0;
}

void sbull_invalidate(struct timer_list *ldev)
{
	struct sbull_dev *dev = from_timer(dev, ldev, timer);

	spin_lock(&dev->lock);
	if (dev->users || !dev->data)
		printk(KERN_INFO "sbull:timer sanity check failed\n");
	else
		dev->media_change = 1;

	spin_unlock(&dev->lock);
}

int sbull_ioctl(struct block_device *bdev, fmode_t mode, unsigned cmd, unsigned long arg)
{
	long size;
	struct hd_geometry geo;
	struct sbull_dev *dev = bdev->bd_disk->private_data;

	switch(cmd) {
	case HDIO_GETGEO:
		size = dev->size*(hardsect_size/KERNEL_SECTOR_SIZE);
		geo.cylinders = (size & ~0x3f) >> 6;
		geo.heads = 4;
		geo.sectors = 16;
		geo.start = 4;
		if (copy_to_user((void __user *) arg, &geo, sizeof(geo)))
			return -EFAULT;
	}

	return -ENOTTY;
}

/*
 * The device operations structure
 */
static struct block_device_operations sbull_ops = {
		.owner = THIS_MODULE,
		.open = sbull_open,
		.release = sbull_release,
		//.check_events = sbull_check_events,
		.media_changed = sbull_media_changed,
		.revalidate_disk = sbull_revalidate_disk,
		.ioctl = sbull_ioctl
};

static struct blk_mq_ops mq_ops_simple = {
	.queue_rq = sbull_request,
};

static struct blk_mq_ops mq_ops_full = {
	.queue_rq = sbull_full_request,
};

static void setup_device(struct sbull_dev *dev, int which)
{
	dev->size = nr_sectors * hardsect_size;
	dev->data = vzalloc(dev->size);
	if (dev->data == NULL) {
		printk(KERN_INFO "dev->data malloc failure\n");
		return;
	}

	spin_lock_init(&dev->lock);
	timer_setup(&dev->timer, sbull_invalidate, 0);

	switch (request_mode) {
	case RM_NOQUEUE:
		dev->queue =  blk_generic_alloc_queue(sbull_make_request, NUMA_NO_NODE);
		if (dev->queue == NULL)
			goto out_vfree;
		printk(KERN_INFO "dev->queue has ben set in RM_NOQUEUE request mode\n");
		break;

	case RM_FULL:
		dev->queue = blk_mq_init_sq_queue(&dev->tag_set, &mq_ops_full, 128, BLK_MQ_F_SHOULD_MERGE);
		if (dev->queue == NULL)
			goto out_vfree;
		printk(KERN_INFO "dev->queue has ben set in RM_FULL request mode\n");
		break;

	default:
		printk(KERN_INFO "Bad request mode %d, using simple\n", request_mode);
	
	case RM_SIMPLE:
		dev->queue = blk_mq_init_sq_queue(&dev->tag_set, &mq_ops_simple, 128, BLK_MQ_F_SHOULD_MERGE);
		if (dev->queue == NULL)
			goto out_vfree;
		printk(KERN_INFO "dev->queue has ben set in RM_SIMPLE request mode\n");
		break;
	}
	/*
	 * configuring a disk
	 */
	blk_queue_logical_block_size(dev->queue, hardsect_size);
	printk(KERN_INFO "disk configuring begins\n");
	dev->queue->queuedata = dev;
	dev->gd = alloc_disk(SBULL_MINORS);
	if (dev->gd == NULL) {
		printk(KERN_INFO "alloc_disk failure\n");
		goto out_vfree;
	}

	dev->gd->major = sbull_major;
	dev->gd->first_minor = which * SBULL_MINORS;
	dev->gd->fops = &sbull_ops;
	dev->gd->queue = dev->queue;
	dev->gd->private_data = dev;
	snprintf(dev->gd->disk_name, 32, "sbull%c", which + 'a');
	set_capacity(dev->gd, nr_sectors);
	add_disk(dev->gd);
	printk(KERN_INFO "disk %s has been added\n", dev->gd->disk_name);
	printk(KERN_INFO "disk size         -- %d bytes\n", dev->size);
	printk(KERN_INFO "number of sectors -- %d \n", nr_sectors);

	return;

out_vfree:
	if(dev->data)
		vfree(dev->data);
}

static void register_sbulld_dev(struct sbull_dev *dev, int index)
{
	int ret;

	dev->sdev.name = dev->name;
	dev->sdev.dev.init_name = "sbulld0";
	dev->sdev.dev.id = index;
	ret = register_sbull_device(&dev->sdev);
	if (ret) {
		printk(KERN_INFO "device not registered, errno = %d\n", ret);
		return;
	}

	printk(KERN_INFO "device registered ???\n");
	return;
}

/*
 * wait for user command function  
 */
static int wait_function(void *unused)
{
	int i;

	while(1) {
		printk(KERN_INFO "Waiting for user command ...\n");
		wait_event_interruptible(wait_queue_usr, wait_queue_flag != 0);
		if (wait_queue_flag == WAIT_EXIT_CLEANUP) {
			printk(KERN_INFO "wait_queue interrupted by EXIT function\n");
			return 0;
		}

		if (wait_queue_flag == WAIT_EXIT_USERCMD) {
			/*
			 * device creation by user command
 			 */
			Devices = kzalloc(nr_devices * sizeof(struct sbull_dev), GFP_KERNEL);
			if (Devices == NULL) {
				printk(KERN_INFO "device doesnt exist - goto UNREGISTER\n");
			}

			for (i = 0; i < nr_devices; i++) {
				/*
				 * device registration, not working :(
				 */
#if 0
				printk(">>> register device\n");
				register_sbulld_dev(Devices + i, i);
#endif
				printk(KERN_INFO "Setup device has been started\n");
				setup_device(Devices + i, i);
				printk(KERN_INFO "Setup device has been completed\n");
			}
		}
		wait_queue_flag = 0;
	}
	return 0;
}

/*
 * module init & exit functions
 */ 
static int sbull_init(void)
{	
	int ret;
	int i;

	printk(KERN_INFO ">>> Sbull module has been loaded\n");
	ret = bus_register(&sbull_bus_type);
	if (ret)
		return ret;

	ret = device_register(&sbull_bus);
	if (ret)
		printk(KERN_INFO "unable to register sbull-0\n");	

	/*
	 * block device registration
	 */
	sbull_major = register_blkdev(sbull_major, "sbull");
	if (sbull_major <= 0) {
		printk(KERN_INFO "sbull: unable to get major number\n");
		return -EBUSY;
	}
	/*
	 * driver registration
	 */
	register_sbull_driver(&sbull_driver);
	/*
	 * Allocate the device array, and initialize each one
	 */	
	if (creation_mode == 0) {
		Devices = kzalloc(nr_devices * sizeof(struct sbull_dev), GFP_KERNEL);
		if (Devices == NULL) {
			printk(KERN_INFO "device doesnt exist - goto UNREGISTER\n");
			goto out_unregister;
		}

		for (i = 0; i < nr_devices; i++) {
			/*
			 * device registration, not working :(
			 */
#if 0
			printk(">>> register device\n");
			register_sbulld_dev(Devices + i, i);
#endif
			printk(KERN_INFO "Setup device has been started\n");
			setup_device(Devices + i, i);
			printk(KERN_INFO "Setup device has been completed\n");
		}
		return 0;
	} else {
		printk(KERN_INFO ">>> you are in <1> device creation mode\n");
		printk(KERN_INFO ">>> please use <set_devsize> and <add_device> commands on /sys/../sbulld/user_cmd for device creation\n");
		/*
		 * initialize wait queue
		 */
		init_waitqueue_head(&wait_queue_usr);

		wait_thread = kthread_create(wait_function, NULL, "WaitThread");
		if (wait_thread)
			wake_up_process(wait_thread);
		else 
			printk(KERN_INFO "thread NOT created\n");
		
		return 0;
	}

out_unregister:
	unregister_blkdev(sbull_major, "sbull");
	return -ENOMEM;
}

static void sbull_cleanup(void)
{	
	int i;

	printk(KERN_INFO ">>> exit function has been started\n");
	wait_queue_flag = WAIT_EXIT_CLEANUP;
	wake_up_interruptible(&wait_queue_usr);
	if (Devices) {
		for (i = 0; i < nr_devices; i++) {
			struct sbull_dev *dev = Devices + i;
#if 0
			unregister_sbull_device(&dev->sdev);
#endif
			del_timer_sync(&dev->timer);
			if (dev->gd) {
				del_gendisk(dev->gd);
				put_disk(dev->gd);
				printk(KERN_INFO "disk %s has been removed\n", dev->gd->disk_name);
			}
			if (dev->queue) 
				blk_cleanup_queue(dev->queue);
			if (dev->data) 
				vfree(dev->data);
		}
		unregister_sbull_driver(&sbull_driver);
		unregister_blkdev(sbull_major, "sbull");
		kfree(Devices);
	}
	device_unregister(&sbull_bus);
	bus_unregister(&sbull_bus_type);
	printk(KERN_INFO ">>> Sbull module has been unloaded\n");
}

module_init(sbull_init);
module_exit(sbull_cleanup);