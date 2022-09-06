#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/string.h>
#include <linux/stat.h>
#include <linux/completion.h>

#include "../ipc-shm.h"

#define MODULE_NAME "S32_IPCF"
#define MODULE_VER "0.1"

MODULE_AUTHOR("Nexter Robotics");
MODULE_LICENSE("Dual BSD/GPL");
// MODULE_AUTHOR(“Falher Alexandre”);
// MODULE_DESCRIPTION(“NXP S32G2 IPCF module”);
MODULE_ALIAS(MODULE_NAME);
MODULE_VERSION(MODULE_VER);

#define LOCAL_SHM_ADDR 0x34100000
#define IPC_SHM_SIZE 0x100000
#define REMOTE_SHM_ADDR (LOCAL_SHM_ADDR + IPC_SHM_SIZE)
#define INTER_CORE_TX_IRQ 2u
#define INTER_CORE_RX_IRQ 1u
#define BUF_LEN 1024

#define sample_fmt(fmt) MODULE_NAME": %s(): "fmt
#define sample_err(fmt, ...) pr_err(sample_fmt(fmt), __func__, ##__VA_ARGS__)
#define sample_warn(fmt, ...) pr_warn(sample_fmt(fmt), __func__, ##__VA_ARGS__)
#define sample_info(fmt, ...) pr_info(MODULE_NAME": "fmt, ##__VA_ARGS__)
#define sample_dbg(fmt, ...) pr_debug(sample_fmt(fmt), __func__, ##__VA_ARGS__)

// static int msg_sizes[IPC_SHM_MAX_POOLS] = {BUF_LEN};
// static int msg_sizes_argc = 1;

static struct ipc_sample_app {
	char last_tx_msg[BUF_LEN];
	char last_rx_msg[BUF_LEN];
	struct kobject *ipc_kobj;
	struct kobj_attribute ping_attr;
} app;

////////////////////////////////////////////////////////
///                      IPC CODE                    ///
///                                                  ///

static void chan_rx_cb(void *cb_arg, int chan_id, void *buf, size_t size);

static int init_ipc_shm(void)
{
	int err;

	/* memory buffer pools */
	struct ipc_shm_pool_cfg buf_pools[] = {
		{
			.num_bufs = 5,
			.buf_size = BUF_LEN
		}
	};

	/* data channel configuration */
	struct ipc_shm_channel_cfg data_chan_cfg = {
		.type = IPC_SHM_MANAGED,
		.ch = {
			.managed = {
				.num_pools = ARRAY_SIZE(buf_pools),
				.pools = buf_pools,
				.rx_cb = chan_rx_cb,
				// .cb_arg = &app,
				.cb_arg = NULL
			},
		}
	};

	/* use same configuration for all data channels */
	struct ipc_shm_channel_cfg channels[] = {
		data_chan_cfg
	};

	/* ipc shm configuration */
	struct ipc_shm_cfg shm_cfg = {
		.local_shm_addr = LOCAL_SHM_ADDR,
		.remote_shm_addr = REMOTE_SHM_ADDR,
		.shm_size = IPC_SHM_SIZE,
		.inter_core_tx_irq = INTER_CORE_TX_IRQ,
		.inter_core_rx_irq = INTER_CORE_RX_IRQ,
		.remote_core = {
			.type = IPC_CORE_DEFAULT,
			.index = 0,
		},
		.num_channels = ARRAY_SIZE(channels),
		.channels = channels
	};

		err = ipc_shm_init(&shm_cfg);
	if (err)
		return err;

	return 0;
}

static void chan_rx_cb(void *arg, int chan_id, void *buf, size_t size) {
	int err = 0;

	sample_info("ch %d << %ld bytes: %s\n", chan_id, size, (char *)buf);

	memcpy(app.last_rx_msg, buf, size);

	err = ipc_shm_release_buf(chan_id, buf);
	if (err) {
		sample_err("failed to free buffer for channel %d,"
			    "err code %d\n", chan_id, err);
	}
}

static int send_msg(int msg_len, const char* msg, int chan_id) {
	char *buf = NULL;

	if (ipc_shm_is_remote_ready()) {
		return -1;
	}

	buf = ipc_shm_acquire_buf(chan_id, msg_len);
	if (buf != NULL) {
		return -2;
	}

	buf = msg;

	strcpy(app.last_tx_msg, buf);

	if (ipc_shm_tx(chan_id, buf, msg_len)) {
		return -3;
	}

	return 0;
}

///                                                  ///
///                      IPC CODE                    ///
////////////////////////////////////////////////////////



////////////////////////////////////////////////////////
///                     SYSFS CODE                   ///
///                                                  ///

static ssize_t ipc_sysfs_show(struct kobject *kobj, struct kobj_attribute *attr,
				char *buf)
{
	int value = 0;

	if (strcmp(attr->attr.name, app.ping_attr.attr.name) == 0) {
		value = app.last_tx_msg;
	}

	return sprintf(buf, "%d\n", value);
}

static ssize_t ipc_sysfs_store(struct kobject *kobj,
			       struct kobj_attribute *attr, const char *buf,
			       size_t count)
{
	if (strcmp(attr->attr.name, app.ping_attr.attr.name) == 0) {
		send_msg(count, buf, 0);
	}

	return count;
}

static int init_sysfs(void)
{
	int err = 0;
	struct kobj_attribute ping_attr =
		__ATTR(ping, 0600, ipc_sysfs_show, ipc_sysfs_store);
	app.ping_attr = ping_attr;

	app.ipc_kobj = kobject_create_and_add(MODULE_NAME, kernel_kobj);
	if (!app.ipc_kobj)
		return -ENOMEM;

	err = sysfs_create_file(app.ipc_kobj, &app.ping_attr.attr);
	if (err) {
		sample_err("sysfs file creation failed, error code %d\n", err);
		goto err_kobj_free;
	}

	return 0;

err_kobj_free:
	kobject_put(app.ipc_kobj);
	return err;
}

static void free_sysfs(void)
{
	kobject_put(app.ipc_kobj);
}

///                                                  ///
///                     SYSFS CODE                   ///
////////////////////////////////////////////////////////



////////////////////////////////////////////////////////
///         KERNEL MODULE IMPERATIVE CODE            ///
///                                                  ///

static int __init ipc_init(void) {
	int err = 0;

	sample_dbg("module version "MODULE_VER" init\n");

	err = init_ipc_shm();
	if (err)
		return err;

	err = init_sysfs();
	if (err)
		return err;

	return 0;
}

static void __exit ipc_exit(void) {
	sample_dbg("module version "MODULE_VER" exit\n");

	free_sysfs();
	ipc_shm_free();
}

module_init(ipc_init);
module_exit(ipc_exit);

///                                                  ///
///         KERNEL MODULE IMPERATIVE CODE            ///
////////////////////////////////////////////////////////
