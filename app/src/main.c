/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <zephyr.h>
#include <drivers/gpio.h>
#include <soc.h>
#include "logging/log.h"
#include <settings/settings.h>
#include <arch/cpu.h>
#include <sys/util.h>
#include <dk_buttons_and_leds.h>

#include <drivers/ipm.h>

#include <openamp/open_amp.h>
#include <metal/sys.h>
#include <metal/device.h>
#include <metal/alloc.h>

//#include <net/buf.h>

#define LOG_LEVEL LOG_LEVEL_INFO
#define LOG_MODULE_NAME dualcore_app
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define RUN_STATUS_LED          DK_LED1
#define CON_STATUS_LED          DK_LED2
#define RUN_LED_BLINK_INTERVAL  1000

#define USER_LED                DK_LED3

#define USER_BUTTON             DK_BTN1_MSK

static bool app_button_state;

//void bt_rpmsg_rx(u8_t *data, size_t len);

static K_SEM_DEFINE(ready_sem, 0, 1);
static K_SEM_DEFINE(rx_sem, 0, 1);

static K_THREAD_STACK_DEFINE(bt_rpmsg_rx_thread_stack,
			     CONFIG_DUALCORE_RPMSG_NRF53_RX_STACK_SIZE);
static struct k_thread bt_rpmsg_rx_thread_data;

static struct device *ipm_tx_handle;
static struct device *ipm_rx_handle;

/* Configuration defines */

#define SHM_NODE            DT_CHOSEN(zephyr_ipc_shm)
#define SHM_BASE_ADDRESS    DT_REG_ADDR(SHM_NODE)

#define SHM_START_ADDR      (SHM_BASE_ADDRESS + 0x400)
#define SHM_SIZE            0x7c00
#define SHM_DEVICE_NAME     "sram0.shm"

BUILD_ASSERT((SHM_START_ADDR + SHM_SIZE - SHM_BASE_ADDRESS)
		<= DT_REG_SIZE(SHM_NODE),
	"Allocated size exceeds available shared memory reserved for IPC");

#define VRING_COUNT         2
#define VRING_TX_ADDRESS    (SHM_START_ADDR + SHM_SIZE - 0x400)
#define VRING_RX_ADDRESS    (VRING_TX_ADDRESS - 0x400)
#define VRING_ALIGNMENT     4
#define VRING_SIZE          16

#define VDEV_STATUS_ADDR    SHM_BASE_ADDRESS

/* End of configuration defines */

static metal_phys_addr_t shm_physmap[] = { SHM_START_ADDR };
static struct metal_device shm_device = {
	.name = SHM_DEVICE_NAME,
	.bus = NULL,
	.num_regions = 1,
	.regions = {
		{
			.virt       = (void *) SHM_START_ADDR,
			.physmap    = shm_physmap,
			.size       = SHM_SIZE,
			.page_shift = 0xffffffff,
			.page_mask  = 0xffffffff,
			.mem_flags  = 0,
			.ops        = { NULL },
		},
	},
	.node = { NULL },
	.irq_num = 0,
	.irq_info = NULL
};

static struct virtqueue *vq[2];
static struct rpmsg_endpoint ep;

static void button_changed(u32_t button_state, u32_t has_changed)
{
	if (has_changed & USER_BUTTON) {
		app_button_state = button_state ? true : false;
	}
}

static int init_button(void)
{
	int err;

	err = dk_buttons_init(button_changed);
	if (err) {
		printk("Cannot init buttons (err: %d)\n", err);
	}

	return err;
}

static unsigned char virtio_get_status(struct virtio_device *vdev)
{
	return VIRTIO_CONFIG_STATUS_DRIVER_OK;
}

static void virtio_set_status(struct virtio_device *vdev, unsigned char status)
{
	sys_write8(status, VDEV_STATUS_ADDR);
}

static u32_t virtio_get_features(struct virtio_device *vdev)
{
	return BIT(VIRTIO_RPMSG_F_NS);
}

static void virtio_set_features(struct virtio_device *vdev, u32_t features)
{
	/* No need for implementation */
}

static void virtio_notify(struct virtqueue *vq)
{
	int status;

	status = ipm_send(ipm_tx_handle, 0, 0, NULL, 0);
	if (status != 0) {
		LOG_ERR("ipm_send failed to notify: %d", status);
	}
}

const struct virtio_dispatch dispatch = {
	.get_status = virtio_get_status,
	.set_status = virtio_set_status,
	.get_features = virtio_get_features,
	.set_features = virtio_set_features,
	.notify = virtio_notify,
};

static void ipm_callback(void *context, u32_t id, volatile void *data)
{
	LOG_DBG("Got callback of id %u", id);
	k_sem_give(&rx_sem);
}

static int endpoint_cb(struct rpmsg_endpoint *ept, void *data, size_t len,
	u32_t src, void *priv)
{
	LOG_DBG("Received message of %u bytes.", len);
	//LOG_HEXDUMP_DBG((uint8_t *)data, len, "Data:");
	LOG_INF("Data: \n\n%s\n", log_strdup((uint8_t *)data));

	//bt_rpmsg_rx(data, len);

	return RPMSG_SUCCESS;
}

static void rpmsg_service_unbind(struct rpmsg_endpoint *ep)
{
	rpmsg_destroy_ept(ep);
}

static void ns_bind_cb(struct rpmsg_device *rdev, const char *name, u32_t dest)
{
	(void)rpmsg_create_ept(&ep,
				rdev,
				name,
				RPMSG_ADDR_ANY,
				dest,
				endpoint_cb,
				rpmsg_service_unbind);

	k_sem_give(&ready_sem);
}

static void bt_rpmsg_rx_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		int status = k_sem_take(&rx_sem, K_FOREVER);

		if (status == 0) {
			virtqueue_notification(vq[0]);
		}
	}
}

int my_rpmsg_platform_init(void)
{
	int err;
	int status = 0;

	struct metal_init_params metal_params = METAL_INIT_DEFAULTS;
	static struct virtio_vring_info     rvrings[2];
	static struct rpmsg_virtio_shm_pool shpool;
	static struct virtio_device         vdev;
	static struct rpmsg_virtio_device   rvdev;
	static struct metal_io_region       *io;
	static struct metal_device          *device;

	LOG_INF("Initializing rpmsg platform");

	/* Setup thread for RX data processing. */
	k_thread_create(&bt_rpmsg_rx_thread_data, bt_rpmsg_rx_thread_stack,
			K_THREAD_STACK_SIZEOF(bt_rpmsg_rx_thread_stack),
			bt_rpmsg_rx_thread, NULL, NULL, NULL,
			K_PRIO_COOP(CONFIG_DUALCORE_RPMSG_NRF53_RX_PRIO),
			0, K_NO_WAIT);

	/* Libmetal setup */
	err = metal_init(&metal_params);
	if (err) {
		LOG_ERR("metal_init: failed - error code %d", err);
		return err;
	}

	err = metal_register_generic_device(&shm_device);
	if (err) {
		LOG_ERR("Couldn't register shared memory device: %d", err);
		return err;
	}

	err = metal_device_open("generic", SHM_DEVICE_NAME, &device);
	if (err) {
		LOG_ERR("metal_device_open failed: %d", err);
		return err;
	}

	io = metal_device_io_region(device, 0);
	if (!io) {
		LOG_ERR("metal_device_io_region failed to get region");
		return -ENODEV;
	}

	/* IPM setup */
	ipm_tx_handle = device_get_binding("IPM_0");
	if (!ipm_tx_handle) {
		LOG_ERR("Could not get TX IPM device handle");
		return -ENODEV;
	}

	ipm_rx_handle = device_get_binding("IPM_1");
	if (!ipm_rx_handle) {
		LOG_ERR("Could not get RX IPM device handle");
		return -ENODEV;
	}

	ipm_register_callback(ipm_rx_handle, ipm_callback, NULL);

	//testing this
	status = ipm_set_enabled(ipm_rx_handle, 1);
	if (status != 0) {
		printk("ipm_set_enabled failed\n");
		return -1;
	}

	/* Virtqueue setup */
	vq[0] = virtqueue_allocate(VRING_SIZE);
	if (!vq[0]) {
		LOG_ERR("virtqueue_allocate failed to alloc vq[0]");
		return -ENOMEM;
	}

	vq[1] = virtqueue_allocate(VRING_SIZE);
	if (!vq[1]) {
		LOG_ERR("virtqueue_allocate failed to alloc vq[1]");
		return -ENOMEM;
	}

	rvrings[0].io = io;
	rvrings[0].info.vaddr = (void *)VRING_TX_ADDRESS;
	rvrings[0].info.num_descs = VRING_SIZE;
	rvrings[0].info.align = VRING_ALIGNMENT;
	rvrings[0].vq = vq[0];

	rvrings[1].io = io;
	rvrings[1].info.vaddr = (void *)VRING_RX_ADDRESS;
	rvrings[1].info.num_descs = VRING_SIZE;
	rvrings[1].info.align = VRING_ALIGNMENT;
	rvrings[1].vq = vq[1];

	vdev.role = RPMSG_MASTER;
	vdev.vrings_num = VRING_COUNT;
	vdev.func = &dispatch;
	vdev.vrings_info = &rvrings[0];

	rpmsg_virtio_init_shm_pool(&shpool, (void *)SHM_START_ADDR, SHM_SIZE);
	err = rpmsg_init_vdev(&rvdev, &vdev, ns_bind_cb, io, &shpool);
	if (err) {
		LOG_ERR("rpmsg_init_vdev failed %d", err);
		return err;
	}

	/* Wait til nameservice ep is setup */
	k_sem_take(&ready_sem, K_FOREVER);

	return 0;
}

//bt_rpmsg_platform_send
int send_data(void *data, size_t len)
{
	LOG_INF("Sending data...");
	return rpmsg_send(&ep, data, len);
}

void main(void)
{
	int blink_status = 0;
	int err;

	printk("Starting example\n");

	err = dk_leds_init();
	if (err) {
		printk("LEDs init failed (err %d)\n", err);
		return;
	}

	err = init_button();
	if (err) {
		printk("Button init failed (err %d)\n", err);
		return;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = my_rpmsg_platform_init();
	if (err) {
		printk("Rpmsg platform init failed (err %d)\n", err);
		return;
	}

	uint8_t msg[15] = "Hello World!";
	msg[14] = 0;
	
	err = send_data(msg, sizeof(msg));
	if (err<0) {
		printk("send_data failed (err %d)\n", err);
		return;
	}

	for (;;) {
		dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	}
}

/*
int init_status_flag(struct device *arg)
{
	virtio_set_status(NULL, 0);

	return 0;
}

SYS_INIT(init_status_flag, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
*/