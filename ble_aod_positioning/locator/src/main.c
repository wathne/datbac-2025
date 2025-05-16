/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stddef.h>
#include <errno.h>
#include <zephyr/kernel.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/direction.h>

#include <zephyr/sys/printk.h> // For printk().

#include "beacon.h"
#include "beacon_database.h"
#include "bt_addr_utils.h" // For BT_ADDR_SIZE (6).
#include "iq_data.h"
#include "iq_data_work_queue.h"
#include "locator.h"

// TODO(wathne): Revise all #include directives, with comments.

// TODO(wathne): Implement a per_adv_context_manager to manage when to sync to
// known beacons and when to scan for new beacons. Responses will include
// enumerated actions, for example 0 ~ "stop syncing and start scanning", and
// 1 ~ "stop scanning and start syncing".

// NOTE(wathne): This main.c file has intentionally been reverted to a version
// that is minimally different from the initial BLE AoD connectionless receiver
// sample. "git diff" and "git difftool" will reveal minimally invasive
// additions to the initial sample code. Tabular indentation has also been
// preserved to avoid artifical differences. This aims to make it easy to
// contrast this version against the initial sample code. Maybe this will be
// appreciated by the next person working on this code. Unfortunately, this
// version is not able to properly cycle between beacons. It tends to sync to
// the same beacon indefinitely. It will repeatedly calculate direction cosines,
// azimuth, and elevation from the first beacon it encounters, but it will not
// be able to calculate a locator position without direction cosines from a
// second beacon. The locator can be tricked into syncing with a second beacon
// by power cycling both beacons. Each beacon transition will result in a new
// locator position. There has been a version of main.c capable of cycling
// between beacons, but that version was rushed in preparation of a simple
// proof-of-concept demonstration. The rushed modifications were reverted
// beacuse they were not appropriate for the longterm development of the
// locator.

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#define PEER_NAME_LEN_MAX 30
/* The Bluetooth Core specification allows controller to wait 6
 * periodic advertising events for
 * synchronization establishment, hence timeout must be longer than that.
 */
#define SYNC_CREATE_TIMEOUT_INTERVAL_NUM 7
/* Maximum length of advertising data represented in hexadecimal format */
#define ADV_DATA_HEX_STR_LEN_MAX (BT_GAP_ADV_MAX_EXT_ADV_DATA_LEN * 2 + 1)

// IQ data work queue.
static struct iq_data_work_queue iq_data_work_queue;

static struct bt_le_per_adv_sync *sync;
static bt_addr_le_t per_addr;
static volatile bool per_adv_found;
static bool scan_enabled;
static uint8_t per_sid;
static uint32_t sync_create_timeout_ms;

static K_SEM_DEFINE(sem_per_adv, 0, 1);
static K_SEM_DEFINE(sem_per_sync, 0, 1);
static K_SEM_DEFINE(sem_per_sync_lost, 0, 1);

#if defined(CONFIG_BT_DF_CTE_RX_AOA)
/* Example sequence of antenna switch patterns for antenna matrix designed by
 * Nordic. For more information about antenna switch patterns see README.rst.
 */
static const uint8_t ant_patterns[] = { 0x2, 0x0, 0x5, 0x6, 0x1, 0x4,
					0xC, 0x9, 0xE, 0xD, 0x8, 0xA };
#endif /* CONFIG_BT_DF_CTE_RX_AOA */

static inline uint32_t adv_interval_to_ms(uint16_t interval)
{
	return interval * 5 / 4;
}

static const char *phy2str(uint8_t phy)
{
	switch (phy) {
	case 0: return "No packets";
	case BT_GAP_LE_PHY_1M: return "LE 1M";
	case BT_GAP_LE_PHY_2M: return "LE 2M";
	case BT_GAP_LE_PHY_CODED: return "LE Coded";
	default: return "Unknown";
	}
}

static const char *cte_type2str(uint8_t type)
{
	switch (type) {
	case BT_DF_CTE_TYPE_AOA: return "AOA";
	case BT_DF_CTE_TYPE_AOD_1US: return "AOD 1 [us]";
	case BT_DF_CTE_TYPE_AOD_2US: return "AOD 2 [us]";
	case BT_DF_CTE_TYPE_NONE: return "";
	default: return "Unknown";
	}
}

static const char *packet_status2str(uint8_t status)
{
	switch (status) {
	case BT_DF_CTE_CRC_OK: return "CRC OK";
	case BT_DF_CTE_CRC_ERR_CTE_BASED_TIME: return "CRC not OK, CTE Info OK";
	case BT_DF_CTE_CRC_ERR_CTE_BASED_OTHER: return "CRC not OK, Sampled other way";
	case BT_DF_CTE_INSUFFICIENT_RESOURCES: return "No resources";
	default: return "Unknown";
	}
}

static bool data_cb(struct bt_data *data, void *user_data)
{
	char *name = user_data;
	uint8_t len;

	switch (data->type) {
	case BT_DATA_NAME_SHORTENED:
	case BT_DATA_NAME_COMPLETE:
		len = MIN(data->data_len, PEER_NAME_LEN_MAX - 1);
		memcpy(name, data->data, len);
		name[len] = '\0';
		return false;
	default:
		return true;
	}
}

static void sync_cb(struct bt_le_per_adv_sync *sync,
		    struct bt_le_per_adv_sync_synced_info *info)
{
	char le_addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

	printk("PER_ADV_SYNC[%u]: [DEVICE]: %s synced, "
	       "Interval 0x%04x (%u ms), PHY %s\n",
	       bt_le_per_adv_sync_get_index(sync), le_addr, info->interval,
	       adv_interval_to_ms(info->interval), phy2str(info->phy));

	k_sem_give(&sem_per_sync);
}

static void term_cb(struct bt_le_per_adv_sync *sync,
		    const struct bt_le_per_adv_sync_term_info *info)
{
	char le_addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

	printk("PER_ADV_SYNC[%u]: [DEVICE]: %s sync terminated\n",
	       bt_le_per_adv_sync_get_index(sync), le_addr);

	k_sem_give(&sem_per_sync_lost);
}

static void recv_cb(struct bt_le_per_adv_sync *sync,
		    const struct bt_le_per_adv_sync_recv_info *info,
		    struct net_buf_simple *buf)
{
	static char data_str[ADV_DATA_HEX_STR_LEN_MAX];
	char le_addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));
	bin2hex(buf->data, buf->len, data_str, sizeof(data_str));

	printk("PER_ADV_SYNC[%u]: [DEVICE]: %s, tx_power %i, "
	       "RSSI %i, CTE %s, data length %u, data: %s\n",
	       bt_le_per_adv_sync_get_index(sync), le_addr, info->tx_power,
	       info->rssi, cte_type2str(info->cte_type), buf->len, data_str);
}

static void cte_recv_cb(struct bt_le_per_adv_sync *sync,
			struct bt_df_per_adv_sync_iq_samples_report const *report)
{
	// Timestamp of when the IQ samples report arrived in this cte_recv_cb()
	// callback function. Elapsed time since the system booted, in milliseconds.
	int64_t report_timestamp = k_uptime_get();

	struct bt_le_per_adv_sync_info info;

	printk("Retrieving Periodic Advertising Sync Info...");
	int err = bt_le_per_adv_sync_get_info(sync, &info);
	if (err) {
		printk("failed (err %d)\n", err);
	}
	printk("success\n");

	char addr_s[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(&info.addr, addr_s, sizeof(addr_s));
	printk("Periodic Advertiser Address: %s\n", addr_s);
	//printk("Advertiser SID: %u\n", info.sid);
	//printk("Advertiser PHY: %u\n", info.phy);
	//printk("Periodic advertising interval (N * 1.25 ms): %u\n", info.interval);

	printk("Channel index: %u\n", report->chan_idx);

	/*
	printk("CTE[%u]: samples count %d, cte type %s, slot durations: %u [us], "
	       "packet status %s, RSSI %i\n",
	       bt_le_per_adv_sync_get_index(sync), report->sample_count,
	       cte_type2str(report->cte_type), report->slot_durations,
	       packet_status2str(report->packet_status), report->rssi);
	*/

	// Intermediate structure for raw IQ samples extracted from an IQ samples
	// report.
	struct iq_raw_samples iq_raw_samples;

	// Initialize the raw IQ samples structure from the IQ samples report.
	iq_raw_samples_init(&iq_raw_samples, report, &info, report_timestamp);

	// Submit the raw IQ samples structure to the IQ data work queue.
	// This is a specialized work queue with LIFO processing and FIFO eviction.
	// The work queue is unfair and will process the most recently submitted
	// work first (LIFO processing). It is expected that more work will be
	// submitted to the work queue than the work queue is able to process. The
	// oldest work will be evicted from the work queue when the work queue is
	// full (FIFO eviction).
	iq_data_work_queue_submit(&iq_data_work_queue, &iq_raw_samples);
}

static struct bt_le_per_adv_sync_cb sync_callbacks = {
	.synced = sync_cb,
	.term = term_cb,
	.recv = recv_cb,
	.cte_report_cb = cte_recv_cb,
};

static void scan_recv(const struct bt_le_scan_recv_info *info,
		      struct net_buf_simple *buf)
{
	char le_addr[BT_ADDR_LE_STR_LEN];
	char name[PEER_NAME_LEN_MAX];

	(void)memset(name, 0, sizeof(name));

	bt_data_parse(buf, data_cb, name);

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

	printk("[DEVICE]: %s, AD evt type %u, Tx Pwr: %i, RSSI %i %s C:%u S:%u "
	       "D:%u SR:%u E:%u Prim: %s, Secn: %s, Interval: 0x%04x (%u ms), "
	       "SID: %u\n",
	       le_addr, info->adv_type, info->tx_power, info->rssi, name,
	       (info->adv_props & BT_GAP_ADV_PROP_CONNECTABLE) != 0,
	       (info->adv_props & BT_GAP_ADV_PROP_SCANNABLE) != 0,
	       (info->adv_props & BT_GAP_ADV_PROP_DIRECTED) != 0,
	       (info->adv_props & BT_GAP_ADV_PROP_SCAN_RESPONSE) != 0,
	       (info->adv_props & BT_GAP_ADV_PROP_EXT_ADV) != 0, phy2str(info->primary_phy),
	       phy2str(info->secondary_phy), info->interval, adv_interval_to_ms(info->interval),
	       info->sid);

	if (!per_adv_found && info->interval) {
		sync_create_timeout_ms =
			adv_interval_to_ms(info->interval) * SYNC_CREATE_TIMEOUT_INTERVAL_NUM;
		per_adv_found = true;
		per_sid = info->sid;
		bt_addr_le_copy(&per_addr, info->addr);

		k_sem_give(&sem_per_adv);
	}
}

static struct bt_le_scan_cb scan_callbacks = {
	.recv = scan_recv,
};

static void create_sync(void)
{
	struct bt_le_per_adv_sync_param sync_create_param;
	int err;

	printk("Creating Periodic Advertising Sync...");
	bt_addr_le_copy(&sync_create_param.addr, &per_addr);
	sync_create_param.options = 0;
	sync_create_param.sid = per_sid;
	sync_create_param.skip = 0;
	sync_create_param.timeout = 0xa;
	err = bt_le_per_adv_sync_create(&sync_create_param, &sync);
	if (err) {
		printk("failed (err %d)\n", err);
		return;
	}
	printk("success.\n");
}

static int delete_sync(void)
{
	int err;

	printk("Deleting Periodic Advertising Sync...");
	err = bt_le_per_adv_sync_delete(sync);
	if (err) {
		printk("failed (err %d)\n", err);
		return err;
	}
	printk("success\n");

	return 0;
}

static void enable_cte_rx(void)
{
	int err;

	const struct bt_df_per_adv_sync_cte_rx_param cte_rx_params = {
		.max_cte_count = 5,
#if defined(CONFIG_BT_DF_CTE_RX_AOA)
		.cte_types = BT_DF_CTE_TYPE_ALL,
		.slot_durations = 0x2,
		.num_ant_ids = ARRAY_SIZE(ant_patterns),
		.ant_ids = ant_patterns,
#else
		.cte_types = BT_DF_CTE_TYPE_AOD_1US | BT_DF_CTE_TYPE_AOD_2US,
#endif /* CONFIG_BT_DF_CTE_RX_AOA */
	};

	printk("Enable receiving of CTE...\n");
	err = bt_df_per_adv_sync_cte_rx_enable(sync, &cte_rx_params);
	if (err) {
		printk("failed (err %d)\n", err);
		return;
	}
	printk("success. CTE receive enabled.\n");
}

static int scan_init(void)
{
	printk("Scan callbacks register...");
	bt_le_scan_cb_register(&scan_callbacks);
	printk("success.\n");

	printk("Periodic Advertising callbacks register...");
	bt_le_per_adv_sync_cb_register(&sync_callbacks);
	printk("success.\n");

	return 0;
}

static int scan_enable(void)
{
	struct bt_le_scan_param param = {
		.type = BT_LE_SCAN_TYPE_ACTIVE,
		.options = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
		.interval = BT_GAP_SCAN_FAST_INTERVAL,
		.window = BT_GAP_SCAN_FAST_WINDOW,
		.timeout = 0U,
	};

	int err;

	if (!scan_enabled) {
		printk("Start scanning...");
		err = bt_le_scan_start(&param, NULL);
		if (err) {
			printk("failed (err %d)\n", err);
			return err;
		}
		printk("success\n");
		scan_enabled = true;
	}

	return 0;
}

static void scan_disable(void)
{
	int err;

	printk("Scan disable...");
	err = bt_le_scan_stop();
	if (err) {
		printk("failed (err %d)\n", err);
		return;
	}
	printk("Success.\n");

	scan_enabled = false;
}

int main(void)
{
	int err;

	printk("Starting Connectionless Locator Demo\n");

	printk("Initializing global beacon database...");
	err = beacon_database_init_global();
	if (err) {
		printk("failed (err %d)\n", err);
		return 0;
	}
	printk("success\n");

	// TODO(wathne): Populating the beacon database with beacon data from within
	// this main() function is a temporary solution. Beacon data for the beacon
	// database should instead be sourced from a local file or from an external
	// server.

	// Beacon 1, 1050638918, F6:66:CD:FD:DC:EB.
	printk("Initializing beacon 1 struct (1050638918, F6:66:CD:FD:DC:EB)...");
	struct beacon beacon_1;
	uint8_t beacon_1_mac[BT_ADDR_SIZE] = {0xF6, 0x66, 0xCD, 0xFD, 0xDC, 0xEB};
	err = beacon_init(&beacon_1, beacon_1_mac, 10, 0, 0, 0, 0, 0);
	if (err) {
		printk("failed (err %d)\n", err);
		return 0;
	}
	printk("success\n");

	printk("Adding beacon 1 struct to global beacon database...");
	err = beacon_database_put(&g_beacon_db, &beacon_1);
	if (err) {
		printk("failed (err %d)\n", err);
		return 0;
	}
	printk("success\n");

	// TODO(wathne): The debugger on Beacon 2 has become unresponsive. It may be
	// possible to flash Beacon 2 from another NRF52833DK. Beacon 2 is currently
	// decomissioned.
	// Beacon 2, 1050625843, CE:96:F5:15:D2:45.
	printk("Initializing beacon 2 struct (1050625843, CE:96:F5:15:D2:45)...");
	struct beacon beacon_2;
	uint8_t beacon_2_mac[BT_ADDR_SIZE] = {0xCE, 0x96, 0xF5, 0x15, 0xD2, 0x45};
	err = beacon_init(&beacon_2, beacon_2_mac, 0, 0, 0, 0, 0, 0);
	if (err) {
		printk("failed (err %d)\n", err);
		return 0;
	}
	printk("success\n");

	printk("Adding beacon 2 struct to global beacon database...");
	err = beacon_database_put(&g_beacon_db, &beacon_2);
	if (err) {
		printk("failed (err %d)\n", err);
		return 0;
	}
	printk("success\n");

	// Beacon 3,  685689749, D5:55:32:1F:94:9F.
	printk("Initializing beacon 3 struct ( 685689749, D5:55:32:1F:94:9F)...");
	struct beacon beacon_3;
	uint8_t beacon_3_mac[BT_ADDR_SIZE] = {0xD5, 0x55, 0x32, 0x1F, 0x94, 0x9F};
	err = beacon_init(&beacon_3, beacon_3_mac, 0, 0, 0, 0, 0, 0);
	if (err) {
		printk("failed (err %d)\n", err);
		return 0;
	}
	printk("success\n");

	printk("Adding beacon 3 struct to global beacon database...");
	err = beacon_database_put(&g_beacon_db, &beacon_3);
	if (err) {
		printk("failed (err %d)\n", err);
		return 0;
	}
	printk("success\n");

	// Beacon 1, 1050638918, F6:66:CD:FD:DC:EB.
	// Beacon 2, 1050625843, CE:96:F5:15:D2:45.
	// Beacon 3,  685689749, D5:55:32:1F:94:9F.
	printk("Printing global beacon database entries:\n");
	struct beacon beacon_temp;
	uint8_t beacons_mac_big_endian[3][6] = {
			{0xF6, 0x66, 0xCD, 0xFD, 0xDC, 0xEB},
			{0xCE, 0x96, 0xF5, 0x15, 0xD2, 0x45},
			{0xD5, 0x55, 0x32, 0x1F, 0x94, 0x9F}};
	uint8_t beacons_mac_little_endian[3][6] = {
			{0xEB, 0xDC, 0xFD, 0xCD, 0x66, 0xF6},
			{0x45, 0xD2, 0x15, 0xF5, 0x96, 0xCE},
			{0x9F, 0x94, 0x1F, 0x32, 0x55, 0xD5}};
	for (int i = 0; i < 3; i++) {
		beacon_database_get(
				&g_beacon_db,
				&beacon_temp,
				beacons_mac_little_endian[i]);
		printk(
				"mac = %02X:%02X:%02X:%02X:%02X:%02X\n"
				"\n"
				"(x, y, z) = (%.2f, %.2f, %.2f)\n"
				"\n"
				"    [ i_x j_x k_x ]   [ %6.2f %6.2f %6.2f ]\n"
				"R = [ i_y j_y k_y ] = [ %6.2f %6.2f %6.2f ]\n"
				"    [ i_z j_z k_z ]   [ %6.2f %6.2f %6.2f ]\n"
				"\n",
				beacon_temp.mac_big_endian[0], beacon_temp.mac_big_endian[1],
				beacon_temp.mac_big_endian[2], beacon_temp.mac_big_endian[3],
				beacon_temp.mac_big_endian[4], beacon_temp.mac_big_endian[5],
				beacon_temp.x, beacon_temp.y, beacon_temp.z,
				beacon_temp.i_x, beacon_temp.j_x, beacon_temp.k_x,
				beacon_temp.i_y, beacon_temp.j_y, beacon_temp.k_y,
				beacon_temp.i_z, beacon_temp.j_z, beacon_temp.k_z);
	}

	printk("Initializing global locator with global beacon database...");
	err = locator_init_global(&g_beacon_db);
	if (err) {
		printk("failed (err %d)\n", err);
		return 0;
	}
	printk("success\n");

	printk("Initializing work queue with LIFO processing and FIFO eviction...");
	iq_data_work_queue_init(
			&iq_data_work_queue,
			&k_sys_work_q,
			iq_data_process);
	printk("success\n");

	printk("Bluetooth initialization...");
	err = bt_enable(NULL);
	if (err) {
		printk("failed (err %d)\n", err);
	}
	printk("success\n");

	scan_init();

	scan_enabled = false;

	while (true) {
		per_adv_found = false;
		scan_enable();

		printk("Waiting for periodic advertising...\n");
		err = k_sem_take(&sem_per_adv, K_FOREVER);
		if (err) {
			printk("failed (err %d)\n", err);
			return 0;
		}
		printk("success. Found periodic advertising.\n");

		create_sync();

		printk("Waiting for periodic sync...\n");
		err = k_sem_take(&sem_per_sync, K_MSEC(sync_create_timeout_ms));
		if (err) {
			printk("failed (err %d)\n", err);
			err = delete_sync();
			if (err) {
				return 0;
			}
			continue;
		}
		printk("success. Periodic sync established.\n");

		enable_cte_rx();

		/* Disable scan to cleanup output */
		scan_disable();

		printk("Waiting for periodic sync lost...\n");
		err = k_sem_take(&sem_per_sync_lost, K_FOREVER);
		if (err) {
			printk("failed (err %d)\n", err);
			return 0;
		}
		printk("Periodic sync lost.\n");
	}
}
