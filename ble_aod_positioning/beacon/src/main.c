/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/direction.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

/* Length of CTE in unit of 8[us] */
#define CTE_LEN (0x14U)
/* Number of CTE send in single periodic advertising train */
#define PER_ADV_EVENT_CTE_COUNT 5

static const struct bt_data ad[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static void adv_sent_cb(struct bt_le_ext_adv *adv,
			struct bt_le_ext_adv_sent_info *info);

const static struct bt_le_ext_adv_cb adv_callbacks = {
	.sent = adv_sent_cb,
};

static struct bt_le_ext_adv *adv_set;

const static struct bt_le_adv_param param =
		BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_USE_IDENTITY,
				     BT_GAP_ADV_FAST_INT_MIN_2,
				     BT_GAP_ADV_FAST_INT_MAX_2,
				     NULL);

static struct bt_le_ext_adv_start_param ext_adv_start_param = {
	.timeout = 0,
	.num_events = 0,
};

const static struct bt_le_per_adv_param per_adv_param = {
	.interval_min = BT_GAP_ADV_SLOW_INT_MIN,
	.interval_max = BT_GAP_ADV_SLOW_INT_MAX,
	.options = BT_LE_ADV_OPT_USE_TX_POWER,
};

static const struct gpio_dt_spec aodtx_mode_enable =
		GPIO_DT_SPEC_GET(DT_NODELABEL(switch0_aodtx_mode_enable), gpios);
static const struct gpio_dt_spec chip_enable =
		GPIO_DT_SPEC_GET(DT_NODELABEL(switch0_chip_enable), gpios);

// Only use a single antenna?
#define BT_CTLR_DF_AOD_ANT_SINGLE_MODE 0

// Only use an antenna row?
#define BT_CTLR_DF_AOD_ANT_ROW_MODE 0

// Only use an antenna column?
#define BT_CTLR_DF_AOD_ANT_COLUMN_MODE 0

// Only use the outer antennas?
#define BT_CTLR_DF_AOD_ANT_OUTER_MODE 0

// Use all 16 antennas if none of the above antenna modes are set to 1.

/* Sequence of antenna switch patterns for a CoreHW CHW1010-ANT2-1.1 antenna
 * array board. A switch pattern is defined as an octet (8 bits). Each bit
 * determines the state of a DFE GPIO connected to the RF switch on the antenna
 * array board. Uniquely identifying 16 antennas requires a minimum of 4 bits.
 * See the radio DTS properties in ../boards/nrf52833dk_nrf52833.overlay.
 * See also Bluetooth Core Specification 5.4, Vol 6, Part A, Section 5.1.
 * 
 * See the "CONFIG_BT_CTLR_DF_MAX_ANT_SW_PATTERN_LEN=16" option in ../prj.conf.
 * 
 * The ant_patterns octets are committed to an underlying SWITCHPATTERN buffer.
 * See the radio_df_ant_switch_pattern_set() function in
 * zephyr/subsys/bluetooth/controller/ll_sw/nordic/hal/nrf5/radio/radio_df.c.
 * 
 * The following list is for the default sample spacing of 4 μs where CTEType
 * field value is 2 for "AoD Constant Tone Extension with 2 μs slots":
 */
#if BT_CTLR_DF_AOD_ANT_SINGLE_MODE
/* CoreHW CHW1010-ANT2-1.1 antenna grid for a single antenna:
 *  +----+----+----+----+
 *  |    |    |    |    |
 *  +----+----+----+----+
 *  |    |    | 10 |    |
 *  +----+----+----+----+
 *  |    |    |    |    |
 *  +----+----+----+----+
 *  |    |    |    |    |
 *  +----+----+----+----+
 * 
 * SWITCHPATTERN[0]  = 0x0,  radio.dfe-pdu-antenna,  idle period (PDU Tx/Rx).
 * SWITCHPATTERN[1]  = 0xA,  ant_patterns[0],        guard and reference period.
 * SWITCHPATTERN[2]  = 0xA,  ant_patterns[1],         1st sample slot.
 * SWITCHPATTERN[3]  = 0xA,  ant_patterns[0],         2nd sample slot.
 * SWITCHPATTERN[2]  = 0xA,  ant_patterns[1],         3rd sample slot.
 * SWITCHPATTERN[3]  = 0xA,  ant_patterns[0],         4th sample slot.
 * SWITCHPATTERN[2]  = 0xA,  ant_patterns[1],         5th sample slot.
 * SWITCHPATTERN[3]  = 0xA,  ant_patterns[0],         6th sample slot.
 * SWITCHPATTERN[2]  = 0xA,  ant_patterns[1],         7th sample slot.
 * SWITCHPATTERN[3]  = 0xA,  ant_patterns[0],         8th sample slot.
 * SWITCHPATTERN[2]  = 0xA,  ant_patterns[1],         9th sample slot.
 * SWITCHPATTERN[3]  = 0xA,  ant_patterns[0],        10th sample slot.
 * SWITCHPATTERN[2]  = 0xA,  ant_patterns[1],        11th sample slot.
 * SWITCHPATTERN[3]  = 0xA,  ant_patterns[0],        12th sample slot.
 * SWITCHPATTERN[2]  = 0xA,  ant_patterns[1],        13th sample slot.
 * SWITCHPATTERN[3]  = 0xA,  ant_patterns[0],        14th sample slot.
 * SWITCHPATTERN[2]  = 0xA,  ant_patterns[1],        15th sample slot.
 * SWITCHPATTERN[3]  = 0xA,  ant_patterns[0],        16th sample slot.
 * SWITCHPATTERN[2]  = 0xA,  ant_patterns[1],        17th sample slot.
 * SWITCHPATTERN[3]  = 0xA,  ant_patterns[0],        18th sample slot.
 * SWITCHPATTERN[2]  = 0xA,  ant_patterns[1],        19th sample slot.
 * SWITCHPATTERN[3]  = 0xA,  ant_patterns[0],        20th sample slot.
 * SWITCHPATTERN[2]  = 0xA,  ant_patterns[1],        21st sample slot.
 * SWITCHPATTERN[3]  = 0xA,  ant_patterns[0],        22nd sample slot.
 * SWITCHPATTERN[2]  = 0xA,  ant_patterns[1],        23rd sample slot.
 * SWITCHPATTERN[3]  = 0xA,  ant_patterns[0],        24th sample slot.
 * SWITCHPATTERN[2]  = 0xA,  ant_patterns[1],        25th sample slot.
 * SWITCHPATTERN[3]  = 0xA,  ant_patterns[0],        26th sample slot.
 * SWITCHPATTERN[2]  = 0xA,  ant_patterns[1],        27th sample slot.
 * SWITCHPATTERN[3]  = 0xA,  ant_patterns[0],        28th sample slot.
 * SWITCHPATTERN[2]  = 0xA,  ant_patterns[1],        29th sample slot.
 * SWITCHPATTERN[3]  = 0xA,  ant_patterns[0],        30th sample slot.
 * SWITCHPATTERN[2]  = 0xA,  ant_patterns[1],        31st sample slot.
 * SWITCHPATTERN[3]  = 0xA,  ant_patterns[0],        32nd sample slot.
 * SWITCHPATTERN[2]  = 0xA,  ant_patterns[1],        33rd sample slot.
 * SWITCHPATTERN[3]  = 0xA,  ant_patterns[0],        34th sample slot.
 * SWITCHPATTERN[2]  = 0xA,  ant_patterns[1],        35th sample slot.
 * SWITCHPATTERN[3]  = 0xA,  ant_patterns[0],        36th sample slot.
 * SWITCHPATTERN[2]  = 0xA,  ant_patterns[1],        37th sample slot.
 */
static uint8_t ant_patterns[2] = {
	0xA, 0xA
};
#elif BT_CTLR_DF_AOD_ANT_ROW_MODE
/* CoreHW CHW1010-ANT2-1.1 antenna grid for an antenna row:
 *  +----+----+----+----+
 *  |    |    |    |    |
 *  +----+----+----+----+
 *  |    |    |    |    |
 *  +----+----+----+----+
 *  |    |    |    |    |
 *  +----+----+----+----+
 *  |  2 |  3 |  4 |  6 |
 *  +----+----+----+----+
 * 
 * SWITCHPATTERN[0]  = 0x0,  radio.dfe-pdu-antenna,  idle period (PDU Tx/Rx).
 * SWITCHPATTERN[1]  = 0x2,  ant_patterns[0],        guard and reference period.
 * SWITCHPATTERN[2]  = 0x3,  ant_patterns[1],         1st sample slot.
 * SWITCHPATTERN[3]  = 0x4,  ant_patterns[2],         2nd sample slot.
 * SWITCHPATTERN[4]  = 0x6,  ant_patterns[3],         3rd sample slot.
 * SWITCHPATTERN[5]  = 0x2,  ant_patterns[0],         4th sample slot.
 * SWITCHPATTERN[2]  = 0x3,  ant_patterns[1],         5th sample slot.
 * SWITCHPATTERN[3]  = 0x4,  ant_patterns[2],         6th sample slot.
 * SWITCHPATTERN[4]  = 0x6,  ant_patterns[3],         7th sample slot.
 * SWITCHPATTERN[5]  = 0x2,  ant_patterns[0],         8th sample slot.
 * SWITCHPATTERN[2]  = 0x3,  ant_patterns[1],         9th sample slot.
 * SWITCHPATTERN[3]  = 0x4,  ant_patterns[2],        10th sample slot.
 * SWITCHPATTERN[4]  = 0x6,  ant_patterns[3],        11th sample slot.
 * SWITCHPATTERN[5]  = 0x2,  ant_patterns[0],        12th sample slot.
 * SWITCHPATTERN[2]  = 0x3,  ant_patterns[1],        13th sample slot.
 * SWITCHPATTERN[3]  = 0x4,  ant_patterns[2],        14th sample slot.
 * SWITCHPATTERN[4]  = 0x6,  ant_patterns[3],        15th sample slot.
 * SWITCHPATTERN[5]  = 0x2,  ant_patterns[0],        16th sample slot.
 * SWITCHPATTERN[2]  = 0x3,  ant_patterns[1],        17th sample slot.
 * SWITCHPATTERN[3]  = 0x4,  ant_patterns[2],        18th sample slot.
 * SWITCHPATTERN[4]  = 0x6,  ant_patterns[3],        19th sample slot.
 * SWITCHPATTERN[5]  = 0x2,  ant_patterns[0],        20th sample slot.
 * SWITCHPATTERN[2]  = 0x3,  ant_patterns[1],        21st sample slot.
 * SWITCHPATTERN[3]  = 0x4,  ant_patterns[2],        22nd sample slot.
 * SWITCHPATTERN[4]  = 0x6,  ant_patterns[3],        23rd sample slot.
 * SWITCHPATTERN[5]  = 0x2,  ant_patterns[0],        24th sample slot.
 * SWITCHPATTERN[2]  = 0x3,  ant_patterns[1],        25th sample slot.
 * SWITCHPATTERN[3]  = 0x4,  ant_patterns[2],        26th sample slot.
 * SWITCHPATTERN[4]  = 0x6,  ant_patterns[3],        27th sample slot.
 * SWITCHPATTERN[5]  = 0x2,  ant_patterns[0],        28th sample slot.
 * SWITCHPATTERN[2]  = 0x3,  ant_patterns[1],        29th sample slot.
 * SWITCHPATTERN[3]  = 0x4,  ant_patterns[2],        30th sample slot.
 * SWITCHPATTERN[4]  = 0x6,  ant_patterns[3],        31st sample slot.
 * SWITCHPATTERN[5]  = 0x2,  ant_patterns[0],        32nd sample slot.
 * SWITCHPATTERN[2]  = 0x3,  ant_patterns[1],        33rd sample slot.
 * SWITCHPATTERN[3]  = 0x4,  ant_patterns[2],        34th sample slot.
 * SWITCHPATTERN[4]  = 0x6,  ant_patterns[3],        35th sample slot.
 * SWITCHPATTERN[5]  = 0x2,  ant_patterns[0],        36th sample slot.
 * SWITCHPATTERN[2]  = 0x3,  ant_patterns[1],        37th sample slot.
 */
static uint8_t ant_patterns[4] = {
	0x2, 0x3, 0x4, 0x6
};
#elif BT_CTLR_DF_AOD_ANT_COLUMN_MODE
/* CoreHW CHW1010-ANT2-1.1 antenna grid for an antenna column:
 *  +----+----+----+----+
 *  |    |    |    |  9 |
 *  +----+----+----+----+
 *  |    |    |    |  8 |
 *  +----+----+----+----+
 *  |    |    |    |  7 |
 *  +----+----+----+----+
 *  |    |    |    |  6 |
 *  +----+----+----+----+
 * 
 * SWITCHPATTERN[0]  = 0x0,  radio.dfe-pdu-antenna,  idle period (PDU Tx/Rx).
 * SWITCHPATTERN[1]  = 0x6,  ant_patterns[0],        guard and reference period.
 * SWITCHPATTERN[2]  = 0x7,  ant_patterns[1],         1st sample slot.
 * SWITCHPATTERN[3]  = 0x8,  ant_patterns[2],         2nd sample slot.
 * SWITCHPATTERN[4]  = 0x9,  ant_patterns[3],         3rd sample slot.
 * SWITCHPATTERN[5]  = 0x6,  ant_patterns[0],         4th sample slot.
 * SWITCHPATTERN[2]  = 0x7,  ant_patterns[1],         5th sample slot.
 * SWITCHPATTERN[3]  = 0x8,  ant_patterns[2],         6th sample slot.
 * SWITCHPATTERN[4]  = 0x9,  ant_patterns[3],         7th sample slot.
 * SWITCHPATTERN[5]  = 0x6,  ant_patterns[0],         8th sample slot.
 * SWITCHPATTERN[2]  = 0x7,  ant_patterns[1],         9th sample slot.
 * SWITCHPATTERN[3]  = 0x8,  ant_patterns[2],        10th sample slot.
 * SWITCHPATTERN[4]  = 0x9,  ant_patterns[3],        11th sample slot.
 * SWITCHPATTERN[5]  = 0x6,  ant_patterns[0],        12th sample slot.
 * SWITCHPATTERN[2]  = 0x7,  ant_patterns[1],        13th sample slot.
 * SWITCHPATTERN[3]  = 0x8,  ant_patterns[2],        14th sample slot.
 * SWITCHPATTERN[4]  = 0x9,  ant_patterns[3],        15th sample slot.
 * SWITCHPATTERN[5]  = 0x6,  ant_patterns[0],        16th sample slot.
 * SWITCHPATTERN[2]  = 0x7,  ant_patterns[1],        17th sample slot.
 * SWITCHPATTERN[3]  = 0x8,  ant_patterns[2],        18th sample slot.
 * SWITCHPATTERN[4]  = 0x9,  ant_patterns[3],        19th sample slot.
 * SWITCHPATTERN[5]  = 0x6,  ant_patterns[0],        20th sample slot.
 * SWITCHPATTERN[2]  = 0x7,  ant_patterns[1],        21st sample slot.
 * SWITCHPATTERN[3]  = 0x8,  ant_patterns[2],        22nd sample slot.
 * SWITCHPATTERN[4]  = 0x9,  ant_patterns[3],        23rd sample slot.
 * SWITCHPATTERN[5]  = 0x6,  ant_patterns[0],        24th sample slot.
 * SWITCHPATTERN[2]  = 0x7,  ant_patterns[1],        25th sample slot.
 * SWITCHPATTERN[3]  = 0x8,  ant_patterns[2],        26th sample slot.
 * SWITCHPATTERN[4]  = 0x9,  ant_patterns[3],        27th sample slot.
 * SWITCHPATTERN[5]  = 0x6,  ant_patterns[0],        28th sample slot.
 * SWITCHPATTERN[2]  = 0x7,  ant_patterns[1],        29th sample slot.
 * SWITCHPATTERN[3]  = 0x8,  ant_patterns[2],        30th sample slot.
 * SWITCHPATTERN[4]  = 0x9,  ant_patterns[3],        31st sample slot.
 * SWITCHPATTERN[5]  = 0x6,  ant_patterns[0],        32nd sample slot.
 * SWITCHPATTERN[2]  = 0x7,  ant_patterns[1],        33rd sample slot.
 * SWITCHPATTERN[3]  = 0x8,  ant_patterns[2],        34th sample slot.
 * SWITCHPATTERN[4]  = 0x9,  ant_patterns[3],        35th sample slot.
 * SWITCHPATTERN[5]  = 0x6,  ant_patterns[0],        36th sample slot.
 * SWITCHPATTERN[2]  = 0x7,  ant_patterns[1],        37th sample slot.
 */
static uint8_t ant_patterns[4] = {
	0x6, 0x7, 0x8, 0x9
};
#elif BT_CTLR_DF_AOD_ANT_OUTER_MODE
/* CoreHW CHW1010-ANT2-1.1 antenna grid for the outer antennas:
 *  +----+----+----+----+
 *  | 13 | 12 | 11 |  9 |
 *  +----+----+----+----+
 *  | 14 |    |    |  8 |
 *  +----+----+----+----+
 *  |  1 |    |    |  7 |
 *  +----+----+----+----+
 *  |  2 |  3 |  4 |  6 |
 *  +----+----+----+----+
 * 
 * SWITCHPATTERN[0]  = 0x0,  radio.dfe-pdu-antenna,  idle period (PDU Tx/Rx).
 * SWITCHPATTERN[1]  = 0x1,  ant_patterns[0],        guard and reference period.
 * SWITCHPATTERN[2]  = 0x2,  ant_patterns[1],         1st sample slot.
 * SWITCHPATTERN[3]  = 0x3,  ant_patterns[2],         2nd sample slot.
 * SWITCHPATTERN[4]  = 0x4,  ant_patterns[3],         3rd sample slot.
 * SWITCHPATTERN[5]  = 0x6,  ant_patterns[4],         4th sample slot.
 * SWITCHPATTERN[6]  = 0x7,  ant_patterns[5],         5th sample slot.
 * SWITCHPATTERN[7]  = 0x8,  ant_patterns[6],         6th sample slot.
 * SWITCHPATTERN[8]  = 0x9,  ant_patterns[7],         7th sample slot.
 * SWITCHPATTERN[9]  = 0xB,  ant_patterns[8],         8th sample slot.
 * SWITCHPATTERN[10] = 0xC,  ant_patterns[9],         9th sample slot.
 * SWITCHPATTERN[11] = 0xD,  ant_patterns[10],       10th sample slot.
 * SWITCHPATTERN[12] = 0xE,  ant_patterns[11],       11th sample slot.
 * SWITCHPATTERN[13] = 0x1,  ant_patterns[0],        12th sample slot.
 * SWITCHPATTERN[2]  = 0x2,  ant_patterns[1],        13th sample slot.
 * SWITCHPATTERN[3]  = 0x3,  ant_patterns[2],        14th sample slot.
 * SWITCHPATTERN[4]  = 0x4,  ant_patterns[3],        15th sample slot.
 * SWITCHPATTERN[5]  = 0x6,  ant_patterns[4],        16th sample slot.
 * SWITCHPATTERN[6]  = 0x7,  ant_patterns[5],        17th sample slot.
 * SWITCHPATTERN[7]  = 0x8,  ant_patterns[6],        18th sample slot.
 * SWITCHPATTERN[8]  = 0x9,  ant_patterns[7],        19th sample slot.
 * SWITCHPATTERN[9]  = 0xB,  ant_patterns[8],        20th sample slot.
 * SWITCHPATTERN[10] = 0xC,  ant_patterns[9],        21st sample slot.
 * SWITCHPATTERN[11] = 0xD,  ant_patterns[10],       22nd sample slot.
 * SWITCHPATTERN[12] = 0xE,  ant_patterns[11],       23rd sample slot.
 * SWITCHPATTERN[13] = 0x1,  ant_patterns[0],        24th sample slot.
 * SWITCHPATTERN[2]  = 0x2,  ant_patterns[1],        25th sample slot.
 * SWITCHPATTERN[3]  = 0x3,  ant_patterns[2],        26th sample slot.
 * SWITCHPATTERN[4]  = 0x4,  ant_patterns[3],        27th sample slot.
 * SWITCHPATTERN[5]  = 0x6,  ant_patterns[4],        28th sample slot.
 * SWITCHPATTERN[6]  = 0x7,  ant_patterns[5],        29th sample slot.
 * SWITCHPATTERN[7]  = 0x8,  ant_patterns[6],        30th sample slot.
 * SWITCHPATTERN[8]  = 0x9,  ant_patterns[7],        31st sample slot.
 * SWITCHPATTERN[9]  = 0xB,  ant_patterns[8],        32nd sample slot.
 * SWITCHPATTERN[10] = 0xC,  ant_patterns[9],        33rd sample slot.
 * SWITCHPATTERN[11] = 0xD,  ant_patterns[10],       34th sample slot.
 * SWITCHPATTERN[12] = 0xE,  ant_patterns[11],       35th sample slot.
 * SWITCHPATTERN[13] = 0x1,  ant_patterns[0],        36th sample slot.
 * SWITCHPATTERN[2]  = 0x2,  ant_patterns[1],        37th sample slot.
 */
static uint8_t ant_patterns[12] = {
	0x1, 0x2, 0x3, 0x4, 0x6, 0x7, 0x8, 0x9,
	0xB, 0xC, 0xD, 0xE
};
#else
/* CoreHW CHW1010-ANT2-1.1 antenna grid for all antennas:
 *  +----+----+----+----+
 *  | 13 | 12 | 11 |  9 |
 *  +----+----+----+----+
 *  | 14 | 15 | 10 |  8 |
 *  +----+----+----+----+
 *  |  1 |  0 |  5 |  7 |
 *  +----+----+----+----+
 *  |  2 |  3 |  4 |  6 |
 *  +----+----+----+----+
 * 
 * SWITCHPATTERN[0]  = 0x0,  radio.dfe-pdu-antenna,  idle period (PDU Tx/Rx).
 * SWITCHPATTERN[1]  = 0x0,  ant_patterns[0],        guard and reference period.
 * SWITCHPATTERN[2]  = 0x1,  ant_patterns[1],         1st sample slot.
 * SWITCHPATTERN[3]  = 0x2,  ant_patterns[2],         2nd sample slot.
 * SWITCHPATTERN[4]  = 0x3,  ant_patterns[3],         3rd sample slot.
 * SWITCHPATTERN[5]  = 0x4,  ant_patterns[4],         4th sample slot.
 * SWITCHPATTERN[6]  = 0x5,  ant_patterns[5],         5th sample slot.
 * SWITCHPATTERN[7]  = 0x6,  ant_patterns[6],         6th sample slot.
 * SWITCHPATTERN[8]  = 0x7,  ant_patterns[7],         7th sample slot.
 * SWITCHPATTERN[9]  = 0x8,  ant_patterns[8],         8th sample slot.
 * SWITCHPATTERN[10] = 0x9,  ant_patterns[9],         9th sample slot.
 * SWITCHPATTERN[11] = 0xA,  ant_patterns[10],       10th sample slot.
 * SWITCHPATTERN[12] = 0xB,  ant_patterns[11],       11th sample slot.
 * SWITCHPATTERN[13] = 0xC,  ant_patterns[12],       12th sample slot.
 * SWITCHPATTERN[14] = 0xD,  ant_patterns[13],       13th sample slot.
 * SWITCHPATTERN[15] = 0xE,  ant_patterns[14],       14th sample slot.
 * SWITCHPATTERN[16] = 0xF,  ant_patterns[15],       15th sample slot.
 * SWITCHPATTERN[17] = 0x0,  ant_patterns[0],        16th sample slot.
 * SWITCHPATTERN[2]  = 0x1,  ant_patterns[1],        17th sample slot.
 * SWITCHPATTERN[3]  = 0x2,  ant_patterns[2],        18th sample slot.
 * SWITCHPATTERN[4]  = 0x3,  ant_patterns[3],        19th sample slot.
 * SWITCHPATTERN[5]  = 0x4,  ant_patterns[4],        20th sample slot.
 * SWITCHPATTERN[6]  = 0x5,  ant_patterns[5],        21st sample slot.
 * SWITCHPATTERN[7]  = 0x6,  ant_patterns[6],        22nd sample slot.
 * SWITCHPATTERN[8]  = 0x7,  ant_patterns[7],        23rd sample slot.
 * SWITCHPATTERN[9]  = 0x8,  ant_patterns[8],        24th sample slot.
 * SWITCHPATTERN[10] = 0x9,  ant_patterns[9],        25th sample slot.
 * SWITCHPATTERN[11] = 0xA,  ant_patterns[10],       26th sample slot.
 * SWITCHPATTERN[12] = 0xB,  ant_patterns[11],       27th sample slot.
 * SWITCHPATTERN[13] = 0xC,  ant_patterns[12],       28th sample slot.
 * SWITCHPATTERN[14] = 0xD,  ant_patterns[13],       29th sample slot.
 * SWITCHPATTERN[15] = 0xE,  ant_patterns[14],       30th sample slot.
 * SWITCHPATTERN[16] = 0xF,  ant_patterns[15],       31st sample slot.
 * SWITCHPATTERN[17] = 0x0,  ant_patterns[0],        32nd sample slot.
 * SWITCHPATTERN[2]  = 0x1,  ant_patterns[1],        33rd sample slot.
 * SWITCHPATTERN[3]  = 0x2,  ant_patterns[2],        34th sample slot.
 * SWITCHPATTERN[4]  = 0x3,  ant_patterns[3],        35th sample slot.
 * SWITCHPATTERN[5]  = 0x4,  ant_patterns[4],        36th sample slot.
 * SWITCHPATTERN[6]  = 0x5,  ant_patterns[5],        37th sample slot.
 */
static uint8_t ant_patterns[16] = {
	0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7,
	0x8, 0x9, 0xA, 0xB, 0xC, 0xD, 0xE, 0xF
};
#endif

struct bt_df_adv_cte_tx_param cte_params = { .cte_len = CTE_LEN,
										     .cte_count = PER_ADV_EVENT_CTE_COUNT,
										     .cte_type = BT_DF_CTE_TYPE_AOD_2US,
										     .num_ant_ids = ARRAY_SIZE(ant_patterns),
										     .ant_ids = ant_patterns
};

static void adv_sent_cb(struct bt_le_ext_adv *adv,
			struct bt_le_ext_adv_sent_info *info)
{
	printk("Advertiser[%d] %p sent %d\n", bt_le_ext_adv_get_index(adv),
	       adv, info->num_sent);
}

int main(void)
{
	char addr_s[BT_ADDR_LE_STR_LEN];
	struct bt_le_oob oob_local;
	int err;

	printk("Starting Connectionless Beacon Demo\n");

	printk("Antenna Switch 0 D0 AoDTX-mode Enable GPIO initialization...");
	if (!gpio_is_ready_dt(&aodtx_mode_enable)) {
		printk("failed (AoDTX-mode Enable GPIO spec is not ready for use.)\n");
		return 0;
	}
	err = gpio_pin_configure_dt(&aodtx_mode_enable, GPIO_OUTPUT_INACTIVE);
	if (err) {
		printk("failed (err %d)\n", err);
		return 0;
	}
	printk("success\n");

	printk("Antenna Switch 0 EN Chip Enable GPIO initialization...");
	if (!gpio_is_ready_dt(&chip_enable)) {
		printk("failed (Chip Enable GPIO spec is not ready for use.)\n");
		return 0;
	}
	err = gpio_pin_configure_dt(&chip_enable, GPIO_OUTPUT_INACTIVE);
	if (err) {
		printk("failed (err %d)\n", err);
		return 0;
	}
	printk("success\n");

	printk("Enable AoDTX-mode...");
	err = gpio_pin_set_dt(&aodtx_mode_enable, 1);
	if (err) {
		printk("failed (err %d)\n", err);
		return 0;
	}
	printk("success\n");

	printk("Enable antenna switch...");
	err = gpio_pin_set_dt(&chip_enable, 1);
	if (err) {
		printk("failed (err %d)\n", err);
		return 0;
	}
	printk("success\n");

	/* Initialize the Bluetooth Subsystem */
	printk("Bluetooth initialization...");
	err = bt_enable(NULL);
	if (err) {
		printk("failed (err %d)\n", err);
		return 0;
	}
	printk("success\n");

	printk("Advertising set create...");
	err = bt_le_ext_adv_create(&param, &adv_callbacks, &adv_set);
	if (err) {
		printk("failed (err %d)\n", err);
		return 0;
	}
	printk("success\n");

	printk("Set advertising data...");
	err = bt_le_ext_adv_set_data(adv_set, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("failed (err %d)\n", err);
		return 0;
	}
	printk("success\n");

	printk("Update CTE params...");
	err = bt_df_set_adv_cte_tx_param(adv_set, &cte_params);
	if (err) {
		printk("failed (err %d)\n", err);
		return 0;
	}
	printk("success\n");

	printk("Periodic advertising params set...");
	err = bt_le_per_adv_set_param(adv_set, &per_adv_param);
	if (err) {
		printk("failed (err %d)\n", err);
		return 0;
	}
	printk("success\n");

	printk("Enable CTE...");
	err = bt_df_adv_cte_tx_enable(adv_set);
	if (err) {
		printk("failed (err %d)\n", err);
		return 0;
	}
	printk("success\n");

	printk("Periodic advertising enable...");
	err = bt_le_per_adv_start(adv_set);
	if (err) {
		printk("failed (err %d)\n", err);
		return 0;
	}
	printk("success\n");

	printk("Extended advertising enable...");
	err = bt_le_ext_adv_start(adv_set, &ext_adv_start_param);
	if (err) {
		printk("failed (err %d)\n", err);
		return 0;
	}
	printk("success\n");

	bt_le_ext_adv_oob_get_local(adv_set, &oob_local);
	bt_addr_le_to_str(&oob_local.addr, addr_s, sizeof(addr_s));

	printk("Started extended advertising as %s\n", addr_s);

	return 0;
}
