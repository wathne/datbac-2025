#include "beacon_database.h" // For beacon database structure and BEACON_DATABASE_CAPACITY.
#include <errno.h> // For ENOENT (2), EINVAL (22), and ENOSPC (28).
#include <stddef.h> // For NULL ((void *)0).
#include <stdint.h> // For uint8_t.
#include "beacon.h" // For beacon structure.
#include "bt_addr_utils.h" // For BT_ADDR_SIZE (6) and bt_addr_mac_compare().

// The global beacon database instance.
// See the beacon_database_init_global() function.
struct beacon_database g_beacon_db;

int beacon_database_init(struct beacon_database *beacon_db) {
    if (beacon_db == NULL) {
        return -EINVAL; // -22 ~ "Invalid argument".
    }

    beacon_db->count = 0;

    return 0; // 0 ~ "Success".
}

int beacon_database_init_global() {
    return beacon_database_init(&g_beacon_db);
}

int beacon_database_put(
        struct beacon_database *beacon_db,
        const struct beacon *beacon) {
    if (beacon_db == NULL || beacon == NULL) {
        return -EINVAL; // -22 ~ "Invalid argument".
    }

    int ret;
    for (int i = 0; i < beacon_db->count; i++) {
        ret = bt_addr_mac_compare(
                beacon->mac_little_endian,
                beacon_db->beacons[i].mac_little_endian);
        if (ret == 1) {
            // Update beacon.
            beacon_db->beacons[i] = *beacon;
            return 0; // 0 ~ "Success".
        }
    }

    if (beacon_db->count >= BEACON_DATABASE_CAPACITY) {
        return -ENOSPC; // -28 ~ "No space left on device".
    }

    // Add beacon.
    beacon_db->beacons[beacon_db->count] = *beacon;
    beacon_db->count++;
    return 0; // 0 ~ "Success".
}

int beacon_database_get(
        struct beacon_database *beacon_db,
        struct beacon *beacon,
        const uint8_t mac_little_endian[BT_ADDR_SIZE]) {
    if (beacon_db == NULL || beacon == NULL || mac_little_endian == NULL) {
        return -EINVAL; // -22 ~ "Invalid argument".
    }

    int ret;
    for (int i = 0; i < beacon_db->count; i++) {
        ret = bt_addr_mac_compare(
                mac_little_endian,
                beacon_db->beacons[i].mac_little_endian);
        if (ret == 1) {
            // Get beacon.
            *beacon = beacon_db->beacons[i];
            return 0; // 0 ~ "Success".
        }
    }

    return -ENOENT; // -2 ~ "No such file or directory".
}