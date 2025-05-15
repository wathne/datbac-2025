#ifndef BEACON_DATABASE_H
#define BEACON_DATABASE_H

#include <stdint.h> // For uint8_t.
#include "beacon.h" // For beacon structure.
#include "bt_addr_utils.h" // For BT_ADDR_SIZE (6).

#define BEACON_DATABASE_CAPACITY 16

// Beacon database structure.
// See the beacon_database_init() function.
struct beacon_database {
    struct beacon beacons[BEACON_DATABASE_CAPACITY];

    int count;
};

// The global beacon database instance.
// See the beacon_database_init_global() function.
extern struct beacon_database g_beacon_db;

// Initialize a beacon database structure (count = 0).
// Returns 0 (0 ~ "Success") if the beacon database structure is initialized.
// Returns -EINVAL (-22 ~ "Invalid argument") if beacon_db pointer is NULL.
int beacon_database_init(struct beacon_database *beacon_db);

// Initialize the global beacon database instance g_beacon_db (count = 0).
// Returns 0 (0 ~ "Success") if the beacon database structure is initialized.
int beacon_database_init_global();

// Update or add a beacon, to a beacon database.
// Returns 0 (0 ~ "Success") if a beacon in the database is updated, or if the
// beacon is added to the database.
// Returns -ENOSPC (-28 ~ "No space left on device") if the database is full.
// Returns -EINVAL (-22 ~ "Invalid argument") if beacon_db pointer is NULL, or
// if beacon pointer is NULL.
int beacon_database_put(
        struct beacon_database *beacon_db,
        const struct beacon *beacon);

// Get a beacon by MAC address, from a beacon database.
// Uses little-endian MAC address format (protocol/reversed octet order) for
// beacon lookup. This allows direct use of MAC addresses as received from the
// BLE controller.
// Returns 0 (0 ~ "Success") if a beacon is found.
// Returns -ENOENT (-2 ~ "No such file or directory") if no beacon is found.
// Returns -EINVAL (-22 ~ "Invalid argument") if beacon_db pointer is NULL, or
// if beacon pointer is NULL, or if mac_little_endian pointer is NULL.
int beacon_database_get(
        struct beacon_database *beacon_db,
        struct beacon *beacon,
        const uint8_t mac_little_endian[BT_ADDR_SIZE]);

#endif // BEACON_DATABASE_H