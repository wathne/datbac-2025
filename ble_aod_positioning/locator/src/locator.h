#ifndef LOCATOR_H
#define LOCATOR_H

#include <stdint.h> // For uint8_t.
#include "beacon_database.h" // For beacon database structure.
#include "bt_addr_utils.h" // For BT_ADDR_SIZE (6).

#define LOCATOR_ERROR_PARALLEL_LINES 92 // An arbitrary error number.

#define LOCATOR_POSITION_CAPACITY 256

// Locator position structure.
// Global coordinates and error radius.
// TODO(wathne): Add more documentation.
struct locator_position {
    // Position in the global coordinate system relative to the global origin,
    // in meters.
    float x; // Global X coordinate.
    float y; // Global Y coordinate.
    float z; // Global Z coordinate.

    float error_radius;
};

// Locator structure.
// TODO(wathne): Add more documentation.
// See the locator_init() function.
struct locator {
    struct beacon_database *beacon_db;

    struct locator_position position_history[LOCATOR_POSITION_CAPACITY];
    int history_count;
    int history_next;
};

// The global locator instance.
// See the locator_init_global() function.
extern struct locator g_locator;

// Initialize a locator structure.
// Returns 0 (0 ~ "Success") if the locator structure is initialized.
// Returns -EINVAL (-22 ~ "Invalid argument") if locator pointer is NULL, or if
// beacon_db pointer is NULL.
int locator_init(
        struct locator *locator,
        struct beacon_database *beacon_db);

// Initialize the global locator instance g_locator.
// Returns 0 (0 ~ "Success") if the locator structure is initialized.
// Returns -EINVAL (-22 ~ "Invalid argument") if beacon_db pointer is NULL.
int locator_init_global(struct beacon_database *beacon_db);

// TODO(wathne): Add documentation.
int locator_estimate_position_from_skew_lines(
        struct locator *locator,
        const uint8_t beacon_1_mac_little_endian[BT_ADDR_SIZE],
        float beacon_1_local_direction_cosine_x,
        float beacon_1_local_direction_cosine_y,
        float beacon_1_local_direction_cosine_z,
        const uint8_t beacon_2_mac_little_endian[BT_ADDR_SIZE],
        float beacon_2_local_direction_cosine_x,
        float beacon_2_local_direction_cosine_y,
        float beacon_2_local_direction_cosine_z);

#endif // LOCATOR_H