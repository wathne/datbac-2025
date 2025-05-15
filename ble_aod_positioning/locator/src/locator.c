#include "locator.h" // For locator structure, locator position structure, LOCATOR_ERROR_PARALLEL_LINES (92), and LOCATOR_POSITION_CAPACITY.
#include <errno.h> // For EINVAL (22).
#include <math.h> // For fabsf() and sqrtf().
#include <stddef.h> // For NULL ((void *)0).
#include <stdint.h> // For uint8_t.
#include <zephyr/sys/printk.h> // For printk().
#include "beacon.h" // For beacon structure and beacon_get_global_direction_cosines().
#include "beacon_database.h" // For beacon database structure and beacon_database_get().
#include "bt_addr_utils.h" // For BT_ADDR_SIZE (6).

// The global locator instance.
// See the locator_init_global() function.
struct locator g_locator;

int locator_init(
        struct locator *locator,
        struct beacon_database *beacon_db) {
    if (locator == NULL || beacon_db == NULL) {
        return -EINVAL; // -22 ~ "Invalid argument".
    }

    locator->beacon_db = beacon_db;

    locator->history_count = 0;
    locator->history_next = 0;

    return 0;
}

int locator_init_global(struct beacon_database *beacon_db) {
    return locator_init(&g_locator, beacon_db);
}

int locator_estimate_position_from_skew_lines(
        struct locator *locator,
        const uint8_t beacon_1_mac_little_endian[BT_ADDR_SIZE],
        float beacon_1_local_direction_cosine_x,
        float beacon_1_local_direction_cosine_y,
        float beacon_1_local_direction_cosine_z,
        const uint8_t beacon_2_mac_little_endian[BT_ADDR_SIZE],
        float beacon_2_local_direction_cosine_x,
        float beacon_2_local_direction_cosine_y,
        float beacon_2_local_direction_cosine_z) {
    if (locator == NULL) {
        printk("DEBUG: locator is NULL\n");
        return -EINVAL; // -22 ~ "Invalid argument".
    }

    if (locator->beacon_db == NULL) {
        printk("DEBUG: locator->beacon_db is NULL\n");
        return -EINVAL; // -22 ~ "Invalid argument".
    }

    int ret;

    struct beacon beacon_1;
    struct beacon beacon_2;
    ret = beacon_database_get(
            locator->beacon_db,
            &beacon_1,
            beacon_1_mac_little_endian);
    if (ret != 0) {
        printk("DEBUG: first beacon is not in database\n");
        return ret;
    }
    ret = beacon_database_get(
            locator->beacon_db,
            &beacon_2,
            beacon_2_mac_little_endian);
    if (ret != 0) {
        printk("DEBUG: second beacon is not in database\n");
        return ret;
    }

    // Global position of first beacon.
    float p1x = beacon_1.x;
    float p1y = beacon_1.y;
    float p1z = beacon_1.z;

    // Global position of second beacon.
    float p2x = beacon_2.x;
    float p2y = beacon_2.y;
    float p2z = beacon_2.z;

    // Global direction cosines from first beacon.
    float d1x;
    float d1y;
    float d1z;

    // Global direction cosines from second beacon.
    float d2x;
    float d2y;
    float d2z;

    beacon_get_global_direction_cosines(
            &beacon_1,
            beacon_1_local_direction_cosine_x,
            beacon_1_local_direction_cosine_y,
            beacon_1_local_direction_cosine_z,
            &d1x,
            &d1y,
            &d1z);

    beacon_get_global_direction_cosines(
            &beacon_2,
            beacon_2_local_direction_cosine_x,
            beacon_2_local_direction_cosine_y,
            beacon_2_local_direction_cosine_z,
            &d2x,
            &d2y,
            &d2z);

    // Global line parameters.
    float t1;
    float t2;

    // A global position vector P and a global direction vector D, when
    // parameterized by t, form a global line L(t).
    //
    //          [ p1x + t1*d1x ]
    // L1(t1) = [ p1y + t1*d1y ]
    //          [ p1z + t1*d1z ]
    //
    //          [ p2x + t2*d2x ]
    // L2(t2) = [ p2y + t2*d2y ]
    //          [ p2z + t2*d2z ]
    //
    // l1x(t1) = p1x + t1*d1x
    // l1y(t1) = p1y + t1*d1y
    // l1z(t1) = p1z + t1*d1z
    //
    // l2x(t2) = p2x + t2*d2x
    // l2y(t2) = p2y + t2*d2y
    // l2z(t2) = p2z + t2*d2z
    //
    // The locator position would ideally be at some point where the two lines
    // intersect. The problem is that there is an additional degree of freedom
    // in 3-dimensional space. A perfect intersection will basically never
    // be the case. The immediate solution is to find the shortest line
    // between the two lines. The midpoint of this new line is then taken as
    // the most likely position for the locator.
    //
    // The lines from both beacons are connected by a vector C.
    // C = L2(t2) - L1(t1) = (P2 + t2*D2) - (P1 + t1*D1)
    // C = (P2 - P1) + t2*D2 - t1*D1
    //
    // Find t1 and t2 such that the distance |L1(t1) - L2(t2)| is minimized. A
    // key insight is that the shortest line between L1 and L2 must always be
    // perpendicular to both L1 and L2.
    // C dot D1 = 0    (1)
    // C dot D2 = 0    (2)
    //
    // Substitute for C.
    // ((P2 - P1) + t2*D2 - t1*D1) dot D1 = 0    (1)
    // ((P2 - P1) + t2*D2 - t1*D1) dot D2 = 0    (2)
    //
    // Define V21.
    // V21 = (P2 - P1)
    //
    // Substitute for (P2 - P1).
    // (V21 + t2*D2 - t1*D1) dot D1 = 0    (1)
    // (V21 + t2*D2 - t1*D1) dot D2 = 0    (2)
    //
    // D1_dot_V21 + t2 * D1_dot_D2 - t1 * D1_dot_D1 = 0    (1)
    // D2_dot_V21 + t2 * D2_dot_D2 - t1 * D1_dot_D2 = 0    (2)
    //
    // Note that D1 and D2 are unit vectors of length 1 since they are formed
    // by normalized direction cosines.
    // D1 dot D1 = 1
    // D2 dot D2 = 1
    //
    // D1_dot_V21 + t2 * D1_dot_D2 - t1 = 0    (1)
    // D2_dot_V21 + t2 - t1 * D1_dot_D2 = 0    (2)
    //
    // t1 =  D1_dot_V21 + t2 * D1_dot_D2    (1)
    // t2 = -D2_dot_V21 + t1 * D1_dot_D2    (2)
    //
    // t1 =  D1_dot_V21 + (-D2_dot_V21 + t1 * D1_dot_D2) * D1_dot_D2    (2->1)
    // t2 = -D2_dot_V21 + ( D1_dot_V21 + t2 * D1_dot_D2) * D1_dot_D2    (1->2)
    //
    // t1 = (D1_dot_V21 - D2_dot_V21 * D1_dot_D2) / (1 - D1_dot_D2 * D1_dot_D2)
    // t2 = (D1_dot_V21 * D1_dot_D2 - D2_dot_V21) / (1 - D1_dot_D2 * D1_dot_D2)
    //
    // numerator1 = (D1_dot_V21 - D2_dot_V21 * D1_dot_D2)
    // numerator2 = (D1_dot_V21 * D1_dot_D2 - D2_dot_V21)
    // denominator = (1 - D1_dot_D2 * D1_dot_D2)
    //
    // D1_dot_D2 is the Cosine of the angle between the two direction vectors.
    // D1_dot_V21 is the scalar component of (P2 - P1) in the direction of D1.
    // D2_dot_V21 is the scalar component of (P2 - P1) in the direction of D2.

    // D1 dot D2.
    float D1_dot_D2 =
            d1x * d2x +
            d1y * d2y +
            d1z * d2z;

    // V21 = (P2 - P1).
    float v21x = p2x - p1x;
    float v21y = p2y - p1y;
    float v21z = p2z - p1z;

    // D1 dot (P2 - P1), or D1 dot V21.
    float D1_dot_V21 =
            d1x * v21x +
            d1y * v21y +
            d1z * v21z;

    // D2 dot (P2 - P1), or D2 dot V21.
    float D2_dot_V21 =
            d2x * v21x +
            d2y * v21y +
            d2z * v21z;

    float denominator = 1.0f - D1_dot_D2 * D1_dot_D2;

    // Reject skew lines that are too parallel.
    // The denominator (1 - (D1_dot_D2)^2) approaches 0 when the skew lines are
    // parallel. Positional calculations become unstable. For now, let
    // |denominator| < 0.001 be the benchmark for parallelity.
    if (fabsf(denominator) < 0.001f) {
        return -LOCATOR_ERROR_PARALLEL_LINES; // -92 ~ "Parallel lines".
    }

    float numerator1 = D1_dot_V21 - D2_dot_V21 * D1_dot_D2;
    float numerator2 = D1_dot_V21 * D1_dot_D2 - D2_dot_V21;

    t1 = numerator1 / denominator;
    t2 = numerator2 / denominator;

    // Closest point Q1 on L1.
    float q1x = p1x + t1*d1x;
    float q1y = p1y + t1*d1y;
    float q1z = p1z + t1*d1z;

    // Closest point Q2 on L2.
    float q2x = p2x + t2*d2x;
    float q2y = p2y + t2*d2y;
    float q2z = p2z + t2*d2z;

    // Midpoint of Q1 and Q2.
    float midpoint_x = (q1x + q2x) / 2.0f;
    float midpoint_y = (q1y + q2y) / 2.0f;
    float midpoint_z = (q1z + q2z) / 2.0f;

    // Distance from Q1 to Q2.
    float distance_x = q2x - q1x;
    float distance_y = q2y - q1y;
    float distance_z = q2z - q1z;
    float distance_absolute = sqrtf(
            distance_x * distance_x +
            distance_y * distance_y +
            distance_z * distance_z);

    struct locator_position position;
    position.x = midpoint_x;
    position.y = midpoint_y;
    position.z = midpoint_z;
    position.error_radius = distance_absolute / 2.0f;

    // TODO(wathne): Move this ring buffer logic into a proper put function.
    struct locator_position *next_position =
            &locator->position_history[locator->history_next];
    *next_position = position;
    locator->history_next =
            (locator->history_next + 1) % LOCATOR_POSITION_CAPACITY;
    if (locator->history_count < LOCATOR_POSITION_CAPACITY) {
        locator->history_count++;
    }

    printk("X = %.2f\nY = %.2f\nZ = %.2f\n",
            position.x, position.y, position.z);

    return 0;
}