#include "bt_addr_utils.h" // For BT_ADDR_SIZE (6).
#include <errno.h> // For EINVAL (22).
#include <stddef.h> // For NULL ((void *)0).
#include <stdint.h> // For uint8_t.
#include <string.h> // For memcmp().

int bt_addr_mac_compare(
        const uint8_t mac_1[BT_ADDR_SIZE],
        const uint8_t mac_2[BT_ADDR_SIZE]) {
    if (mac_1 == NULL || mac_2 == NULL) {
        return -EINVAL; // -22 ~ "Invalid argument".
    }

    if (memcmp(mac_1, mac_2, BT_ADDR_SIZE) == 0) {
        return 1; // 1 ~ "True".
    }

    return 0; // 0 ~ "False".
}