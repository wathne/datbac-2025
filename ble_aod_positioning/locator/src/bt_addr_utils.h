#ifndef BT_ADDR_UTILS_H
#define BT_ADDR_UTILS_H

#include <stdint.h> // For uint8_t.
#include <zephyr/bluetooth/addr.h> // For BT_ADDR_SIZE (6).

#ifndef BT_ADDR_SIZE
#define BT_ADDR_SIZE 6
#endif

// Compare Bluetooth MAC addresses.
// Returns 1 (1 ~ "True") if the MAC addresses are equal.
// Returns 0 (0 ~ "False") if the MAC addresses are not equal.
// Returns -EINVAL (-22 ~ "Invalid argument") if mac_1 pointer is NULL, or if
// mac_2 pointer is NULL.
int bt_addr_mac_compare(
        const uint8_t mac_1[BT_ADDR_SIZE],
        const uint8_t mac_2[BT_ADDR_SIZE]);

#endif // BT_ADDR_UTILS_H