#ifndef BLE_CHANNEL_CONSTANTS_H
#define BLE_CHANNEL_CONSTANTS_H

#include <stdint.h> // For uint8_t and uint16_t.

// Speed of light in meters per second.
// c = 299792458 meters/second.

// pi = 3.1415926535897932384626433832795

// Lookup table (LUT) for BLE channel frequencies in MHz.
// BLE channels 0-36 are secondary advertising channels.
// BLE channels 37-39 are primary advertising channels.
extern const uint16_t ble_channel_frequencies[40];

// Lookup table (LUT) for BLE channel wavelengths in millimeters.
// lambda = c/f, where c = 299792458 meters/second, and f is BLE channel
// frequency in MHz.
// BLE channels 0-36 are secondary advertising channels.
// BLE channels 37-39 are primary advertising channels.
extern const float ble_channel_wavelengths[40];

// Lookup table (LUT) for BLE channel wavenumbers in radians per millimeter.
// k = 2*pi/lambda, where pi = 3.1415926535897932384626433832795, and lambda is
// BLE channel wavelength in millimeters.
// BLE channels 0-36 are secondary advertising channels.
// BLE channels 37-39 are primary advertising channels.
extern const float ble_channel_wavenumbers[40];

// Get BLE channel frequency in MHz.
// BLE channels 0-36 are secondary advertising channels.
// BLE channels 37-39 are primary advertising channels.
uint16_t ble_channel_get_frequency(uint8_t ble_channel_index);

// Get BLE channel wavelength in millimeters.
// lambda = c/f, where c = 299792458 meters/second, and f is BLE channel
// frequency in MHz.
// BLE channels 0-36 are secondary advertising channels.
// BLE channels 37-39 are primary advertising channels.
float ble_channel_get_wavelength(uint8_t ble_channel_index);

// Get BLE channel wavenumber in radians per millimeter.
// k = 2*pi/lambda, where pi = 3.1415926535897932384626433832795, and lambda is
// BLE channel wavelength in millimeters.
// BLE channels 0-36 are secondary advertising channels.
// BLE channels 37-39 are primary advertising channels.
float ble_channel_get_wavenumber(uint8_t ble_channel_index);

#endif // BLE_CHANNEL_CONSTANTS_H