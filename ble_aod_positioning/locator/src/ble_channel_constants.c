#include "ble_channel_constants.h"
#include <stdint.h> // For uint8_t and uint16_t.

const uint16_t ble_channel_frequencies[40] = {
    2404, 2406, 2408, 2410, 2412,
    2414, 2416, 2418, 2420, 2422,
    2424, 2428, 2430, 2432, 2434,
    2436, 2438, 2440, 2442, 2444,
    2446, 2448, 2450, 2452, 2454,
    2456, 2458, 2460, 2462, 2464,
    2466, 2468, 2470, 2472, 2474,
    2476, 2478, 2402, 2426, 2480
};

const float ble_channel_wavelengths[40] = {
    124.705681f, 124.602019f, 124.498529f, 124.395211f, 124.292064f,
    124.189088f, 124.086282f, 123.983647f, 123.881181f, 123.778884f,
    123.676757f, 123.473006f, 123.371382f, 123.269925f, 123.168635f,
    123.067511f, 122.966554f, 122.865761f, 122.765134f, 122.664672f,
    122.564374f, 122.464239f, 122.364269f, 122.264461f, 122.164816f,
    122.065333f, 121.966012f, 121.866853f, 121.767855f, 121.669017f,
    121.570340f, 121.471823f, 121.373465f, 121.275266f, 121.177226f,
    121.079345f, 120.981621f, 124.809516f, 123.574797f, 120.884056f
};

const float ble_channel_wavenumbers[40] = {
    0.050384f, 0.050426f, 0.050468f, 0.050510f, 0.050552f,
    0.050594f, 0.050636f, 0.050678f, 0.050719f, 0.050761f,
    0.050803f, 0.050887f, 0.050929f, 0.050971f, 0.051013f,
    0.051055f, 0.051097f, 0.051139f, 0.051181f, 0.051222f,
    0.051264f, 0.051306f, 0.051348f, 0.051390f, 0.051432f,
    0.051474f, 0.051516f, 0.051558f, 0.051600f, 0.051642f,
    0.051684f, 0.051725f, 0.051767f, 0.051809f, 0.051851f,
    0.051893f, 0.051935f, 0.050342f, 0.050845f, 0.051977f
};

uint16_t ble_channel_get_frequency(uint8_t ble_channel_index) {
    if (ble_channel_index < 40) {
        return ble_channel_frequencies[ble_channel_index];
    } else {
        // TODO(wathne): Gracefully default to BLE channel index 18 (2442 Mhz)?
        // return ble_channel_frequencies[18];
        return 0;
    }
}

float ble_channel_get_wavelength(uint8_t ble_channel_index) {
    if (ble_channel_index < 40) {
        return ble_channel_wavelengths[ble_channel_index];
    } else {
        // TODO(wathne): Gracefully default to BLE channel index 18 (2442 Mhz)?
        // return ble_channel_wavelengths[18];
        return 0.0f;
    }
}

float ble_channel_get_wavenumber(uint8_t ble_channel_index) {
    if (ble_channel_index < 40) {
        return ble_channel_wavenumbers[ble_channel_index];
    } else {
        // TODO(wathne): Gracefully default to BLE channel index 18 (2442 Mhz)?
        // return ble_channel_wavenumbers[18];
        return 0.0f;
    }
}