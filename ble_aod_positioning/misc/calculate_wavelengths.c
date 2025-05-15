#include <stdio.h>

/*
$ gcc -o calculate_wavelengths calculate_wavelengths.c
$ ./calculate_wavelengths

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
*/

int main() {
    const double c = 299792458.0;
    const double pi = 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679;

    const int ble_channel_frequencies[40] = {
        2404, 2406, 2408, 2410, 2412,
        2414, 2416, 2418, 2420, 2422,
        2424, 2428, 2430, 2432, 2434,
        2436, 2438, 2440, 2442, 2444,
        2446, 2448, 2450, 2452, 2454,
        2456, 2458, 2460, 2462, 2464,
        2466, 2468, 2470, 2472, 2474,
        2476, 2478, 2402, 2426, 2480
    };

    printf("const float ble_channel_wavelengths[40] = {\n    ");

    int count = 0;
    for (int i = 0; i < 40; i++) {
        double frequency_hz = ble_channel_frequencies[i] * 1000000.0;
        double wavelength_mm = (c / frequency_hz) * 1000.0;

        printf("%.6ff", wavelength_mm);

        if (i < 39) printf(", ");
        count++;

        if (count == 5 && i < 39) {
            printf("\n    ");
            count = 0;
        }
    }

    printf("\n};\n");

    return 0;
}