#include <stdio.h>

/*
$ gcc -o calculate_wavenumbers calculate_wavenumbers.c
$ ./calculate_wavenumbers

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

    printf("const float ble_channel_wavenumbers[40] = {\n    ");

    int count = 0;
    for (int i = 0; i < 40; i++) {
        double frequency_hz = ble_channel_frequencies[i] * 1000000.0;
        double wavelength_mm = (c / frequency_hz) * 1000.0;
        double wavenumber = 2.0 * pi / wavelength_mm;

        printf("%.6ff", wavenumber);

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