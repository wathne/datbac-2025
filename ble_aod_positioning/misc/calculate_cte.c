#include <stdio.h>

/*
$ gcc -o calculate_cte calculate_cte.c
$ ./calculate_cte

const float cte_frequency = 250000.0f;
const float cte_wavelength_mm = 1199169.832000f;
const float cte_wavelength_m = 1199.169832f;
const float cte_wavelength_km = 1.199170f;
const float cte_wavenumber_rad_mm = 0.000005f;
const float cte_wavenumber_rad_m = 0.005240f;
const float cte_wavenumber_rad_km = 5.239613f;
const float cte_antenna_spacing_orthogonal_rad = 0.000196f;
const float cte_antenna_spacing_orthogonal_mrad = 0.196485f;
const float cte_antenna_spacing_diagonal_rad = 0.000278f;
const float cte_antenna_spacing_diagonal_mrad = 0.277872f;
*/

int main() {
    const double c = 299792458.0;
    const double pi = 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679;
    const double frequency_hz = 250000.0;

    const float antenna_spacing_orthogonal = 37.5f;
    const float d = antenna_spacing_orthogonal;
    const float antenna_spacing_diagonal = 53.033009f;
    const float d_diag = antenna_spacing_diagonal;

    double wavelength_mm = (c / frequency_hz) * 1000.0;
    double wavelength_m = (c / frequency_hz);
    double wavelength_km = (c / frequency_hz) / 1000.0;
    double wavenumber_rad_mm = 2.0 * pi / wavelength_mm;
    double wavenumber_rad_m = 2.0 * pi / wavelength_m;
    double wavenumber_rad_km = 2.0 * pi / wavelength_km;
    double k_d_orth_rad = wavenumber_rad_mm * d;
    double k_d_orth_mrad = wavenumber_rad_m * d;
    double k_d_diag_rad = wavenumber_rad_mm * d_diag;
    double k_d_diag_mrad = wavenumber_rad_m * d_diag;

    printf("const float cte_frequency = 250000.0f;\n");
    printf("const float cte_wavelength_mm = %.6ff;\n", wavelength_mm);
    printf("const float cte_wavelength_m = %.6ff;\n", wavelength_m);
    printf("const float cte_wavelength_km = %.6ff;\n", wavelength_km);
    printf("const float cte_wavenumber_rad_mm = %.6ff;\n", wavenumber_rad_mm);
    printf("const float cte_wavenumber_rad_m = %.6ff;\n", wavenumber_rad_m);
    printf("const float cte_wavenumber_rad_km = %.6ff;\n", wavenumber_rad_km);
    printf("const float cte_antenna_spacing_orthogonal_rad = %.6ff;\n", k_d_orth_rad);
    printf("const float cte_antenna_spacing_orthogonal_mrad = %.6ff;\n", k_d_orth_mrad);
    printf("const float cte_antenna_spacing_diagonal_rad = %.6ff;\n", k_d_diag_rad);
    printf("const float cte_antenna_spacing_diagonal_mrad = %.6ff;\n", k_d_diag_mrad);

    return 0;
}