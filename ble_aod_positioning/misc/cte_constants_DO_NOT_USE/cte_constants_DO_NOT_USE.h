#ifndef CTE_CONSTANTS_H
#define CTE_CONSTANTS_H

// DO NOT USE.
// BLE AoA/AoD calculations must be based on channel specific BLE carrier
// frequencies (~2.4 GHz), and not the CTE frequency (250 kHz).
// These CTE constants are kept here as a reference in case someone wants to
// include them and perform calculations despite this warning.

// Speed of light in meters per second.
// c = 299792458 meters/second.

// pi = 3.1415926535897932384626433832795

// CTE frequency in hertz.
extern const float cte_frequency;

// CTE wavelength in millimeters, meters, or kilometers.
// lambda = c/f, where c = 299792458 meters/second, and f is CTE frequency in
// hertz.

// CTE wavelength in millimeters.
extern const float cte_wavelength_mm;
// CTE wavelength in meters.
extern const float cte_wavelength_m;
// CTE wavelength in kilometers.
extern const float cte_wavelength_km;

// CTE wavenumber in radians per millimeter, meter, or kilometer.
// k = 2*pi/lambda, where pi = 3.1415926535897932384626433832795, and lambda is
// CTE wavelength in millimeters, meters, or kilometers.

// CTE wavenumber in radians per millimeter.
extern const float cte_wavenumber_rad_mm;
// CTE wavenumber in radians per meter.
extern const float cte_wavenumber_rad_m;
// CTE wavenumber in radians per kilometer.
extern const float cte_wavenumber_rad_km;

// CoreHW CHW1010-ANT2-1.1 antenna spacing for orthogonally adjacent antennas,
// from antenna center to antenna center, in radians, at the CTE frequency.
// CTE wavenumber multiplied by orthogonal antenna spacing.
// k * antenna_spacing_orthogonal, in radians.
extern const float cte_antenna_spacing_orthogonal_rad;
// CoreHW CHW1010-ANT2-1.1 antenna spacing for orthogonally adjacent antennas,
// from antenna center to antenna center, in milliradians, at the CTE frequency.
// CTE wavenumber multiplied by orthogonal antenna spacing.
// k * antenna_spacing_orthogonal, in milliradians.
extern const float cte_antenna_spacing_orthogonal_mrad;

// CoreHW CHW1010-ANT2-1.1 antenna spacing for diagonally adjacent antennas,
// from antenna center to antenna center, in radians, at the CTE frequency.
// CTE wavenumber multiplied by diagonal antenna spacing.
// k * antenna_spacing_diagonal, in radians.
extern const float cte_antenna_spacing_diagonal_rad;
// CoreHW CHW1010-ANT2-1.1 antenna spacing for diagonally adjacent antennas,
// from antenna center to antenna center, in milliradians, at the CTE frequency.
// CTE wavenumber multiplied by diagonal antenna spacing.
// k * antenna_spacing_diagonal, in milliradians.
extern const float cte_antenna_spacing_diagonal_mrad;

#endif // CTE_CONSTANTS_H