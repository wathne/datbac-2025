#ifndef CHW1010_ANT2_SPECS_H
#define CHW1010_ANT2_SPECS_H

// CoreHW CHW1010-ANT2-1.1 antenna grid:
//  +----+----+----+----+
//  | 13 | 12 | 11 |  9 |
//  +----+----+----+----+
//  | 14 | 15 | 10 |  8 |
//  +----+----+----+----+
//  |  1 |  0 |  5 |  7 |
//  +----+----+----+----+
//  |  2 |  3 |  4 |  6 |
//  +----+----+----+----+

// CoreHW CHW1010-ANT2-1.1 antenna coordinate system:
//            Y
//            |
//    13   12 | 11    9
//            |
//    14   15 | 10    8
//            +------------ X
//     1    0    5    7
//
//     2    3    4    6
//
// Right-handed Cartesian (x, y, z) coordinate system.
// X-axis: Points rightward when facing the array.
// Y-axis: Points upward when facing the array.
// Z-axis: Points outward from the array toward the locator.
// Origin (0, 0, 0) is at the center of the array.
// 
// Azimuth is the angle in the XZ-plane with respect to the Z-axis.
//   - Positive when the locator is to the right of the array (x > 0).
//   - Negative when the locator is to the left of the array (x < 0).
//   - Range is [-pi, pi].
// Elevation is the angle from the XZ-plane toward the Y-axis.
//   - Positive when the locator is above the XZ-plane (y > 0).
//   - Negative when the locator is below the XZ-plane (y < 0).
//   - Range is [-pi/2, pi/2].
//
// The above definition for Azimuth and Elevation is intuitive and
// conventional, but note that many BLE direction finding references may use
// an alternative convention:
// (Alt.) Azimuth is the angle in the XY-plane with respect to the X-axis.
// (Alt.) Elevation is the angle from the XY-plane toward the Z-axis.

// CoreHW CHW1010-ANT2-1.1 antenna spacing for orthogonally adjacent antennas,
// from antenna center to antenna center, in millimeters.
extern const float antenna_spacing_orthogonal;

// CoreHW CHW1010-ANT2-1.1 antenna spacing for diagonally adjacent antennas,
// from antenna center to antenna center, in millimeters.
extern const float antenna_spacing_diagonal;

// CoreHW CHW1010-ANT2-1.1 antenna center positions (x, y, z) in millimeters.
extern const float antenna_positions_xyz[16][3];

// CoreHW CHW1010-ANT2-1.1 antenna center positions (x, y) in millimeters,
// z = 0.
extern const float antenna_positions_xy[16][2];

// CoreHW CHW1010-ANT2-1.1 antenna center positions x-coordinate in millimeters,
// z = 0.
extern const float antenna_positions_x[16];

// CoreHW CHW1010-ANT2-1.1 antenna center positions y-coordinate in millimeters,
// z = 0.
extern const float antenna_positions_y[16];

#endif // CHW1010_ANT2_SPECS_H