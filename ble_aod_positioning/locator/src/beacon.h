#ifndef BEACON_H
#define BEACON_H

#include <stdint.h> // For uint8_t.
#include "bt_addr_utils.h" // For BT_ADDR_SIZE (6).

// Manage relative position and orientation of beacons in a "global" coordinate
// system. Note that this "global" coordinate system is not yet positioned or
// oriented relative to the actual global geographic coordinate system (GCS) of
// the Earth. The actual global GCS latitude, longitude, height, and orientation
// must be managed in a separate data structure to anchor the "global"
// coordinate system itself to somewhere on the surface of the Earth.

// Beacon structure.
// MAC address, global coordinates, and global orientation.
// See the beacon_init() function.
struct beacon {
    // MAC address in big-endian format (conventional/human-readable order,
    // e.g., F6:66:CD:FD:DC:EB).
    uint8_t mac_big_endian[BT_ADDR_SIZE];

    // MAC address in little-endian format (protocol/reversed octet order,
    // e.g., EB:DC:FD:CD:66:F6).
    // "Multi-octet fields ... shall be transmitted with the least significant
    // octet first."
    // - Bluetooth Core Specification 5.4, Vol 6, Part B, Sections 1.2 - 1.3.
    // The Nordic Semiconductor BLE implementation follows the same reversed
    // octet ordering when storing BLE device addresses.
    uint8_t mac_little_endian[BT_ADDR_SIZE];

    // Position of the local origin (0, 0, 0) in the global coordinate system
    // relative to the global origin, in meters.
    float x; // Global X coordinate.
    float y; // Global Y coordinate.
    float z; // Global Z coordinate.

    // Global orientation as orthonormal basis vectors (i, j, k). These basis
    // vectors form the rotation matrix R from local coordinates to global
    // coordinates.
    //
    //     [ i_x  j_x  k_x ]
    // R = [ i_y  j_y  k_y ]
    //     [ i_z  j_z  k_z ]
    //
    // For transforming local direction cosines to global direction cosines.
    //
    // [ dx_global ]   [ i_x  j_x  k_x ] [ dx_local ]
    // [ dy_global ] = [ i_y  j_y  k_y ] [ dy_local ]
    // [ dz_global ]   [ i_z  j_z  k_z ] [ dz_local ]
    //
    float i_x, i_y, i_z; // Local X-axis (i) (rightward) in global coordinates.
    float j_x, j_y, j_z; // Local Y-axis (j) (upward) in global coordinates.
    float k_x, k_y, k_z; // Local Z-axis (k) (outward) in global coordinates.
};

// Initialize a beacon structure.
// Converts the MAC address from big-endian format to little-endian format, and
// stores both MAC address formats as mac_big_endian and mac_little_endian.
// TODO(wathne): Add more documentation. Meanwhile, see the documentation for
// the beacon structure, and see the documentation for the
// beacon_set_global_orientation() function.
// Returns 0 (0 ~ "Success") if the beacon structure is initialized.
// Returns -EINVAL (-22 ~ "Invalid argument") if beacon pointer is NULL, or if
// mac_big_endian pointer is NULL, or if yaw, pitch, or roll are out of range.
int beacon_init(
        struct beacon *beacon,
        const uint8_t mac_big_endian[BT_ADDR_SIZE],
        float global_x,
        float global_y,
        float global_z,
        float yaw,
        float pitch,
        float roll);

// Set global orientation for a beacon by converting Yaw, Pitch, and Roll to
// orthonormal basis vectors (i, j, k).
// Returns 0 (0 ~ "Success") if the global orientation is set.
// Returns -EINVAL (-22 ~ "Invalid argument") if beacon pointer is NULL, or if
// yaw, pitch, or roll are out of range.
//
// Tait–Bryan angles α, β, and γ, when applied in an intrinsic rotation sequence
// z-y'-x'', are known as Yaw, Pitch, and Roll. The equivalent extrinsic
// rotation sequence is x-y-z. These angles follow a right-hand rule. Point the
// right thumb along the positive direction of the axis of rotation, then curl
// the fingers in the direction of positive rotation.
// Yaw is a counterclockwise rotation of α about the Z-axis when viewed from the
// positive Z-axis toward the origin, range [-pi, pi].
// Pitch is a counterclockwise rotation of β about the Y-axis when viewed from
// the positive Y-axis toward the origin, range [-pi/2 , pi/2].
// Roll is a counterclockwise rotation of γ about the X-axis when viewed from
// the positive X-axis toward the origin, range [-pi, pi].
// An intrinsic rotation sequence z-y'-x'' is applied in the local coordinate
// system in a Yaw-Pitch-Roll order. First apply Yaw, then apply Pitch, and
// finally apply Roll. In other words, first rotate about the global Z-axis,
// then rotate about the new Y'-axis, and finally rotate about the new X''-axis.
// An extrinsic rotation sequence x-y-z is applied in the global coordinate
// system in a Roll-Pitch-Yaw order. First apply Roll, then apply Pitch, and
// finally apply Yaw. In other words, first rotate about the global X-axis, then
// rotate about the global Y-axis, and finally rotate about the global Z-axis.
// The extrinsic rotation sequence forms the rotation matrix R from local
// coordinates to global coordinates. Note that matrix multiplication is not
// commutative. Matrix multiplication is evaluated from right to left, similar
// to function compositions.
//
// R = Rz(α) Ry(β) Rx(γ)
//
//                [  cos(α) -sin(α)    0    ]
// Yaw:   Rz(α) = [  sin(α)  cos(α)    0    ]
//                [    0       0       1    ]
//
//                [  cos(β)    0     sin(β) ]
// Pitch: Ry(β) = [    0       1       0    ]
//                [ -sin(β)    0     cos(β) ]
//
//                [    1       0       0    ]
// Roll:  Rx(γ) = [    0     cos(γ) -sin(γ) ]
//                [    0     sin(γ)  cos(γ) ]
//
//             Yaw                 Pitch                 Roll
//     [ cos(α) -sin(α) 0 ] [  cos(β) 0 sin(β) ] [ 1   0       0    ]
// R = [ sin(α)  cos(α) 0 ] [    0    1   0    ] [ 0 cos(γ) -sin(γ) ]
//     [   0       0    1 ] [ -sin(β) 0 cos(β) ] [ 0 sin(γ)  cos(γ) ]
//
//     [ cos(α)*cos(β) cos(α)*sin(β)*sin(γ)-sin(α)*cos(γ) cos(α)*sin(β)*cos(γ)+sin(α)*sin(γ) ]
// R = [ sin(α)*cos(β) sin(α)*sin(β)*sin(γ)+cos(α)*cos(γ) sin(α)*sin(β)*cos(γ)-cos(α)*sin(γ) ]
//     [    -sin(β)              cos(β)*sin(γ)                      cos(β)*cos(γ)            ]
//
//     [ i_x  j_x  k_x ]
// R = [ i_y  j_y  k_y ]
//     [ i_z  j_z  k_z ]
//
// i_x = cos(α)*cos(β)
// i_y = sin(α)*cos(β)
// i_z = -sin(β)
//
// j_x = cos(α)*sin(β)*sin(γ) - sin(α)*cos(γ)
// j_y = sin(α)*sin(β)*sin(γ) + cos(α)*cos(γ)
// j_z = cos(β)*sin(γ)
//
// k_x = cos(α)*sin(β)*cos(γ) + sin(α)*sin(γ)
// k_y = sin(α)*sin(β)*cos(γ) - cos(α)*sin(γ)
// k_z = cos(β)*cos(γ)
int beacon_set_global_orientation(
        struct beacon *beacon,
        float yaw,
        float pitch,
        float roll);

// Get global direction cosines by transforming local direction cosines to
// global direction cosines. Input validation is intentionally omitted. The
// local direction cosines must form a normalized direction vector. The beacon
// argument must be a pointer to an initialized beacon structure.
// See the beacon_init() function.
// The set of direction cosines form a normalized direction vector D such that
// cos(a)^2 + cos(b)^2 + cos(c)^2 = 1.
// Direction cos(a) is the cosine of the angle a between D and the X-axis.
// Direction cos(b) is the cosine of the angle b between D and the Y-axis.
// Direction cos(c) is the cosine of the angle c between D and the Z-axis.
//
//     [ cos(a) ]
// D = [ cos(b) ]
//     [ cos(c) ]
//
// D_global = R D_local
//
// [ dx_global ]   [ i_x  j_x  k_x ] [ dx_local ]
// [ dy_global ] = [ i_y  j_y  k_y ] [ dy_local ]
// [ dz_global ]   [ i_z  j_z  k_z ] [ dz_local ]
//
// dx_global =
//         dx_local * i_x +
//         dy_local * j_x +
//         dz_local * k_x;
//
// dy_global =
//         dx_local * i_y +
//         dy_local * j_y +
//         dz_local * k_y;
//
// dz_global =
//         dx_local * i_z +
//         dy_local * j_z +
//         dz_local * k_z;
void beacon_get_global_direction_cosines(
        const struct beacon *beacon,
        float local_direction_cosine_x,
        float local_direction_cosine_y,
        float local_direction_cosine_z,
        float *global_direction_cosine_x,
        float *global_direction_cosine_y,
        float *global_direction_cosine_z);

#endif // BEACON_H