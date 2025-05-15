#include "chw1010_ant2_specs.h"

const float antenna_spacing_orthogonal = 37.5f;

const float antenna_spacing_diagonal = 53.033009f;

/*static const float d = antenna_spacing_orthogonal;
const float antenna_positions_xyz[16][3] = {
    {-0.5f*d, -0.5f*d, 0.0f}, // antenna  0, in bottom left quadrant.
    {-1.5f*d, -0.5f*d, 0.0f}, // antenna  1, in bottom left quadrant.
    {-1.5f*d, -1.5f*d, 0.0f}, // antenna  2, in bottom left quadrant.
    {-0.5f*d, -1.5f*d, 0.0f}, // antenna  3, in bottom left quadrant.
    { 0.5f*d, -1.5f*d, 0.0f}, // antenna  4, in bottom right quadrant.
    { 0.5f*d, -0.5f*d, 0.0f}, // antenna  5, in bottom right quadrant.
    { 1.5f*d, -1.5f*d, 0.0f}, // antenna  6, in bottom right quadrant.
    { 1.5f*d, -0.5f*d, 0.0f}, // antenna  7, in bottom right quadrant.
    { 1.5f*d,  0.5f*d, 0.0f}, // antenna  8, in top right quadrant.
    { 1.5f*d,  1.5f*d, 0.0f}, // antenna  9, in top right quadrant.
    { 0.5f*d,  0.5f*d, 0.0f}, // antenna 10, in top right quadrant.
    { 0.5f*d,  1.5f*d, 0.0f}, // antenna 11, in top right quadrant.
    {-0.5f*d,  1.5f*d, 0.0f}, // antenna 12, in top left quadrant.
    {-1.5f*d,  1.5f*d, 0.0f}, // antenna 13, in top left quadrant.
    {-1.5f*d,  0.5f*d, 0.0f}, // antenna 14, in top left quadrant.
    {-0.5f*d,  0.5f*d, 0.0f}  // antenna 15, in top left quadrant.
};*/

const float antenna_positions_xyz[16][3] = {
    {-18.75f, -18.75f, 0.0f}, // antenna  0, in bottom left quadrant.
    {-56.25f, -18.75f, 0.0f}, // antenna  1, in bottom left quadrant.
    {-56.25f, -56.25f, 0.0f}, // antenna  2, in bottom left quadrant.
    {-18.75f, -56.25f, 0.0f}, // antenna  3, in bottom left quadrant.
    { 18.75f, -56.25f, 0.0f}, // antenna  4, in bottom right quadrant.
    { 18.75f, -18.75f, 0.0f}, // antenna  5, in bottom right quadrant.
    { 56.25f, -56.25f, 0.0f}, // antenna  6, in bottom right quadrant.
    { 56.25f, -18.75f, 0.0f}, // antenna  7, in bottom right quadrant.
    { 56.25f,  18.75f, 0.0f}, // antenna  8, in top right quadrant.
    { 56.25f,  56.25f, 0.0f}, // antenna  9, in top right quadrant.
    { 18.75f,  18.75f, 0.0f}, // antenna 10, in top right quadrant.
    { 18.75f,  56.25f, 0.0f}, // antenna 11, in top right quadrant.
    {-18.75f,  56.25f, 0.0f}, // antenna 12, in top left quadrant.
    {-56.25f,  56.25f, 0.0f}, // antenna 13, in top left quadrant.
    {-56.25f,  18.75f, 0.0f}, // antenna 14, in top left quadrant.
    {-18.75f,  18.75f, 0.0f}  // antenna 15, in top left quadrant.
};

const float antenna_positions_xy[16][2] = {
    {-18.75f, -18.75f}, // antenna  0, in bottom left quadrant.
    {-56.25f, -18.75f}, // antenna  1, in bottom left quadrant.
    {-56.25f, -56.25f}, // antenna  2, in bottom left quadrant.
    {-18.75f, -56.25f}, // antenna  3, in bottom left quadrant.
    { 18.75f, -56.25f}, // antenna  4, in bottom right quadrant.
    { 18.75f, -18.75f}, // antenna  5, in bottom right quadrant.
    { 56.25f, -56.25f}, // antenna  6, in bottom right quadrant.
    { 56.25f, -18.75f}, // antenna  7, in bottom right quadrant.
    { 56.25f,  18.75f}, // antenna  8, in top right quadrant.
    { 56.25f,  56.25f}, // antenna  9, in top right quadrant.
    { 18.75f,  18.75f}, // antenna 10, in top right quadrant.
    { 18.75f,  56.25f}, // antenna 11, in top right quadrant.
    {-18.75f,  56.25f}, // antenna 12, in top left quadrant.
    {-56.25f,  56.25f}, // antenna 13, in top left quadrant.
    {-56.25f,  18.75f}, // antenna 14, in top left quadrant.
    {-18.75f,  18.75f}  // antenna 15, in top left quadrant.
};

const float antenna_positions_x[16] = {
    -18.75f, // antenna  0, in bottom left quadrant.
    -56.25f, // antenna  1, in bottom left quadrant.
    -56.25f, // antenna  2, in bottom left quadrant.
    -18.75f, // antenna  3, in bottom left quadrant.
     18.75f, // antenna  4, in bottom right quadrant.
     18.75f, // antenna  5, in bottom right quadrant.
     56.25f, // antenna  6, in bottom right quadrant.
     56.25f, // antenna  7, in bottom right quadrant.
     56.25f, // antenna  8, in top right quadrant.
     56.25f, // antenna  9, in top right quadrant.
     18.75f, // antenna 10, in top right quadrant.
     18.75f, // antenna 11, in top right quadrant.
    -18.75f, // antenna 12, in top left quadrant.
    -56.25f, // antenna 13, in top left quadrant.
    -56.25f, // antenna 14, in top left quadrant.
    -18.75f  // antenna 15, in top left quadrant.
};

const float antenna_positions_y[16] = {
    -18.75f, // antenna  0, in bottom left quadrant.
    -18.75f, // antenna  1, in bottom left quadrant.
    -56.25f, // antenna  2, in bottom left quadrant.
    -56.25f, // antenna  3, in bottom left quadrant.
    -56.25f, // antenna  4, in bottom right quadrant.
    -18.75f, // antenna  5, in bottom right quadrant.
    -56.25f, // antenna  6, in bottom right quadrant.
    -18.75f, // antenna  7, in bottom right quadrant.
     18.75f, // antenna  8, in top right quadrant.
     56.25f, // antenna  9, in top right quadrant.
     18.75f, // antenna 10, in top right quadrant.
     56.25f, // antenna 11, in top right quadrant.
     56.25f, // antenna 12, in top left quadrant.
     56.25f, // antenna 13, in top left quadrant.
     18.75f, // antenna 14, in top left quadrant.
     18.75f  // antenna 15, in top left quadrant.
};