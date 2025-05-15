#include <stdio.h>

/*
$ gcc -o calculate_antenna_positions calculate_antenna_positions.c
$ ./calculate_antenna_positions

const float antenna_positions_xyz[16][3] = {
    {-18.75f, -18.75f, 0.0f}, // antenna 0
    {-56.25f, -18.75f, 0.0f}, // antenna 1
    {-56.25f, -56.25f, 0.0f}, // antenna 2
    {-18.75f, -56.25f, 0.0f}, // antenna 3
    { 18.75f, -56.25f, 0.0f}, // antenna 4
    { 18.75f, -18.75f, 0.0f}, // antenna 5
    { 56.25f, -56.25f, 0.0f}, // antenna 6
    { 56.25f, -18.75f, 0.0f}, // antenna 7
    { 56.25f,  18.75f, 0.0f}, // antenna 8
    { 56.25f,  56.25f, 0.0f}, // antenna 9
    { 18.75f,  18.75f, 0.0f}, // antenna 10
    { 18.75f,  56.25f, 0.0f}, // antenna 11
    {-18.75f,  56.25f, 0.0f}, // antenna 12
    {-56.25f,  56.25f, 0.0f}, // antenna 13
    {-56.25f,  18.75f, 0.0f}, // antenna 14
    {-18.75f,  18.75f, 0.0f}  // antenna 15
};
*/

int main() {
    const float antenna_spacing_orthogonal = 37.5f;
    const float d = antenna_spacing_orthogonal;
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
    };

    printf("const float antenna_positions_xyz[16][3] = {\n");

    float x;
    float y;
    for (int i = 0; i < 15; i++) {
        x = antenna_positions_xyz[i][0];
        y = antenna_positions_xyz[i][1];
        printf("    {%6.2ff, %6.2ff, 0.0f}, // antenna %d\n",
                x, y, i);
    }
    x = antenna_positions_xyz[15][0];
    y = antenna_positions_xyz[15][1];
    printf("    {%6.2ff, %6.2ff, 0.0f}  // antenna %d\n",
            x, y, 15);

    printf("};\n");

    return 0;
}