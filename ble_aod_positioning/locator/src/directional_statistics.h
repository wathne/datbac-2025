#ifndef DIRECTIONAL_STATISTICS_H
#define DIRECTIONAL_STATISTICS_H

// Search for the intrinsic circular mean of a set of angles (radians).
// An intrinsic circular mean minimizes angular distances.
// Returns a circular mean in radians [-pi, pi].
// Returns 0.0f if angles_count is 0.
// Returns angles[0] if angles_count is 1.
// Iteratively minimizes angular distances. Each iteration calculates the
// angular distance between each provided angle and the current intrinsic mean.
// The maximum number of iterations is constrained by the
// max_intrinsic_iterations argument. Each iteration will check against the
// tolerance for sufficient convergence. The iteration loop will exit
// prematurely if sufficient convergence has been achieved. Passing
// max_intrinsic_iterations = 0 to the function will make the function return
// the extrinsic circular mean, and the function will not search for the
// intrinsic circular mean. For computational efficiency it is recommended to
// pass a tolerance argument in the range [0.1, 0.01], and a
// max_intrinsic_iterations argument in the range [0, 5].
// Note that this function may descend onto a local minimum instead of the
// global minimum if the provided angles are very scattered. The function aims
// to be good enough, readable, and computationally efficient.
// TODO(wathne): Add more documentation.
// TODO(wathne): Refactor for more mathematical rigor.
float directional_statistics_circular_mean(
        const float *angles,
        int angles_count,
        int max_intrinsic_iterations,
        float tolerance);

// Calculate the extrinsic circular mean of a set of angles (radians).
// An extrinsic circular mean minimizes Euclidean distances.
// Returns a circular mean in radians [-pi, pi].
// Returns 0.0f if angles_count is 0.
// Returns angles[0] if angles_count is 1.
// This function is a wrapper function for the
// directional_statistics_circular_mean() function, with
// max_intrinsic_iterations set to 0. The extrinsic circular mean is more
// computationally efficient, but it may be less accurate than the intrinsic
// circular mean if the provided angles are scattered.
float directional_statistics_circular_mean_extrinsic(
        const float *angles,
        int angles_count);

#endif // DIRECTIONAL_STATISTICS_H