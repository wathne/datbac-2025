#include "directional_statistics.h"
#include <math.h> // For cosf(), sinf(), atan2f(), sqrtf(), fabsf(), and M_PI (3.1415927f).

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

float directional_statistics_circular_mean(
        const float *angles,
        int angles_count,
        int max_intrinsic_iterations,
        float tolerance) {
    // The tolerance argument will be constrained by TOLERANCE_MININUM.
    // Tolerance defaults to TOLERANCE_MININUM if the tolerance argument is
    // equal to 0 or generally less than TOLERANCE_MININUM.
    static const float TOLERANCE_MININUM = 0.000001f;

    // Gracefully handle angles_count < 2.
    if (angles_count < 2) {
        if (angles_count == 1) {
            return angles[0];
        }
        return 0.0f;
    }

    // Calculate the extrinsic circular mean.
    // Minimizes Euclidean distances.
    //
    // atan2f((1/n)*S, (1/n)*C) is equivalent to atan2f(S, C), where n is the
    // angles_count, S is the sum of sin(phi), and C is the sum of cos(phi).
    // Note that the (1/n) terms are cancelled.
    float sum_cos_phi = 0.0f;
    float sum_sin_phi = 0.0f;
    for (int i = 0; i < angles_count; i++) {
        sum_cos_phi = sum_cos_phi + cosf(angles[i]);
        sum_sin_phi = sum_sin_phi + sinf(angles[i]);
    }
    float extrinsic_mean = atan2f(sum_sin_phi, sum_cos_phi);

    // Return the extrinsic circular mean if the max_intrinsic_iterations
    // argument is equal to 0 or generally less than 1.
    if (max_intrinsic_iterations < 1) {
        return extrinsic_mean;
    }

    // The tolerance argument is constrained by TOLERANCE_MININUM. Tolerance
    // defaults to TOLERANCE_MININUM if the tolerance argument is equal to 0 or
    // generally less than TOLERANCE_MININUM.
    if (tolerance < TOLERANCE_MININUM) {
        tolerance = TOLERANCE_MININUM;
    }

    // Set the initial intrinsic circular mean equal to the extrinsic circular
    // mean.
    float intrinsic_mean = extrinsic_mean;

    // Iteratively search for the intrinsic circular mean.
    // Iteratively minimize angular distances. Each iteration calculates the
    // angular distance between each provided angle and the current intrinsic
    // mean. The maximum number of iterations is constrained by the
    // max_intrinsic_iterations argument. Each iteration will check against the
    // tolerance for sufficient convergence. The iteration loop will exit
    // prematurely if sufficient convergence has been achieved.
    //
    // epsilon(i) = phi(i) - mu
    // ε(i) = φ(i) - μ
    //
    // Cosine is an even function:  cos(-x) =  cos(x) for all x in R
    // Sine is an odd function:     sin(-x) = -sin(x) for all x in R
    //
    // Cosine is symmetric about 0:
    // cos(0) = 1
    // cos(ε) < 1 for  0 < ε < π
    // cos(ε) < 1 for -π < ε < 0
    // This Cosine symmetry means that the sum of Cosines inherently measures
    // how clustered the angles are about the current intrinsic mean.
    // sum_cos_epsilon approaches angles_count for very clustered angles, with
    // each cos_epsilon approaching 1 near the current intrinsic mean. Consider
    // atan2f(sum_sin_epsilon, sum_cos_epsilon), the magnitude of
    // atan2f() is inversely related to the magnitude of sum_cos_epsilon.
    // Clustered angles will effectively dampen the magnitude of atan2f(), and
    // scattered angles will effectively boost the magnitude of atan2f().
    //
    // Sine changes sign about 0:
    // sin(0) = 0
    // sin(ε) > 0 for  0 < ε < π
    // sin(ε) < 0 for -π < ε < 0
    // The sign inherently provides direction for adjusting the next
    // intrinsic mean, either clockwise or counterclockwise. Positive and
    // negative angular distances are balanced about the current intrinsic mean
    // when sum_sin_epsilon approaches 0. This balance is also relevant for the
    // convergence check: |sum_sin_epsilon| < tolerance. The direction and 
    // magnitude of the sum of Sines effectively measures the imbalance about
    // the current intrinsic mean.
    //
    // atan2f((1/n)*S, (1/n)*C) is equivalent to atan2f(S, C), where n is the
    // angles_count, S is the sum of sin(phi), and C is the sum of cos(phi).
    // Note that the (1/n) terms are cancelled.
    float epsilon;
    float sum_cos_epsilon;
    float sum_sin_epsilon;
    float previous_intrinsic_mean = intrinsic_mean;
    for (int iteration = 0; iteration < max_intrinsic_iterations; iteration++) {
        sum_cos_epsilon = 0.0f;
        sum_sin_epsilon = 0.0f;

        for (int i = 0; i < angles_count; i++) {
            epsilon = angles[i] - intrinsic_mean;

            // Unwrap epsilon if absolute value is greater than pi, that is,
            // if epsilon is not within [-pi, pi]. This ensures the shortest
            // angular distance from the current intrinsic mean.
            if (epsilon > M_PI) {
                epsilon = epsilon - 2 * M_PI;
            }
            if (epsilon < -M_PI) {
                epsilon = epsilon + 2 * M_PI;
            }

            sum_cos_epsilon = sum_cos_epsilon + cosf(epsilon);
            sum_sin_epsilon = sum_sin_epsilon + sinf(epsilon);
        }

        // Set the current intrinsic circular mean.
        intrinsic_mean = intrinsic_mean + atan2f(
                sum_sin_epsilon,
                sum_cos_epsilon);

        // Unwrap intrinsic mean if absolute value is greater than pi, that is,
        // if intrinsic mean is not within [-pi, pi].
        if (intrinsic_mean > M_PI) {
            intrinsic_mean = intrinsic_mean - 2 * M_PI;
        }
        if (intrinsic_mean < -M_PI) {
            intrinsic_mean = intrinsic_mean + 2 * M_PI;
        }

        // Check against the tolerance for sufficient convergence.
        if (fabsf(sum_sin_epsilon) < tolerance) {
            return intrinsic_mean;
        }

        // Check against the tolerance for sufficient convergence.
        if (fabsf(intrinsic_mean - previous_intrinsic_mean) < tolerance) {
            return intrinsic_mean;
        }
        previous_intrinsic_mean = intrinsic_mean;
    }

    return intrinsic_mean;
}

float directional_statistics_circular_mean_extrinsic(
        const float *angles,
        int angles_count) {
    return directional_statistics_circular_mean(
            angles,
            angles_count,
            0,
            0);
}