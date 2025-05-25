#!/usr/bin/env python3


# This function uses a very specialized trilateration algorithm in the XZ-plane
# where y=0. beacon 1 has position (x=10, y=0, z=0) and beacon 3 has position
# (x=0, y=0, z=0). This function can only be used for these exact beacon
# positions. This is not a general trilateration algorithm.
def calculate_test_ground_truth(distance_from_beacon_1, distance_from_beacon_3):
    d1 = distance_from_beacon_1
    d3 = distance_from_beacon_3

    # d1² = (x - b1x)² + (y - b1y)² + (z - b1z)²
    # d3² = (x - b3x)² + (y - b3y)² + (z - b3z)²
    #
    # y = 0
    # ~> d1² = (x - b1x)² + (z - b1z)²
    # ~> d3² = (x - b3x)² + (z - b3z)²
    #
    # b1z = 0, b3z = 0
    # ~> d1² = (x - b1x)² + z²
    # ~> d3² = (x - b3x)² + z²
    #
    # b1x = 10, b3x = 0
    # ~> d1² = (x - 10)² + z²
    # ~> d3² = x² + z²
    #
    # d1² = x² - 20x + 100 + z²
    # d3² = x² + z²
    #
    # d1² = (x² + z²) - 20x + 100
    # d3² = (x² + z²)
    #
    # d1² = d3² - 20x + 100
    #
    # 20x = d3² - d1² + 100
    #
    # x = (d3² - d1² + 100) / 20
    #
    # z² = d3² - x²
    #
    # Let z = +sqrt(d3² - x²)

    x = (d3**2 - d1**2 + 100) / 20

    z_squared = d3**2 - x**2
    if z_squared < 0:
        z_squared = 0
    z = z_squared**0.5

    return (x, z)


def main():
    # Beacon 1 position: (x=10, y=0, z=0) in meters.
    # Beacon 3 position: (x=0, y=0, z=0) in meters.

    # Test 1 ground truth:
    # 11.27 meters distance from beacon 1 (10, 0, 0),
    #  9.64 meters distance from beacon 3 ( 0, 0, 0).
    test_1_gt = calculate_test_ground_truth(11.27, 9.64)

    # Test 2 ground truth:
    #  9.4 meters distance from beacon 1 (10, 0, 0),
    # 13.5 meters distance from beacon 3 ( 0, 0, 0).
    test_2_gt = calculate_test_ground_truth(9.4, 13.5)

    # Test 3 ground truth:
    # 14.8 meters distance from beacon 1 (10, 0, 0),
    #  9.4 meters distance from beacon 3 ( 0, 0, 0).
    test_3_gt = calculate_test_ground_truth(14.8, 9.4)

    # Test 1 ground truth: (3.30, 0, 9.06)
    print(f"Test 1 ground truth: ({test_1_gt[0]:.2f}, 0, {test_1_gt[1]:.2f})")
    # Test 2 ground truth: (9.69, 0, 9.40)
    print(f"Test 2 ground truth: ({test_2_gt[0]:.2f}, 0, {test_2_gt[1]:.2f})")
    # Test 3 ground truth: (-1.53, 0, 9.27)
    print(f"Test 3 ground truth: ({test_3_gt[0]:.2f}, 0, {test_3_gt[1]:.2f})")


if __name__ == "__main__":
    main()
