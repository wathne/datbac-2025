#!/usr/bin/env python3

import os
import sys
import matplotlib.pyplot as plt

# Test 1 ground truth: (3.30, 0, 9.06)
# Test 2 ground truth: (9.69, 0, 9.40)
# Test 3 ground truth: (-1.53, 0, 9.27)

test_count = 3
test_ground_truths = [(3.30, 0, 9.06), (9.69, 0, 9.40), (-1.53, 0, 9.27)]
test_colors = ['green', 'blue', 'red']
test_beacons = [(10, 0, 0), (0, 0, 0)]


def calculate_test_errors(input_filenames):
    for i, input_filename in enumerate(input_filenames[:test_count]):
        coordinates = []
        x = None
        y = None
        z = None

        with open(input_filename, 'r') as input_file:
            for line in input_file:
                words = line.split(',')
                if len(words) >= 3:
                    x = float(words[0])
                    y = float(words[1])
                    z = float(words[2])

                if (x is not None and y is not None and z is not None):
                    coordinates.append((x, y, z))
                    x = None
                    y = None
                    z = None

        count = len(coordinates)
        if count == 0:
            continue

        sum_x = 0
        sum_y = 0
        sum_z = 0
        for c in coordinates:
            sum_x = sum_x + c[0]
            sum_y = sum_y + c[1]
            sum_z = sum_z + c[2]

        mean_x = sum_x / count
        mean_y = sum_y / count
        mean_z = sum_z / count

        ground_truth = test_ground_truths[i]

        ground_truth_x = ground_truth[0]
        ground_truth_y = ground_truth[1]
        ground_truth_z = ground_truth[2]

        distance_x = mean_x - ground_truth_x
        distance_y = mean_y - ground_truth_y
        distance_z = mean_z - ground_truth_z

        error = (distance_x**2 + distance_y**2 + distance_z**2)**0.5

        print(f"Test {i+1}: error = {error:.3f} meters")

    return None


def plot_test_coordinates(input_filenames):
    output_filename = "plot.png"

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(projection='3d')

    for i, input_filename in enumerate(input_filenames[:test_count]):
        coordinates = []
        x = None
        y = None
        z = None

        with open(input_filename, 'r') as input_file:
            for line in input_file:
                words = line.split(',')
                if len(words) >= 3:
                    x = float(words[0])
                    y = float(words[1])
                    z = float(words[2])

                if (x is not None and y is not None and z is not None):
                    coordinates.append((x, y, z))
                    x = None
                    y = None
                    z = None

        ax.scatter(*zip(*coordinates), color=test_colors[i], alpha=0.6)

    #ax.scatter(*zip(*test_beacons), color='black', marker='s', s=100) #front
    #ax.scatter(*zip(*test_beacons), color='black', marker='_', s=100) #above
    ax.scatter(*zip(*test_beacons), color='black', s=100, alpha=1.0) #other

    ax.scatter(*zip(*test_ground_truths), color='black', s=25, alpha=1.0)

    ax.set_xlim(-20, 30)
    ax.set_ylim(-25, 25)
    ax.set_zlim(-5, 45)
    ax.set_box_aspect((50, 50, 50))

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    #ax.view_init(elev=90, azim=0, roll=90) #front
    #ax.view_init(elev=0, azim=90, roll=180) #above
    ax.view_init(elev=30, azim=60, roll=130) #other

    plt.tight_layout()

    plt.savefig(output_filename, dpi=600, bbox_inches='tight')

    plt.show()

    return None


def main():
    filenames = []

    for filename in sys.argv[1:]:
        if not os.path.isfile(filename):
            print(f"Error: {filename} is not a file")
            continue
        filenames.append(filename)

    calculate_test_errors(filenames)
    plot_test_coordinates(filenames)


if __name__ == "__main__":
    main()
