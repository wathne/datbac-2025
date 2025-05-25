#!/usr/bin/env python3

import os
import sys


def extract_test_coordinates(input_filename):
    output_filename = f"{input_filename}_coordinates.csv"

    coordinates = []
    x = None
    y = None
    z = None

    with open(input_filename, 'r') as input_file:
        for line in input_file:
            line = line.strip()
            if "X = " in line:
                x = float(line.split("X = ")[1])
            elif "Y = " in line:
                y = float(line.split("Y = ")[1])
            elif "Z = " in line:
                z = float(line.split("Z = ")[1])

            if (x is not None and y is not None and z is not None):
                coordinates.append((x, y, z))
                x = None
                y = None
                z = None

    with open(output_filename, 'w') as output_file:
        for x, y, z in coordinates:
            output_file.write(f"{x:.2f},{y:.2f},{z:.2f}\n")

    return None


def main():
    for filename in sys.argv[1:]:
        if not os.path.isfile(filename):
            print(f"Error: {filename} is not a file")
            continue

        extract_test_coordinates(filename)


if __name__ == "__main__":
    main()
