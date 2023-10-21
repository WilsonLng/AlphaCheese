#!/usr/bin/env python3
import sys
import math

if len(sys.argv) != 3:
    print("Usage: python3 thetas.py xf yf")
else:
    xf = int(sys.argv[1])
    yf = int(sys.argv[2])

    l1 = 15.75
    l2 = 13.25
    l3 = 17.40
    
    k = xf ** 2 + yf ** 2 + l1 ** 2 - l2 ** 2 + 2 * yf * l3 + l3 ** 2
    n = math.sqrt((2 * xf * l1) ** 2 + (2 * yf * l1 + 2 * l1 * l3) ** 2)

    alphr = math.atan((2 * yf * l1 + 2 * l1 * l3) / (2 * xf * l1))
    alphd = math.degrees(alphr)

    thetas = []

    # theta11 = alphd + math.degrees(math.acos(k / n))
    theta11 = alphd + math.acos(k / n)
    # theta12 = alphd - math.degrees(math.acos(k / n))
    theta12 = 0

    theta21 = - theta11 + math.degrees(math.acos(((xf - l1 * math.cos(math.radians(theta11))) / l2)))
    # theta22 = - theta12 + math.degrees(math.acos(((xf - l1 * math.cos(math.radians(theta12))) / l2)))
    theta22 = 0

    theta31 = -90 - theta11 - theta21
    # theta32 = 270 - theta12 - theta22
    theta32 = 0

    thetas.extend([theta11, theta12, theta21, theta22, theta31, theta32])

    for i in range(len(thetas)):
        if thetas[i] < 0:
            thetas[i] = thetas[i] + 360
        elif thetas[i] > 360:
            thetas[i] = thetas[i] - 360

    for i in range(2, len(thetas)):
        thetas[i] = thetas[i] - 120

    thetas[0] = thetas[0] + 60
    thetas[1] = thetas[1] + 60
    thetas[2] = 300 - thetas[2]

    for i in range(0, len(thetas), 2):
        print("FP: ", end="")
        print(thetas[i], end=", ")

    print()

    for i in range(0, len(thetas), 2):
        print("FP: ", end="")
        print(thetas[i] * 1024 / 300, end=", ")

    print()

    for i in range(1, len(thetas), 2):
        print("SP: ", end="")
        print(thetas[i], end=", ")

    print()

    for i in range(1, len(thetas), 2):
        print("SP: ", end="")
        print(thetas[i] * 1024 / 300, end=", ")

    print()