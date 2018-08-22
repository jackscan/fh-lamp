import math

gamma = 2.6
for x in range(256):
    y = int(math.pow(x / 255.0, gamma) * 255.0 * 4.0 + 0.5)
    print("{:3},".format(y)),

