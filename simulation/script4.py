import math
import numpy as np

PI = 3.141592653589

segment_length = 1

def convert(angle, radians=True):
    if radians:
        angle = angle * PI / 180
    else:
        angle = angle * 180 / PI
    return angle

def dist(p1, p2 = [0, 0, 0]):
    if (len(p1) == 2):
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
    else:
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 + (p1[2] - p2[2]) ** 2)

def constrain(x, min = -PI / 2, max = PI / 2):
    if x < min:
        return min
    elif x > max:
        return max
    else:
        return x
    
def fix_angle(x):
    if x > PI:
        return x - 2 * PI
    elif x < -PI:
        return x + 2 * PI
    else:
        return x

def parameterized_intersection(point, orientation):
    A = dist(point[:2])
    d = dist(point)
    R = math.sqrt(segment_length ** 2 - 0.25 * d ** 2)
    V = [0, 0, 0]
    
    if A == 0:
        V = [1, 0, 0]
    else:
        V = [1 / A * -point[1], 1 / A * point[0], 0]
    U = [1 / d * point[0], 1 / d * point[1], 1 / d * point[2]]
    Z = np.linalg.cross(U, V)

    return [0.5 * d * U[i] + R * math.cos(orientation) * V[i] + R * math.sin(orientation) * Z[i] for i in range(3)]

def calculate_angles(point):
    x_circ = math.sqrt(segment_length ** 2 - point[1] ** 2)
    angles = [0, 0]

    angles[0] = -2 * math.atan2(point[2] - x_circ, point[0])
    angles[1] = -2 * math.atan2(x_circ, point[1] + segment_length)

    return angles

def calculate_target_angles(point, orientation, radians=True):
    segment1_target = parameterized_intersection(point, orientation)
    joint1_angles = calculate_angles(segment1_target)

    w = [point[0] - segment1_target[0], point[1] - segment1_target[1], point[2] - segment1_target[2]]

    row1 = [math.cos(joint1_angles[0]), 0, -math.sin(joint1_angles[0])]
    row2 = [-math.sin(joint1_angles[0]) * math.sin(joint1_angles[1]), math.cos(joint1_angles[1]), -math.cos(joint1_angles[0]) * math.sin(joint1_angles[1])]
    row3 = [math.sin(joint1_angles[0]) * math.cos(joint1_angles[1]), math.sin(joint1_angles[1]), math.cos(joint1_angles[0]) * math.cos(joint1_angles[1])]
    transform_inverse = np.array([row1, row2, row3])

    #print(f"transform_inverse: {transform_inverse}")
    #print(f"w: {w}")

    W = np.matmul(transform_inverse, w)
    joint2_angles = calculate_angles(W)
    angles = joint1_angles + joint2_angles
    #print(f"W: {W}")

    angles = [fix_angle(i) for i in angles]

    if not radians:
        angles = [i * 180 / PI for i in angles]

    #if (angles[2] > 0 and angles[2] > PI / 2):
        #angles[2] = PI - angles[2]

    return angles

def check_angle_bounds(angles):
    success = True
    for i in range(3):
        if abs(angles[i + 1]) > PI / 2:
            #print(f"motor {i + 1} out of range with angle {angles[i + 1] * 180 / PI}")
            success = False

    return success

results = [0, 0]

min_dist = segment_length * math.sqrt(2)

#print(f"output: {[convert(i, False) for i in calculate_target_angles((-0.9323101942233875, 1.1110840233783252, 0.2557476826774733), 0)]}")

#exit()
failed_distances = []

for t1 in range(90):
    for t2 in range(90):
        for r in range(8):
            R = (2 * segment_length - min_dist) * (r + 2) * 0.1 + min_dist
            scale = math.cos(convert(t2 * 2 - 90)) * R
            x = -math.sin(convert(t1 * 2 - 90)) * scale
            y = math.cos(convert(t1 * 2 - 90)) * scale
            z = math.sin(convert(t2 * 2 - 90)) * R

            target = calculate_target_angles((x, y, z), 0)
            if not check_angle_bounds(target):
                success = False
                for t0 in range(72):
                    target = calculate_target_angles((x, y, z), convert(t0 * 5))

                    if check_angle_bounds(target):
                        success = True
                        break

                if not success:
                    #print(f"failed for position ({x}, {y}, {z})")
                    #print(f"output: {[convert(i, False) for i in target]}")
                    results[0] += 1
                else:
                    results[1] += 1
            else:
                results[1] += 1

print(f"Results: {results[0]} failed, {results[1]} success")
print(f"Success rate: {results[1] / (results[0] + results[1]) * 100}%")