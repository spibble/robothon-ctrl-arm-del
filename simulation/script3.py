import pybullet as p
import numpy as np
import time
import pybullet_data
import math

PI = 3.141592653589

physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0, 0, 0)
planeId = p.loadURDF("plane.urdf")
startPos = [0, 0, 1.25]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
boxId = p.loadURDF("arm.urdf", startPos, startOrientation, useFixedBase=True)

startPos2 = [0.35, 0, 1.5]
#sphereId = p.loadURDF("sphere.urdf", startPos2, startOrientation, useFixedBase=True)
startPos3 = [1.15, 0, 1.2]
#sphereId2 = p.loadURDF("sphere.urdf", startPos3, startOrientation, useFixedBase=True)

#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
camera_distance = 4
camera_pitch = -30
camera_target = [1, 0, 0.5]  # Focus point

print(f"Joints: {p.getNumJoints(boxId)}")

segment_length = 1

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

def move_joint(joint, angle):
    p.setJointMotorControl2(boxId, joint, p.POSITION_CONTROL, targetPosition=angle, force=15)

def move_segment(joint, angles, radians=False):
    if not radians:
        angles = [angles[0] * PI / 180, angles[1] * PI / 180]

    angles = [fix_angle(angles[0]), fix_angle(angles[1])]
    
    match joint:
        case 0:
            move_joint(1, angles[0])
            move_joint(2, angles[1])

            if (abs(angles[1]) > PI / 2):
                print(f"motor 2 out of range with angle {angles[1] * 180 / PI}")
        case 1:
            move_joint(3, angles[0])
            move_joint(5, angles[1])

            if (abs(angles[0]) > PI / 2):
                print(f"motor 3 out of range with angle {angles[0] * 180 / PI}")
            if (abs(angles[1]) > PI / 2):
                print(f"motor 4 out of range with angle {angles[1] * 180 / PI}")

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
            return False
    
    return success

def move_to_point(point):
    angles = calculate_target_angles(point, 0)
    success = check_angle_bounds(angles)

    if not success:
        for t in range(72):
            angles = calculate_target_angles(point, t * PI / 36)
            if check_angle_bounds(angles):
                success = True
                break

    if success:
        move_segment(0, angles[:2], True)
        move_segment(1, angles[2:], True)
        return 1
    else:
        print("failed to find valid motor angles")
        return 0

x = 0
count = 0

yaw = 0

#move_segment(0, [0, 0])
#move_segment(1, [0, 0])
#print(calculate_target_angles([0.5, 0.67, 0.76], 0, False))
#move_to_point([0.5, 0.67, 0.76], 0)
print(calculate_target_angles([-0.9323101942233875, 1.1110840233783252, 0.2557476826774733], 0, False))

test_square = [[-1, 1.2, 1 - i / 100] for i in range(200)] + [[-1 + i / 100, 1.2, -1] for i in range(200)] + [[1, 1.2, -1 + i / 100] for i in range(200)] + [[1 - i / 100, 1.2, 1] for i in range(200)]

while count > -1:
    x += 1
    
    #move_to_point(test_square[x % 800])

    p.resetDebugVisualizerCamera(cameraDistance=camera_distance,cameraYaw=yaw, cameraPitch=camera_pitch, cameraTargetPosition=camera_target)

    keys = p.getKeyboardEvents()
    if keys.get(p.B3G_LEFT_ARROW):
        yaw -= 1
    if keys.get(p.B3G_RIGHT_ARROW):
        yaw += 1
    if keys.get(p.B3G_UP_ARROW): 
        camera_distance -= 0.1
    if keys.get(p.B3G_DOWN_ARROW):
        camera_distance += 0.1

    p.stepSimulation()
    time.sleep(1.0 / 240)  # 240 Hz simulation

    count += 1