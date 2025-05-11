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

camera_distance = 4
camera_pitch = -30
camera_target = [1, 0, 0.5]

print(f"Joints: {p.getNumJoints(boxId)}")

# start of functions that need to be translated
##########################################################################################

segment_length = 1 # length of arm segment, technically doesn't matter and can be just 1

# converts angles
# x: angle to convert
# radians: True to convert into radians, False to convert to degrees
# return: x but better
def convert(x, radians=True):
    if radians:
        return x * PI / 180
    else:
        return x * 180 / PI

# distance between p1 and p2
# p1: point [x, y, z] or [x, y]
# p2: point [x, y, z] or [x, y] or origin if not specified
# return: distance
def dist(p1, p2 = [0, 0, 0]): 
    if (len(p1) == 2):
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
    else:
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 + (p1[2] - p2[2]) ** 2)

# limits x between max and min
# x: some number (probably an angle)
# min: self-explanatory
# max: see above
# return: x but better
def constrain(x, min = -PI / 2, max = PI / 2): 
    if x < min:
        return min
    elif x > max:
        return max
    else:
        return x
    
# limits an angle between -pi and +pi
# x: angle
# return: x but better
def fix_angle(x): 
    x %= (2 * PI)
    if x > PI:
        return x - 2 * PI
    elif x < -PI:
        return x + 2 * PI
    else:
        return x

def move_joint(joint, angle): # basically just the same as writing an angle to a servo
    p.setJointMotorControl2(boxId, joint, p.POSITION_CONTROL, targetPosition=angle, force=15)

# sends move_joint commands to 2 motors at once
# joint 0: shoulder
# joint 1: elbow
# angles: [theta1, theta2] for each motor in the joint
# radians: true to use radians
# return: None
def move_segment(joint, angles, radians=False): 
    if not radians:
        angles = [convert(angles[0]), convert(angles[1])]

    angles = [fix_angle(angles[0]), fix_angle(angles[1])]
    
    match joint:
        case 0:
            move_joint(1, angles[0])
            move_joint(2, angles[1])

            if (abs(angles[1]) > PI / 2):
                print(f"motor 2 out of range with angle {convert(angles[1], False)}")
        case 1:
            move_joint(3, angles[0])
            move_joint(5, angles[1])

            if (abs(angles[0]) > PI / 2):
                print(f"motor 3 out of range with angle {convert(angles[0], False)}")
            if (abs(angles[1]) > PI / 2):
                print(f"motor 4 out of range with angle {convert(angles[1], False)}")

# point: [x, y, z] target point of hand
# orientation: position along intersection circle
# return: [x, y, z]
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
    Z = np.cross(U, V) # vector cross product between U and V

    return [0.5 * d * U[i] + R * math.cos(orientation) * V[i] + R * math.sin(orientation) * Z[i] for i in range(3)]

# finds transformation needed to reach a point
# point: [x, y, z]
# return: [theta1, theta2]
def calculate_angles(point):
    x_circ = math.sqrt(segment_length ** 2 - point[1] ** 2)
    angles = [0, 0]

    angles[0] = -2 * math.atan2(point[2] - x_circ, point[0])
    angles[1] = -2 * math.atan2(x_circ, point[1] + segment_length)

    return angles

# finds all 4 angles needed to reach a point
# point: [x, y, z]
# orientation: orientation of the hand
# radians: true to output in radians
# return: [theta1, theta2, theta3, theta4] (is not guaranteed to be within valid limits)
def calculate_target_angles(point, orientation, radians=True):
    segment1_target = parameterized_intersection(point, orientation)
    joint1_angles = calculate_angles(segment1_target)

    w = [point[0] - segment1_target[0], point[1] - segment1_target[1], point[2] - segment1_target[2]]

    row1 = [math.cos(joint1_angles[0]), 0, -math.sin(joint1_angles[0])]
    row2 = [-math.sin(joint1_angles[0]) * math.sin(joint1_angles[1]), math.cos(joint1_angles[1]), -math.cos(joint1_angles[0]) * math.sin(joint1_angles[1])]
    row3 = [math.sin(joint1_angles[0]) * math.cos(joint1_angles[1]), math.sin(joint1_angles[1]), math.cos(joint1_angles[0]) * math.cos(joint1_angles[1])]
    transform_inverse = np.array([row1, row2, row3]) # matrix representation of the above 3 arrays

    W = np.matmul(transform_inverse, w)  # multiplies transform_inverse with w
    joint2_angles = calculate_angles(W)
    angles = joint1_angles + joint2_angles

    angles = [fix_angle(i) for i in angles]

    if not radians:
        angles = [convert(i, False) for i in angles]

    return angles

# checks if angles are within valid limits
# theta1 is the 360 servo and does not need to be bounded
# angles: [theta1, theta2, theta3, theta4]
# return: True if all angles are valid, False if not
def check_angle_bounds(angles):
    success = True
    for i in range(3):
        if abs(angles[i + 1]) > PI / 2:
            return False
    
    return success

# moves to a point (intended function for interfacing with the arm)
# point: [x, y, z]
# return: target angles [theta1, theta2, theta3, theta4] if successfully moved, 0 if no valid angles are found
def move_to_point(point):
    angles = calculate_target_angles(point, 0)
    success = check_angle_bounds(angles)

    if not success:
        for t in range(72): # finds a valid orientation if the first one is invalid
            angles = calculate_target_angles(point, t * PI / 36)
            if check_angle_bounds(angles):
                success = True
                break

    if success:
        move_segment(0, angles[:2], True)
        move_segment(1, angles[2:], True)
        return angles
    else:
        print("failed to find valid motor angles")
        return 0

# end of functions that need to be implemented
######################################################################################
# everything below this point is testing

# tests after implementing
#print(move_to_point([1, 1, 1])) # expected: [0.2849241266220632, -0.548028407620313, 0.9004038352924489, -1.0471975511965974]
#print(move_to_point([0, 1, 1])) # expected: 0
#print(move_to_point([-1.5, 0.25, 1])) # expected: [-0.5510269726559951, -1.4109847059054328, -1.55258853232945, -0.8549582665697857]
#print(move_to_point([-1, 0.5, -1])) # expected: [-1.5786151632304684, -1.1071609446474566, -1.567299665340304, -1.445468495626831]
#print(move_to_point([0.25, 0, 1.5])) # expected: [0.16514867741462438, -0.8638445989076784, 0.0, -1.4139034557744354]
#print(move_to_point([0, 2, 0])) # expected: [0.0, 0.0, 0.0, 0.0]

x = 0
count = 0

yaw = 0

test_square = [[-1, 1.2, 1 - i / 100] for i in range(200)] + [[-1 + i / 100, 1.2, -1] for i in range(200)] + [[1, 1.2, -1 + i / 100] for i in range(200)] + [[1 - i / 100, 1.2, 1] for i in range(200)]

while count > -1:
    x += 1
    
    move_to_point(test_square[x % 800])

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