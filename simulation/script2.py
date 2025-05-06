import pybullet as p
import numpy as np
import time
import pybullet_data
import math
import sys

PI = 3.141592653589

physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0, 0, 0)
planeId = p.loadURDF("plane.urdf")
startPos = [0, 0, 1.25]
startOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("arm.urdf", startPos, startOrientation, useFixedBase=True)

startPos2 = [0.35, 0, 1.5]
#sphereId = p.loadURDF("sphere.urdf", startPos2, startOrientation, useFixedBase=True)
startPos3 = [1.15, 0, 1.2]
sphereId2 = p.loadURDF("sphere.urdf", startPos3, startOrientation, useFixedBase=True)

#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
camera_distance = 4
camera_pitch = -30
camera_target = [1, 0, 0.5]  # Focus point

print(f"Joints: {p.getNumJoints(boxId)}")

segmentOrientation1 = (0, 0)
segmentLength = 0.8

# Main loop
yaw = 180  # Start yaw angle
p.resetDebugVisualizerCamera(cameraDistance=camera_distance,
                                  cameraYaw=yaw,
                                  cameraPitch=camera_pitch,
                                  cameraTargetPosition=camera_target)
count = 0

def sign(x):
    if x > 0:
        return 1
    elif x <= 0:
        return -1

def dist(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 + (p1[2] - p2[2]) ** 2)

def move_joint(joint, angle):
    p.setJointMotorControl2(boxId, joint, p.POSITION_CONTROL, targetPosition=angle, force=15)

def safe_asin(x):
    return math.asin(max(-1, min(1, x)))

def get_orientation(coords):
    return (-math.atan2(coords[0], coords[1]), safe_asin(coords[2] / segmentLength))

def get_joint_angles(euler):
    if (euler[1] == 0):
        euler = (euler[0], sys.float_info.min)

    mult = math.sqrt(math.sin(euler[0]) ** 2 + math.tan(euler[1]) ** 2)
    
    return (safe_asin(math.cos(euler[1]) * mult), PI / 2 + 2 * math.atan(math.cos(euler[1]) / math.sin(euler[1]) * (mult - math.sin(euler[0]))))

def get_segment1_target(target, parameter):
    if (dist(target, (0, 0, 0)) > 2 * segmentLength):
        print("Target is unreachable at distance: ", dist(target, (0, 0, 0)))
        return (0, 0, 0)
    
    R = math.sqrt(segmentLength ** 2 - 0.25 * (target[0] ** 2 + target[1] ** 2 + target[2] ** 2))
    A = math.sqrt(target[0] ** 2 + target[1] ** 2)
    d = math.sqrt(target[0] ** 2 + target[1] ** 2 + target[2] ** 2)

    return (0.5 * target[0] - R * (target[1] / A * math.cos(parameter) + target[0] * target[2] / (A * d) * math.sin(parameter)), 0.5 * target[1] - R * (target[0] / A * math.cos(parameter) - target[1] * target[2] / (A * d) * math.sin(parameter)), 0.5 * target[2] + R * A / d * math.sin(parameter))

def get_segment2_angles(target, parameter):
    #v = [-segmentLength * math.sin(segmentOrientation1[0]) * math.cos(segmentOrientation1[1]), segmentLength * math.cos(segmentOrientation1[0]) * math.cos(segmentOrientation1[1]), segmentLength * math.sin(segmentOrientation1[1])]
    v = get_segment1_target(target, parameter)
    print(f"v: {v}")
    segmentOrientation1 = get_orientation(v)

    w = [target[0] - v[0], target[1] - v[1], target[2] - v[2]]
    print(f"pre transform w: {w}")

    r1 = [math.cos(segmentOrientation1[0]) * math.cos(segmentOrientation1[1]), -math.sin(segmentOrientation1[0]) * math.cos(segmentOrientation1[1]), math.sin(segmentOrientation1[0]) * math.sin(segmentOrientation1[1])]
    r2 = [math.sin(segmentOrientation1[0]) * math.cos(segmentOrientation1[1]), math.cos(segmentOrientation1[0]) * math.cos(segmentOrientation1[1]), -math.cos(segmentOrientation1[0]) * math.sin(segmentOrientation1[1])]
    r3 = [0, math.sin(segmentOrientation1[1]), math.cos(segmentOrientation1[1])]
    transform = np.array([r1, r2, r3])
    print(f"transform: {transform}")
    print(f"det: {np.linalg.det(transform)}")
    w = np.matmul(np.linalg.inv(transform), w)

    mag = np.linalg.norm(w)
    print(f"w: {w}, mag: {mag}")
    return get_orientation((w[0], w[1], w[2]))

def move(segment1, segment2):
    move_segment1(segment1, False)

    move_segment2((segment2[0], segment2[1]), segment1, False)

print(f"target 1: {get_segment1_target((0, 0.5, 0.5), 0)}")
print(f"target 2: {get_segment2_angles((0, 0.5, 0.5), 0)}")
#print(f"target orientation: {get_segment2_angles((0, 0.8, 0.8))}")

def move_segment1(euler, radians = True):
    if not radians:
        euler = (euler[0] * PI / 180, euler[1] * PI / 180)
    output = get_joint_angles(euler)
    p.setJointMotorControl2(boxId, 1, p.POSITION_CONTROL, targetPosition=output[1], force=35)
    p.setJointMotorControl2(boxId, 2, p.POSITION_CONTROL, targetPosition=output[0], force=35)

    print(f"Target joint angles: {output[1] * 180 / PI}, {output[0] * 180 / PI}")

def move_segment2(euler, origin_euler, radians = True):
    if not radians:
        euler = (euler[0] * PI / 180, euler[1] * PI / 180)
    output = get_joint_angles(euler)
    output = (output[0] + 0 * (origin_euler[1] < 0), output[1])

    print(f"segmentOrientation1: {segmentOrientation1}")
    p.setJointMotorControl2(boxId, 3, p.POSITION_CONTROL, targetPosition=output[1], force=35)
    p.setJointMotorControl2(boxId, 5, p.POSITION_CONTROL, targetPosition=-output[0], force=35)

    print(f"Target joint angles: {output[0] * 180 / PI}, {output[1] * 180 / PI}")

def set_control(angles, radians = True):
    if not radians:
        angles = (angles[0] * PI / 180, angles[1] * PI / 180)
    p.setJointMotorControl2(boxId, 1, p.POSITION_CONTROL, targetPosition=angles[0], force=15)
    p.setJointMotorControl2(boxId, 2, p.POSITION_CONTROL, targetPosition=angles[1], force=15)

def reset_position():
    p.setJointMotorControl2(boxId, 1, p.POSITION_CONTROL, targetPosition=PI / 2, force=15)
    p.setJointMotorControl2(boxId, 2, p.POSITION_CONTROL, targetPosition=0, force=15)

def test_segment1(angle1, radians = True):
    euler = (angle1, 0)
    if not radians:
        euler = (angle1 * PI / 180, 0)
    output = get_joint_angles(euler)
    p.setJointMotorControl2(boxId, 1, p.POSITION_CONTROL, targetPosition=-output[0], force=15)
    p.setJointMotorControl2(boxId, 2, p.VELOCITY_CONTROL, targetVelocity=1)

    print(f"Target joint angles: {output[0] * 180 / PI}, {output[1] * 180 / PI}")

#test_segment1(30, radians=False)  # Move to initial position

settle_count = 0
x = 0
move_segment1((0, 0), radians=False)  # Move to initial position
#set_control((90, 0), radians=False)  # Move to initial position
#move_segment1((PI/2, 0), radians=True)
#move_segment2((45, 45), radians=False)
#set_control((0, 90), radians=False)  # Move to initial position

while count > -1:
    p.resetDebugVisualizerCamera(cameraDistance=camera_distance,cameraYaw=yaw, cameraPitch=camera_pitch, cameraTargetPosition=camera_target)
    #t = get_orientation(get_segment1_target((0, 0.5, 0.5), 0))
    #s = get_segment2_angles((0, 0.5, 0.5), 0)
    #move_segment1(t)
    #move_segment2(s)

    if settle_count > 1:
        t = get_orientation(get_segment1_target((0, 0.5, 0.5), x / 100 * 2 * PI))
        s = get_segment2_angles((0, 0.5, 0.5), x / 100 * 2 * PI)
        move_segment1((0, -x), radians=False)
        #move_segment2(s)
        x += 0.05
        if (x >= 90):
            x = 0

    #settle_count += 1

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
    time.sleep(1.0)  # 240 Hz simulation

    count += 1
