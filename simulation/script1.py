import pybullet as p
import time
import pybullet_data

physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0, 0, -9.8)
planeId = p.loadURDF("plane.urdf")
startPos = [0, 0, 1.25]
startOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("arm.urdf", startPos, startOrientation, useFixedBase=True)

#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
camera_distance = 4
camera_pitch = -30
camera_target = [1, 0, 0.5]  # Focus point

print(f"Joints: {p.getNumJoints(boxId)}")

# Main loop
yaw = 180  # Start yaw angle
p.resetDebugVisualizerCamera(cameraDistance=camera_distance,
                                  cameraYaw=yaw,
                                  cameraPitch=camera_pitch,
                                  cameraTargetPosition=camera_target)
count = 0
while count < 2000:
    p.setJointMotorControlArray(boxId, [0, 1, 2, 3, 5, 6, 8], p.VELOCITY_CONTROL, targetVelocities=[0.3] * 7, forces=[50] * 7)
    
    p.stepSimulation()
    time.sleep(1.0 / 240.0)  # 240 Hz simulation

    count += 1
