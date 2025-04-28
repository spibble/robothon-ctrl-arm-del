import pybullet as p
import time
import pybullet_data

physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0, 0, 0)
planeId = p.loadURDF("plane.urdf")
startPos = [0, 0, 0.25]
startOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("arm.urdf", startPos, startOrientation, useFixedBase=True)

#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
camera_distance = 2
camera_pitch = -30
camera_target = [0, 0, 0.5]  # Focus point

# Main loop
yaw = 0  # Start yaw angle
while True:
    # Orbit: Increase yaw over time
    yaw += 0.1
    if yaw > 360:
        yaw -= 360
    
    # Update camera view
    p.resetDebugVisualizerCamera(cameraDistance=camera_distance,
                                  cameraYaw=yaw,
                                  cameraPitch=camera_pitch,
                                  cameraTargetPosition=camera_target)
    
    p.stepSimulation()
    time.sleep(1.0 / 240.0)  # 240 Hz simulation
