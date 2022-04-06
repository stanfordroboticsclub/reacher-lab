from reacher import reacher_kinematics
import pybullet as p
import reacher.data as pd
import time
import math
import numpy as np
from absl import app
from absl import flags
import copy
from pupper_hardware_interface import interface
from serial.tools import list_ports
from sys import platform

flags.DEFINE_bool("run_on_robot", False, "Whether to run on robot or in simulation.")
FLAGS = flags.FLAGS
import pybullet_data

KP = 8.0
KD = 0.8
MAX_CURRENT = 6.0

HIP_OFFSET = 0.0335
L1 = 0.08
L2 = 0.11

def configure_pybullet():
  p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
  p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
  p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
  p.setAdditionalSearchPath(pybullet_data.getDataPath())
  p.resetDebugVisualizerCamera(cameraDistance=0.3, cameraYaw=-200, cameraPitch=-30, cameraTargetPosition=[0,0,0.1])

def main(argv):
  run_on_robot = FLAGS.run_on_robot
  p.connect(p.GUI)
  configure_pybullet()
  
  URDF_PATH = pd.getDataPath() + "/pupper_arm.urdf"
  reacher = p.loadURDF(URDF_PATH, useFixedBase=True)
  target_visual_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.015)
  target_position = np.array([0,0,0])
  sphere_id = p.createMultiBody(baseVisualShapeIndex=target_visual_shape, basePosition=target_position)

  joint_ids = []
  param_ids = []

  p.setPhysicsEngineParameter(numSolverIterations=10) # Affects performance?
  p.changeDynamics(reacher, -1, linearDamping=0, angularDamping=0)

  for j in range(p.getNumJoints(reacher)):
    p.changeDynamics(reacher, j, linearDamping=0, angularDamping=0)
    info = p.getJointInfo(reacher, j)
    jointName = info[1]
    jointType = info[2]
    if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
      joint_ids.append(j)
      param_ids.append(p.addUserDebugParameter(jointName.decode("utf-8"), -math.pi, math.pi, 0))

  if run_on_robot:
    if platform == "linux" or platform == "linux2":
      serial_port = next(list_ports.grep(".*ttyACM0.*")).device
    elif platform == "darwin":
      serial_port = next(list_ports.grep("usbmodem")).device
    hardware_interface = interface.Interface(serial_port)
    time.sleep(0.25)
    hardware_interface.set_joint_space_parameters(
        kp=KP, kd=KD, max_current=MAX_CURRENT)

  p.setRealTimeSimulation(1)
  counter = 0
  while (1):
    counter += 1
    joint_angles = np.zeros(3)

    # Read 
    for i in range(len(param_ids)):
      c = param_ids[i]
      target_pos = p.readUserDebugParameter(c)
      joint_angles[i] = target_pos
      p.setJointMotorControl2(reacher, joint_ids[i], p.POSITION_CONTROL, target_pos, force=20.)

    if run_on_robot:
      full_actions = np.zeros([3, 4])
      full_actions[:, 3] = np.reshape(joint_angles, 3)
      hardware_interface.set_actuator_postions(np.array(full_actions))
    
    # Use forward kinematics to set the position of the sphere to the
    # xyz coordinates computed by your forward kinematics code.
    # If it's working, the sphere should overlap with the 
    # actual robot end-effector. There may be some lag due to the 
    # inertia of the robot arm.
    end_effector_pos = reacher_kinematics.calculate_forward_kinematics_robot(joint_angles)
    p.resetBasePositionAndOrientation(sphere_id, posObj=end_effector_pos, ornObj=[0, 0, 0, 1])
    time.sleep(0.01)

app.run(main)