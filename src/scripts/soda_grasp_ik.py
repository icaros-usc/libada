import adapy
import numpy as np
import rospy
import tf.transformations
import adarrt
# rospy.init_node('soda_grasp')

def createBwMatrixforTSR():
  Bw = np.zeros([6,2])
  Bw[0,0] = 0
  Bw[0,1] = 0
  Bw[1,0] = 0
  Bw[1,1] = 0
  Bw[2,0] = -0.02
  Bw[2,1] = 0.02
  Bw[3,0] = 0
  Bw[3,1] = 0
  Bw[4,0] = 0
  Bw[4,1] = 0
  Bw[5,0] = -np.pi
  Bw[5,1] = np.pi
  return Bw


def createSodaTSR(sodaPose):
  sodaTSR = adapy.get_default_TSR()
  sodaTSR.set_T0_w(sodaPose)
  rot_trans = np.eye(4)
  rot_trans[0:3,0:3] = np.array([[0,0,-1],[1,0,0],[0,-1,0]])

  sodaTSR_Tw_e = np.matmul(rot_trans, hand.get_endeffector_transform("cylinder"))
  sodaTSR_Tw_e[0,3] = -0.04
  sodaTSR_Tw_e[2,3] = 0.08

  sodaTSR.set_Tw_e(sodaTSR_Tw_e)
  Bw = createBwMatrixforTSR()
  sodaTSR.set_Bw(Bw)
  return sodaTSR


def closeHand(hand, displacement):
  hand.execute_preshape(displacement)


if __name__ == '__main__':
  sim = True
  ada = adapy.Ada(True)
  viewer = ada.start_viewer("dart_markers/sode_grasp", "map")
  world = ada.get_world()
  hand = ada.get_hand()
  hand_node = hand.get_endeffector_body_node()
  arm_skeleton = ada.get_arm_skeleton()
  arm_state_space = ada.get_arm_state_space()

  armHome = [-1.5,3.22,1.23,-2.19,1.8,1.2]
  if (sim):
    ada.set_positions(armHome)

  viewer.add_frame(hand_node);

  sodaPose = np.eye(4)
  sodaPose[0,3] = 0.25
  sodaPose[1,3] = -0.35
  sodaURDFUri = "package://pr_assets/data/objects/can.urdf"
  world.add_body_from_urdf_matrix(sodaURDFUri, sodaPose)

  tableURDFUri = "package://pr_assets/data/furniture/uw_demo_table.urdf"
  tablePose = [0.3, 0.0, -0.7, 0.707107, 0, 0, 0.707107]
  table = world.add_body_from_urdf(tableURDFUri, tablePose)
  
  closeHand(hand, [0.75,0.75])

  rospy.sleep(1.)

   
  sodaTSR = createSodaTSR(sodaPose)
  
  #marker = viewer.add_tsr_marker(sodaTSR)


  var = raw_input("Press Enter to continue...")
  exit()

  ik_sampleable = adapy.create_ik(arm_skeleton,
                                  arm_state_space,
                                  sodaTSR,
                                  hand_node)

  ik_generator = ik_sampleable.create_sample_generator()

  configurations = []

  samples = 0
  maxSamples = 10
  while samples < maxSamples and ik_generator.can_sample():
      goal_state = ik_generator.sample(arm_state_space)

      samples += 1
      if len(goal_state) == 0:
          continue
      configurations.append(goal_state)

  if len(configurations) == 0:
      print("No valid configurations found!")

  if (sim):
    ada.set_positions(armHome)

  print("Computing RRT plan")
  trajectory = None
  for configuration in configurations:
      #trajectory = ada.plan_to_configuration(arm_state_space, arm_skeleton, configuration)
      trajectory = adarrt.runRRT(start_state = np.array(armHome), goal_state = np.array(configuration), step_size = 0.1, goal_precision = 0.1,ada = ada)

      if trajectory:
          break

  if not trajectory:
      print("Failed to find a solution!")
  else:
      #trajectory = ada.compute_retime_path(arm_skeleton, trajectory)
      ada.execute_trajectory(trajectory)
  var = raw_input("Press Enter to exit...")
