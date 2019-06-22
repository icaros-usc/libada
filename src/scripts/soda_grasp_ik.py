import adapy
import numpy as np
import rospy
import tf.transformations

# rospy.init_node('soda_grasp')

ada = adapy.Ada(True)
viewer = ada.start_viewer("dart_markers/sode_grasp", "map")
world = ada.get_world()
hand = ada.get_hand()
hand_node = hand.get_endeffector_body_node()
arm_skeleton = ada.get_arm_skeleton()
arm_state_space = ada.get_arm_state_space()

rotation = tf.transformations.quaternion_matrix([0, 0, 0, 1])
translation = tf.transformations.translation_matrix([0.5, -0.142525, 0.502])
sodaPose = np.matmul(rotation, translation)
sodaURDFUri = "package://pr_assets/data/objects/can.urdf"
world.add_body_from_urdf_matrix(sodaURDFUri, sodaPose)

var = raw_input("Press Enter to continue...")

rospy.sleep(1.)

sodaTSR = adapy.get_default_TSR()

sodaTSR.set_T0_w(sodaPose)
sodaTSR_Tw_e = sodaTSR.get_Tw_e()
sodaTSR.set_Tw_e(np.matmul(sodaTSR_Tw_e, hand.get_endeffector_transform("cylinder")))

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

trajectory = None
for configuration in configurations:
    trajectory = ada.plan_to_configuration(arm_state_space, arm_skeleton, configuration)
    if trajectory:
        break

if not trajectory:
    print("Failed to find a solution!")
else:
    trajectory = ada.compute_retime_path(arm_skeleton, trajectory)
    ada.execute_trajectory(trajectory)
    var = raw_input("Press Enter to exit...")
