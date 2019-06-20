import adapy
import rospy

# rospy.init_node('soda_grasp')

ada = adapy.Ada(True)
viewer = ada.start_viewer("dart_markers/sode_grasp", "map")
world = ada.get_world()

sodaPose = [0.5, -0.142525, 0.502, 0, 0, 0, 1]
sodaURDFUri = "package://pr_assets/data/objects/can.urdf"
world.add_body_from_urdf(sodaURDFUri, sodaPose)

rospy.sleep(1.)

sodaTSR = adapy.get_default_TSR()

import pdb
pdb.set_trace()

sodaTSR.set_mT0_w(sodaPose)

var = raw_input("Press Enter to continue...")
