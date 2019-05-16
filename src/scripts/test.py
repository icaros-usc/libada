#!/usr/bin/env python

import aikidopy
import adapy
import rospy
from roscpp_initializer import roscpp_initializer
import aikidopy 

ada = adapy.Ada()
viewer = ada.startViewer("dart_markers/simple_trajectories", "map")

import IPython; IPython.embed()
#roscpp_initializer.roscpp_init("load_ada", [])
#rospy.init_node("load_ada")
#rate = rospy.Rate(10)



#if not rospy.is_shutdown():
#  ada = adapy.Ada()
#  #world = ada.getWorld()
#  skeleton = ada.getSkeleton()
#  print(ada.getName())
#  print "starting viewer"
#  viewer = InteractiveMarkerViewer("interactive_markers", "map")
#  viewer.addSkeleton(skeleton);
#  viewer.setAutoUpdate(true);
#  print "bye"
