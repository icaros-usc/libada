import adapy
import rospy
import numpy as np
import time


class AdaRRT():
    """
    Rapidly-Exploring Random Trees (RRT)
    """
    class Node():
        def __init__(self, state, parent):
            self.state = state.copy()
            self.parent = parent
            self.children = []


        def __iter__(self):
            nodelist = [self]
            while nodelist:
                node = nodelist.pop(0)
                nodelist.extend(node.children)
                yield node

    def __init__(self,
                 start_state,
                 goal_state,
                 ada,
                 step_size,
                 goal_precision,
                 max_itr=10000):
        self.start = AdaRRT.Node(start_state, None)
        self.goal = AdaRRT.Node(goal_state, None)
        self.max_itr = max_itr
        self.ada = ada
        self.position_lower_limits = np.array([-3.14, 1.57, 0.33, -3.14, 0, 0])
        self.position_upper_limits = np.array([3.14, 5.00, 5.00, 3.14, 3.14, 3.14])
        self.step_size = step_size
        self.goal_precision = goal_precision
        self.min_err = 1000
        
        # add objects to world
        canURDFUri = "package://pr_assets/data/objects/can.urdf"
        sodaCanPose = [0.25, -0.35, 0.0, 0, 0, 0, 1]
        tableURDFUri = "package://pr_assets/data/furniture/uw_demo_table.urdf"
        tablePose = [0.3, 0.0, -0.7, 0.707107, 0, 0, 0.707107]
        world = ada.get_world()
        can = world.add_body_from_urdf(canURDFUri, sodaCanPose)
        table = world.add_body_from_urdf(tableURDFUri, tablePose)

        collision_free_constraint = ada.set_up_collision_detection(self.ada.get_arm_state_space(),
                                                                   self.ada.get_arm_skeleton(),
                                                                   [can, table])
        self.full_collision_free_constraint = ada.get_full_collision_constraint(self.ada.get_arm_state_space(),
                                                                                self.ada.get_arm_skeleton(),
                                                                                collision_free_constraint)
  
 
    def build(self):
        for k in range(self.max_itr):
  
            # Sample a random node
            if np.random.random_sample(1)>0.8:
              random_state = self._get_random_state()
            else:
              random_state = self._get_random_state_near_goal()
            # find nearest neighbor
            nearest_neighbor = self._get_nearest_neighbor(random_state)
  
            # too close
            if np.linalg.norm(nearest_neighbor.state - random_state) < self.step_size:
              continue
          

            # extend RRT
            new_node = self._extend_sample(random_state, nearest_neighbor)
            if not new_node:
                continue

            # check for completion
            err = np.linalg.norm(new_node.state - self.goal.state)
            if err < self.min_err:
              self.min_err = err
              print(self.min_err)
            if err < self.goal_precision:
                self.goal.parent = new_node
                path = self._trace_path_to_goal()
                return path

    def _get_random_state_near_goal(self):
        state = np.random.random_sample(self.position_lower_limits.size)
        state = state * 0.1
        state = self.goal.state-0.05 + state
        return state

    def _get_random_state(self):
 
        state = np.random.random_sample(self.position_lower_limits.size)
        state = state * (self.position_upper_limits - self.position_lower_limits)
        state = state + self.position_lower_limits


        return state

    def _get_nearest_neighbor(self, sample):
        dist2sample = [(np.linalg.norm(sample - n.state), n) for n in self.start]
        return min(dist2sample)[1]

    def _extend_sample(self, sample, neighbor):
        dir2sample = (sample - neighbor.state) / np.linalg.norm(sample - neighbor.state)

        # scale by step_size
        dir2sample = dir2sample * self.step_size

        # check if new location is valid
        new_node_state = neighbor.state + dir2sample
        if self._check_for_collision(new_node_state):
            return

        # build a new node and add it to the tree
        new_node = AdaRRT.Node(new_node_state, neighbor)
        neighbor.children.append(new_node)
        return new_node

    def _trace_path_to_goal(self, node=None):
        if node is None:
            node = self.goal
        path = [node]

        # backtrace through the tree
        while node is not self.start:
            node = node.parent
            path.insert(0, node)

        # convert path to waypoints
        waypoints = [p.state for p in path]

        # return a single list of numpy arrays
        return waypoints

    def _check_for_collision(self, new_node_state):
        return self.full_collision_free_constraint.is_satisfied(self.ada.get_arm_state_space(), self.ada.get_arm_skeleton(), new_node_state)


def runRRT(start_state, goal_state, step_size, goal_precision, ada):
      # easy goal
    adaRRT = AdaRRT(start_state = start_state,goal_state = goal_state,step_size = step_size, 
        goal_precision = goal_precision, ada = ada)
    path = adaRRT.build()
    if path is not None:
        print(path)
        waypoints = []
        for i, waypoint in enumerate(path):
            waypoints.append((0.0 + i, waypoint))
    traj = ada.compute_smooth_joint_space_path(ada.get_arm_state_space(), waypoints)
    return traj

if __name__ == '__main__':
    sim = True

    # instantiate an ada
    ada = adapy.Ada(True)

    armHome = [-1.5,3.22,1.23,-2.19,1.8,1.2]

    goalConfig =[-1.72,4.44,2.02,-2.04,2.66, 1.39]

    if (sim):
      ada.set_positions(goalConfig)
     

    # launch viewer
    viewer = ada.start_viewer("dart_markers/simple_trajectories", "map")

    # easy goal
    adaRRT = AdaRRT(start_state=np.array(armHome),
                    goal_state=np.array(goalConfig),
                    step_size = 0.1, 
                    goal_precision = 0.1,
                    ada=ada)


    rospy.sleep(1.0)

    path = adaRRT.build()
    if path is not None:
        print(path)
        waypoints = []
        for i, waypoint in enumerate(path):
            waypoints.append((0.0 + i, waypoint))

        t0= time.clock()
        traj = ada.compute_smooth_joint_space_path(ada.get_arm_state_space(), waypoints)
        t= time.clock()-t0
        print(str(t)+"seconds elapsed")
        raw_input('Press ENTER to exit')
        ada.execute_trajectory(traj)
        rospy.sleep(1.0)

