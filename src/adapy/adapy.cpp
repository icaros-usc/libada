#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/cast.h>
#include <pybind11/stl.h>

#include <memory>
#include <aikido/planner/World.hpp>
#include <aikido/constraint/Satisfied.hpp>
#include <aikido/constraint/dart/CollisionFree.hpp>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>

#include "libada/Ada.hpp"
#include <ros/ros.h>

#include <dart/dart.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/CollisionGroup.hpp>

namespace py = pybind11;

Eigen::Isometry3d vectorToIsometry(std::vector<double> &poseVector) {
  double *ptr = &poseVector[0];
  Eigen::Map<Eigen::VectorXd> p(ptr, 7);

  Eigen::Isometry3d pose(Eigen::Isometry3d::Identity());
  pose.translation() = p.head(3);
  Eigen::Quaterniond q(p[3], p[4], p[5], p[6]);
  pose.linear() = Eigen::Matrix3d(q);
  return pose;
}

PYBIND11_MODULE(adapy, m) {
  //====================================LIBADA==========================================================
  py::class_<ada::Ada, std::shared_ptr<ada::Ada>>(m, "Ada")
      .def(py::init([](bool simulation) -> std::shared_ptr<ada::Ada> {
        return std::make_shared<ada::Ada>(
            aikido::planner::World::create(), simulation);
      }))
      .def("get_self_collision_constraint",
           [](ada::Ada *self) -> std::shared_ptr<aikido::constraint::dart::CollisionFree> {
             return self->getSelfCollisionConstraint(
                 self->getStateSpace(),
                 self->getMetaSkeleton()
             );
           })
      .def("start_trajectory_executor", [](ada::Ada *self) -> void {
        self->startTrajectoryExecutor();
      })
      .def("get_name",
           [](ada::Ada *self) -> std::string { return self->getName(); })
      .def("get_world",
           [](ada::Ada *self) -> aikido::planner::WorldPtr {
             return self->getWorld();
           })
      .def("get_skeleton", [](ada::Ada *self) -> dart::dynamics::MetaSkeletonPtr {
        return self->getMetaSkeleton();
      })
      .def("get_arm_skeleton", [](ada::Ada *self) -> dart::dynamics::MetaSkeletonPtr {
        return self->getArm()->getMetaSkeleton();
      })
      .def("get_arm_state_space", [](ada::Ada *self) -> aikido::statespace::dart::MetaSkeletonStateSpacePtr {
        return self->getArm()->getStateSpace();
      })
      .def("set_up_collision_detection",
           [](ada::Ada *self,
              const aikido::statespace::dart::MetaSkeletonStateSpacePtr &armSpace,
              const dart::dynamics::MetaSkeletonPtr& armSkeleton,
              const std::vector<dart::dynamics::SkeletonPtr>& envSkeletons) -> std::shared_ptr<aikido::constraint::dart::CollisionFree> {
             dart::collision::FCLCollisionDetectorPtr collisionDetector = dart::collision::FCLCollisionDetector::create();
             std::shared_ptr<dart::collision::CollisionGroup> armCollisionGroup =
                 collisionDetector->createCollisionGroup(self->getMetaSkeleton().get());
             std::shared_ptr<dart::collision::CollisionGroup> envCollisionGroup =
                 collisionDetector->createCollisionGroup();
             for (const auto& envSkeleton : envSkeletons) {
               envCollisionGroup->addShapeFramesOf(envSkeleton.get());
             }
             std::shared_ptr<aikido::constraint::dart::CollisionFree> collisionFreeConstraint =
                 std::make_shared<aikido::constraint::dart::CollisionFree>(armSpace,
                                                                           armSkeleton,
                                                                           collisionDetector);
             collisionFreeConstraint->addPairwiseCheck(armCollisionGroup, envCollisionGroup);
             return collisionFreeConstraint;
           })
      .def("get_full_collision_constraint",
           [](ada::Ada *self,
              const aikido::statespace::dart::MetaSkeletonStateSpacePtr &armSpace,
              const dart::dynamics::MetaSkeletonPtr& armSkeleton,
              const aikido::constraint::dart::CollisionFreePtr& collisionFreeConstraint) -> aikido::constraint::TestablePtr {
        return self->getFullCollisionConstraint(armSpace, armSkeleton, collisionFreeConstraint);
      })
      .def("compute_joint_space_path",
           [](ada::Ada *self,
              const aikido::statespace::dart::MetaSkeletonStateSpacePtr &stateSpace,
              const std::vector<std::pair<double, Eigen::VectorXd>> &waypoints)
               -> aikido::trajectory::TrajectoryPtr {
             return self->computeJointSpacePath(stateSpace, waypoints);
           })
      .def("compute_smooth_joint_space_path",
           [](ada::Ada *self,
              const aikido::statespace::dart::MetaSkeletonStateSpacePtr &stateSpace,
              const std::vector<std::pair<double, Eigen::VectorXd>> &waypoints)
               -> aikido::trajectory::TrajectoryPtr {
             return self->computeSmoothJointSpacePath(stateSpace, waypoints);
           })
      .def("execute_trajectory",
           [](ada::Ada *self,
              const aikido::trajectory::TrajectoryPtr &trajectory)
               -> void {
             auto future = self->executeTrajectory(trajectory);
             future.wait();
           })
      .def("start_viewer",
           [](ada::Ada *self, const std::string &topicName, const std::string &baseFrameName)
               -> std::shared_ptr<aikido::rviz::WorldInteractiveMarkerViewer> {
             int argc = 1;
             char *argv[] = {"adapy"};
             ros::init(argc, argv, "ada");
             auto viewer = std::make_shared<aikido::rviz::WorldInteractiveMarkerViewer>(
                 self->getWorld(),
                 topicName,
                 baseFrameName);
             viewer->setAutoUpdate(true);
             return viewer;
           });


  //====================================AIKIDO=============================================================================
  py::class_<aikido::planner::World, std::shared_ptr<aikido::planner::World>>(m, "World")
      .def("add_body_from_urdf", [](aikido::planner::World *self, const std::string &uri,
                                    std::vector<double> objectPose) -> std::shared_ptr<::dart::dynamics::Skeleton> {
        auto transform = vectorToIsometry(objectPose);

        dart::utils::DartLoader urdfLoader;
        const auto resourceRetriever
            = std::make_shared<aikido::io::CatkinResourceRetriever>();
        const auto skeleton = urdfLoader.parseSkeleton(uri, resourceRetriever);

        if (!skeleton)
          throw std::runtime_error("unable to load '" + uri + "'");

        dynamic_cast<dart::dynamics::FreeJoint *>(skeleton->getJoint(0))
            ->setTransform(transform);

        self->addSkeleton(skeleton);
        return skeleton;
      })
      .def("get_skeleton", [](aikido::planner::World *self, int i) -> dart::dynamics::SkeletonPtr {
        if (i + 1 <= int(self->getNumSkeletons())) {
          return self->getSkeleton(i);
        }
        else {
          return nullptr;
        }
      });

  py::class_<aikido::rviz::WorldInteractiveMarkerViewer, std::shared_ptr<aikido::rviz::WorldInteractiveMarkerViewer>>(
      m, "WorldInteractiveMarkerViewer")
      .def("add_tsr_marker",
           [](aikido::rviz::WorldInteractiveMarkerViewer *self,
              std::shared_ptr<aikido::constraint::dart::TSR> tsr)
               -> aikido::rviz::TSRMarkerPtr {
             return self->addTSRMarker(*tsr.get());
           });

  py::class_<aikido::constraint::dart::CollisionFree, std::shared_ptr<aikido::constraint::dart::CollisionFree>>(
      m,
      "CollisionFree");
  py::class_<aikido::statespace::dart::MetaSkeletonStateSpace, aikido::statespace::dart::MetaSkeletonStateSpacePtr>(
      m,
      "MetaSkeletonStateSpace");
  py::class_<aikido::trajectory::Trajectory, aikido::trajectory::TrajectoryPtr>(m, "Trajectory");

  py::class_<aikido::constraint::Testable, aikido::constraint::TestablePtr>(m, "FullCollisionFree").
      def("is_satisfied",
          [](aikido::constraint::Testable *self,
             const aikido::statespace::dart::MetaSkeletonStateSpacePtr& armSpace,
             const dart::dynamics::MetaSkeletonPtr& armSkeleton,
             const Eigen::VectorXd& positions) -> bool {
            auto armState = armSpace->createState();
            armSpace->convertPositionsToState(positions, armState);
            auto currentState = armSpace->getScopedStateFromMetaSkeleton(armSkeleton.get());
            aikido::constraint::DefaultTestableOutcome fullCollisionCheckOutcome;
            bool collisionResult = self->isSatisfied(armState, &fullCollisionCheckOutcome);
            armSpace->setState(armSkeleton.get(), currentState);
            return collisionResult;
          });

  //===================================================DART======================================================================
  py::class_<dart::dynamics::Skeleton, std::shared_ptr<dart::dynamics::Skeleton>>(m, "Skeleton")
      .def("get_name",
           [](dart::dynamics::Skeleton *self) -> std::string { return self->getName(); })
      .def("get_positions",
           [](dart::dynamics::Skeleton *self) -> Eigen::VectorXd { return self->getPositions(); })
      .def("get_num_joints",
           [](dart::dynamics::Skeleton *self) -> int {
             return self->getNumJoints();
           });

  py::class_<dart::dynamics::MetaSkeleton, std::shared_ptr<dart::dynamics::MetaSkeleton>>(m, "MetaSkeleton")
      .def("get_name",
           [](dart::dynamics::MetaSkeleton *self) -> std::string { return self->getName(); })
      .def("get_positions",
           [](dart::dynamics::MetaSkeleton *self) -> Eigen::VectorXd { return self->getPositions(); })
      .def("get_num_joints",
           [](dart::dynamics::MetaSkeleton *self) -> int {
             return self->getNumJoints();
           });
}
