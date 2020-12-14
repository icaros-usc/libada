#include <dart/dynamics/InverseKinematics.hpp>
#include <aikido/constraint/dart/InverseKinematicsSampleable.hpp>
#include <aikido/constraint/dart/TSR.hpp>
#include <aikido/constraint/dart/JointStateSpaceHelpers.hpp>
#include <aikido/constraint/Sampleable.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/cast.h>
#include <pybind11/stl.h>
#include "libada/Ada.hpp"

namespace py = pybind11;

void IK(pybind11::module& m) {
  m.def("create_ik", [](
      dart::dynamics::MetaSkeletonPtr armSkeleton,
      aikido::statespace::dart::MetaSkeletonStateSpacePtr armSpace,
      aikido::constraint::dart::TSR objectTSR,
      ada::AdaHandPtr handPtr) ->
  aikido::constraint::dart::InverseKinematicsSampleable {
    auto goalTSR = std::make_shared<aikido::constraint::dart::TSR>(objectTSR);
    auto ik = dart::dynamics::InverseKinematics::create(handPtr->getEndEffectorBodyNode());
    ik->setDofs(armSkeleton->getDofs());
    auto rng = std::unique_ptr<aikido::common::RNG>(new aikido::common::RNGWrapper<std::default_random_engine>(0));
    aikido::constraint::dart::InverseKinematicsSampleable ikSampleable(armSpace,
        armSkeleton,
        goalTSR,
        aikido::constraint::dart::createSampleableBounds(armSpace, std::move(rng)),
        ik,
        10);
    return ikSampleable;
  });

  py::class_<aikido::constraint::dart::InverseKinematicsSampleable, std::shared_ptr<aikido::constraint::dart::InverseKinematicsSampleable>>(m, "IKSampleable")
    .def("create_sample_generator", [](aikido::constraint::dart::InverseKinematicsSampleable* self) -> std::unique_ptr<aikido::constraint::SampleGenerator> {
      return self->createSampleGenerator();
    });

  py::class_<aikido::constraint::SampleGenerator>(m, "IKSampleGenerator")
    .def("can_sample", [](aikido::constraint::SampleGenerator *self) -> bool {
      return self->canSample();
    })
    .def("sample", [](aikido::constraint::SampleGenerator *self,
                      const aikido::statespace::dart::MetaSkeletonStateSpacePtr& armSpace,
                      const dart::dynamics::MetaSkeletonPtr& armSkeleton) -> Eigen::VectorXd {
      auto goalState = armSpace->createState();
      std::lock_guard<std::mutex> lock(armSkeleton->getBodyNode(0)->getSkeleton()->getMutex());
      bool sampled = self->sample(goalState);
      Eigen::VectorXd positions;
      if (!sampled) return positions;
      armSpace->convertStateToPositions(goalState, positions);
      return positions;
    });

  py::class_<dart::dynamics::BodyNode, std::shared_ptr<dart::dynamics::BodyNode>>(m, "BodyNode");
}

