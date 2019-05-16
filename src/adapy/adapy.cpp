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

namespace py = pybind11;

namespace ada {
namespace python {


PYBIND11_MODULE(adapy, m)
//void Ada(pybind11::module &m)
{
	
py::object StateSpace = py::module::import("aikidopy").attr("StateSpace");
py::object Interpolated = py::module::import("aikidopy").attr("Interpolated");

 py::class_<ada::Ada, std::shared_ptr<ada::Ada>>(m, "Ada")
      .def(::pybind11::init([]{ return std::make_shared<ada::Ada>(
aikido::planner::World::create(), true); }))
      .def("getName",
[](ada::Ada* self) -> std::string { return self->getName(); })
      .def("getWorld",
        [](ada::Ada* self) -> aikido::planner::WorldPtr {
          return self->getWorld();
})
    .def("startViewer",
            [](ada::Ada* self, const std::string& topicName, const std::string& baseFrameName)
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
})
;

 py::class_<aikido::rviz::WorldInteractiveMarkerViewer, std::shared_ptr<aikido::rviz::WorldInteractiveMarkerViewer>>(m, "WorldInteractiveMarkerViewer");
 py::class_<aikido::planner::World, std::shared_ptr<aikido::planner::World>>(m, "World");
   //   .def("getSkeleton",
   //     [](ada::Ada* self) -> dart::dynamics::SkeletonPtr {
   //       return self->getWorld()->getSkeleton("j2n6s200");
//});

 // py::class_<dart::dynamics::Skeleton, std::shared_ptr<dart::dynamics::Skeleton>>(m, "Skeleton");

}
}
}
