#include <pybind11/pybind11.h>

namespace py = pybind11;

void Ada(pybind11::module& m);
void Aikido(pybind11::module& m);
void Dart(pybind11::module& m);

PYBIND11_MODULE(adapy, m) {
  Ada(m);

  Aikido(m);

  Dart(m);
}
