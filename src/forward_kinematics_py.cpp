#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>
#include <pybind11/numpy.h>
#include <py_wrappers/forward_kinematics.hpp>

namespace py = pybind11;


PYBIND11_MAKE_OPAQUE(std::vector<double>);
PYBIND11_MODULE(forward_kinematics_py, m) {
    m.doc() = R"pbdoc(
        Pybind11 wrapper around forward kinematics code using Orocos KDL
        -----------------------
        .. currentmodule:: forward_kinematics
        .. autosummary::
           :toctree: _generate
           
           subtract
    )pbdoc";



    py::bind_vector<std::vector<double>>(m, "VectorDouble");



    py::class_<ForwardKinematics>(m, "ForwardKinematics")
        .def(py::init<const std::string &>())
        .def("calculate", [](ForwardKinematics* fk, const std::string &root, const std::string &tip, const py::array_t<double> joint_states) -> Pose{

            py::buffer_info buf1 = joint_states.request();
            if (buf1.ndim != 1)
                throw std::runtime_error("Number of dimensions must be one");

            auto start_ptr = (double*)buf1.ptr;
            auto end_ptr = start_ptr + buf1.shape[0];
            std::vector<double> states_vec(start_ptr, end_ptr);
            return fk->calculate(root, tip, states_vec);
        });
        
    py::class_<Vector>(m, "Vector")
        .def_readwrite("x", &Vector::x)
        .def_readwrite("y", &Vector::y)
        .def_readwrite("z", &Vector::z);

    py::class_<Pose>(m, "Pose")
        .def_readwrite("translation", &Pose::translation)
        .def_readwrite("rotation_euler", &Pose::rotation_euler);

#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}