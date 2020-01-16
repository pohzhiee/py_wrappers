cmake_minimum_required(VERSION 3.10)
project(py_wrappers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

add_subdirectory(pybind11)
pybind11_add_module(test_module src/main.cpp)
set_property(TARGET test_module PROPERTY POSITION_INDEPENDENT_CODE ON)

install(TARGETS test_module
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages
        RESOURCE DESTINATION resource
        RUNTIME DESTINATION bin
)

find_package(urdf REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(kdl_parser REQUIRED)


add_library(forward_kinematics src/forward_kinematics.cpp)
set_property(TARGET forward_kinematics PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(forward_kinematics PUBLIC include)
ament_target_dependencies(
        forward_kinematics
        orocos_kdl
        urdf
        urdfdom_headers
        kdl_parser
)
install(TARGETS forward_kinematics
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(forward_kinematics)
#ament_export_dependencies(py_wrappers)
pybind11_add_module(forward_kinematics_py
    src/forward_kinematics_py.cpp)
target_link_libraries(forward_kinematics_py PRIVATE forward_kinematics)

set_property(TARGET forward_kinematics_py PROPERTY POSITION_INDEPENDENT_CODE ON)
install(TARGETS forward_kinematics_py
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages
        RESOURCE DESTINATION resource
        RUNTIME DESTINATION bin
)

ament_package()
