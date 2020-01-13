# cmake_minimum_required(VERSION 3.10)
# project(py_wrappers2)

# # Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# # Default to C++17
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 17)
# endif()

# find_package(python_cmake_module REQUIRED)
# find_package(PythonExtra REQUIRED)

# message(STATUS "Python executable: ${PYTHON_EXECUTABLE}")

# # Uncomment the 2 set commands if using self compiled boost such as 1.69
# # Hint to cmake that our boost is in this directory
# #set(BOOST_ROOT "/opt/boost_1_69_0")
# # Do not find default boost from system path (1.65 on Ubuntu 18.04)
# #set(Boost_NO_SYSTEM_PATHS ON)
# #find_package(Boost 1.65)
# find_package(Boost 1.65 COMPONENTS python${PYTHON_VERSION_MAJOR} REQUIRED)
# #find_package(Boost 1.67 COMPONENTS python${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR} REQUIRED)

# message(STATUS "PYTHON_LIBRARIES = ${PYTHON_LIBRARIES}")
# message(STATUS "PYTHON_EXECUTABLE = ${PYTHON_EXECUTABLE}")
# message(STATUS "PYTHON_INCLUDE_DIRS = ${PYTHON_INCLUDE_DIRS}")
# message(STATUS "Boost_LIBRARIES = ${Boost_LIBRARIES}")
# message(STATUS "Boost_INCLUDE_DIRS = ${Boost_INCLUDE_DIRS}")


# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies for the forward kinematics library
# find_package(urdf REQUIRED)
# find_package(urdfdom_headers REQUIRED)
# find_package(kdl_parser REQUIRED)

# add_library(forward_kinematics src/forward_kinematics.cpp)
# set_property(TARGET forward_kinematics PROPERTY POSITION_INDEPENDENT_CODE ON)
# target_include_directories(forward_kinematics PUBLIC include)
# ament_target_dependencies(
#         forward_kinematics
#         orocos_kdl
#         urdf
#         urdfdom_headers
#         kdl_parser
# )

# install(TARGETS forward_kinematics
#         ARCHIVE DESTINATION lib
#         LIBRARY DESTINATION lib
#         RUNTIME DESTINATION bin
# )
# install(DIRECTORY include/
#         DESTINATION include)

# PYTHON_ADD_MODULE(forward_kinematics_py src/forward_kinematics_py.cpp)
# target_include_directories(forward_kinematics_py PUBLIC ${PYTHON_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} include)
# target_link_libraries(forward_kinematics_py PRIVATE ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} forward_kinematics)
# install(TARGETS forward_kinematics_py
#         ARCHIVE DESTINATION lib
#         LIBRARY DESTINATION lib
#         RUNTIME DESTINATION bin
#         )

# ament_package()


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

pybind11_add_module(forward_kinematics_py
    src/forward_kinematics_py.cpp)
target_link_libraries(forward_kinematics_py PRIVATE forward_kinematics)

set_property(TARGET forward_kinematics_py PROPERTY POSITION_INDEPENDENT_CODE ON)
install(TARGETS forward_kinematics_py
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages
        RUNTIME DESTINATION bin
)

ament_package()