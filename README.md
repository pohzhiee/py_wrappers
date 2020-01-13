# Introduction
This package is meant to create python wrappers for various components. As of 13 Jan 2020,
this package is only used to wrap the forward kinematics logic, as pyKDL doesn't work and C++
code is written which uses the normal KDL. This logic is then wrapped into a python function
such that we can perform forward kinematics from python from a urdf file.

The aim of this package is to generate `.so` shared libraries and then get python to import.