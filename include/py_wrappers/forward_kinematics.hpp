//
// Created by pohzhiee on 29/10/19.
//

#ifndef PY_WRAPPERS_FORWARD_KINEMATICS_HPP
#define PY_WRAPPERS_FORWARD_KINEMATICS_HPP

#include "kdl/tree.hpp"
#include <string>


struct RotationQuaternion {
    double x;
    double y;
    double z;
    double w;
};
struct Vector {
    double x;
    double y;
    double z;
};
struct RotationEuler {
    double x;
    double y;
    double z;
};

struct Pose {
    Vector translation;
    Vector rotation_euler;
};

class ForwardKinematics {
public:
//    explicit ForwardKinematics(const KDL::Tree &tree);
//    ForwardKinematics();
    explicit ForwardKinematics(const std::string &urdf_path);

    Pose calculate(const std::string &root, const std::string &tip, const std::vector<double> &joint_states);


private:
    KDL::Tree tree_;
};


#endif //PY_WRAPPERS_FORWARD_KINEMATICS_HPP
