//
// Created by pohzhiee on 29/10/19.
//

#include "py_wrappers/forward_kinematics.hpp"
#include <functional>
#include <string>
#include "kdl/jntarray.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "urdf/model.h"


//ForwardKinematics::ForwardKinematics(const KDL::Tree &tree) : tree_(tree) {}

ForwardKinematics::ForwardKinematics(const std::string &urdf_path) {
    urdf::Model model;
    if (!model.initFile(urdf_path)) {
        fprintf(stderr, "Unable to initialize urdf::model from %s\n", urdf_path.c_str());
        return;
    }

    if (!kdl_parser::treeFromUrdfModel(model, tree_)) {
        fprintf(stderr, "Failed to extract kdl tree from xml robot description\n");
        return;
    }
}

Pose
ForwardKinematics::calculate(const std::string &root, const std::string &tip, const std::vector<double> &joint_states) {
    KDL::Chain chain;
    Pose pose{};
    auto getChainResult = tree_.getChain(root, tip, chain);
    if (!getChainResult) {
        fprintf(stderr, "Failed to get chain\n");
        return pose;
    }
    KDL::JntArray jointAngles = KDL::JntArray(joint_states.size());
    for(size_t i = 0;i<joint_states.size();i++){
        jointAngles(i) = joint_states[i];
    }
    KDL::ChainFkSolverPos_recursive FKSolver =
            KDL::ChainFkSolverPos_recursive(chain);
    KDL::Frame eeFrame;
    auto result = FKSolver.JntToCart(jointAngles, eeFrame);
    if(result != 0){
        fprintf(stderr, "FK solve fail\n");
    }

    double alpha, beta, gamma;
    eeFrame.M.GetEulerZYZ(alpha, beta, gamma);
    pose.translation.x = eeFrame.p.x();
    pose.translation.y = eeFrame.p.y();
    pose.translation.z = eeFrame.p.z();
    pose.rotation_euler.x = alpha;
    pose.rotation_euler.y = beta;
    pose.rotation_euler.z = gamma;
    return pose;
}
