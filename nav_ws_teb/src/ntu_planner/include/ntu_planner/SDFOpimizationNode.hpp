#ifndef SDF_OPTIMIZATION_HPP
#define SDF_OPTIMIZATION_HPP
#include "TFSubscriberNode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "initial_optimized_msgs/msg/initial_optimized_trajectory.hpp"
#include "sdf_optimized_msgs/msg/sdf_optimized_trajectory.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "yaml-cpp/yaml.h"
#include "Eigen/Dense"
#include <LBFGS.h>
#include "utils/minco.hpp"
#include "utils/lmbm.h"
#include "SDF.cuh"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "std_msgs/msg/float64_multi_array.hpp"
class SDF_Opimization_Node : public rclcpp::Node {
public:
    SDF_Opimization_Node(std::shared_ptr<TFSubscriberNode> tf_subscriber_node);

    static double costFunctionLmbmParallelWrapper(void *ptr, const double *x_variable, double *g, const int n) {
        SDF_Opimization_Node *node = static_cast<SDF_Opimization_Node *>(ptr);
        return node->costFunctionLmbmParallel(ptr, x_variable, g, n);
    }

    static int earlyExitLMBMWrapper(void *instance, const double *x, int k) {
        SDF_Opimization_Node *node = static_cast<SDF_Opimization_Node *>(instance);
        return node->earlyExitLMBM(instance, x, k);
    }

private:
    void Callback(const initial_optimized_msgs::msg::InitialOptimizedTrajectory::ConstSharedPtr& init_traj_msg);
    double costFunctionLmbmParallel(void *ptr, const double *x_variable, double *g, const int n);
    int earlyExitLMBM(void *instance, const double *x, const int k);
    void forwardT(const Eigen::VectorXd &tau, Eigen::VectorXd &T);
    template <typename EIGENVEC>
    void backwardT(const Eigen::VectorXd &T, EIGENVEC &tau);
    void forwardP(const Eigen::VectorXd &xi, Eigen::Matrix3Xd &P);
    template <typename EIGENVEC>
    void backwardP(const Eigen::Matrix3Xd &P, EIGENVEC &xi);
    void backwardGradT(const double* tau, const Eigen::VectorXd &gradT, double * gradTau, const int sizeTau);
    void backwardGradP(const Eigen::Matrix3Xd &gradP, double *gradXi, const int Space_dim);

    template <typename EIGENVEC>
    void backwardGradT(const Eigen::VectorXd &tau, const Eigen::VectorXd &gradT, EIGENVEC &gradTau);
    template <typename EIGENVEC>
    void backwardGradP(const Eigen::VectorXd &xi, const Eigen::Matrix3Xd &gradP, EIGENVEC &gradXi);
    void visualizeSweptVolume(std::vector<SDF_Map_> SDF_Map, int Point_Num);
    void visualizeTrajectory(const Eigen::MatrixXd &b, const Eigen::VectorXd &times);
    Eigen::Vector3d generatePolynomialTrajectory(const Eigen::MatrixXd& coefficients, const Eigen::VectorXd& times, double t);

    // Topic subscription
    rclcpp::Subscription<initial_optimized_msgs::msg::InitialOptimizedTrajectory>::SharedPtr Init_Traj_Subscriber_;

    // Topic publishing
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr Swept_Volume_Publisher_;             // Publish swept volume markers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr Trajectory_Opimization_Publisher_;   // Publish optimized trajectory markers
    rclcpp::Publisher<sdf_optimized_msgs::msg::SDFOptimizedTrajectory>::SharedPtr SDF_Opimization_Publisher_;   // Publish final optimized trajectory

    std::shared_ptr<TFSubscriberNode> tf_subscriber_node_;  // Shared pointer to tf_subscriber_node

    Optimized_Trajectory_ traj;
    std::string config_yaml_path;
    YAML::Node config;

    Matrix Car_Odom_Matrix;      // Car -> World frame transform matrix
    Matrix Odom_Car_Matrix;      // Car -> World frame transform matrix
    Eigen::MatrixXd Car_Odom_Rotation_Translation_Matrix;
    Eigen::MatrixXd Odom_Car_Rotation_Translation_Matrix;
    geometry_msgs::msg::Quaternion Car_Quaternion;
    std_msgs::msg::Header Header;

    minco::MINCO_S3NU minco;
    int spatialDim;
    int temporalDim;
    // double *x_variable;
    Eigen::Matrix3Xd Obstacle_Points;
    
    int opimization_times;
    int opimization_times_max;
    
    double weight_time;
    double weight_energy_x;
    double weight_energy_y;
    double weight_energy_w;
};

#endif