#ifndef TRAJECTORY_OPTIMIZATION_HPP
#define TRAJECTORY_OPTIMIZATION_HPP
#include "TFSubscriberNode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "astar_msgs/msg/a_star_path_array.hpp"
#include "initial_optimized_msgs/msg/initial_optimized_trajectory.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "yaml-cpp/yaml.h"
#include "Eigen/Dense"
#include <LBFGS.h>
#include "utils/minco.hpp"


class Trajectory_Opimization_Node : public rclcpp::Node {
public:
    Trajectory_Opimization_Node(std::shared_ptr<TFSubscriberNode> tf_subscriber_node);

private:
    typedef struct{
        Eigen::Vector3d position;
    }A_Star_Path_;

    typedef struct{
        Eigen::MatrixX2d b;      
        Eigen::VectorXd times;
    }Initial_Optimized_Trajectory_;


    void Callback(const astar_msgs::msg::AStarPathArray::ConstSharedPtr& astar_path_msg);
    double ComputeCostAndGradient(const Eigen::VectorXd& params, Eigen::VectorXd& grad);
    double ComputeCostAndGradient_Position(Eigen::MatrixX3d& GradByPoints, Eigen::VectorXd& GradByTimes);
    void visualizeTrajectory(const Eigen::MatrixXd &b, const Eigen::VectorXd &times);
    Eigen::Vector3d generatePolynomialTrajectory(const Eigen::MatrixXd& coefficients, const Eigen::VectorXd& times, double t);
    void forwardT(const Eigen::VectorXd &tau, Eigen::VectorXd &T);
    template <typename EIGENVEC>
    void backwardT(const Eigen::VectorXd &T, EIGENVEC &tau);
    void forwardP(const Eigen::VectorXd &xi, Eigen::Matrix3Xd &P);
    template <typename EIGENVEC>
    void backwardP(const Eigen::Matrix3Xd &P, EIGENVEC &xi);
    template <typename EIGENVEC>
    void backwardGradT(const Eigen::VectorXd &tau, const Eigen::VectorXd &gradT, EIGENVEC &gradTau);
    template <typename EIGENVEC>
    void backwardGradP(const Eigen::VectorXd &xi, const Eigen::Matrix3Xd &gradP, EIGENVEC &gradXi);

    // Topic subscription
    rclcpp::Subscription<astar_msgs::msg::AStarPathArray>::SharedPtr Astar_Path_Subscriber_;

    // Topic publishing
    rclcpp::Publisher<initial_optimized_msgs::msg::InitialOptimizedTrajectory>::SharedPtr Initial_Optimized_Trajectory_Publisher_; // Publish initial optimized trajectory
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr Trajectory_Opimization_Publisher_;                          // Publish trajectory optimization markers

    std::shared_ptr<TFSubscriberNode> tf_subscriber_node_;  // Shared pointer to tf_subscriber_node

    std::string config_yaml_path;
    YAML::Node config;

    std::vector<A_Star_Path_> A_Star_Path;

    minco::MINCO_S3NU minco;
    int pieceN;
    int spatialDim;
    int temporalDim;
    Eigen::Matrix3Xd points;        
    Eigen::VectorXd times;

    Matrix Car_Odom_Matrix;      // Car -> World frame transform matrix

    Eigen::MatrixXd Car_Odom_Rotation_Translation_Matrix;
    std_msgs::msg::Header Header;

    double weight_time;
    double weight_energy_x;
    double weight_energy_y;
    double weight_energy_w;
    double weight_position_x;
    double weight_position_y;
    double weight_position_w;
};

#endif