#ifndef NTU_PLANNER__TRAJECTORY_OPTIMIZER_H_
#define NTU_PLANNER__TRAJECTORY_OPTIMIZER_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <vector>
#include <LBFGS.h>
#include "utils/minco.hpp"
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>

namespace ntu_planner
{

/**
 * @brief 轨迹优化器类
 *
 * 功能：将全局路径转换为 MINCO 表示，并进行一级优化
 * 输出：优化后的 MINCO 参考点和时间参数
 */
class TrajectoryOptimizer
{
public:
    TrajectoryOptimizer();
    ~TrajectoryOptimizer() = default;

    /**
     * @brief 初始化优化器参数
     * @param nh 节点句柄，用于获取参数
     */
    void initialize(ros::NodeHandle &nh);

    /**
     * @brief 优化全局路径
     * @param global_plan 输入的全局路径（来自 NavFN）
     * @param optimized_points 输出：优化后的参考点 (3 x N 矩阵)
     * @param optimized_times 输出：每段的时间参数
     * @return 是否优化成功
     */
    bool optimizePath(const std::vector<geometry_msgs::PoseStamped> &global_plan,
                      Eigen::Matrix3Xd &optimized_points,
                      Eigen::VectorXd &optimized_times,
                      Eigen::Matrix3d &init_state,
                      Eigen::Matrix3d &final_state);

    /**
     * @brief 获取 MINCO 系数（用于轨迹生成和可视化）
     * @return MINCO 系数矩阵
     */
    const Eigen::MatrixXd& getMincoCoefficients() const { return minco_.b; }

    /**
     * @brief 获取优化后的时间参数
     */
    const Eigen::VectorXd& getOptimizedTimes() const { return times_; }

    /**
     * @brief 可视化优化后的轨迹
     * @param frame_id 坐标系 ID
     */
    void visualizeTrajectory(const std::string &frame_id);

    /**
     * @brief 更新轨迹优化器参数（从 dynamic_reconfigure 回调调用）
     */
    void updateParameters(
        double weight_time, double weight_energy_x, double weight_energy_y, double weight_energy_w,
        double weight_position_x, double weight_position_y, double weight_position_w,
        double init_time, int astar_point_interval, int max_iterations,
        double min_step, double epsilon,
        bool viz_only_control_points, double viz_time_step);

private:
    // MINCO 对象
    minco::MINCO_S3NU minco_;

    // 优化参数
    int piece_num_;              // 轨迹段数
    int spatial_dim_;            // 空间维度 (3 * (piece_num - 1))
    int temporal_dim_;           // 时间维度 (piece_num)

    // 权重参数（从 ROS 参数服务器加载）
    double weight_time_;         // 时间权重
    double weight_energy_x_;     // X 方向能量权重
    double weight_energy_y_;     // Y 方向能量权重
    double weight_energy_w_;     // W 方向能量权重（航向角）
    double weight_position_x_;   // X 方向位置权重
    double weight_position_y_;   // Y 方向位置权重
    double weight_position_w_;   // W 方向位置权重

    // 优化设置
    double init_time_;           // 初始时间间隔（秒）
    int astar_point_interval_;   // 路径采样间隔
    int max_iterations_;         // 最大迭代次数
    double min_step_;            // LBFGS 最小步长
    double epsilon_;             // 收敛阈值

    // 可视化设置
    bool viz_only_control_points_;  // 是否仅显示控制点
    double viz_time_step_;           // 可视化时间步长（秒）

    // 内部数据
    Eigen::Matrix3Xd points_;    // 当前优化的点
    Eigen::VectorXd times_;      // 当前优化的时间
    std::vector<Eigen::Vector3d> sampled_path_;  // 采样后的路径点

    bool initialized_;

    // ROS 发布器
    ros::Publisher trajectory_viz_pub_;  // 轨迹可视化发布器

    // 辅助函数
    /**
     * @brief 从全局路径中采样点
     */
    void samplePath(const std::vector<geometry_msgs::PoseStamped> &global_plan);

    /**
     * @brief 代价函数及其梯度计算
     */
    double computeCostAndGradient(const Eigen::VectorXd &params, Eigen::VectorXd &grad);

    /**
     * @brief 位置代价函数及其梯度
     */
    double computePositionCost(Eigen::MatrixX3d &grad_by_points, Eigen::VectorXd &grad_by_times);

    // 参数变换函数（tau <-> T, xi <-> P）
    void forwardT(const Eigen::VectorXd &tau, Eigen::VectorXd &T);
    void backwardT(const Eigen::VectorXd &T, Eigen::VectorXd &tau);
    void forwardP(const Eigen::VectorXd &xi, Eigen::Matrix3Xd &P);
    void backwardP(const Eigen::Matrix3Xd &P, Eigen::VectorXd &xi);
    void backwardGradT(const Eigen::VectorXd &tau, const Eigen::VectorXd &gradT, Eigen::VectorXd &gradTau);
    void backwardGradP(const Eigen::VectorXd &xi, const Eigen::Matrix3Xd &gradP, Eigen::VectorXd &gradXi);

    /**
     * @brief 从 MINCO 多项式生成轨迹点
     */
    Eigen::Vector3d generatePolynomialTrajectory(const Eigen::MatrixXd &coefficients,
                                                  const Eigen::VectorXd &times,
                                                  double t);

    /**
     * @brief 创建原始路径的简单 MINCO 表示（fallback）
     */
    bool createFallbackTrajectory(const std::vector<geometry_msgs::PoseStamped> &global_plan,
                                    Eigen::Matrix3Xd &fallback_points,
                                    Eigen::VectorXd &fallback_times);
};

} // namespace ntu_planner

#endif // NTU_PLANNER__TRAJECTORY_OPTIMIZER_H_
