#ifndef NTU_PLANNER__SDF_OPTIMIZER_H_
#define NTU_PLANNER__SDF_OPTIMIZER_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <vector>
#include "utils/minco.hpp"
#include "utils/lmbm.h"
#include "SDF.cuh"

namespace ntu_planner
{

/**
 * @brief SDF优化器类
 *
 * 功能：对第一次优化后的轨迹进行SDF优化
 * 输入：第一次优化的结果（MINCO参考点和时间参数）
 * 输出：SDF优化后的轨迹
 */
class SDFOptimizer
{
public:
    SDFOptimizer();
    ~SDFOptimizer() = default;

    /**
     * @brief 初始化优化器参数
     * @param nh 节点句柄，用于获取参数
     */
    void initialize(ros::NodeHandle &nh);

    /**
     * @brief SDF优化
     * @param initial_points 第一次优化后的参考点 (3 x N 矩阵)
     * @param initial_times 第一次优化后的时间参数
     * @param obstacle_points 障碍物点云 (3 x M 矩阵)
     * @param optimized_points 输出：SDF优化后的参考点
     * @param optimized_times 输出：SDF优化后的时间参数
     * @return 是否优化成功
     */
    bool optimizePath(const Eigen::Matrix3Xd &initial_points,
                      const Eigen::VectorXd &initial_times,
                      const Eigen::Matrix3Xd &obstacle_points,
                      Eigen::Matrix3Xd &optimized_points,
                      Eigen::VectorXd &optimized_times);

    /**
     * @brief 获取 MINCO 系数（用于轨迹生成和可视化）
     */
    const Eigen::MatrixXd& getMincoCoefficients() const { return minco_.b; }

    /**
     * @brief 获取优化后的时间参数
     */
    const Eigen::VectorXd& getOptimizedTimes() const { return traj_.times; }

    /**
     * @brief 可视化优化后的轨迹和扫掠体积
     */
    void visualizeTrajectory(const std::string &frame_id);
    void visualizeSweptVolume(const std::string &frame_id);

    /**
     * @brief 更新SDF优化器参数（从 dynamic_reconfigure 回调调用）
     */
    void updateParameters(
        double coarse_grid_size, double find_t_star_time_step, double eta, double c,
        double tol, int max_iter, bool sdf_test, double sdf_resolution,
        double sdf_safety_hor, int sdf_opimiz_times_max, double sdf_opimiz_weight_time,
        double sdf_opimiz_weight_energy_x, double sdf_opimiz_weight_energy_y,
        double sdf_opimiz_weight_energy_w, double sdf_opimiz_weight_swept_volume,
        double sdf_opimiz_weight_safety);

    // LMBM包装函数（静态函数，供LMBM调用）
    static double costFunctionLmbmWrapper(void *ptr, const double *x_variable, double *g, const int n);
    static int earlyExitLMBMWrapper(void *instance, const double *x, int k);

private:
    // MINCO 对象
    minco::MINCO_S3NU minco_;

    // 轨迹结构
    Optimized_Trajectory_ traj_;

    // 优化参数
    int piece_num_;              // 轨迹段数
    int spatial_dim_;            // 空间维度 (3 * (piece_num - 1))
    int temporal_dim_;           // 时间维度 (piece_num)

    // Find t_star 参数
    double coarse_grid_size_;           // 粗网格大小
    double find_t_star_time_step_;      // 时间步长
    double eta_;                         // 学习率
    double c_;                           // Armijo控制参数
    double tol_;                         // 容差
    int max_iter_;                       // 最大迭代次数

    // Compute SDF 参数
    bool sdf_test_;                      // SDF测试模式
    double sdf_resolution_;              // SDF分辨率
    double sdf_safety_hor_;              // 安全距离

    // Optimization 参数
    int sdf_opimiz_times_max_;           // 最大优化次数
    double sdf_opimiz_weight_time_;      // 时间权重
    double sdf_opimiz_weight_energy_x_;  // X方向能量权重
    double sdf_opimiz_weight_energy_y_;  // Y方向能量权重
    double sdf_opimiz_weight_energy_w_;  // W方向能量权重
    double sdf_opimiz_weight_swept_volume_;  // 扫掠体积权重
    double sdf_opimiz_weight_safety_;    // 安全性权重

    // 内部数据
    Eigen::Matrix3Xd obstacle_points_;   // 障碍物点云
    int optimization_times_;             // 当前优化次数

    bool initialized_;

    // ROS 发布器
    ros::Publisher trajectory_viz_pub_;     // 轨迹可视化发布器
    ros::Publisher swept_volume_pub_;       // 扫掠体积可视化发布器

    // 辅助函数
    /**
     * @brief 代价函数及其梯度计算
     */
    double costFunctionLmbmParallel(void *ptr, const double *x_variable, double *g, const int n);

    /**
     * @brief 提前退出检查
     */
    int earlyExitLMBM(void *instance, const double *x, const int k);

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
};

} // namespace ntu_planner

#endif // NTU_PLANNER__SDF_OPTIMIZER_H_
