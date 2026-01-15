#ifndef NTU_PLANNER__SDF_OPTIMIZER_H_
#define NTU_PLANNER__SDF_OPTIMIZER_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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
     * @param robot_base_frame 机器人坐标系名称
     * @param global_frame 全局坐标系名称
     */
    void initialize(ros::NodeHandle &nh, const std::string &robot_base_frame, const std::string &global_frame);

    /**
     * @brief SDF优化
     * @param initial_points 第一次优化后的参考点 (3 x N 矩阵)
     * @param initial_times 第一次优化后的时间参数
     * @param obstacle_points 障碍物点云 (3 x M 矩阵)
     * @param optimized_points 输出：SDF优化后的参考点
     * @param optimized_times 输出：SDF优化后的时间参数
     * @param init_state 初始状态（位置、速度、加速度）
     * @param final_state 终止状态（位置、速度、加速度）
     * @param frame_id 坐标系名称（用于可视化）
     * @return 是否优化成功
     */
    bool optimizePath(const Eigen::Matrix3Xd &initial_points,
                      const Eigen::VectorXd &initial_times,
                      const Eigen::Matrix3Xd &obstacle_points,
                      Eigen::Matrix3Xd &optimized_points,
                      Eigen::VectorXd &optimized_times,
                      const Eigen::Matrix3d &init_state,
                      const Eigen::Matrix3d &final_state,
                      const std::string &frame_id = "map");

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
        double tol, int max_iter, double sdf_resolution,
        double sdf_safety_hor, int sdf_opimiz_times_max, double sdf_opimiz_weight_time,
        double sdf_opimiz_weight_energy_x, double sdf_opimiz_weight_energy_y,
        double sdf_opimiz_weight_energy_w, double sdf_opimiz_weight_yaw,
        double sdf_opimiz_weight_safety, double width, double height,
        const std::string& footprint,
        bool viz_only_control_points, double viz_time_step);

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
    double sdf_resolution_;              // SDF分辨率
    double sdf_safety_hor_;              // 安全距离

    // Optimization 参数
    int sdf_opimiz_times_max_;           // 最大优化次数
    double sdf_opimiz_weight_time_;      // 时间权重
    double sdf_opimiz_weight_energy_x_;  // X方向能量权重
    double sdf_opimiz_weight_energy_y_;  // Y方向能量权重
    double sdf_opimiz_weight_energy_w_;  // W方向能量权重
    double sdf_opimiz_weight_yaw_;  // 扫掠体积权重
    double sdf_opimiz_weight_safety_;    // 安全性权重

    // Costmap参数
    double width_;                       // 局部costmap宽度
    double height_;                      // 局部costmap高度

    // 车辆footprint（字符串格式）
    std::string footprint_;
    // 解析后的footprint（4个顶点）
    std::vector<std::pair<double, double>> parsed_footprint_;
    
    // 可视化设置
    bool viz_only_control_points_;  // 是否仅显示控制点
    double viz_time_step_;           // 可视化时间步长（秒）

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
     * @brief 解析footprint字符串为4个顶点的数组
     * @return 解析是否成功
     */
    bool parseFootprint(const std::string& footprint_str);

    /**
     * @brief 从 MINCO 多项式生成轨迹点
     */
    Eigen::Vector3d generatePolynomialTrajectory(const Eigen::MatrixXd &coefficients,
                                                  const Eigen::VectorXd &times,
                                                  double t);

    /**
     * @brief 使用TF获取当前机器人位姿，并将轨迹转换到局部坐标系
     * @param global_traj 全局坐标系的轨迹
     * @param local_traj 输出：局部坐标系的轨迹
     * @param origin_x 输出：局部坐标系原点x（全局坐标）
     * @param origin_y 输出：局部坐标系原点y（全局坐标）
     * @param origin_yaw 输出：局部坐标系朝向（全局坐标）
     * @return 是否成功获取TF变换
     */
    bool convertToLocalFrameUsingTF(const Optimized_Trajectory_ &global_traj,
                                    Optimized_Trajectory_ &local_traj,
                                    double &origin_x,
                                    double &origin_y,
                                    double &origin_yaw);

    // TF相关成员
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string robot_base_frame_;  // 机器人基座坐标系名称
    std::string global_frame_;       // 全局坐标系名称
};

} // namespace ntu_planner

#endif // NTU_PLANNER__SDF_OPTIMIZER_H_
