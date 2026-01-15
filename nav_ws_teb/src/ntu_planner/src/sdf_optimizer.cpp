#include "ntu_planner/sdf_optimizer.h"
#include <cmath>
#include <chrono>

namespace ntu_planner
{

SDFOptimizer::SDFOptimizer()
    : initialized_(false),
      piece_num_(0),
      spatial_dim_(0),
      temporal_dim_(0),
      optimization_times_(0)
{
}

void SDFOptimizer::initialize(ros::NodeHandle &nh)
{
    // 加载Find t_star参数
    nh.param("sdf_optimization/coarse_grid_size", coarse_grid_size_, 0.1);
    nh.param("sdf_optimization/find_t_star_time_step", find_t_star_time_step_, 0.1);
    nh.param("sdf_optimization/eta", eta_, 0.01);
    nh.param("sdf_optimization/c", c_, 0.1);
    nh.param("sdf_optimization/tol", tol_, 1e-6);
    nh.param("sdf_optimization/max_iter", max_iter_, 1000);

    // 加载Compute SDF参数
    nh.param("sdf_optimization/sdf_test", sdf_test_, false);
    nh.param("sdf_optimization/sdf_resolution", sdf_resolution_, 0.8);
    nh.param("sdf_optimization/sdf_safety_hor", sdf_safety_hor_, 1.0);

    // 加载Optimization参数
    nh.param("sdf_optimization/sdf_opimiz_times_max", sdf_opimiz_times_max_, 10000);
    nh.param("sdf_optimization/sdf_opimiz_weight_time", sdf_opimiz_weight_time_, 1.0);
    nh.param("sdf_optimization/sdf_opimiz_weight_energy_x", sdf_opimiz_weight_energy_x_, 1.0);
    nh.param("sdf_optimization/sdf_opimiz_weight_energy_y", sdf_opimiz_weight_energy_y_, 1.0);
    nh.param("sdf_optimization/sdf_opimiz_weight_energy_w", sdf_opimiz_weight_energy_w_, 1.0);
    nh.param("sdf_optimization/sdf_opimiz_weight_swept_volume", sdf_opimiz_weight_swept_volume_, 0.1);
    nh.param("sdf_optimization/sdf_opimiz_weight_safety", sdf_opimiz_weight_safety_, 1.0);

    ROS_INFO("SDFOptimizer initialized with parameters:");
    ROS_INFO("  coarse_grid_size: %.3f", coarse_grid_size_);
    ROS_INFO("  sdf_resolution: %.3f", sdf_resolution_);
    ROS_INFO("  sdf_opimiz_times_max: %d", sdf_opimiz_times_max_);
    ROS_INFO("  sdf_opimiz_weight_time: %.3f", sdf_opimiz_weight_time_);
    ROS_INFO("  sdf_opimiz_weight_swept_volume: %.3f", sdf_opimiz_weight_swept_volume_);
    ROS_INFO("  sdf_opimiz_weight_safety: %.3f", sdf_opimiz_weight_safety_);

    // 创建可视化发布器
    trajectory_viz_pub_ = nh.advertise<visualization_msgs::MarkerArray>("sdf_optimized_trajectory_viz", 1);
    swept_volume_pub_ = nh.advertise<visualization_msgs::MarkerArray>("swept_volume_viz", 1);

    // 创建SDF配置结构体（用于GPU处理）
    SDFConfig config;
    config.coarse_grid_size = coarse_grid_size_;
    config.find_t_star_time_step = find_t_star_time_step_;
    config.eta = eta_;
    config.c = c_;
    config.tol = tol_;
    config.max_iter = max_iter_;
    config.sdf_resolution = sdf_resolution_;
    config.sdf_safety_hor = sdf_safety_hor_;
    config.weight_safety = sdf_opimiz_weight_safety_;
    config.weight_swept_volume = sdf_opimiz_weight_swept_volume_;

    // 初始化GPU配置
    GPUProcessConfig(config);

    initialized_ = true;
}

bool SDFOptimizer::optimizePath(const Eigen::Matrix3Xd &initial_points,
                                 const Eigen::VectorXd &initial_times,
                                 const Eigen::Matrix3Xd &obstacle_points,
                                 Eigen::Matrix3Xd &optimized_points,
                                 Eigen::VectorXd &optimized_times)
{
    if (!initialized_)
    {
        ROS_ERROR("SDFOptimizer not initialized!");
        return false;
    }

    if (initial_points.cols() == 0)
    {
        ROS_ERROR("Initial points empty!");
        return false;
    }

    // 保存障碍物点云
    obstacle_points_ = obstacle_points;

    // 设置轨迹参数
    piece_num_ = initial_points.cols() + 1;
    temporal_dim_ = piece_num_;
    spatial_dim_ = 3 * (piece_num_ - 1);

    traj_.pieceN = piece_num_;
    traj_.b = Eigen::MatrixX3d::Zero(6 * piece_num_, 3);
    traj_.times = initial_times;
    traj_.points = initial_points;

    // 设置边界条件（使用初始轨迹的起点和终点）
    Eigen::Matrix3d init_state = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d final_state = Eigen::Matrix3d::Zero();
    init_state.col(0) = initial_points.col(0);
    final_state.col(0) = initial_points.col(initial_points.cols() - 1);

    minco_.setConditions(init_state, final_state, piece_num_);
    minco_.setParameters(traj_.points, traj_.times);
    traj_.b = minco_.b;

    // 初始化优化变量
    const int total_opt_variable_num = temporal_dim_ + spatial_dim_;
    Eigen::VectorXd x(total_opt_variable_num);
    Eigen::Map<Eigen::VectorXd> tau(x.data(), temporal_dim_);
    Eigen::Map<Eigen::VectorXd> xi(x.data() + temporal_dim_, spatial_dim_);

    // 初始化时间参数
    Eigen::VectorXd tau_temp;
    backwardT(traj_.times, tau_temp);
    tau = tau_temp;

    // 初始化空间参数
    for (int i = 0; i < piece_num_ - 1; ++i)
    {
        xi.segment(3 * i, 3) = traj_.points.col(i);
    }

    // LMBM优化
    optimization_times_ = 0;
    lmbm::lmbm_parameter_t param;
    double final_cost;

    ROS_INFO("Starting SDF optimization...");
    auto start_time = std::chrono::high_resolution_clock::now();

    int ret = lmbm::lmbm_optimize(
        total_opt_variable_num,
        x.data(),
        &final_cost,
        costFunctionLmbmWrapper,
        this,
        earlyExitLMBMWrapper,
        &param);

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

    // 提取优化结果
    forwardT(tau, traj_.times);
    forwardP(xi, traj_.points);

    ROS_INFO("SDF optimization return code: %d", ret);

    if (ret > 0)
    {
        ROS_INFO("SDF optimization successful! Process took %.3f s", duration / 1000000.0);

        // 设置最终参数
        minco_.setParameters(traj_.points, traj_.times);
        traj_.b = minco_.b;

        // 计算扫掠体积
        std::vector<SDF_Map_> SDF_Map;
        double area = 0;
        GPUProcessSDF(traj_, SDF_Map, area, sdf_test_);
        ROS_INFO("Swept area: %.3f", area);

        // 输出优化结果
        optimized_points = traj_.points;
        optimized_times = traj_.times;

        return true;
    }
    else
    {
        ROS_WARN("SDF optimization unsuccessful! Process took %.3f s", duration / 1000000.0);
        return false;
    }
}

double SDFOptimizer::costFunctionLmbmWrapper(void *ptr, const double *x_variable, double *g, const int n)
{
    SDFOptimizer *optimizer = static_cast<SDFOptimizer *>(ptr);
    return optimizer->costFunctionLmbmParallel(ptr, x_variable, g, n);
}

int SDFOptimizer::earlyExitLMBMWrapper(void *instance, const double *x, int k)
{
    SDFOptimizer *optimizer = static_cast<SDFOptimizer *>(instance);
    return optimizer->earlyExitLMBM(instance, x, k);
}

double SDFOptimizer::costFunctionLmbmParallel(void *ptr, const double *x_variable, double *g, const int n)
{
    const int dimTau = temporal_dim_;
    const int dimXi = spatial_dim_;

    Eigen::Map<const Eigen::VectorXd> tau(x_variable, dimTau);
    Eigen::Map<const Eigen::VectorXd> xi(x_variable + dimTau, dimXi);
    Eigen::Map<Eigen::VectorXd> gradTau(g, dimTau);
    Eigen::Map<Eigen::VectorXd> gradXi(g + dimTau, dimXi);

    // 前向变换
    forwardT(tau, traj_.times);
    forwardP(xi, traj_.points);

    double cost;
    Eigen::MatrixX3d GradByCoeffs(6 * traj_.pieceN, 3);
    Eigen::VectorXd GradByTimes(traj_.pieceN);
    Eigen::Matrix3Xd gradByPoints;
    Eigen::VectorXd gradByTimes;

    // 设置MINCO参数
    minco_.setParameters(traj_.points, traj_.times);
    traj_.b = minco_.b;

    // 1. 能量代价
    minco_.getEnergy(cost, sdf_opimiz_weight_energy_x_, sdf_opimiz_weight_energy_y_, sdf_opimiz_weight_energy_w_);
    minco_.getEnergyPartialGradByCoeffs(GradByCoeffs, sdf_opimiz_weight_energy_x_, sdf_opimiz_weight_energy_y_, sdf_opimiz_weight_energy_w_);
    minco_.getEnergyPartialGradByTimes(GradByTimes, sdf_opimiz_weight_energy_x_, sdf_opimiz_weight_energy_y_, sdf_opimiz_weight_energy_w_);
    minco_.propogateGrad(GradByCoeffs, GradByTimes, gradByPoints, gradByTimes);

    // 2. 时间代价
    cost += sdf_opimiz_weight_time_ * traj_.times.sum();
    gradByTimes.array() += sdf_opimiz_weight_time_;

    // 3. 障碍物代价（通过GPU计算SDF梯度）
    Eigen::MatrixX3d GradByPoints_Ob = Eigen::MatrixX3d::Zero(traj_.pieceN, 3);
    Eigen::VectorXd GradByTimes_Ob = Eigen::VectorXd::Zero(traj_.pieceN);
    double cost_Ob;
    GPUProcessGradSDF(traj_, obstacle_points_, GradByPoints_Ob, GradByTimes_Ob, cost_Ob);
    gradByPoints += GradByPoints_Ob.topRows(GradByPoints_Ob.rows() - 1).transpose();
    gradByTimes += GradByTimes_Ob;
    cost += cost_Ob;

    // 4. 航向角代价（通过GPU计算）
    Eigen::MatrixX3d GradByPoints_Yaw = Eigen::MatrixX3d::Zero(traj_.pieceN, 3);
    Eigen::VectorXd GradByTimes_Yaw = Eigen::VectorXd::Zero(traj_.pieceN);
    double cost_Yaw;
    GPUProcessGradYaw(traj_, GradByPoints_Yaw, GradByTimes_Yaw, cost_Yaw);
    gradByPoints += GradByPoints_Yaw.topRows(GradByPoints_Yaw.rows() - 1).transpose();
    gradByTimes += GradByTimes_Yaw;
    cost += cost_Yaw;

    // 反向传播梯度
    Eigen::VectorXd gradTau_temp, gradXi_temp;
    backwardGradP(xi, gradByPoints, gradXi_temp);
    backwardGradT(tau, gradByTimes, gradTau_temp);
    gradTau = gradTau_temp;
    gradXi = gradXi_temp;

    return cost;
}

int SDFOptimizer::earlyExitLMBM(void *instance, const double *x, const int k)
{
    optimization_times_++;
    return optimization_times_ > sdf_opimiz_times_max_;
}

// ==================== 参数变换函数 ====================

void SDFOptimizer::forwardT(const Eigen::VectorXd &tau, Eigen::VectorXd &T)
{
    const int size_tau = tau.size();
    T.resize(size_tau);
    for (int i = 0; i < size_tau; ++i)
    {
        T(i) = tau(i) > 0.0
                   ? ((0.5 * tau(i) + 1.0) * tau(i) + 1.0)
                   : 1.0 / ((0.5 * tau(i) - 1.0) * tau(i) + 1.0);
    }
}

void SDFOptimizer::backwardT(const Eigen::VectorXd &T, Eigen::VectorXd &tau)
{
    const int size_T = T.size();
    tau.resize(size_T);
    for (int i = 0; i < size_T; ++i)
    {
        tau(i) = T(i) > 1.0
                     ? (sqrt(2.0 * T(i) - 1.0) - 1.0)
                     : (1.0 - sqrt(2.0 / T(i) - 1.0));
    }
}

void SDFOptimizer::forwardP(const Eigen::VectorXd &xi, Eigen::Matrix3Xd &P)
{
    const int size_P = xi.size() / 3;
    P.resize(3, size_P);
    for (int i = 0; i < size_P; ++i)
    {
        P.col(i) = xi.segment(3 * i, 3);
    }
}

void SDFOptimizer::backwardP(const Eigen::Matrix3Xd &P, Eigen::VectorXd &xi)
{
    const int size_P = P.cols();
    xi.resize(3 * size_P);
    for (int i = 0; i < size_P; ++i)
    {
        xi.segment(3 * i, 3) = P.col(i);
    }
}

void SDFOptimizer::backwardGradT(const Eigen::VectorXd &tau,
                                  const Eigen::VectorXd &gradT,
                                  Eigen::VectorXd &gradTau)
{
    const int size_tau = tau.size();
    gradTau.resize(size_tau);
    double den_sqrt;

    for (int i = 0; i < size_tau; ++i)
    {
        if (tau(i) > 0)
        {
            gradTau(i) = gradT(i) * (tau(i) + 1.0);
        }
        else
        {
            den_sqrt = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
            gradTau(i) = gradT(i) * (1.0 - tau(i)) / (den_sqrt * den_sqrt);
        }
    }
}

void SDFOptimizer::backwardGradP(const Eigen::VectorXd &xi,
                                  const Eigen::Matrix3Xd &gradP,
                                  Eigen::VectorXd &gradXi)
{
    const int size_P = gradP.cols();
    gradXi.resize(3 * size_P);
    for (int i = 0; i < size_P; ++i)
    {
        gradXi.segment(3 * i, 3) = gradP.col(i);
    }
}

// ==================== 可视化函数 ====================

Eigen::Vector3d SDFOptimizer::generatePolynomialTrajectory(const Eigen::MatrixXd &coefficients,
                                                            const Eigen::VectorXd &times,
                                                            double t)
{
    int current_segment = 0;
    double segment_start_time = 0.0;

    // 确定当前时间点属于哪个轨迹段
    while (current_segment < times.size() && t > (segment_start_time + times[current_segment]))
    {
        segment_start_time += times[current_segment];
        current_segment++;
    }

    // 如果时间超过了所有轨迹段的总时间，则返回最后一个点的位置
    if (current_segment >= times.size())
    {
        current_segment = times.size() - 1;
        t = segment_start_time;
    }

    // 计算局部时间（相对于当前轨迹段的时间）
    double local_t = t - segment_start_time;

    // 根据当前轨迹段的多项式系数计算位置
    Eigen::MatrixXd current_coeff = coefficients.block(6 * current_segment, 0, 6, 3);

    double x = 0.0, y = 0.0;
    for (int j = 0; j < 6; ++j)
    {
        x += current_coeff(j, 0) * std::pow(local_t, j);
        y += current_coeff(j, 1) * std::pow(local_t, j);
    }

    return Eigen::Vector3d(x, y, 0.0);
}

void SDFOptimizer::visualizeTrajectory(const std::string &frame_id)
{
    if (trajectory_viz_pub_.getNumSubscribers() == 0)
        return;

    visualization_msgs::MarkerArray marker_array;

    // 删除旧标记
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = frame_id;
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);
    trajectory_viz_pub_.publish(marker_array);
    marker_array.markers.clear();

    // 生成新的轨迹标记
    double total_duration = traj_.times.sum();
    int ID = 0;

    for (double t = 0.0; t <= total_duration; t += 0.05)
    {
        Eigen::Vector3d point = generatePolynomialTrajectory(traj_.b, traj_.times, t);

        visualization_msgs::Marker point_marker;
        point_marker.header.frame_id = frame_id;
        point_marker.header.stamp = ros::Time::now();
        point_marker.ns = "sdf_trajectory";
        point_marker.type = visualization_msgs::Marker::SPHERE;
        point_marker.action = visualization_msgs::Marker::ADD;
        point_marker.id = ID++;

        point_marker.scale.x = 0.1;
        point_marker.scale.y = 0.1;
        point_marker.scale.z = 0.1;

        // 白色表示SDF优化后的轨迹
        point_marker.color.r = 1.0;
        point_marker.color.g = 1.0;
        point_marker.color.b = 1.0;
        point_marker.color.a = 1.0;

        point_marker.pose.position.x = point.x();
        point_marker.pose.position.y = point.y();
        point_marker.pose.position.z = 0.3;

        point_marker.pose.orientation.x = 0.0;
        point_marker.pose.orientation.y = 0.0;
        point_marker.pose.orientation.z = 0.0;
        point_marker.pose.orientation.w = 1.0;

        marker_array.markers.push_back(point_marker);
    }

    trajectory_viz_pub_.publish(marker_array);
    ROS_INFO("Published SDF optimized trajectory visualization with %d markers", ID);
}

void SDFOptimizer::visualizeSweptVolume(const std::string &frame_id)
{
    if (swept_volume_pub_.getNumSubscribers() == 0)
        return;

    // 计算扫掠体积
    std::vector<SDF_Map_> SDF_Map;
    double area = 0;
    GPUProcessSDF(traj_, SDF_Map, area, sdf_test_);

    visualization_msgs::MarkerArray marker_array;

    // 删除旧标记
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = frame_id;
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);
    swept_volume_pub_.publish(marker_array);
    marker_array.markers.clear();

    int ID = 0;
    for (size_t i = 0; i < SDF_Map.size(); ++i)
    {
        double f_sdf_min = SDF_Map[i].SDF;

        // 检查是否在物体内部（负值表示在内部）
        if (f_sdf_min <= 0)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = frame_id;
            marker.header.stamp = ros::Time::now();
            marker.ns = "swept_volume";
            marker.id = ID++;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position.x = SDF_Map[i].x;
            marker.pose.position.y = SDF_Map[i].y;
            marker.pose.position.z = 0.0;

            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = sdf_resolution_;
            marker.scale.y = sdf_resolution_;
            marker.scale.z = sdf_resolution_;

            marker.color.a = 0.8;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;

            marker_array.markers.push_back(marker);
        }
    }

    swept_volume_pub_.publish(marker_array);
    ROS_INFO("Published swept volume visualization with %d markers", ID);
}

void SDFOptimizer::updateParameters(
    double coarse_grid_size, double find_t_star_time_step, double eta, double c,
    double tol, int max_iter, bool sdf_test, double sdf_resolution,
    double sdf_safety_hor, int sdf_opimiz_times_max, double sdf_opimiz_weight_time,
    double sdf_opimiz_weight_energy_x, double sdf_opimiz_weight_energy_y,
    double sdf_opimiz_weight_energy_w, double sdf_opimiz_weight_swept_volume,
    double sdf_opimiz_weight_safety)
{
    // 更新所有参数
    coarse_grid_size_ = coarse_grid_size;
    find_t_star_time_step_ = find_t_star_time_step;
    eta_ = eta;
    c_ = c;
    tol_ = tol;
    max_iter_ = max_iter;
    sdf_test_ = sdf_test;
    sdf_resolution_ = sdf_resolution;
    sdf_safety_hor_ = sdf_safety_hor;
    sdf_opimiz_times_max_ = sdf_opimiz_times_max;
    sdf_opimiz_weight_time_ = sdf_opimiz_weight_time;
    sdf_opimiz_weight_energy_x_ = sdf_opimiz_weight_energy_x;
    sdf_opimiz_weight_energy_y_ = sdf_opimiz_weight_energy_y;
    sdf_opimiz_weight_energy_w_ = sdf_opimiz_weight_energy_w;
    sdf_opimiz_weight_swept_volume_ = sdf_opimiz_weight_swept_volume;
    sdf_opimiz_weight_safety_ = sdf_opimiz_weight_safety;

    // 更新GPU配置
    SDFConfig config;
    config.coarse_grid_size = coarse_grid_size_;
    config.find_t_star_time_step = find_t_star_time_step_;
    config.eta = eta_;
    config.c = c_;
    config.tol = tol_;
    config.max_iter = max_iter_;
    config.sdf_resolution = sdf_resolution_;
    config.sdf_safety_hor = sdf_safety_hor_;
    config.weight_safety = sdf_opimiz_weight_safety_;
    config.weight_swept_volume = sdf_opimiz_weight_swept_volume_;
    GPUProcessConfig(config);

    ROS_INFO("SDFOptimizer parameters updated via dynamic_reconfigure:");
    ROS_INFO("  coarse_grid_size: %.3f", coarse_grid_size_);
    ROS_INFO("  sdf_resolution: %.3f", sdf_resolution_);
    ROS_INFO("  sdf_opimiz_times_max: %d", sdf_opimiz_times_max_);
    ROS_INFO("  weights: time=%.1f, swept_volume=%.1f, safety=%.1f",
             sdf_opimiz_weight_time_, sdf_opimiz_weight_swept_volume_, sdf_opimiz_weight_safety_);
}

} // namespace ntu_planner
