#include "ntu_planner/trajectory_optimizer.h"
#include <cmath>

namespace ntu_planner
{

TrajectoryOptimizer::TrajectoryOptimizer()
    : initialized_(false),
      piece_num_(0),
      spatial_dim_(0),
      temporal_dim_(0)
{
}

void TrajectoryOptimizer::initialize(ros::NodeHandle &nh)
{
    // 加载参数
    nh.param("trajectory_optimizer/weight_time", weight_time_, 0.1);
    nh.param("trajectory_optimizer/weight_energy_x", weight_energy_x_, 1.0);
    nh.param("trajectory_optimizer/weight_energy_y", weight_energy_y_, 1.0);
    nh.param("trajectory_optimizer/weight_energy_w", weight_energy_w_, 0.1);
    nh.param("trajectory_optimizer/weight_position_x", weight_position_x_, 100.0);
    nh.param("trajectory_optimizer/weight_position_y", weight_position_y_, 100.0);
    nh.param("trajectory_optimizer/weight_position_w", weight_position_w_, 1.0);
    nh.param("trajectory_optimizer/init_time", init_time_, 0.5);
    nh.param("trajectory_optimizer/astar_point_interval", astar_point_interval_, 2);
    nh.param("trajectory_optimizer/max_iterations", max_iterations_, 1000);
    nh.param("trajectory_optimizer/min_step", min_step_, 1e-5);
    nh.param("trajectory_optimizer/epsilon", epsilon_, 1e-5);
    nh.param("trajectory_optimizer/viz_only_control_points", viz_only_control_points_, false);
    nh.param("trajectory_optimizer/viz_time_step", viz_time_step_, 0.05);

    ROS_INFO("=== DEBUG: Final values ===");

    ROS_INFO("TrajectoryOptimizer initialized with parameters:");
    ROS_INFO("  weight_time: %.3f", weight_time_);
    ROS_INFO("  weight_position_x: %.3f, weight_position_y: %.3f", weight_position_x_, weight_position_y_);
    ROS_INFO("  init_time: %.3f s", init_time_);
    ROS_INFO("  viz_only_control_points: %s, viz_time_step: %.3f s",
             viz_only_control_points_ ? "true" : "false", viz_time_step_);

    // 创建轨迹可视化发布器
    trajectory_viz_pub_ = nh.advertise<visualization_msgs::MarkerArray>("optimized_trajectory_viz", 1);

    initialized_ = true;
}

bool TrajectoryOptimizer::optimizePath(const std::vector<geometry_msgs::PoseStamped> &global_plan,
                                        Eigen::Matrix3Xd &optimized_points,
                                        Eigen::VectorXd &optimized_times)
{
    if (!initialized_)
    {
        ROS_ERROR("TrajectoryOptimizer not initialized!");
        return false;
    }

    if (global_plan.size() < 2)
    {
        ROS_ERROR("Global plan too short (size=%zu)", global_plan.size());
        return false;
    }

    // 1. 采样路径点
    samplePath(global_plan);

    if (sampled_path_.empty())
    {
        ROS_WARN("Sampled path is empty, using original plan");
        return false;
    }

    // 2. 设置 MINCO 参数
    piece_num_ = sampled_path_.size() + 1;
    temporal_dim_ = piece_num_;
    spatial_dim_ = 3 * (piece_num_ - 1);

    // 起点和终点
    Eigen::Vector3d start_point(global_plan.front().pose.position.x,
                                 global_plan.front().pose.position.y,
                                 0.0);
    Eigen::Vector3d end_point(global_plan.back().pose.position.x,
                               global_plan.back().pose.position.y,
                               0.0);

    // 设置边界条件
    Eigen::Matrix3d init_state = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d final_state = Eigen::Matrix3d::Zero();
    init_state.col(0) = start_point;
    final_state.col(0) = end_point;
    minco_.setConditions(init_state, final_state, piece_num_);

    // 3. 初始化优化变量
    Eigen::VectorXd x(temporal_dim_ + spatial_dim_);
    Eigen::Map<Eigen::VectorXd> tau(x.data(), temporal_dim_);
    Eigen::Map<Eigen::VectorXd> xi(x.data() + temporal_dim_, spatial_dim_);

    // 初始化时间参数
    Eigen::VectorXd T = init_time_ * Eigen::VectorXd::Ones(piece_num_);
    Eigen::VectorXd tau_temp;
    backwardT(T, tau_temp);
    tau = tau_temp;

    // 初始化空间参数（使用采样点）
    for (int i = 0; i < piece_num_ - 1; ++i)
    {
        xi.segment(3 * i, 3) = sampled_path_[i];
    }

    // 4. LBFGS 优化
    LBFGSpp::LBFGSParam<double> param;
    LBFGSpp::LBFGSSolver<double> solver(param);
    param.min_step = min_step_;
    param.epsilon = epsilon_;
    param.max_iterations = max_iterations_;

    // 创建代价函数（使用 lambda 捕获 this）
    std::function<double(const Eigen::VectorXd &, Eigen::VectorXd &)> cost_function =
        [this](const Eigen::VectorXd &params, Eigen::VectorXd &grad) {
            return this->computeCostAndGradient(params, grad);
        };

    double min_cost;
    int niter = solver.minimize(cost_function, x, min_cost);

    ROS_INFO("Optimization finished: %d iterations, cost=%.3f", niter, min_cost);

    // 5. 提取优化结果
    forwardT(tau, times_);
    forwardP(xi, points_);

    // 设置最终参数
    minco_.setConditions(init_state, final_state, piece_num_);
    minco_.setParameters(points_, times_);

    // 6. 计算位置代价以评估优化质量
    Eigen::MatrixX3d grad_by_points_pos = Eigen::MatrixX3d::Zero(piece_num_ - 1, 3);
    Eigen::VectorXd grad_by_times_pos = Eigen::VectorXd::Zero(piece_num_);
    double position_cost = computePositionCost(grad_by_points_pos, grad_by_times_pos);

    // 7. 判断优化质量，如果差则使用 fallback
    bool optimization_success = (position_cost < 100.0);

    if (!optimization_success)
    {
        ROS_WARN("Optimization quality poor (position_cost=%.3f), using fallback trajectory", position_cost);

        // 创建 fallback 轨迹（原始路径的 MINCO 格式）
        if (!createFallbackTrajectory(global_plan, optimized_points, optimized_times))
        {
            ROS_ERROR("Fallback trajectory creation failed!");
            return false;
        }
    }
    else
    {
        // 使用优化后的结果
        optimized_points = points_;
        optimized_times = times_;
    }

    // 8. 可视化轨迹
    std::string frame_id = global_plan.empty() ? "map" : global_plan[0].header.frame_id;
    visualizeTrajectory(frame_id);

    return true;
}

void TrajectoryOptimizer::samplePath(const std::vector<geometry_msgs::PoseStamped> &global_plan)
{
    sampled_path_.clear();

    // 跳过起点和终点，按间隔采样中间点
    for (size_t i = 1; i < global_plan.size() - 1; i += (astar_point_interval_ + 1))
    {
        Eigen::Vector3d point(global_plan[i].pose.position.x,
                              global_plan[i].pose.position.y,
                              0.0);
        sampled_path_.push_back(point);
    }

    ROS_INFO("Sampled %zu points from global plan (size=%zu)", sampled_path_.size(), global_plan.size());
}

double TrajectoryOptimizer::computeCostAndGradient(const Eigen::VectorXd &params, Eigen::VectorXd &grad)
{
    double cost = 0.0;
    grad.setZero();

    // 映射参数
    Eigen::Map<const Eigen::VectorXd> tau(params.data(), temporal_dim_);
    Eigen::Map<const Eigen::VectorXd> xi(params.data() + temporal_dim_, spatial_dim_);
    Eigen::Map<Eigen::VectorXd> grad_tau(grad.data(), temporal_dim_);
    Eigen::Map<Eigen::VectorXd> grad_xi(grad.data() + temporal_dim_, spatial_dim_);

    // 前向变换
    forwardT(tau, times_);
    forwardP(xi, points_);
    minco_.setParameters(points_, times_);

    // 1. 能量代价
    Eigen::MatrixX3d grad_by_coeffs = Eigen::MatrixX3d::Zero(6 * piece_num_, 3);
    Eigen::VectorXd grad_by_times = Eigen::VectorXd::Zero(piece_num_);
    Eigen::Matrix3Xd grad_by_points = Eigen::Matrix3Xd::Zero(3, piece_num_ - 1);
    Eigen::VectorXd grad_by_times_smooth = Eigen::VectorXd::Zero(piece_num_);

    minco_.getEnergy(cost, weight_energy_x_, weight_energy_y_, weight_energy_w_);
    minco_.getEnergyPartialGradByCoeffs(grad_by_coeffs, weight_energy_x_, weight_energy_y_, weight_energy_w_);
    minco_.getEnergyPartialGradByTimes(grad_by_times, weight_energy_x_, weight_energy_y_, weight_energy_w_);
    minco_.propogateGrad(grad_by_coeffs, grad_by_times, grad_by_points, grad_by_times_smooth);

    // 2. 时间代价
    cost += weight_time_ * times_.sum();
    grad_by_times_smooth.array() += weight_time_;

    // 3. 位置代价（保持接近采样点）
    Eigen::MatrixX3d grad_by_points_pos = Eigen::MatrixX3d::Zero(piece_num_ - 1, 3);
    Eigen::VectorXd grad_by_times_pos = Eigen::VectorXd::Zero(piece_num_);
    double position_cost = computePositionCost(grad_by_points_pos, grad_by_times_pos);

    grad_by_points += grad_by_points_pos.transpose();
    grad_by_times_smooth += grad_by_times_pos;
    cost += position_cost;

    // 反向传播梯度
    Eigen::VectorXd grad_tau_temp, grad_xi_temp;
    backwardGradT(tau, grad_by_times_smooth, grad_tau_temp);
    backwardGradP(xi, grad_by_points, grad_xi_temp);
    grad_tau = grad_tau_temp;
    grad_xi = grad_xi_temp;

    return cost;
}

double TrajectoryOptimizer::computePositionCost(Eigen::MatrixX3d &grad_by_points, Eigen::VectorXd &grad_by_times)
{
    double cost = 0.0;
    for (int i = 0; i < static_cast<int>(sampled_path_.size()); i++)
    {
        Eigen::Vector3d P = minco_.b.block((i + 1) * 6 + 0, 0, 1, 3).transpose();
        Eigen::Vector3d V = minco_.b.block((i + 1) * 6 + 1, 0, 1, 3).transpose();

        // 计算误差
        Eigen::Vector3d delta = P - sampled_path_[i];
        Eigen::Vector3d delta_sq(delta(0) * delta(0), delta(1) * delta(1), delta(2) * delta(2));
        Eigen::Vector3d weights(weight_position_x_, weight_position_y_, weight_position_w_);

        // 代价
        cost += delta_sq.dot(weights);

        // 梯度（对分段起点位置的梯度）
        grad_by_points.row(i) = 2.0 * delta.cwiseProduct(weights).transpose();

        // 梯度（对时间的梯度，通过速度传播）
        grad_by_times(i + 1) = grad_by_points.row(i).dot(V);
    }

    return cost;
}

// ==================== 参数变换函数 ====================

void TrajectoryOptimizer::forwardT(const Eigen::VectorXd &tau, Eigen::VectorXd &T)
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

void TrajectoryOptimizer::backwardT(const Eigen::VectorXd &T, Eigen::VectorXd &tau)
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

void TrajectoryOptimizer::forwardP(const Eigen::VectorXd &xi, Eigen::Matrix3Xd &P)
{
    const int size_P = xi.size() / 3;
    P.resize(3, size_P);
    for (int i = 0; i < size_P; ++i)
    {
        P.col(i) = xi.segment(3 * i, 3);
    }
}

void TrajectoryOptimizer::backwardP(const Eigen::Matrix3Xd &P, Eigen::VectorXd &xi)
{
    const int size_P = P.cols();
    xi.resize(3 * size_P);
    for (int i = 0; i < size_P; ++i)
    {
        xi.segment(3 * i, 3) = P.col(i);
    }
}

void TrajectoryOptimizer::backwardGradT(const Eigen::VectorXd &tau,
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

void TrajectoryOptimizer::backwardGradP(const Eigen::VectorXd &xi,
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

// ==================== 可视化和辅助函数 ====================

Eigen::Vector3d TrajectoryOptimizer::generatePolynomialTrajectory(const Eigen::MatrixXd &coefficients,
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
    { // 计算五次多项式
        x += current_coeff(j, 0) * std::pow(local_t, j); // X方向
        y += current_coeff(j, 1) * std::pow(local_t, j); // Y方向
    }

    return Eigen::Vector3d(x, y, 0.0);
}

void TrajectoryOptimizer::visualizeTrajectory(const std::string &frame_id)
{
    if (trajectory_viz_pub_.getNumSubscribers() == 0)
        return;

    visualization_msgs::MarkerArray marker_array;

    // 1. 删除旧标记
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = frame_id;
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);
    trajectory_viz_pub_.publish(marker_array);
    marker_array.markers.clear();

    int ID = 0;

    ROS_INFO("Visualization mode: %s", viz_only_control_points_ ? "CONTROL_POINTS_ONLY" : "SAMPLED_TRAJECTORY");

    if (viz_only_control_points_)
    {
        // 模式 1: 仅显示控制点
        ROS_INFO("Visualizing control points only (%ld points)", points_.cols());

        for (int i = 0; i < points_.cols(); ++i)
        {
            visualization_msgs::Marker point_marker;
            point_marker.header.frame_id = frame_id;
            point_marker.header.stamp = ros::Time::now();
            point_marker.ns = "control_points";
            point_marker.type = visualization_msgs::Marker::SPHERE;
            point_marker.action = visualization_msgs::Marker::ADD;
            point_marker.id = ID++;

            // 控制点大小稍大，便于区分
            point_marker.scale.x = 0.08;
            point_marker.scale.y = 0.08;
            point_marker.scale.z = 0.08;

            // 颜色：红色（控制点）
            point_marker.color.r = 1.0;
            point_marker.color.g = 0.0;
            point_marker.color.b = 0.0;
            point_marker.color.a = 1.0;

            // 位置
            point_marker.pose.position.x = points_(0, i);
            point_marker.pose.position.y = points_(1, i);
            point_marker.pose.position.z = 0.1;

            // 方向（单位四元数，无旋转）
            point_marker.pose.orientation.x = 0.0;
            point_marker.pose.orientation.y = 0.0;
            point_marker.pose.orientation.z = 0.0;
            point_marker.pose.orientation.w = 1.0;

            marker_array.markers.push_back(point_marker);
        }
    }
    else
    {
        // 模式 2: 使用时间步长采样轨迹
        double total_duration = times_.sum();
        ROS_INFO("Visualizing sampled trajectory with time step %.3f s (total duration: %.3f s)",
                 viz_time_step_, total_duration);

        for (double t = 0.0; t <= total_duration; t += viz_time_step_)
        {
            Eigen::Vector3d point = generatePolynomialTrajectory(minco_.b, times_, t);

            visualization_msgs::Marker point_marker;
            point_marker.header.frame_id = frame_id;
            point_marker.header.stamp = ros::Time::now();
            point_marker.ns = "optimized_trajectory";
            point_marker.type = visualization_msgs::Marker::SPHERE;
            point_marker.action = visualization_msgs::Marker::ADD;
            point_marker.id = ID++;

            // 点的大小
            point_marker.scale.x = 0.05;
            point_marker.scale.y = 0.05;
            point_marker.scale.z = 0.05;

            // 颜色：蓝色（采样轨迹）
            point_marker.color.r = 0.0;
            point_marker.color.g = 0.0;
            point_marker.color.b = 1.0;
            point_marker.color.a = 1.0;

            // 位置
            point_marker.pose.position.x = point.x();
            point_marker.pose.position.y = point.y();
            point_marker.pose.position.z = 0.1;

            // 方向（单位四元数，无旋转）
            point_marker.pose.orientation.x = 0.0;
            point_marker.pose.orientation.y = 0.0;
            point_marker.pose.orientation.z = 0.0;
            point_marker.pose.orientation.w = 1.0;

            marker_array.markers.push_back(point_marker);
        }
    }

    // 发布轨迹标记
    trajectory_viz_pub_.publish(marker_array);
    ROS_INFO("Published optimized trajectory visualization with %d markers", ID);
}

bool TrajectoryOptimizer::createFallbackTrajectory(const std::vector<geometry_msgs::PoseStamped> &global_plan,
                                                     Eigen::Matrix3Xd &fallback_points,
                                                     Eigen::VectorXd &fallback_times)
{
    if (global_plan.size() < 2)
    {
        return false;
    }

    // 采样路径点
    samplePath(global_plan);

    if (sampled_path_.empty())
    {
        ROS_WARN("Fallback: sampled path is empty");
        return false;
    }

    // 使用采样点作为 fallback
    int n_points = sampled_path_.size();
    fallback_points.resize(3, n_points);
    fallback_times.resize(n_points + 1);

    for (int i = 0; i < n_points; ++i)
    {
        fallback_points.col(i) = sampled_path_[i];
        fallback_times(i) = init_time_;
    }
    fallback_times(n_points) = init_time_;

    // 设置 MINCO 边界条件
    Eigen::Vector3d start_point(global_plan.front().pose.position.x,
                                  global_plan.front().pose.position.y,
                                  0.0);
    Eigen::Vector3d end_point(global_plan.back().pose.position.x,
                               global_plan.back().pose.position.y,
                               0.0);

    Eigen::Matrix3d init_state = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d final_state = Eigen::Matrix3d::Zero();
    init_state.col(0) = start_point;
    final_state.col(0) = end_point;

    minco_.setConditions(init_state, final_state, n_points + 1);
    minco_.setParameters(fallback_points, fallback_times);

    // 保存到成员变量
    points_ = fallback_points;
    times_ = fallback_times;

    ROS_INFO("Created fallback trajectory with %d points", n_points);
    return true;
}

void TrajectoryOptimizer::updateParameters(
    double weight_time, double weight_energy_x, double weight_energy_y, double weight_energy_w,
    double weight_position_x, double weight_position_y, double weight_position_w,
    double init_time, int astar_point_interval, int max_iterations,
    double min_step, double epsilon,
    bool viz_only_control_points, double viz_time_step)
{
    // 更新所有参数
    weight_time_ = weight_time;
    weight_energy_x_ = weight_energy_x;
    weight_energy_y_ = weight_energy_y;
    weight_energy_w_ = weight_energy_w;
    weight_position_x_ = weight_position_x;
    weight_position_y_ = weight_position_y;
    weight_position_w_ = weight_position_w;

    init_time_ = init_time;
    astar_point_interval_ = astar_point_interval;
    max_iterations_ = max_iterations;
    min_step_ = min_step;
    epsilon_ = epsilon;

    viz_only_control_points_ = viz_only_control_points;
    viz_time_step_ = viz_time_step;

    ROS_INFO("TrajectoryOptimizer parameters updated via dynamic_reconfigure:");
    ROS_INFO("  weight_time: %.3f", weight_time_);
    ROS_INFO("  weight_position: [%.1f, %.1f, %.1f]",
             weight_position_x_, weight_position_y_, weight_position_w_);
    ROS_INFO("  weight_energy: [%.1f, %.1f, %.1f]",
             weight_energy_x_, weight_energy_y_, weight_energy_w_);
    ROS_INFO("  init_time: %.3f, max_iterations: %d", init_time_, max_iterations_);
    ROS_INFO("  viz_only_control_points: %s, viz_time_step: %.3f",
             viz_only_control_points_ ? "true" : "false", viz_time_step_);
}

} // namespace ntu_planner
