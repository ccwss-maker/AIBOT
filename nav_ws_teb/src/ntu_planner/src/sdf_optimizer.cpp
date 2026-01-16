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

void SDFOptimizer::initialize(ros::NodeHandle &nh, const std::string &robot_base_frame, const std::string &global_frame)
{
    // 保存坐标系参数
    robot_base_frame_ = robot_base_frame;
    global_frame_ = global_frame;
    
    // 加载Find t_star参数
    nh.param("sdf_optimization/coarse_grid_size", coarse_grid_size_, 0.1);
    nh.param("sdf_optimization/find_t_star_time_step", find_t_star_time_step_, 0.1);
    nh.param("sdf_optimization/eta", eta_, 0.01);
    nh.param("sdf_optimization/c", c_, 0.1);
    nh.param("sdf_optimization/tol", tol_, 1e-6);
    nh.param("sdf_optimization/max_iter", max_iter_, 1000);

    // 加载Compute SDF参数
    nh.param("sdf_optimization/sdf_resolution", sdf_resolution_, 0.8);
    nh.param("sdf_optimization/sdf_safety_hor", sdf_safety_hor_, 1.0);

    // 加载Optimization参数
    nh.param("sdf_optimization/sdf_opimiz_times_max", sdf_opimiz_times_max_, 10000);
    nh.param("sdf_optimization/sdf_opimiz_weight_time", sdf_opimiz_weight_time_, 1.0);
    nh.param("sdf_optimization/sdf_opimiz_weight_energy_x", sdf_opimiz_weight_energy_x_, 1.0);
    nh.param("sdf_optimization/sdf_opimiz_weight_energy_y", sdf_opimiz_weight_energy_y_, 1.0);
    nh.param("sdf_optimization/sdf_opimiz_weight_energy_w", sdf_opimiz_weight_energy_w_, 1.0);
    nh.param("sdf_optimization/sdf_opimiz_weight_yaw", sdf_opimiz_weight_yaw_, 0.1);
    nh.param("sdf_optimization/sdf_opimiz_weight_safety", sdf_opimiz_weight_safety_, 1.0);

    // 加载Costmap参数
    nh.param("sdf_optimization/width", width_, 10.0);
    nh.param("sdf_optimization/height", height_, 10.0);

    // 加载车辆footprint参数
    nh.param("sdf_optimization/footprint", footprint_, std::string("[[0.45, 0.35], [0.45, -0.35], [-0.45, -0.35], [-0.45, 0.35]]"));

    // 加载可视化参数
    nh.param("sdf_optimization/viz_only_control_points", viz_only_control_points_, false);
    nh.param("sdf_optimization/viz_time_step", viz_time_step_, 0.05);
    
    // 初始化TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    ROS_INFO("SDFOptimizer initialized with parameters:");
    ROS_INFO("  coarse_grid_size: %.3f", coarse_grid_size_);
    ROS_INFO("  sdf_resolution: %.3f", sdf_resolution_);
    ROS_INFO("  sdf_opimiz_times_max: %d", sdf_opimiz_times_max_);
    ROS_INFO("  sdf_opimiz_weight_time: %.3f", sdf_opimiz_weight_time_);
    ROS_INFO("  sdf_opimiz_weight_yaw: %.3f", sdf_opimiz_weight_yaw_);
    ROS_INFO("  sdf_opimiz_weight_safety: %.3f", sdf_opimiz_weight_safety_);
    ROS_INFO("  costmap size: %.3f x %.3f meters",
             width_, height_);
    ROS_INFO("  vehicle footprint vertices:");
    for (const auto &vertex : parsed_footprint_)
    {
        ROS_INFO("(%.3f, %.3f)", vertex.first, vertex.second);
    }
    ROS_INFO("  viz_only_control_points: %s, viz_time_step: %.3f s",
             viz_only_control_points_ ? "true" : "false", viz_time_step_);

    // 解析车辆footprint字符串
    parseFootprint(footprint_);


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
    config.footprint = parsed_footprint_;
    
    double half_width = width_ / 2.0;
    double half_height = height_ / 2.0;
    config.x_min = -half_width;
    config.x_max = half_width;
    config.y_min = -half_height;
    config.y_max = half_height;

    // 初始化GPU配置
    GPUProcessConfig(config);

    initialized_ = true;
}

bool SDFOptimizer::optimizePath(const Eigen::Matrix3Xd &initial_points,
                                 const Eigen::VectorXd &initial_times,
                                 const Eigen::Matrix3Xd &obstacle_points,
                                 Eigen::Matrix3Xd &optimized_points,
                                 Eigen::VectorXd &optimized_times,
                                 const Eigen::Matrix3d &init_state,
                                 const Eigen::Matrix3d &final_state,
                                 const std::string &frame_id)
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

        // // 计算扫掠体积
        // std::vector<SDF_Map_> SDF_Map;
        // double area = 0;
        // GPUProcessSDF(traj_, SDF_Map, area);
        // ROS_INFO("Swept area: %.3f", area);

        // 输出优化结果
        optimized_points = traj_.points;
        optimized_times = traj_.times;

        // 可视化轨迹
        visualizeTrajectory(frame_id);
        visualizeSweptVolume(frame_id);
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
    // std::cout << obstacle_points_.col(0).transpose() << std::endl;

    GPUProcessGradSDF(traj_, obstacle_points_, GradByPoints_Ob, GradByTimes_Ob, cost_Ob);
    gradByPoints += GradByPoints_Ob.topRows(GradByPoints_Ob.rows() - 1).transpose();
    gradByTimes += GradByTimes_Ob;
    cost += cost_Ob;

    // 4. 航向角代价（通过GPU计算）
    Eigen::MatrixX3d GradByPoints_Yaw = Eigen::MatrixX3d::Zero(traj_.pieceN, 3);
    Eigen::MatrixX3d GradByVelocity_Yaw = Eigen::MatrixX3d::Zero(traj_.pieceN, 3);
    Eigen::MatrixX3d GPUGradByVx_Vy_Yaw = Eigen::MatrixX3d::Zero(traj_.pieceN, 3);
    Eigen::VectorXd GradByTimes_Yaw = Eigen::VectorXd::Zero(traj_.pieceN);
    double cost_Yaw;
    GPUProcessGradYaw(traj_, sdf_opimiz_weight_yaw_, GPUGradByVx_Vy_Yaw, GradByTimes_Yaw, cost_Yaw);
    GradByVelocity_Yaw.col(0) = GPUGradByVx_Vy_Yaw.col(0);
    GradByVelocity_Yaw.col(1) = GPUGradByVx_Vy_Yaw.col(1);
    GradByPoints_Yaw.col(2) = GPUGradByVx_Vy_Yaw.col(2);
    GradByCoeffs.setZero();
    minco_.getVel0PartialGradByCoeffs(GradByCoeffs, GradByVelocity_Yaw, 1.0, 1.0, 1.0);
    Eigen::VectorXd TotalGradByTimes;
    Eigen::Matrix3Xd GradByPoints_Yaw_;
    minco_.propogateGrad(GradByCoeffs, GradByTimes_Yaw, GradByPoints_Yaw_, TotalGradByTimes);
    GradByPoints_Yaw.col(0).topRows(GradByPoints_Yaw.rows() - 1) = GradByPoints_Yaw_.row(0).transpose();
    GradByPoints_Yaw.col(1).topRows(GradByPoints_Yaw.rows() - 1) = GradByPoints_Yaw_.row(1).transpose();
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

// ==================== 坐标系转换函数 ====================

bool SDFOptimizer::convertToLocalFrameUsingTF(const Optimized_Trajectory_ &global_traj,
                                             Optimized_Trajectory_ &local_traj,
                                             double &origin_x,
                                             double &origin_y,
                                             double &origin_yaw)
{
    try
    {
        // 使用TF获取从全局坐标系到机器人坐标系的变换
        geometry_msgs::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
            global_frame_, robot_base_frame_, ros::Time(0), ros::Duration(1.0));
        
        // 从变换中提取机器人在全局坐标系中的位置
        origin_x = transform_stamped.transform.translation.x;
        origin_y = transform_stamped.transform.translation.y;
        
        // 从四元数提取yaw角
        tf2::Quaternion q(
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z,
            transform_stamped.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, origin_yaw);
        
        ROS_INFO("Got TF transform: origin=(%.3f, %.3f), yaw=%.3f rad (%.1f deg)",
                 origin_x, origin_y, origin_yaw, origin_yaw * 180.0 / M_PI);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Failed to get TF transform: %s. Using trajectory start point as fallback.", ex.what());
        
        // 备用方案：使用轨迹起点作为局部坐标系原点
        origin_x = global_traj.b(0, 0);
        origin_y = global_traj.b(0, 1);
        origin_yaw = global_traj.b(0, 2);
    }
    
    // 复制结构
    local_traj = global_traj;
    
    double cos_yaw = std::cos(origin_yaw);
    double sin_yaw = std::sin(origin_yaw);
    
    // 转换所有轨迹系数（仅处理位置的常数项）
    for (int i = 0; i < local_traj.b.rows(); i += 6)
    {
        double global_x = global_traj.b(i, 0);
        double global_y = global_traj.b(i, 1);
        
        // 平移到原点
        double dx = global_x - origin_x;
        double dy = global_y - origin_y;
        
        // 旋转到局部坐标系（逆时针旋转-origin_yaw）
        local_traj.b(i, 0) = dx * cos_yaw + dy * sin_yaw;
        local_traj.b(i, 1) = -dx * sin_yaw + dy * cos_yaw;
        local_traj.b(i, 2) = global_traj.b(i, 2) - origin_yaw;
    }
    
    // 转换控制点
    for (int i = 0; i < local_traj.points.cols(); ++i)
    {
        double global_x = global_traj.points(0, i);
        double global_y = global_traj.points(1, i);
        
        double dx = global_x - origin_x;
        double dy = global_y - origin_y;
        
        local_traj.points(0, i) = dx * cos_yaw + dy * sin_yaw;
        local_traj.points(1, i) = -dx * sin_yaw + dy * cos_yaw;
        local_traj.points(2, i) = global_traj.points(2, i) - origin_yaw;
    }
    
    return true;
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
        ROS_INFO("Visualizing control points only (%ld points)", traj_.points.cols());

        for (int i = 0; i < traj_.points.cols(); ++i)
        {
            visualization_msgs::Marker point_marker;
            point_marker.header.frame_id = frame_id;
            point_marker.header.stamp = ros::Time::now();
            point_marker.ns = "sdf_control_points";
            point_marker.type = visualization_msgs::Marker::SPHERE;
            point_marker.action = visualization_msgs::Marker::ADD;
            point_marker.id = ID++;

            // 控制点大小稍大，便于区分
            point_marker.scale.x = 0.08;
            point_marker.scale.y = 0.08;
            point_marker.scale.z = 0.08;

            // 颜色：白色（控制点）
            point_marker.color.r = 1.0;
            point_marker.color.g = 1.0;
            point_marker.color.b = 1.0;
            point_marker.color.a = 1.0;

            // 位置
            point_marker.pose.position.x = traj_.points(0, i);
            point_marker.pose.position.y = traj_.points(1, i);
            point_marker.pose.position.z = 0.3;

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
        double total_duration = traj_.times.sum();
        ROS_INFO("Visualizing sampled trajectory with time step %.3f s (total duration: %.3f s)",
                 viz_time_step_, total_duration);

        for (double t = 0.0; t <= total_duration; t += viz_time_step_)
        {
            Eigen::Vector3d point = generatePolynomialTrajectory(traj_.b, traj_.times, t);

            visualization_msgs::Marker point_marker;
            point_marker.header.frame_id = frame_id;
            point_marker.header.stamp = ros::Time::now();
            point_marker.ns = "sdf_trajectory";
            point_marker.type = visualization_msgs::Marker::SPHERE;
            point_marker.action = visualization_msgs::Marker::ADD;
            point_marker.id = ID++;

            // 点的大小
            point_marker.scale.x = 0.05;
            point_marker.scale.y = 0.05;
            point_marker.scale.z = 0.05;

            // 颜色：白色（采样轨迹）
            point_marker.color.r = 1.0;
            point_marker.color.g = 1.0;
            point_marker.color.b = 1.0;
            point_marker.color.a = 1.0;

            // 位置
            point_marker.pose.position.x = point.x();
            point_marker.pose.position.y = point.y();
            point_marker.pose.position.z = 0.3;

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
    ROS_INFO("Published SDF optimized trajectory visualization with %d markers", ID);
}

void SDFOptimizer::visualizeSweptVolume(const std::string &frame_id)
{
    if (swept_volume_pub_.getNumSubscribers() == 0)
        return;

    // 使用TF转换到局部坐标系
    Optimized_Trajectory_ local_traj;
    double origin_x, origin_y, origin_yaw;
    if (!convertToLocalFrameUsingTF(traj_, local_traj, origin_x, origin_y, origin_yaw))
    {
        ROS_ERROR("Failed to convert to local frame");
        return;
    }
    
    // 使用局部坐标系轨迹计算扫掠体积
    std::vector<SDF_Map_> SDF_Map;
    double area = 0;
    GPUProcessSDF(local_traj, SDF_Map, area);
    
    double cos_yaw = std::cos(origin_yaw);
    double sin_yaw = std::sin(origin_yaw);
    
    // 将 yaw 角转换为四元数（用于 marker 的 orientation）
    tf2::Quaternion q_marker;
    q_marker.setRPY(0.0, 0.0, origin_yaw);

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

            // 使用TF获取的变换将局部坐标转回全局坐标
            double local_x = SDF_Map[i].x;
            double local_y = SDF_Map[i].y;
            
            // 先旋转（顺时针旋转origin_yaw）
            double rotated_x = local_x * cos_yaw - local_y * sin_yaw;
            double rotated_y = local_x * sin_yaw + local_y * cos_yaw;
            
            // 再平移
            marker.pose.position.x = rotated_x + origin_x;
            marker.pose.position.y = rotated_y + origin_y;
            marker.pose.position.z = 0.0;

            // 设置 marker 的朝向为车辆的朝向
            marker.pose.orientation.x = q_marker.x();
            marker.pose.orientation.y = q_marker.y();
            marker.pose.orientation.z = q_marker.z();
            marker.pose.orientation.w = q_marker.w();

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
    double tol, int max_iter, double sdf_resolution,
    double sdf_safety_hor, int sdf_opimiz_times_max, double sdf_opimiz_weight_time,
    double sdf_opimiz_weight_energy_x, double sdf_opimiz_weight_energy_y,
    double sdf_opimiz_weight_energy_w, double sdf_opimiz_weight_yaw,
    double sdf_opimiz_weight_safety, double width, double height,
    const std::string& footprint,
    bool viz_only_control_points, double viz_time_step)
{
    // 更新所有参数
    coarse_grid_size_ = coarse_grid_size;
    find_t_star_time_step_ = find_t_star_time_step;
    eta_ = eta;
    c_ = c;
    tol_ = tol;
    max_iter_ = max_iter;
    sdf_resolution_ = sdf_resolution;
    sdf_safety_hor_ = sdf_safety_hor;
    sdf_opimiz_times_max_ = sdf_opimiz_times_max;
    sdf_opimiz_weight_time_ = sdf_opimiz_weight_time;
    sdf_opimiz_weight_energy_x_ = sdf_opimiz_weight_energy_x;
    sdf_opimiz_weight_energy_y_ = sdf_opimiz_weight_energy_y;
    sdf_opimiz_weight_energy_w_ = sdf_opimiz_weight_energy_w;
    sdf_opimiz_weight_yaw_ = sdf_opimiz_weight_yaw;
    sdf_opimiz_weight_safety_ = sdf_opimiz_weight_safety;
    width_ = width;
    height_ = height;
    footprint_ = footprint;
    parseFootprint(footprint_);
    
    // 更新可视化参数
    viz_only_control_points_ = viz_only_control_points;
    viz_time_step_ = viz_time_step;

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
    config.footprint = parsed_footprint_;

    // 区域参数
    double half_width = width_ / 2.0;
    double half_height = height_ / 2.0;
    config.x_min = -half_width;
    config.x_max = half_width;
    config.y_min = -half_height;
    config.y_max = half_height;

    GPUProcessConfig(config);

    ROS_INFO("SDFOptimizer parameters updated via dynamic_reconfigure:");
    ROS_INFO("  coarse_grid_size: %.3f", coarse_grid_size_);
    ROS_INFO("  sdf_resolution: %.3f", sdf_resolution_);
    ROS_INFO("  width: %.2f, height: %.2f", width_, height_);
    ROS_INFO("  sdf_opimiz_times_max: %d", sdf_opimiz_times_max_);
    ROS_INFO("  weights: time=%.1f, swept_volume=%.1f, safety=%.1f",
             sdf_opimiz_weight_time_, sdf_opimiz_weight_yaw_, sdf_opimiz_weight_safety_);
}

bool SDFOptimizer::parseFootprint(const std::string& footprint_str)
{
    parsed_footprint_.clear();
    
    // 解析JSON格式: "[[x1, y1], [x2, y2], [x3, y3], [x4, y4]]"
    std::string s = footprint_str;
    
    // 移除所有空格
    s.erase(std::remove(s.begin(), s.end(), ' '), s.end());
    
    // 检查基本格式
    if (s.size() < 4 || s[0] != '[' || s[1] != '[' || s[s.size()-1] != ']' || s[s.size()-2] != ']')
    {
        ROS_ERROR("Invalid footprint format: '%s'", footprint_str.c_str());
        ROS_ERROR("Expected format: [[x1, y1], [x2, y2], [x3, y3], [x4, y4]]");
        // 使用默认footprint (矩形: 0.9m x 0.7m)
        parsed_footprint_.push_back({0.45, 0.35});
        parsed_footprint_.push_back({0.45, -0.35});
        parsed_footprint_.push_back({-0.45, -0.35});
        parsed_footprint_.push_back({-0.45, 0.35});
        ROS_WARN("Using default footprint with 4 vertices");
        return false;
    }
    
    // 解析每个点 [x, y]
    // 从第一个 '[' 开始搜索，即位置 1
    size_t pos = 1; // 跳过最外层的第一个 '['
    
    while (pos < s.size() - 1)
    {
        // 查找下一个 '['
        pos = s.find('[', pos);
        if (pos == std::string::npos || pos >= s.size() - 1)
        {
            break;
        }
        
        pos++; // 跳过 '['
        
        // 查找这个数组内的结束括号
        size_t bracket_pos = s.find(']', pos);
        if (bracket_pos == std::string::npos)
        {
            ROS_ERROR("Cannot find closing bracket for coordinate");
            break;
        }
        
        // 在这个数组范围内查找逗号
        std::string coord_str = s.substr(pos, bracket_pos - pos);
        size_t comma_pos_in_coord = coord_str.find(',');
        
        if (comma_pos_in_coord != std::string::npos)
        {
            try
            {
                std::string x_str = coord_str.substr(0, comma_pos_in_coord);
                std::string y_str = coord_str.substr(comma_pos_in_coord + 1);
                
                double x = std::stod(x_str);
                double y = std::stod(y_str);
                
                parsed_footprint_.push_back({x, y});
                
                pos = bracket_pos + 1; // 移动到 ']' 之后
            }
            catch (const std::exception& e)
            {
                ROS_ERROR("Error parsing footprint coordinate '%s': %s", coord_str.c_str(), e.what());
                parsed_footprint_.clear();
                parsed_footprint_.push_back({0.45, 0.35});
                parsed_footprint_.push_back({0.45, -0.35});
                parsed_footprint_.push_back({-0.45, -0.35});
                parsed_footprint_.push_back({-0.45, 0.35});
                ROS_WARN("Using default footprint with 4 vertices");
                return false;
            }
        }
        else
        {
            ROS_ERROR("Cannot find comma in coordinate string: '%s'", coord_str.c_str());
            break;
        }
    }
    
    // 必须是4个顶点
    if (parsed_footprint_.size() != 4)
    {
        ROS_ERROR("Footprint must have exactly 4 vertices, got %zu", parsed_footprint_.size());
        ROS_ERROR("Using default footprint instead");
        parsed_footprint_.clear();
        parsed_footprint_.push_back({0.45, 0.35});
        parsed_footprint_.push_back({0.45, -0.35});
        parsed_footprint_.push_back({-0.45, -0.35});
        parsed_footprint_.push_back({-0.45, 0.35});
        return false;
    }
    
    ROS_INFO("Parsed footprint with 4 vertices:");
    for (size_t i = 0; i < parsed_footprint_.size(); ++i)
    {
        ROS_INFO("  [%.3f, %.3f]", parsed_footprint_[i].first, parsed_footprint_[i].second);
    }
    
    return true;
}

} // namespace ntu_planner
