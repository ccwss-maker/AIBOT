#include "TrajectoryOpimizationNode.hpp"

Trajectory_Opimization_Node::Trajectory_Opimization_Node(std::shared_ptr<TFSubscriberNode> tf_subscriber_node)
    : Node("control_node"), tf_subscriber_node_(tf_subscriber_node) {
    // 初始化订阅者   
    Astar_Path_Subscriber_ = this->create_subscription<astar_msgs::msg::AStarPathArray>("/A_Star_Planned_Path", 10, std::bind(&Trajectory_Opimization_Node::Callback, this, std::placeholders::_1));
    // 初始化发布者
    Initial_Optimized_Trajectory_Publisher_ = this->create_publisher<initial_optimized_msgs::msg::InitialOptimizedTrajectory>("Initial_Optimized_Trajectory", 10);
    Trajectory_Opimization_Publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("trajectory_opimization_marker_array", 10);
    config_yaml_path = "/root/PersonalData/Program/multi-axle-all-wheel-steering-vehicles_ws/src/route_planning/config/route_planning_config.yaml";

}

void Trajectory_Opimization_Node::Callback(const astar_msgs::msg::AStarPathArray::ConstSharedPtr & astar_path)
{
    Header = astar_path->header;
    Car_Odom_Matrix = tf_subscriber_node_ -> Matrix_Read("odom", "car_base");           // 车辆    ->  世界 变换矩阵获取(放在回调函数最开始，保证和初始数据时间戳尽量同步)
    Car_Odom_Rotation_Translation_Matrix = Car_Odom_Matrix.Rotation_Translation_Read();
    config = YAML::LoadFile(config_yaml_path);
    Eigen::Vector3d A_Star_Path_front = Eigen::Vector3d(astar_path->paths.front().position.x, astar_path->paths.front().position.y, astar_path->paths.front().position.z);
    Eigen::Vector3d A_Star_Path_back = Eigen::Vector3d(astar_path->paths.back().position.x, astar_path->paths.back().position.y, astar_path->paths.back().position.z);
    A_Star_Path.clear();

    for (int i=1; i<astar_path->paths.size()-1; i+=(config["A_Star_Point_interval"].as<double>() + 1)) {
        A_Star_Path_ path_point;
        path_point.position = Eigen::Vector3d(astar_path->paths[i].position.x, astar_path->paths[i].position.y, astar_path->paths[i].position.z);
        A_Star_Path.push_back(path_point);
    }

    pieceN      = A_Star_Path.size() + 1;
    temporalDim = pieceN;
    spatialDim  = 3 * (pieceN - 1);
    Eigen::Matrix3d initState = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d finalState = Eigen::Matrix3d::Zero();
    initState.col(0) = A_Star_Path_front;
    finalState.col(0) = A_Star_Path_back;
    minco.setConditions(initState, finalState, pieceN);
    Eigen::VectorXd x(temporalDim + spatialDim);
    Eigen::Map<Eigen::VectorXd> tau(x.data(), temporalDim);
    Eigen::Map<Eigen::VectorXd> xi(x.data() + temporalDim, spatialDim);
    double init_time = config["init_time"].as<double>();
    Eigen::VectorXd T = init_time * Eigen::VectorXd::Ones(pieceN);
    backwardT(T, tau);
    for (int i = 0; i < pieceN - 1; ++i)
    {
        xi.segment(3 * i, 3)  = A_Star_Path[i].position;
    }

    const int dimTau = temporalDim;
    const int dimXi  = spatialDim;

    // 定义LBFGS优化
    LBFGSpp::LBFGSParam<double> param;
    LBFGSpp::LBFGSSolver<double> solver(param);
    param.min_step = 1e-5;
    param.epsilon = 1e-5;
    param.max_iterations = 1000;
    weight_time = config["traj_opimiz_weight_time"].as<double>();
    weight_position_x = config["traj_opimiz_weight_position_x"].as<double>();
    weight_position_y = config["traj_opimiz_weight_position_y"].as<double>();
    weight_position_w = config["traj_opimiz_weight_position_w"].as<double>();
    weight_energy_x = config["traj_opimiz_weight_energy_x"].as<double>();
    weight_energy_y = config["traj_opimiz_weight_energy_y"].as<double>();
    weight_energy_w = config["traj_opimiz_weight_energy_w"].as<double>();
    // 将lambda表达式赋给std::function
    std::function<double(const Eigen::VectorXd&, Eigen::VectorXd&)> cost_function =
        [this](const Eigen::VectorXd& params, Eigen::VectorXd& grad) {
            return this->ComputeCostAndGradient(params, grad);
        };

    double min_cost;
    int niter = solver.minimize(cost_function, x, min_cost);
    forwardT(tau, times);
    forwardP(xi, points);
    minco.setConditions(initState, finalState, pieceN);
    minco.setParameters(points, times);

    Eigen::MatrixX3d GradByPoints_Position = Eigen::MatrixX3d::Zero(pieceN, 3);
    Eigen::VectorXd GradByTimes_Position = Eigen::VectorXd::Zero(pieceN);
    double cost_Yaw = ComputeCostAndGradient_Position(GradByPoints_Position, GradByTimes_Position);

    initial_optimized_msgs::msg::InitialOptimizedTrajectory InitialOptimizedTrajectory;
    InitialOptimizedTrajectory.header.frame_id = "car_base";
    InitialOptimizedTrajectory.header.stamp = astar_path->header.stamp;
    if(cost_Yaw < 100)
    {
        std::cout<<"Initial Optimized Successful: "<<cost_Yaw<<std::endl;
        for(int i=0; i<pieceN; i++)
        {
            InitialOptimizedTrajectory.times.push_back(times[i]);
        }
        // 发布原始点
        geometry_msgs::msg::Vector3 p;
        p.x = A_Star_Path_front(0);
        p.y = A_Star_Path_front(1);
        p.z = A_Star_Path_front(2);
        InitialOptimizedTrajectory.position.push_back(p);
        for(int i=0; i<points.cols(); i++)
        {
            p.x = points(0, i);
            p.y = points(1, i);
            p.z = points(2, i);
            InitialOptimizedTrajectory.position.push_back(p);
        }
        p.x = A_Star_Path_back(0);
        p.y = A_Star_Path_back(1);
        p.z = A_Star_Path_back(2);
        InitialOptimizedTrajectory.position.push_back(p);
    }
    else
    {
        std::cout<<"Initial Optimized Failed: "<<cost_Yaw<<std::endl;
        geometry_msgs::msg::Vector3 p;
        p.x = A_Star_Path_front(0);
        p.y = A_Star_Path_front(1);
        p.z = A_Star_Path_front(2);
        InitialOptimizedTrajectory.position.push_back(p);
        InitialOptimizedTrajectory.times.push_back(init_time);

        for (int i=0; i<A_Star_Path.size(); i++) {
            geometry_msgs::msg::Vector3 p;
            p.x = A_Star_Path[i].position(0);
            p.y = A_Star_Path[i].position(1);
            p.z = A_Star_Path[i].position(2);
            InitialOptimizedTrajectory.position.push_back(p);
            InitialOptimizedTrajectory.times.push_back(init_time);
        }

        p.x = A_Star_Path_back(0);
        p.y = A_Star_Path_back(1);
        p.z = A_Star_Path_back(2);
        InitialOptimizedTrajectory.position.push_back(p);
        InitialOptimizedTrajectory.times.push_back(init_time);
    }

    visualizeTrajectory(minco.b, times);
    InitialOptimizedTrajectory.obstacle_points = astar_path->obstacle_points;
    Initial_Optimized_Trajectory_Publisher_->publish(InitialOptimizedTrajectory);
}

// 代价函数及其梯度计算
double Trajectory_Opimization_Node::ComputeCostAndGradient(const Eigen::VectorXd& params, Eigen::VectorXd& grad) {
    double cost = 0.0;
    grad.setZero();
    int dimTau = temporalDim; // 时间维度
    int dimXi = spatialDim;   // 空间维度
    Eigen::Map<const Eigen::VectorXd> tau(params.data(), dimTau); // 将参数映射为tau
    Eigen::Map<const Eigen::VectorXd> xi(params.data() + dimTau, dimXi); // 将参数映射为xi
    Eigen::Map<Eigen::VectorXd> gradTau(grad.data(), dimTau); // 将梯度映射为gradTau
    Eigen::Map<Eigen::VectorXd> gradXi(grad.data() + dimTau, dimXi); // 将梯度映射为gradXi
    
    Eigen::MatrixX3d GradByCoeffs = Eigen::MatrixX3d::Zero(6 * pieceN, 3);
    Eigen::VectorXd GradByTimes = Eigen::VectorXd::Zero(pieceN);
    Eigen::Matrix3Xd gradByPoints = Eigen::Matrix3Xd::Zero(3, pieceN - 1);
    Eigen::VectorXd gradByTimes = Eigen::VectorXd::Zero(pieceN);
    forwardT(tau, times); // 计算时间参数
    forwardP(xi, points); // 计算空间参数
    minco.setParameters(points, times); // 设置最小曲线参数
    minco.getEnergy(cost, weight_energy_x, weight_energy_y, weight_energy_w);
    minco.getEnergyPartialGradByCoeffs(GradByCoeffs, weight_energy_x, weight_energy_y, weight_energy_w); // ∂E/∂c
    minco.getEnergyPartialGradByTimes(GradByTimes, weight_energy_x, weight_energy_y, weight_energy_w);   // ∂E/∂T
    minco.propogateGrad(GradByCoeffs, GradByTimes, gradByPoints, gradByTimes);
    cost += weight_time * times.sum();
    gradByTimes.array() += weight_time;

    Eigen::MatrixX3d GradByPoints_Position = Eigen::MatrixX3d::Zero(pieceN - 1, 3);
    Eigen::VectorXd GradByTimes_Position = Eigen::VectorXd::Zero(pieceN);


    double cost_Position;
    cost_Position += ComputeCostAndGradient_Position(GradByPoints_Position, GradByTimes_Position);
    gradByPoints += GradByPoints_Position.transpose();
    GradByTimes += GradByTimes_Position;
    cost += cost_Position;

    backwardGradT(tau, gradByTimes, gradTau);
    backwardGradP(xi, gradByPoints, gradXi);
    return cost; // 返回总代价
}

// 代价函数及其梯度计算
double Trajectory_Opimization_Node::ComputeCostAndGradient_Position(Eigen::MatrixX3d& GradByPoints, Eigen::VectorXd& GradByTimes) {
    double cost = 0.0;
    for (int i = 0; i < A_Star_Path.size(); i++) {
        Eigen::Vector3d P = minco.b.block((i + 1) * 6 + 0, 0, 1, 3).transpose();
        Eigen::Vector3d V = minco.b.block((i + 1) * 6 + 1, 0, 1, 3).transpose();
        double delta_x = P(0) - A_Star_Path[i].position(0);
        double delta_y = P(1) - A_Star_Path[i].position(1);
        double delta_w = P(2) - A_Star_Path[i].position(2);
        Eigen::Vector3d delta = P - A_Star_Path[i].position;
        Eigen::Vector3d delta_2 = Eigen::Vector3d(delta_x * delta_x, delta_y * delta_y, delta_w * delta_w);
        Eigen::Vector3d C = Eigen::Vector3d(weight_position_x, weight_position_y, weight_position_w);
        GradByPoints.row(i) = 2 * delta.cwiseProduct(C).transpose();
        GradByTimes(i + 1) = GradByPoints.row(i).dot(V);
        cost += delta_2.dot(C);
    }
    return cost;
}

// tao--->T
void Trajectory_Opimization_Node::forwardT(const Eigen::VectorXd &tau, Eigen::VectorXd &T)
{
    const int sizeTau = tau.size();
    T.resize(sizeTau);
    for (int i = 0; i < sizeTau; i++)
    {
        T(i) = tau(i) > 0.0
                    ? ((0.5 * tau(i) + 1.0) * tau(i) + 1.0)
                    : 1.0 / ((0.5 * tau(i) - 1.0) * tau(i) + 1.0);
    }
    return;
}
// T--->tao
template <typename EIGENVEC>
void Trajectory_Opimization_Node::backwardT(const Eigen::VectorXd &T, EIGENVEC &tau)
{
    const int sizeT = T.size();
    tau.resize(sizeT);
    for (int i = 0; i < sizeT; i++)
    {
        tau(i) = T(i) > 1.0
                        ? (sqrt(2.0 * T(i) - 1.0) - 1.0)
                        : (1.0 - sqrt(2.0 / T(i) - 1.0));
    }
    return;
}

template <typename EIGENVEC>
void Trajectory_Opimization_Node::backwardGradT(const Eigen::VectorXd &tau,
                                    const Eigen::VectorXd &gradT,
                                    EIGENVEC &gradTau)
{
    const int sizeTau = tau.size();
    gradTau.resize(sizeTau);
    double denSqrt;

    for (int i = 0; i < sizeTau; i++)
    {
        if (tau(i) > 0)
        {
            gradTau(i) = gradT(i) * (tau(i) + 1.0);
        }
        else
        {
            denSqrt = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
            gradTau(i) = gradT(i) * (1.0 - tau(i)) / (denSqrt * denSqrt);
        }
    }

    return;
}

template <typename EIGENVEC>
void Trajectory_Opimization_Node::backwardGradP(const Eigen::VectorXd &xi,
                                    const Eigen::Matrix3Xd &gradP,
                                    EIGENVEC &gradXi)
{
    const int sizeP = gradP.cols();
    for (int i = 0; i < sizeP; ++i)
    {
        gradXi.segment(3 * i, 3) = gradP.col(i);
    }
    return;
}

void Trajectory_Opimization_Node::forwardP(const Eigen::VectorXd &xi, Eigen::Matrix3Xd &P)
{
    const int sizeP = xi.size() / 3;
    P.resize(3, sizeP);
    for (int i = 0; i < sizeP; i++)
    {
        P.col(i) = xi.segment(3 * i, 3);
    }
    return;
}

template <typename EIGENVEC>
void Trajectory_Opimization_Node::backwardP(const Eigen::Matrix3Xd &P, EIGENVEC &xi)
{
    const int sizeP = P.cols();
    for (int i = 0; i < sizeP; ++i)
    {
        xi.segment(3 * i, 3) = P.col(i);
    }
    return;
}

void Trajectory_Opimization_Node::visualizeTrajectory(const Eigen::MatrixXd &b, const Eigen::VectorXd &times)
{
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = "odom";
    delete_marker.header.stamp = Header.stamp;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);
    Trajectory_Opimization_Publisher_->publish(marker_array);
    marker_array.markers.clear(); // 清空数组以便添加新标记

    // 生成新的轨迹标记
    double total_duration = times.sum(); // 计算总时间
    int ID = 0;

    // 使用0.05s步长遍历总时间范围
    for (double t = 0.0; t <= total_duration; t += 0.05) {
        // 生成当前时间点的轨迹位置
        Eigen::Vector3d point = generatePolynomialTrajectory(b, times, t);

        visualization_msgs::msg::Marker point_marker;
        point_marker.header.frame_id = "odom";
        point_marker.header.stamp = Header.stamp;
        point_marker.ns = "trajectory";
        point_marker.type = visualization_msgs::msg::Marker::SPHERE;  // 使用SPHERE类型表示单个点
        point_marker.action = visualization_msgs::msg::Marker::ADD;

        // 为每个点设置唯一的ID
        point_marker.id = ID++;  // 为每个点分配唯一的ID

        // 设置Marker的比例
        point_marker.scale.x = 0.05;  // 点的大小
        point_marker.scale.y = 0.05;
        point_marker.scale.z = 0.05;

        // 设置Marker的颜色
        point_marker.color.r = 0.0;  // 红色分量
        point_marker.color.g = 0.0;  // 绿色分量
        point_marker.color.b = 1.0;  // 蓝色分量
        point_marker.color.a = 1.0;  // 透明度（alpha）

        // 设置点的坐标
        Eigen::VectorXd Point_Map(4);
        Point_Map <<    point.x(),
                        point.y(),
                        0,
                        1.0;

        Eigen::VectorXd Transformed_Point_Map = Car_Odom_Rotation_Translation_Matrix * Point_Map;
        point_marker.pose.position.x = Transformed_Point_Map(0, 0);
        point_marker.pose.position.y = Transformed_Point_Map(1, 0);
        point_marker.pose.position.z = 0.3;

        // 将Marker添加到MarkerArray
        marker_array.markers.push_back(point_marker);
    }

    // 发布新的轨迹标记
    Trajectory_Opimization_Publisher_->publish(marker_array);
}

// 生成多项式曲线的点
Eigen::Vector3d Trajectory_Opimization_Node::generatePolynomialTrajectory(const Eigen::MatrixXd& coefficients, const Eigen::VectorXd& times, double t)
{
    int current_segment = 0;
    double segment_start_time = 0.0;

    // 确定当前时间点属于哪个轨迹段
    while (current_segment < times.size() && t > (segment_start_time + times[current_segment])) {
        segment_start_time += times[current_segment];
        current_segment++;
    }

    // 如果时间超过了所有轨迹段的总时间，则返回最后一个点的位置
    if (current_segment >= times.size()) {
        current_segment = times.size() - 1;
        t = segment_start_time;
    }

    // 计算局部时间（相对于当前轨迹段的时间）
    double local_t = t - segment_start_time;

    // 根据当前轨迹段的多项式系数计算位置
    double x = 0.0;
    double y = 0.0;

    Eigen::MatrixXd current_coeff = coefficients.block<6, 3>(6 * current_segment, 0);

    for (int j = 0; j < 6; ++j) { // 计算五次多项式
        x += current_coeff(j, 0) * pow(local_t, j); // X方向
        y += current_coeff(j, 1) * pow(local_t, j); // Y方向
    }

    return Eigen::Vector3d(x, y, 0.0);
}