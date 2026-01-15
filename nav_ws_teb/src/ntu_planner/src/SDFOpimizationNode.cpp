#include "SDFOpimizationNode.hpp"
SDF_Opimization_Node::SDF_Opimization_Node(std::shared_ptr<TFSubscriberNode> tf_subscriber_node)
    : Node("control_node"), tf_subscriber_node_(tf_subscriber_node) {
    // 初始化订阅者   
    Init_Traj_Subscriber_ = this->create_subscription<initial_optimized_msgs::msg::InitialOptimizedTrajectory>("/Initial_Optimized_Trajectory", 10, std::bind(&SDF_Opimization_Node::Callback, this, std::placeholders::_1));
    // 初始化发布者
    Swept_Volume_Publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("Swept_Volume_Publisher", 10);
    Trajectory_Opimization_Publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("SDF_trajectory_opimization_marker_array", 10);
    SDF_Opimization_Publisher_ = this->create_publisher<sdf_optimized_msgs::msg::SDFOptimizedTrajectory>("SDF_opimization_trajectory", 10);
    config_yaml_path = "/root/PersonalData/Program/multi-axle-all-wheel-steering-vehicles_ws/src/route_planning/config/route_planning_config.yaml";
}

void SDF_Opimization_Node::Callback(const initial_optimized_msgs::msg::InitialOptimizedTrajectory::ConstSharedPtr & init_traj_msg)
{
    Header = init_traj_msg->header;
    Car_Odom_Matrix = tf_subscriber_node_ -> Matrix_Read("odom", "car_base");           // 车辆    ->  世界 变换矩阵获取(放在回调函数最开始，保证和初始数据时间戳尽量同步)
    Odom_Car_Matrix = tf_subscriber_node_ -> Matrix_Read("car_base", "odom");
    Car_Odom_Rotation_Translation_Matrix = Car_Odom_Matrix.Rotation_Translation_Read();
    Odom_Car_Rotation_Translation_Matrix = Odom_Car_Matrix.Rotation_Translation_Read();
    Car_Quaternion = Car_Odom_Matrix.Quaternion_Read();
    config = YAML::LoadFile(config_yaml_path);

    Obstacle_Points.resize(3, init_traj_msg->obstacle_points.size());
    for(int i=0; i<init_traj_msg->obstacle_points.size(); i++)
    {
        Obstacle_Points.col(i) <<   init_traj_msg->obstacle_points[i].x,
                                    init_traj_msg->obstacle_points[i].y,
                                    init_traj_msg->obstacle_points[i].z;
    }
    
    traj.pieceN = init_traj_msg->position.size() - 1;
    // std::cout<<traj.pieceN<<std::endl;
    traj.b = Eigen::MatrixX3d::Zero(6 * traj.pieceN, 3);
    traj.times.resize(traj.pieceN);
    for(int i=0; i< traj.pieceN; i++)
    {
        traj.times[i] = init_traj_msg->times[i];
    }
    Eigen::Matrix3d initState = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d finalState = Eigen::Matrix3d::Zero();
    initState.col(0) << init_traj_msg->position.front().x,
                        init_traj_msg->position.front().y,
                        init_traj_msg->position.front().z;
    traj.points.resize(3, init_traj_msg->position.size() - 2);
    for(int i=0; i<init_traj_msg->position.size() - 2; i++)
    {
        traj.points(0, i) = init_traj_msg->position[i + 1].x;
        traj.points(1, i) = init_traj_msg->position[i + 1].y;
        traj.points(2, i) = init_traj_msg->position[i + 1].z;
    }
    finalState.col(0) <<init_traj_msg->position.back().x,
                        init_traj_msg->position.back().y,
                        init_traj_msg->position.back().z;
    
    minco.setConditions(initState, finalState, traj.pieceN);
    minco.setParameters(traj.points, traj.times);
    traj.b = minco.b;
    
    //优化器设置
    temporalDim = traj.pieceN;
    spatialDim = 3 * (traj.pieceN - 1);
    const int total_opt_variable_num = temporalDim + spatialDim;
    Eigen::VectorXd x(total_opt_variable_num);
    Eigen::Map<Eigen::VectorXd> tau(x.data(), temporalDim);
    Eigen::Map<Eigen::VectorXd> xi(x.data() + temporalDim, spatialDim);
    backwardT(traj.times, tau);
    for (int i = 0; i < traj.pieceN - 1; ++i)
    {
        xi.segment(3 * i, 3)  = traj.points.col(i);
    }

    GPUProcessConfig(config);

    opimization_times = 0;
    opimization_times_max = config["SDF_opimiz_times_max"].as<double>();
    weight_time = config["SDF_opimiz_weight_time"].as<double>();
    weight_energy_x = config["SDF_opimiz_weight_energy_x"].as<double>();
    weight_energy_y = config["SDF_opimiz_weight_energy_y"].as<double>();
    weight_energy_w = config["SDF_opimiz_weight_energy_w"].as<double>();
    lmbm::lmbm_parameter_t param;
    double final_cost;
    auto start_time = std::chrono::high_resolution_clock::now();
    int ret = lmbm::lmbm_optimize(  total_opt_variable_num,
                                    x.data(),
                                    &final_cost,
                                    costFunctionLmbmParallelWrapper,
                                    this,
                                    earlyExitLMBMWrapper,
                                    &param);
    forwardT(tau, traj.times);
    forwardP(xi, traj.points);
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

    std::cout<<"ret: "<<ret<<std::endl;
    if(ret > 0)
    {    
        std::cout<<"Successful Process took "<<duration/1000000.0<<" s."<<std::endl;
        minco.setParameters(traj.points, traj.times);
        traj.b = minco.b;
        std::vector<SDF_Map_> SDF_Map;
        double area = 0;
        GPUProcessSDF(traj, SDF_Map, area, config["SDF_TEST"].as<bool>());
        std::cout<<"swept area: "<<area<<std::endl;

        sdf_optimized_msgs::msg::SDFOptimizedTrajectory message;
        message.header = Header;
        
        message.rotation_translation_matrix.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        message.rotation_translation_matrix.layout.dim[0].size = 4;
        message.rotation_translation_matrix.layout.dim[0].stride = 16;  // 4x4=16
        message.rotation_translation_matrix.layout.dim[0].label = "matrix";

        // 将Eigen矩阵的数据压入Float64MultiArray的data字段
        for (int i = 0; i < Car_Odom_Rotation_Translation_Matrix.rows(); ++i) {
            for (int j = 0; j < Car_Odom_Rotation_Translation_Matrix.cols(); ++j) {
                message.rotation_translation_matrix.data.push_back(Car_Odom_Rotation_Translation_Matrix(i, j));
            }
        }
        geometry_msgs::msg::Vector3 p;
        p.x = init_traj_msg->position.front().x;
        p.y = init_traj_msg->position.front().y;
        p.z = init_traj_msg->position.front().z;
            message.position.push_back(p);
        for(int i=0; i<traj.points.cols(); i++)
        {
            p.x = traj.points(0, i);
            p.y = traj.points(1, i);
            p.z = traj.points(2, i);
            message.position.push_back(p);
        }
        p.x = init_traj_msg->position.back().x;
        p.y = init_traj_msg->position.back().y;
        p.z = init_traj_msg->position.back().z;
            message.position.push_back(p);
        for(int i=0; i<traj.pieceN; i++)
        {
            message.times.push_back(traj.times(i));
        }
        SDF_Opimization_Publisher_->publish(message);

        visualizeTrajectory(traj.b, traj.times);
        visualizeSweptVolume(SDF_Map, SDF_Map.size());
    }
    else
    {
        std::cout<<"Unsuccessful Process took "<<duration/1000000.0<<" s."<<std::endl;
    }
}

double SDF_Opimization_Node::costFunctionLmbmParallel(void *ptr, const double *x_variable, double *g, const int n)
{
    const int dimTau     = temporalDim;
    const int dimXi      = spatialDim;
    Eigen::Map<const Eigen::VectorXd> tau(x_variable, dimTau); // 将参数映射为tau
    Eigen::Map<const Eigen::VectorXd> xi(x_variable + dimTau, dimXi); // 将参数映射为xi
    Eigen::Map<Eigen::VectorXd> gradTau(g, dimTau); // 将梯度映射为gradTau
    Eigen::Map<Eigen::VectorXd> gradXi(g + dimTau, dimXi); // 将梯度映射为gradXi

    forwardT(tau, traj.times); 
    forwardP(xi, traj.points);

    double cost;
    Eigen::MatrixX3d GradByCoeffs(6 * traj.pieceN, 3);
    Eigen::VectorXd GradByTimes(traj.pieceN);
    Eigen::Matrix3Xd gradByPoints;
    Eigen::VectorXd gradByTimes;
    minco.setParameters(traj.points, traj.times);
    traj.b = minco.b;
    
    minco.getEnergy(cost, weight_energy_x, weight_energy_y, weight_energy_w);
    minco.getEnergyPartialGradByCoeffs(GradByCoeffs, weight_energy_x, weight_energy_y, weight_energy_w); // ∂E/∂c
    minco.getEnergyPartialGradByTimes(GradByTimes, weight_energy_x, weight_energy_y, weight_energy_w);   // ∂E/∂T
    minco.propogateGrad(GradByCoeffs, GradByTimes, gradByPoints, gradByTimes);
    cost += weight_time * traj.times.sum();
    gradByTimes.array() += weight_time;

    Eigen::MatrixX3d GradByPoints_Ob = Eigen::MatrixX3d::Zero(traj.pieceN, 3);
    Eigen::VectorXd GradByTimes_Ob = Eigen::VectorXd::Zero(traj.pieceN);
    double cost_Ob;
    GPUProcessGradSDF(traj, Obstacle_Points, GradByPoints_Ob, GradByTimes_Ob, cost_Ob);
    gradByPoints += GradByPoints_Ob.topRows(GradByPoints_Ob.rows() - 1).transpose();
    GradByTimes += GradByTimes_Ob;
    cost += cost_Ob;

    Eigen::MatrixX3d GradByPoints_Yaw = Eigen::MatrixX3d::Zero(traj.pieceN, 3);
    Eigen::VectorXd GradByTimes_Yaw = Eigen::VectorXd::Zero(traj.pieceN);
    double cost_Yaw;
    GPUProcessGradYaw(traj, GradByPoints_Yaw, GradByTimes_Yaw, cost_Yaw);
    gradByPoints += GradByPoints_Yaw.topRows(GradByPoints_Yaw.rows() - 1).transpose();
    GradByTimes += GradByTimes_Yaw;
    cost += cost_Yaw;

    backwardGradP(xi, gradByPoints, gradXi);
    backwardGradT(tau, gradByTimes, gradTau);

    visualizeTrajectory(traj.b, traj.times);
    return cost;
}

int SDF_Opimization_Node::earlyExitLMBM(void *instance, const double *x, const int k)
{
    opimization_times++;
    return opimization_times > opimization_times_max;
}

// tao--->T
void SDF_Opimization_Node::forwardT(const Eigen::VectorXd &tau, Eigen::VectorXd &T)
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
void SDF_Opimization_Node::backwardT(const Eigen::VectorXd &T, EIGENVEC &tau)
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
void SDF_Opimization_Node::backwardGradT(const Eigen::VectorXd &tau,
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
void SDF_Opimization_Node::backwardGradP(const Eigen::VectorXd &xi,
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

void SDF_Opimization_Node::backwardGradT(const double* tau, const Eigen::VectorXd &gradT, double * gradTau, const int sizeTau)
{
    double denSqrt;

    for (int i = 0; i < sizeTau; i++)
    {
        if (tau[i] > 0)
        {
            gradTau[i] = gradT[i] * (tau[i] + 1.0);
        }
        else
        {
            denSqrt = (0.5 * tau[i] - 1.0) * tau[i] + 1.0;
            gradTau[i] = gradT[i] * (1.0 - tau[i]) / (denSqrt * denSqrt);
        }
    }

    return;
}

void SDF_Opimization_Node::backwardGradP(const Eigen::Matrix3Xd &gradP, double *gradXi, const int Space_dim)
{

    for (int i = 0; i < Space_dim; ++i)
    {
            Eigen::Map<Eigen::VectorXd>(gradXi+ 3 * i, 3) = gradP.col(i);
    }
    return;
}

void SDF_Opimization_Node::forwardP(const Eigen::VectorXd &xi, Eigen::Matrix3Xd &P)
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
void SDF_Opimization_Node::backwardP(const Eigen::Matrix3Xd &P, EIGENVEC &xi)
{
    const int sizeP = P.cols();
    for (int i = 0; i < sizeP; ++i)
    {
        xi.segment(3 * i, 3) = P.col(i);
    }
    return;
}

void SDF_Opimization_Node::visualizeSweptVolume(std::vector<SDF_Map_> SDF_Map, int Point_Num)
{
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = "odom";
    delete_marker.header.stamp = Header.stamp;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);
    Swept_Volume_Publisher_->publish(marker_array);
    marker_array.markers.clear(); // 清空数组以便添加新标记

    // 初始化一个矩阵来存储所有点的坐标（同一列表示一个点的坐标）
    Eigen::MatrixXd Point_Map(4, SDF_Map.size());
    Point_Map.setZero(); // 初始化所有元素为0
    for (int i=0; i<SDF_Map.size(); i++)
    {
        double f_sdf_min = SDF_Map[i].SDF;

        // 检查是否在物体内部（负值表示在内部）
        if (f_sdf_min <= 0) {
            // 该点在物体内部，将其加入Point_Map
            Point_Map(0, i) = SDF_Map[i].x;
            Point_Map(1, i) = SDF_Map[i].y;
            Point_Map(2, i) = -1.5;  // Z值设为0，假设在平面上
            Point_Map(3, i) = 1.0;  // 齐次坐标
        }
    }

    int ID = 0;
    // 将点的数量调整为实际存储的点数
    Point_Map.conservativeResize(4, Point_Num);
    // 使用矩阵乘法一次性转换所有点
    Eigen::MatrixXd Transformed_Point_Map = Car_Odom_Rotation_Translation_Matrix * Point_Map;
    // 遍历转换后的点，生成标记
    for (int i = 0; i < Point_Num; ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = Header.stamp;
        marker.ns = "swept_volume";
        marker.id = ID++;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        // 使用转换后的坐标
        marker.pose.position.x = Transformed_Point_Map(0, i);
        marker.pose.position.y = Transformed_Point_Map(1, i);
        marker.pose.position.z = Transformed_Point_Map(2, i);
        marker.pose.orientation = Car_Quaternion;
        double resolution = config["SDF_resolution"].as<double>();
        marker.scale.x = resolution; // Cube size for visualization
        marker.scale.y = resolution;
        marker.scale.z = resolution;
        marker.color.a = 0.8;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;

        // 添加到标记数组
        marker_array.markers.push_back(marker);
    }
    // 发布标记数组
    Swept_Volume_Publisher_->publish(marker_array);
}

void SDF_Opimization_Node::visualizeTrajectory(const Eigen::MatrixXd &b, const Eigen::VectorXd &times)
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
        point_marker.scale.x = 0.1;  // 点的大小
        point_marker.scale.y = 0.1;
        point_marker.scale.z = 0.1;

        // 设置Marker的颜色
        point_marker.color.r = 1.0;  // 红色分量
        point_marker.color.g = 1.0;  // 绿色分量
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
Eigen::Vector3d SDF_Opimization_Node::generatePolynomialTrajectory(const Eigen::MatrixXd& coefficients, const Eigen::VectorXd& times, double t)
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