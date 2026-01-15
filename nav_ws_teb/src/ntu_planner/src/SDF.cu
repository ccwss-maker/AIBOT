#include "ntu_planner/SDF.cuh"
#include <chrono>

double Car_Length;
double Car_Length_0;
double Car_Width;
double x_min;
double x_max;
double y_min;
double y_max;
double resolution;
double grid_size;

// 定义常量内存
__constant__ double c_x_min;
__constant__ double c_x_max;
__constant__ double c_y_min;
__constant__ double c_y_max;
__constant__ double c_resolution;
__constant__ double c_grid_size;
__constant__ double c_time_step;
__constant__ double c_eta;
__constant__ double c_c;
__constant__ double c_tol;
__constant__ double c_max_iter;
__constant__ double c_Car_Length;
__constant__ double c_Car_Width;
__constant__ double c_total_duration;
__constant__ double c_safety_hor;
__constant__ double c_weight_safety;
__constant__ double c_weight_swept_volume;
__constant__ int c_num_grid_x;
__constant__ int c_num_grid_y;
__constant__ int c_num_points_x;
__constant__ int c_num_points_y;
__constant__ int c_pieceN;

__device__ void findCurrentSegmentAndLocalTime(GPU_Initial_Optimized_Trajectory_ Gpu_Traj, double t, int* segment, double* local_t) {
    int left = 0;
    int right = c_pieceN - 1;
    double total_time = 0.0;

    // 二分查找
    while (left <= right) {
        int mid = left + (right - left) / 2;
        double segment_end_time = 0.0;

        // 计算到mid段为止的总时间
        for (int i = 0; i <= mid; i++) {
            segment_end_time += Gpu_Traj.times[i];
        }

        if (t < segment_end_time - Gpu_Traj.times[mid]) {
            right = mid - 1;
        } else if (t > segment_end_time) {
            left = mid + 1;
        } else {
            *segment = mid;
            total_time = segment_end_time - Gpu_Traj.times[mid];
            break;
        }
    }

    // 如果时间超过了所有轨迹段的总时间，则返回最后一个段
    if (left > right) {
        *segment = c_pieceN - 1;
        total_time = 0.0;
        for (int i = 0; i < c_pieceN - 1; i++) {
            total_time += Gpu_Traj.times[i];
        }
    }
    // 计算局部时间（相对于当前轨迹段的时间）
    *local_t = t - total_time;
}

// 计算当前时间 t 下的位置信息
__device__ void computePositionAtPiece(GPU_Initial_Optimized_Trajectory_ Gpu_Traj, int current_segment, double local_t, double P[2]) {

    double *coeffx = Gpu_Traj.b + (c_pieceN * 6 * 0 + 6 * current_segment);
    double *coeffy = Gpu_Traj.b + (c_pieceN * 6 * 1 + 6 * current_segment);

    // 使用 Horner 法则计算多项式值
    double result[2] = {0};
    result[0] = coeffx[5];
    result[1] = coeffy[5];
    for (int j = 4; j >= 0; --j) {
        result[0] = result[0] * local_t + coeffx[j];
        result[1] = result[1] * local_t + coeffy[j];
    }
    P[0] = result[0];
    P[1] = result[1];
}

// 计算当前时间 t 下的速度
__device__ void computeVelocityAtPiece(GPU_Initial_Optimized_Trajectory_ Gpu_Traj, int current_segment, double local_t, double V[2]) {

    double* coeffx = Gpu_Traj.b + c_pieceN * 6 * 0 + 6 * current_segment;
    double* coeffy = Gpu_Traj.b + c_pieceN * 6 * 1 + 6 * current_segment;

    // 使用 Horner 法则计算多项式的一阶导数
    double result[2] = {0};
    result[0] = 5 * coeffx[5];
    result[1] = 5 * coeffy[5];            
    for (int j = 4; j > 0; --j) {
        result[0] = result[0] * local_t + j * coeffx[j];
        result[1] = result[1] * local_t + j * coeffy[j];
    }
    V[0] = result[0];
    V[1] = result[1];
}

// 计算当前时间 t 下的加速度
__device__ void computeAccelerationAtPiece(GPU_Initial_Optimized_Trajectory_ Gpu_Traj, int current_segment, double local_t, double A[2]) {
    double* coeffx = Gpu_Traj.b + c_pieceN * 6 * 0 + 6 * current_segment;
    double* coeffy = Gpu_Traj.b + c_pieceN * 6 * 1 + 6 * current_segment;

    // 使用 Horner 法则计算多项式的二阶导数
    double result[2] = {0};
    result[0] = 20 * coeffx[5];
    result[1] = 20 * coeffy[5];
    for (int j = 4; j > 1; --j) {
        result[0] = result[0] * local_t + j * (j - 1) * coeffx[j];
        result[1] = result[1] * local_t + j * (j - 1) * coeffy[j];
    }
    A[0] = result[0];
    A[1] = result[1];
}

// 计算当前时间 t 下的航向角信息
__device__ void computeYawAtPiece(GPU_Initial_Optimized_Trajectory_ Gpu_Traj, int current_segment, double local_t, double *Yaw) {

    double *coeffcita = Gpu_Traj.b + (c_pieceN * 6 * 2 + 6 * current_segment);

    // 使用 Horner 法则计算多项式值
    double result = 0;
    result = coeffcita[5];
    for (int j = 4; j >= 0; --j) {
        result = result * local_t + coeffcita[j];
    }
    *Yaw = result;
}

// 计算当前时间 t 下的速度
__device__ void computeYawVelocityAtPiece(GPU_Initial_Optimized_Trajectory_ Gpu_Traj, int current_segment, double local_t, double *W) {

    double *coeffcita = Gpu_Traj.b + (c_pieceN * 6 * 2 + 6 * current_segment);

    // 使用 Horner 法则计算多项式值
    double result = 0;
    result = 5 * coeffcita[5];
    for (int j = 4; j >= 0; --j) {
        result = result * local_t + j * coeffcita[j];
    }
    *W = result;
}

// 计算当前时间 t 下的旋转矩阵
__device__ void computeRotationMatrix(double yaw, double R[2][2]) {
    // 生成二维旋转矩阵 R(t)
    R[0][0] = cos(yaw);
    R[0][1] = -sin(yaw);
    R[1][0] = sin(yaw);
    R[1][1] = cos(yaw);
}

// 计算当前时间 t 下的旋转矩阵导数
__device__ void computeRotationDot(double V[], double A[], double yaw, double RDot[2][2]) {

    double denominator = V[0] * V[0] + V[1] * V[1];
    
    // 添加一个小的阈值来避免除以零
    const double epsilon = 1e-6;
    
    double yaw_dot;
    if (denominator > epsilon) {
        yaw_dot = (V[1] * A[0] - V[0] * A[1]) / denominator;
    } else {
        // 当速度接近零时，假设 yaw_dot 为零
        yaw_dot = 0.0;
    }

    // 生成二维旋转矩阵 R(t)
    RDot[0][0] = -sin(yaw) * yaw_dot;
    RDot[0][1] = -cos(yaw) * yaw_dot;
    RDot[1][0] = cos(yaw) * yaw_dot;
    RDot[1][1] = -sin(yaw) * yaw_dot;
}

// 计算基于机器人运动的 SDF 函数
__device__ double fsdf(GPU_Initial_Optimized_Trajectory_ Gpu_Traj, double P_ob[2], double t) {
    double P[2] = {0};
    double yaw = 0;
    double R[2][2] = {0};
    double P_rel[2] = {0};
    int current_segment;
    double local_t;
    findCurrentSegmentAndLocalTime(Gpu_Traj, t, &current_segment, &local_t);
    computePositionAtPiece(Gpu_Traj, current_segment, local_t, P);
    computeYawAtPiece(Gpu_Traj, current_segment, local_t, &yaw);
    computeRotationMatrix(yaw, R);

    P_rel[0] = R[0][0] * (P_ob[0] - P[0]) + R[1][0] * (P_ob[1] - P[1]);
    P_rel[1] = R[0][1] * (P_ob[0] - P[0]) + R[1][1] * (P_ob[1] - P[1]);
    // 基于局部坐标计算车辆的 SDF 值
    double half_length = c_Car_Length / 2.0;  // 车辆长的一半
    double half_width = c_Car_Width / 2.0;    // 车辆宽的一半

    double dx = fabs(P_rel[0]) - half_length;
    double dy = fabs(P_rel[1]) - half_width;
    double sdf;
    if(dx > 0 && dy > 0)
    {
        sdf = hypot(dx, dy);
    }
    else
    { 
        sdf = fmax(dx, dy);
    }

    return sdf;
}

// 计算基于机器人运动的 SDF 函数
__device__ double fsdf_P_rel(GPU_Initial_Optimized_Trajectory_ Gpu_Traj, double P_rel[2]) {
    // 基于局部坐标计算车辆的 SDF 值
    double half_length = c_Car_Length / 2.0;  // 车辆长的一半
    double half_width = c_Car_Width / 2.0;    // 车辆宽的一半

    double dx = fabs(P_rel[0]) - half_length;
    double dy = fabs(P_rel[1]) - half_width;
    double sdf;
    if(dx > 0 && dy > 0)
    {
        sdf = hypot(dx, dy);
    }
    else
    { 
        sdf = fmax(dx, dy);
    }

    return sdf;
}
__device__ void fsdf_grad_P(double P_rel[2], double sdf_grad[2])
{
    // 基于局部坐标计算车辆的 SDF 值
    double half_length = c_Car_Length / 2.0;  // 车辆长的一半
    double half_width = c_Car_Width / 2.0;    // 车辆宽的一半

    double dx = fabs(P_rel[0]) - half_length;
    double dy = fabs(P_rel[1]) - half_width;
    if(dx > 0 && dy > 0)
    {
        double dxy = hypot(dx, dy);
        double inv_dxy = 1.0 / fmax(dxy, 1e-10);
        sdf_grad[0] = dx * inv_dxy * copysign(1.0, P_rel[0]);
        sdf_grad[1] = dy * inv_dxy * copysign(1.0, P_rel[1]);
    }
    else if(dx >= dy)
    {
        sdf_grad[0] = copysign(1.0, P_rel[0]);
        sdf_grad[1] = 0;
    }
    else if(dy > dx)
    {
        sdf_grad[0] = 0;
        sdf_grad[1] = copysign(1.0, P_rel[1]);
    }
}

// 计算 SDF 关于时间的导数
__device__ double fsdf_dot(GPU_Initial_Optimized_Trajectory_ Gpu_Traj, double P_ob[2], double t) {    
    // 获取当前时间 t 下的信息
    double P[2] = {0};
    double V[2] = {0};
    double A[2] = {0};
    double yaw = 0;
    double R[2][2] = {0};
    double RDot[2][2] = {0};
    double P_rel[2] = {0};
    int current_segment;
    double local_t;
    findCurrentSegmentAndLocalTime(Gpu_Traj, t, &current_segment, &local_t);
    computePositionAtPiece(Gpu_Traj, current_segment, local_t, P);
    computeVelocityAtPiece(Gpu_Traj, current_segment, local_t, V);
    computeAccelerationAtPiece(Gpu_Traj, current_segment, local_t, A);
    computeYawAtPiece(Gpu_Traj, current_segment, local_t, &yaw);
    computeRotationMatrix(yaw, R);
    computeRotationDot(V, A, yaw, RDot);

    P_rel[0] = R[0][0] * (P_ob[0] - P[0]) + R[1][0] * (P_ob[1] - P[1]);
    P_rel[1] = R[0][1] * (P_ob[0] - P[0]) + R[1][1] * (P_ob[1] - P[1]);
    // 计算梯度 SDF
    double fsdf_grad_Prel[2];
    fsdf_grad_P(P_rel, fsdf_grad_Prel);

    // 计算 RDot * P_rel
    double rot_dot_times_xrel[2];
    rot_dot_times_xrel[0] = RDot[0][0] * (P_rel[0]) + RDot[0][1] * (P_rel[1]);
    rot_dot_times_xrel[1] = RDot[1][0] * (P_rel[0]) + RDot[1][1] * (P_rel[1]);
    
    // 计算 (RDot * P_rel - V)
    double inner_term[2];
    inner_term[0] = rot_dot_times_xrel[0] - V[0];
    inner_term[1] = rot_dot_times_xrel[1] - V[1];
    
    // 计算 R.transpose() * (RDot * P_rel - V)
    double result[2];
    result[0] = R[0][0] * inner_term[0] + R[1][0] * inner_term[1];
    result[1] = R[0][1] * inner_term[0] + R[1][1] * inner_term[1];
    
    // 计算梯度和结果的点积
    double fsdf_dot_value = fsdf_grad_Prel[0] * result[0] + fsdf_grad_Prel[1] * result[1];

    return fsdf_dot_value;
}

__device__ double min_sdf(GPU_Initial_Optimized_Trajectory_ Gpu_Traj, double grid_t_closest[], double P_ob[2], double *t_star)
{
    // 找到点(x, y)所在的大网格
    int grid_x = static_cast<int>((P_ob[0] - c_x_min) / c_grid_size);
    int grid_y = static_cast<int>((P_ob[1] - c_y_min) / c_grid_size);
    double t_start = grid_t_closest[c_num_grid_y * grid_x + grid_y];
    // 使用梯度下降和线搜索找到每个点的最小 SDF
    double t = t_start;
    int iter = 0;
    for (iter = 0; iter < c_max_iter; ++iter) {
        double gradient = fsdf_dot(Gpu_Traj, P_ob, t);
        double direction = -gradient;
        double step = c_eta;

        // Armijo线搜索
        while (fsdf(Gpu_Traj, P_ob, t + step * direction) > fsdf(Gpu_Traj, P_ob, t) + c_c * step * gradient) {
            step /= 2.0;
        }
        double t_next = t + step * direction;
        if (fabs(t_next - t) < c_tol) {
            t = t_next;
            break;
        }
        t = t_next;
    }
    if(t_star != NULL) *t_star = t;
    return fsdf(Gpu_Traj, P_ob, t);
}

// 计算SDF和t_closest
__global__ void find_T_Closest( GPU_Initial_Optimized_Trajectory_ Gpu_Traj, double grid_t_closest[]) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= c_num_grid_x * c_num_grid_y) return;

    int col = idx / c_num_grid_y;
    int row = idx % c_num_grid_y;

    double grid_center[2];
    grid_center[0] = c_x_min + (col + 0.5) * c_grid_size;
    grid_center[1] = c_y_min + (row + 0.5) * c_grid_size;

    double MAX_DOUBLE = 1.7976931348623157e+308;
    double min_sdf = MAX_DOUBLE;
    double t_closest = 0;

    for (double t = 0; t <= c_total_duration; t += c_time_step) {
        double sdf_value = fsdf(Gpu_Traj, grid_center, t);
        if (sdf_value < min_sdf) {
            min_sdf = sdf_value;
            t_closest = t;
        }
    }
    grid_t_closest[idx] = t_closest;
}

// 计算SDF
__global__ void Calculate_SDF( GPU_Initial_Optimized_Trajectory_ Gpu_Traj, double grid_t_closest[], SDF_Map_ point_map[]) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= c_num_points_x * c_num_points_y) return;

    int col = idx / c_num_points_y;
    int row = idx % c_num_points_y;

    double P_ob[2];
    P_ob[0] = c_x_min + c_resolution * col;
    P_ob[1] = c_y_min + c_resolution * row;

    double sdf = min_sdf(Gpu_Traj, grid_t_closest, P_ob, NULL);

    point_map[idx].x = P_ob[0];
    point_map[idx].y = P_ob[1];
    point_map[idx].SDF = sdf;
}

__global__ void Calculate_Grad_SDF_Ob(GPU_Initial_Optimized_Trajectory_ Gpu_Traj, double GPU_Obstacle_Points[], int Obstacle_N, double GPUGradByPositions[], double GPUGradByTimes[], double *GPUCost)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= Obstacle_N * Gpu_Traj.pieceN) return;

    // 声明共享内存
    __shared__ int pieceN;
    if(threadIdx.x == 0) pieceN = Gpu_Traj.pieceN;
    __syncthreads();

    int obstacle_ID = idx / (pieceN);
    int piece_ID = idx % (pieceN);
    
    double t_star = 0;
    double P_ob[2];
    P_ob[0] = GPU_Obstacle_Points[obstacle_ID * 3 + 0];
    P_ob[1] = GPU_Obstacle_Points[obstacle_ID * 3 + 1];

    // 获取当前时间 t* 下的信息
    double P[2] = {0};
    double V[2] = {0};
    double yaw = 0;
    double R[2][2] = {0};
    double P_rel[2] = {0};
    computePositionAtPiece(Gpu_Traj, piece_ID, t_star, P);
    computeVelocityAtPiece(Gpu_Traj, piece_ID, t_star, V);
    computeYawAtPiece(Gpu_Traj, piece_ID, t_star, &yaw);
    computeRotationMatrix(yaw, R);
    P_rel[0] = R[0][0] * (P_ob[0] - P[0]) + R[1][0] * (P_ob[1] - P[1]);
    P_rel[1] = R[0][1] * (P_ob[0] - P[0]) + R[1][1] * (P_ob[1] - P[1]);

    double sdf = fsdf_P_rel(Gpu_Traj, P_rel);
    double jsdf = 0;
    double delta_sdf = c_safety_hor - sdf;
    double delta_sdf_2 = delta_sdf * delta_sdf;
    double C = 0;
    if(delta_sdf > 0)   
    {
        jsdf = c_weight_safety * delta_sdf * delta_sdf_2;
        C = c_weight_safety * (-3) * delta_sdf_2;
    }
    else
    {
        jsdf = 0;
        C = 0;
    }                

    double fsdf_grad_Prel[2];
    fsdf_grad_P(P_rel, fsdf_grad_Prel);

    double grad_sdf_position[2];
    grad_sdf_position[0] = R[0][0] * fsdf_grad_Prel[0] + R[0][1] * fsdf_grad_Prel[1];
    grad_sdf_position[1] = R[1][0] * fsdf_grad_Prel[0] + R[1][1] * fsdf_grad_Prel[1];
    
    atomicAdd(&GPUGradByPositions[0 * pieceN + piece_ID], -C * grad_sdf_position[0]);
    atomicAdd(&GPUGradByPositions[1 * pieceN + piece_ID], -C * grad_sdf_position[1]);
    atomicAdd(&GPUGradByTimes[piece_ID], -C * (grad_sdf_position[0] * V[0] + grad_sdf_position[1] * V[1]));
    atomicAdd(GPUCost, jsdf);
}

__global__ void Calculate_Grad_Yaw(GPU_Initial_Optimized_Trajectory_ Gpu_Traj, double GPUGradByPoints_Yaw[], double GPUGradByTimes_Yaw[], double *GPUCost)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= Gpu_Traj.pieceN) return;

    // 声明共享内存
    __shared__ int pieceN;
    if(threadIdx.x == 0) pieceN = Gpu_Traj.pieceN;
    __syncthreads();

    int obstacle_ID = idx / (pieceN);
    int piece_ID = idx % (pieceN);
    
    double t_star = 0;

    // 获取当前时间 t 下的信息
    double V[2] = {0};
    double A[2] = {0};
    double yaw = 0;
    double w = 0;
    computeVelocityAtPiece(Gpu_Traj, piece_ID, t_star, V);
    computeAccelerationAtPiece(Gpu_Traj, piece_ID, t_star, A);
    computeYawAtPiece(Gpu_Traj, piece_ID, t_star, &yaw);
    computeYawVelocityAtPiece(Gpu_Traj, piece_ID, t_star, &w);
    double Yaw_Route;
    Yaw_Route = atan2(V[1], V[0]);
    double Yaw_delta = yaw - Yaw_Route;
    double cost = c_weight_swept_volume * Yaw_delta * Yaw_delta;
    double V_inv = 1.0 / fmax(V[0] * V[0] + V[1] * V[1], 1e-10);

    double gradient_Yaw = c_weight_swept_volume * 2 * Yaw_delta;
    double gradient_X = c_weight_swept_volume * 2 * Yaw_delta * (-V[1] * V_inv);
    double gradient_Y = c_weight_swept_volume * 2 * Yaw_delta * (V[0] * V_inv);
    double gradient_T = gradient_Yaw * w + gradient_X * V[0] + gradient_Y * V[1];
    atomicAdd(&GPUGradByPoints_Yaw[0 * pieceN + piece_ID], gradient_X);
    atomicAdd(&GPUGradByPoints_Yaw[1 * pieceN + piece_ID], gradient_Y);
    atomicAdd(&GPUGradByPoints_Yaw[2 * pieceN + piece_ID], gradient_Yaw);
    atomicAdd(&GPUGradByTimes_Yaw[piece_ID], gradient_T);
    atomicAdd(GPUCost, cost);
}

void GPUProcessConfig(YAML::Node config)
{
    // 定义车身几何参数
    Car_Length = config["Car_Length"].as<double>();
    Car_Length_0 = 0.15;;
    Car_Width = config["Car_Width"].as<double>();

    // 定义遍历的区域和分辨率（以米为单位）
    x_min = config["x_min"].as<double>();
    x_max = config["x_max"].as<double>();
    y_min = config["y_min"].as<double>();
    y_max = config["y_max"].as<double>();
    resolution = config["SDF_resolution"].as<double>();
    
    // 定义粗网格数据
    grid_size = config["coarse_grid_size"].as<double>();
    double time_step = config["find_t_star_time_step"].as<double>();
    
    // 定义梯度下降搜索参数
    double eta = config["eta"].as<double>();
    double c = config["c"].as<double>();
    double tol = config["tol"].as<double>();
    double max_iter = config["max_iter"].as<double>();

    //定义SDF安全阈值
    double safety_hor = config["SDF_safety_hor"].as<double>();
    double weight_safety = config["SDF_opimiz_weight_safety"].as<double>();

    //定义SV优化系数
    double weight_swept_volume = config["SDF_opimiz_weight_swept_volume"].as<double>();
    
    cudaMemcpyToSymbol(c_safety_hor, &safety_hor, sizeof(double));
    cudaMemcpyToSymbol(c_weight_safety, &weight_safety, sizeof(double));
    cudaMemcpyToSymbol(c_resolution, &resolution, sizeof(double));
    cudaMemcpyToSymbol(c_grid_size, &grid_size, sizeof(double));
    cudaMemcpyToSymbol(c_time_step, &time_step, sizeof(double));
    cudaMemcpyToSymbol(c_eta, &eta, sizeof(double));
    cudaMemcpyToSymbol(c_c, &c, sizeof(double));
    cudaMemcpyToSymbol(c_tol, &tol, sizeof(double));
    cudaMemcpyToSymbol(c_max_iter, &max_iter, sizeof(double));
    cudaMemcpyToSymbol(c_Car_Length, &Car_Length, sizeof(double));
    cudaMemcpyToSymbol(c_Car_Width, &Car_Width, sizeof(double));
    cudaMemcpyToSymbol(c_weight_swept_volume, &weight_swept_volume, sizeof(double));
}

void GPUProcessSDF(Optimized_Trajectory_ traj, std::vector<SDF_Map_>& SDF_Map, double& area, bool test) {

    if(test) 
    {
        Car_Length = Car_Length_0;
        cudaMemcpyToSymbol(c_Car_Length, &Car_Length_0, sizeof(double));
    }
    // 计算遍历区域最小范围（以米为单位）
    x_min = fmax(x_min, traj.points.row(0).minCoeff() - fmax(Car_Length, Car_Width));
    x_max = fmin(x_max, traj.points.row(0).maxCoeff() + fmax(Car_Length, Car_Width));
    y_min = fmax(y_min, traj.points.row(1).minCoeff() - fmax(Car_Length, Car_Width)); 
    y_max = fmin(y_max, traj.points.row(1).maxCoeff() + fmax(Car_Length, Car_Width));

    // 计算总时间
    double total_duration = traj.times.sum();

    cudaMemcpyToSymbol(c_x_min, &x_min, sizeof(double));
    cudaMemcpyToSymbol(c_x_max, &x_max, sizeof(double));
    cudaMemcpyToSymbol(c_y_min, &y_min, sizeof(double));
    cudaMemcpyToSymbol(c_y_max, &y_max, sizeof(double));
    cudaMemcpyToSymbol(c_total_duration, &total_duration, sizeof(double));

    // 获取轨迹信息
    int b_size = traj.pieceN * 6 * 3 * sizeof(double);
    int times_size = traj.pieceN * sizeof(double);
    GPU_Initial_Optimized_Trajectory_ Gpu_Traj;
    cudaMemcpyToSymbol(c_pieceN, &traj.pieceN, sizeof(int));
    // 分配CUDA内存
    cudaMalloc(&Gpu_Traj.b, b_size);
    cudaMalloc(&Gpu_Traj.times, times_size);

    cudaMemcpy(Gpu_Traj.b, traj.b.data(), b_size, cudaMemcpyHostToDevice);
    cudaMemcpy(Gpu_Traj.times, traj.times.data(), times_size, cudaMemcpyHostToDevice);

    int blockSize, numBlocks;
    // 计算粗网格数据
    int num_grid_x = static_cast<int>((x_max - x_min) / grid_size);
    int num_grid_y = static_cast<int>((y_max - y_min) / grid_size);
    cudaMemcpyToSymbol(c_num_grid_x, &num_grid_x, sizeof(int));
    cudaMemcpyToSymbol(c_num_grid_y, &num_grid_y, sizeof(int));
    blockSize = 256;
    numBlocks = (num_grid_x * num_grid_y + blockSize - 1) / blockSize;
    double* grid_t_closest;
    cudaMalloc(&grid_t_closest, num_grid_x * num_grid_y * sizeof(double));
    find_T_Closest<<<numBlocks, blockSize>>>(Gpu_Traj, grid_t_closest);
    cudaDeviceSynchronize();

    // 计算细网格数据
    int num_points_x = static_cast<int>((x_max - x_min) / resolution) + 1;
    int num_points_y = static_cast<int>((y_max - y_min) / resolution) + 1;
    printf("%d Points\n",num_points_x * num_points_y);
    cudaMemcpyToSymbol(c_num_points_x, &num_points_x, sizeof(int));
    cudaMemcpyToSymbol(c_num_points_y, &num_points_y, sizeof(int));
    blockSize = 256;
    numBlocks = (num_points_x * num_points_y + blockSize - 1) / blockSize;
    SDF_Map_* point_map;
    cudaMalloc(&point_map, num_points_x * num_points_y * sizeof(SDF_Map_));
    Calculate_SDF<<<numBlocks, blockSize>>>( Gpu_Traj, grid_t_closest, point_map);
    cudaDeviceSynchronize();

    // 复制结果到主机
    SDF_Map_ host_point_map[num_points_x * num_points_y];
    double area_num = 0;
    double unit_area = resolution * resolution;
    cudaMemcpy(host_point_map, point_map, num_points_x * num_points_y * sizeof(SDF_Map_), cudaMemcpyDeviceToHost);
    for(int idx=0; idx<num_points_x * num_points_y; idx++)
    {
        SDF_Map.push_back(host_point_map[idx]);
        if(host_point_map[idx].SDF <= 0.0)
        {
            area_num++;
        }
    }
    area = area_num * unit_area;
    // 释放CUDA内存
    cudaFree(Gpu_Traj.b);
    cudaFree(Gpu_Traj.times);
    cudaFree(grid_t_closest);
    cudaFree(point_map);
}

void GPUProcessGradSDF(Optimized_Trajectory_ traj, Eigen::Matrix3Xd Obstacle_Points, Eigen::MatrixX3d & GradByPositions, Eigen::VectorXd & GradByTimes, double & cost)
{
    // 获取轨迹信息
    int b_size = traj.pieceN * 6 * 3 * sizeof(double);
    int times_size = traj.pieceN * sizeof(double);
    GPU_Initial_Optimized_Trajectory_ Gpu_Traj;
    cudaMemcpyToSymbol(c_pieceN, &traj.pieceN, sizeof(int));
    // 分配CUDA内存
    cudaMalloc(&Gpu_Traj.b, b_size);
    cudaMalloc(&Gpu_Traj.times, times_size);
    
    cudaMemcpy(Gpu_Traj.b, traj.b.data(), b_size, cudaMemcpyHostToDevice);
    cudaMemcpy(Gpu_Traj.times, traj.times.data(), times_size, cudaMemcpyHostToDevice);
    Gpu_Traj.pieceN = traj.pieceN;

    //获取障碍物信息
    double *GPU_Obstacle_Points;
    cudaMalloc(&GPU_Obstacle_Points, 3 * Obstacle_Points.cols() * sizeof(double));
    cudaMemcpy(GPU_Obstacle_Points, Obstacle_Points.data(), 3 * Obstacle_Points.cols() * sizeof(double), cudaMemcpyHostToDevice);

    int blockSize, numBlocks;
    
    // 计算所有障碍物的梯度
    double *GPUGradByPositions, *GPUGradByTimes, *GPUCost;
    cudaMalloc(&GPUGradByPositions, traj.pieceN * 3 * sizeof(double));
    cudaMalloc(&GPUGradByTimes, traj.pieceN * sizeof(double));
    cudaMalloc(&GPUCost, 1 * sizeof(double));
    cudaMemset(GPUGradByPositions, 0, traj.pieceN * 3 * sizeof(double));
    cudaMemset(GPUGradByTimes, 0, traj.pieceN * sizeof(double));
    cudaMemset(GPUCost, 0, sizeof(double));
    blockSize = 256;
    numBlocks = (Obstacle_Points.cols() * traj.pieceN + blockSize - 1) / blockSize;
    Calculate_Grad_SDF_Ob<<<numBlocks, blockSize>>>(Gpu_Traj, GPU_Obstacle_Points, Obstacle_Points.cols(), GPUGradByPositions, GPUGradByTimes, GPUCost);
    cudaDeviceSynchronize();

    // 复制结果到主机
    double CPU_Cost;
    cudaMemcpy(GradByPositions.data(), GPUGradByPositions, traj.pieceN * 3 * sizeof(double), cudaMemcpyDeviceToHost);
    cudaMemcpy(GradByTimes.data(), GPUGradByTimes, traj.pieceN * sizeof(double), cudaMemcpyDeviceToHost);
    cudaMemcpy(&CPU_Cost, GPUCost, 1 * sizeof(double), cudaMemcpyDeviceToHost);
    cost = CPU_Cost;

    // 释放CUDA内存
    cudaFree(Gpu_Traj.b);
    cudaFree(Gpu_Traj.times);
    cudaFree(GPUGradByPositions);
    cudaFree(GPUGradByTimes);
    cudaFree(GPUCost);
}

void GPUProcessGradYaw(Optimized_Trajectory_ traj, Eigen::MatrixX3d & GradByPoints_Yaw, Eigen::VectorXd & GradByTimes_Yaw, double & cost)
{
    // 获取轨迹信息
    int b_size = traj.pieceN * 6 * 3 * sizeof(double);
    int times_size = traj.pieceN * sizeof(double);
    GPU_Initial_Optimized_Trajectory_ Gpu_Traj;
    cudaMemcpyToSymbol(c_pieceN, &traj.pieceN, sizeof(int));
    // 分配CUDA内存
    cudaMalloc(&Gpu_Traj.b, b_size);
    cudaMalloc(&Gpu_Traj.times, times_size);
    
    cudaMemcpy(Gpu_Traj.b, traj.b.data(), b_size, cudaMemcpyHostToDevice);
    cudaMemcpy(Gpu_Traj.times, traj.times.data(), times_size, cudaMemcpyHostToDevice);
    Gpu_Traj.pieceN = traj.pieceN;

    int blockSize, numBlocks;
    
    // 计算所有障碍物的梯度
    double *GPUGradByPoints_Yaw, *GPUGradByTimes_Yaw, *GPUCost;
    cudaMalloc(&GPUGradByPoints_Yaw, traj.pieceN * 3 * sizeof(double));
    cudaMalloc(&GPUGradByTimes_Yaw, traj.pieceN * sizeof(double));
    cudaMalloc(&GPUCost, 1 * sizeof(double));
    cudaMemset(GPUGradByPoints_Yaw, 0, traj.pieceN * 3 * sizeof(double));
    cudaMemset(GPUGradByTimes_Yaw, 0, traj.pieceN * sizeof(double));
    cudaMemset(GPUCost, 0, sizeof(double));
    blockSize = 256;
    numBlocks = (traj.pieceN + blockSize - 1) / blockSize;
    Calculate_Grad_Yaw<<<numBlocks, blockSize>>>(Gpu_Traj, GPUGradByPoints_Yaw, GPUGradByTimes_Yaw, GPUCost);
    cudaDeviceSynchronize();

    // 复制结果到主机
    double CPU_Cost;
    cudaMemcpy(GradByPoints_Yaw.data(), GPUGradByPoints_Yaw, traj.pieceN * 3 * sizeof(double), cudaMemcpyDeviceToHost);
    cudaMemcpy(GradByTimes_Yaw.data(), GPUGradByTimes_Yaw, traj.pieceN * sizeof(double), cudaMemcpyDeviceToHost);
    cudaMemcpy(&CPU_Cost, GPUCost, 1 * sizeof(double), cudaMemcpyDeviceToHost);
    cost = CPU_Cost;

    // 释放CUDA内存
    cudaFree(Gpu_Traj.b);
    cudaFree(Gpu_Traj.times);
    cudaFree(GPUGradByPoints_Yaw);
    cudaFree(GPUGradByTimes_Yaw);
    cudaFree(GPUCost);
}