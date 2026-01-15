#ifndef SDF_CUH
#define SDF_CUH

#include "Eigen/Dense"
#include <vector>

typedef struct{
    int pieceN;
    Eigen::MatrixX3d b;
    Eigen::VectorXd times;
    Eigen::Matrix3Xd points;
}Optimized_Trajectory_;

typedef struct{
    int pieceN;
    double *b;
    double *times;
}GPU_Initial_Optimized_Trajectory_;

typedef struct{
    double x;
    double y;
    double SDF;
}SDF_Map_;

// SDF配置结构体
struct SDFConfig {
    // Find t_star 参数
    double coarse_grid_size;
    double find_t_star_time_step;
    double eta;
    double c;
    double tol;
    int max_iter;

    // SDF 参数
    double sdf_resolution;
    double sdf_safety_hor;
    double weight_safety;

    // 车辆参数 - footprint 必须是4个顶点
    std::vector<std::pair<double, double>> footprint;

    // 遍历区域
    double x_min;
    double x_max;
    double y_min;
    double y_max;
};

void GPUProcessConfig(const SDFConfig& config);
void GPUProcessSDF(Optimized_Trajectory_ init_traj, std::vector<SDF_Map_>& SDF_Map, double& area);
void GPUProcessGradSDF(Optimized_Trajectory_ traj, Eigen::Matrix3Xd Obstacle_Points, Eigen::MatrixX3d & GradByPositions, Eigen::VectorXd & GradByTimes, double & cost);
void GPUProcessGradYaw(Optimized_Trajectory_ traj, double sdf_opimiz_weight_yaw_, Eigen::MatrixX3d & GradByPoints_Yaw, Eigen::VectorXd & GradByTimes_Yaw, double & cost);
#endif