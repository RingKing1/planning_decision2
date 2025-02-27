#pragma once
#include <iostream>
#include <Eigen/Dense>

#include "local_dp_qp.h"
#include "toolKits.h"
#include "scenario_tools.h"

class Scenario
{
public:
    Scenario(const Eigen::VectorXd &car, const Eigen::MatrixXd &globalPath,
             const std::vector<obses_sd> &obses_limit_SD,
             const std::vector<Eigen::VectorXd> &GlobalcoordinatesystemObsesLimit,
             const double &gpsA,const int indexinglobalpath);
    virtual ~Scenario() = default;

    // 给出决策makedecision的函数
    virtual void MakeDecision() = 0;

    // 执行函数
    virtual bool Process() = 0;

    // 获取当前系统时间
    double Time();

    // 重置决策flags
    void RestFlags(bool DriveStraightLineFlag_,
                   bool ObstacleAvoidanceFlag_,
                   bool DecelerateFlag_,
                   bool Overtakinginlaneflag_,
                   bool righttoleftlane);

    // 获取本周期的局部路径optTrajxy
    inline Eigen::MatrixXd getlocalpath() const { return optTrajxy; }
    inline std::vector<Eigen::Vector4d> getlocalpathsd() const { return optTrajsd; }

    // 更新本周期的局部路径optTrajxy
    inline void setlocalpath(const Eigen::MatrixXd &localpath)
    {
        optTrajxy.resize(localpath.rows(), localpath.cols());
        optTrajxy = localpath;
    }
    inline void setlocalpath(const std::vector<Eigen::Vector4d> &localpath) { optTrajsd = localpath; }

    void Updated(const Eigen::VectorXd &car, const std::vector<obses_sd> &obses_limit_SD_,
                 const std::vector<Eigen::VectorXd> &GlobalcoordinatesystemObsesLimit_,
                 const double &gpsA, const double indexinglobalpath);
    void UpdateLocalPath();
    void setPlanningParam(double start_l_, double end_l_, double target_l_, double delta_l_, double target_v_);
    void setStartPointParam(double start_s_, double start_l_, double start_dl_, double start_ddl_,
                             double start_speed_, double start_acc_);



    // 判断路径是否需要重规划
    void CheckPathReplan();

protected:

    /*********规划相关参数 *********/
    Decisionflags Decisionflags_;
    local_dp_qp LOCAL_;
    Eigen::MatrixXd optTrajxy;                  // 本周期局部路径xy
    std::vector<Eigen::Vector4d> optTrajsd;     // 上周期局部路径sd
    Eigen::MatrixXd lastOptTrajxy;              // 本周期局部路径xy
    std::vector<Eigen::Vector4d> lastOptTrajsd; // 本周期局部路径sd 
    // 存储当前车辆状态
    Eigen::VectorXd car_;
    // 储存全局路径
    Eigen::MatrixXd globalPath;
    // 存储障碍物信息（摄像头、雷达）
    std::vector<obses_sd> obses_limit_SD;
    std::vector<Eigen::VectorXd> GlobalcoordinatesystemObsesLimit;
    double gpsA_;

    // 车辆在全局路径下的坐标点
    int indexinglobalpath_ = 0;
    // 是否找到合适的局部路径
    bool find_local_path_ = false;


    // 车辆参数（全局固定常量）
    static constexpr double vehicle_length_ = 4.532;
    static constexpr double vehicle_width_ = 1.814;
    std::vector<Eigen::MatrixXd> all_local_pointss;
    bool REPALN = true;
    int Numbercycles = 0;

    double delta_l;
    double target_v;
    double start_l;
    double end_l;
    double target_l;
    double ddl;
    double dl;
    double distance_threshold = 9; //停车距离阈值 自车长度 + 前方停车距离 (和前方车辆的距离)
    bool flag;
    double heading_time_;
    double local_start_s;
    double local_start_l;
    double speed;

    /*************AEB**************/
    bool aeb_start_flag = false;
};
