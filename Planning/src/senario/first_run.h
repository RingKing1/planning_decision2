#pragma once
#include "scenario.h"
#include "localmath.h"
#include "QP.h"

class FirstRun : public Scenario
{
public:
    // 定义构造函数，同时传入父类构造函数需要的参数
    FirstRun(const Eigen::VectorXd &car, const Eigen::MatrixXd &globalPath, const Eigen::MatrixXd &obs_lidar,const double& gpsA_);
    void Straight() override;
    void AvoidObstacle() override;
    void Overtake() override;
    void DecelerateFollow() override;
    void ReturnRightLane() override;
    void MakeDecision() override;
private:
    double gpsA;
    double delta_l;
    double target_v;
    double start_l;
    double end_l;
    double heading_time_;
    double ddl;
    /**************Calobs***************/
    std::vector<obses_sd> obses_limit_SD; //在范围内的SD
    std::vector<Eigen::VectorXd> GlobalcoordinatesystemObsesLimit;
};
