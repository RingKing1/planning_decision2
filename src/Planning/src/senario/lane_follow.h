#pragma once
#include "scenario.h"

class LaneFollowScenario : public Scenario
{
public:
    // 定义构造函数，同时传入父类构造函数需要的参数
    LaneFollowScenario(const Eigen::VectorXd &car, const Eigen::MatrixXd &globalPath,
                       const std::vector<obses_sd> &obses_limit_SD,
                       const std::vector<Eigen::VectorXd> &GlobalcoordinatesystemObsesLimit,
                       const double &gpsA, const int indexinglobalpath);
    bool Straight();
    bool AvoidObstacle();
    bool Overtake();
    bool DecelerateFollow();
    bool ReturnRightLane();

    void MakeDecision() override;
    bool Process() override;

private:
    
};
