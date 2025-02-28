#include "near_stop.h"

// 通过初始化列表调用父类 Scenario 的构造函数
NearStop::NearStop(const Eigen::VectorXd &car, const Eigen::MatrixXd &globalPath,
                   const std::vector<obses_sd> &obses_limit_SD,
                   const std::vector<Eigen::VectorXd> &GlobalcoordinatesystemObsesLimit,
                   const double &gpsA, const int indexinglobalpath)
    : Scenario(car, globalPath, obses_limit_SD, GlobalcoordinatesystemObsesLimit, gpsA, indexinglobalpath)
{
}

void NearStop::Straight()
{
}

void NearStop::AvoidObstacle()
{
}

void NearStop::Overtake()
{
}

//靠近终点的减速停车 
bool NearStop::DecelerateFollow() {
    int planninglength = globalPath(5, globalPath.cols() - 1) - globalPath(5, indexinglobalpath_);
    int singleplanninglength;
    if (planninglength < 10 ) { //距离终点很近 
        singleplanninglength = planninglength;
    } else {
        singleplanninglength = std::ceil(planninglength / 2);
    }
    setPlanningParam(-1.5, -1.5, -1.5, 0.5, 0);
    frentPoint FrentPoint_;
    int car_index_localpath;
    senarioTools::findClosestPointInLocalPath(car_(0), car_(1),optTrajxy, car_index_localpath);
    senarioTools::cartofrenet(car_, globalPath, indexinglobalpath_, FrentPoint_);
    Eigen::VectorXd vehicle_state_(6);
    vehicle_state_ << car_(0), car_(1), car_(2), car_(3), car_(4), gpsA_;
    std::array<double, 6> vehicle_state = senarioTools::Decidestartsl(FrentPoint_, car_index_localpath, indexinglobalpath_,
                                                                      optTrajxy, globalPath, vehicle_state_, optTrajsd);
    setStartPointParam(vehicle_state[0], vehicle_state[1], vehicle_state[2],
                       vehicle_state[3], vehicle_state[4], vehicle_state[5]);
    RestFlags(true, false, false, false, false);
    LOCAL_.setPatam(gpsA_, speed, FrentPoint_.s, FrentPoint_.d, dl, ddl, globalPath, planninglength, singleplanninglength, indexinglobalpath_, obses_limit_SD, GlobalcoordinatesystemObsesLimit,
                    start_l, end_l, delta_l, target_v, target_l, Decisionflags_, 0, false, false, 0, 0); // 最后一位时最近障碍物的位置
    find_local_path_ = LOCAL_.GetoptTrajxy(lastOptTrajxy, lastOptTrajsd);
    if (find_local_path_)
    {
        UpdateLocalPath();
    }
    return find_local_path_;


}

void NearStop::ReturnRightLane()
{
}

void NearStop::MakeDecision()
{
}

bool NearStop::Process()
{
    return false;
}
