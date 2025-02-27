#include "approaching_intersection.h"

// 通过初始化列表调用父类 Scenario 的构造函数
ApproachingIntersection::ApproachingIntersection(const Eigen::VectorXd &car, const Eigen::MatrixXd &globalPath,
                                                 const std::vector<obses_sd> &obses_limit_SD,
                                                 const std::vector<Eigen::VectorXd> &GlobalcoordinatesystemObsesLimit,
                                                 const double &gpsA, const int indexinglobalpath)
    : Scenario(car, globalPath, obses_limit_SD, GlobalcoordinatesystemObsesLimit, gpsA, indexinglobalpath)
{
}

bool ApproachingIntersection::Straight() {
    setPlanningParam(-1.5, -1.5, -1.5, 0.5, 5);// 弯道处减速 
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
    LOCAL_.setPatam(gpsA_, speed, FrentPoint_.s, FrentPoint_.d, dl, ddl, globalPath, 30, 10, indexinglobalpath_, obses_limit_SD, GlobalcoordinatesystemObsesLimit,
                    start_l, end_l, delta_l, target_v, target_l, Decisionflags_, 0, false, false, 0, 0); // 最后一位时最近障碍物的位置
    find_local_path_ = LOCAL_.GetoptTrajxy(lastOptTrajxy, lastOptTrajsd);
    if (find_local_path_) {
        UpdateLocalPath();
    }
    return find_local_path_;
}

void ApproachingIntersection::AvoidObstacle()
{
}

void ApproachingIntersection::Overtake()
{
}

void ApproachingIntersection::DecelerateFollow()
{
}

void ApproachingIntersection::ReturnRightLane()
{
}

void ApproachingIntersection::MakeDecision()
{
}

bool ApproachingIntersection::Process()
{
    return false;
}
