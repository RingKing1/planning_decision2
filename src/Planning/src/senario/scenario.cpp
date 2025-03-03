#include "scenario.h"

constexpr int64_t NANOS_PER_SECOND = 1000000000; // 1e9

// 使用初始化列表来初始化成员变量
Scenario::Scenario(const Eigen::VectorXd &car,
                   const Eigen::MatrixXd &globalPath,
                   const std::vector<obses_sd> &obses_limit_SD,
                   const std::vector<Eigen::VectorXd> &GlobalcoordinatesystemObsesLimit,
                   const double &gpsA, const double indexinglobalpath) : car_(car),
                                                                         globalPath(globalPath),
                                                                         obses_limit_SD(obses_limit_SD),
                                                                         GlobalcoordinatesystemObsesLimit(GlobalcoordinatesystemObsesLimit),
                                                                         gpsA_(gpsA),
                                                                         indexinglobalpath_(indexinglobalpath)
{
}

double Scenario::Time()
{
    auto now = std::chrono::high_resolution_clock::now();
    auto nano_time_point = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
    auto epoch = nano_time_point.time_since_epoch();
    uint64_t now_nano = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch).count();
    return static_cast<double>(now_nano) / NANOS_PER_SECOND;
}

// bool ObstacleAvoidanceFlag = false; // 街道超车的标志true为正在避障， false 直线行驶
// bool DecelerateFlag = false;        // 减速停车标志
// bool DriveStraightLineFlag = false; // 直线行使标志
// bool Overtakinginlaneflag = false;  // 车道内超车的标志
// bool righttoleftlane = false;       // 超车完成 返回原车道的标志
void Scenario::RestFlags(bool DriveStraightLineFlag_,
                         bool ObstacleAvoidanceFlag_,
                         bool DecelerateFlag_,
                         bool Overtakinginlaneflag_,
                         bool righttoleftlane)
{
    Decisionflags_.DriveStraightLineFlag = DriveStraightLineFlag_; // 直行决策
    Decisionflags_.ObstacleAvoidanceFlag = ObstacleAvoidanceFlag_; // 避障决策
    Decisionflags_.DecelerateFlag = DecelerateFlag_;               // 减速跟车决策
    Decisionflags_.Overtakinginlaneflag = Overtakinginlaneflag_;   // 超车决策
    Decisionflags_.righttoleftlane = righttoleftlane;              // 返回原车道决策
}

void Scenario::Updated(const Eigen::VectorXd &car, const std::vector<obses_sd> &obses_limit_SD_, const std::vector<Eigen::VectorXd> &GlobalcoordinatesystemObsesLimit_, const double &gpsA, const double indexinglobalpath)
{
    car_ = car;
    obses_limit_SD = obses_limit_SD_;
    GlobalcoordinatesystemObsesLimit = GlobalcoordinatesystemObsesLimit_;
    gpsA_ = gpsA;
    indexinglobalpath_ = indexinglobalpath;
}

void Scenario::UpdateLocalPath()
{
    optTrajxy.resize(lastOptTrajxy.rows(), lastOptTrajxy.cols());
    optTrajxy = lastOptTrajxy;
    optTrajsd = lastOptTrajsd;
}

void Scenario::CheckPathReplan()
{
    if (Numbercycles == 0)
    {

        all_local_pointss.clear();
        // 重新计算时 才会清空
        Eigen::MatrixXd car_Fpoint;
        all_local_pointss.reserve(optTrajxy.cols());
        for (size_t i = 0; i < optTrajxy.cols(); ++i)
        {
            obs::get_car_fourpoint(vehicle_length_, vehicle_width_, optTrajxy.col(i), car_Fpoint); // 由车辆的中心点 计算局部路径上的每个店对应的车辆的四个顶点
            all_local_pointss.emplace_back(car_Fpoint);
        }
    }

    // 判断是否超时，如果超时也需要重新规划
    if (!obs::HasOverlap(all_local_pointss, GlobalcoordinatesystemObsesLimit, optTrajxy))
    {
        Numbercycles += 1;
        REPALN = false;
    }
    if (Numbercycles > 10)
    {
        Numbercycles = 0;
        REPALN = true;
    }
}
