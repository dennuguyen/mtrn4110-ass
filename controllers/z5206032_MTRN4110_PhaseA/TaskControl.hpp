#ifndef TASK_CONTROL_HPP_
#define TASK_CONTROL_HPP_

#include <vector>
#include <webots/Robot.hpp>

#include "DrivePlan.hpp"
#include "Kinematics.hpp"
#include "Localisation.hpp"
#include "Util.hpp"
#include "WallPerception.hpp"

namespace mtrn4110 {
class TaskControl {
   public:
    explicit TaskControl(webots::Robot &);
    explicit TaskControl(TaskControl const &) = delete;
    TaskControl(TaskControl &&) noexcept;
    ~TaskControl() = default;
    auto tick() -> void;
    auto acquireLock() -> void;
    auto releaseLock() -> void;
    auto const isLockBusy() const -> bool;
    auto displayMessage() const -> void;
    auto initcsv() const -> void;
    auto writeMessage2csv() const -> void;

   private:
    auto const getMessage() const -> std::vector<std::pair<std::string, std::string>>;

   public:
    static constexpr auto drivePlanPath_ = "../../MotionPlan.txt";
    static constexpr auto csvPath_ = "../../MotionExecution.csv";
    DrivePlan drivePlan;            // Driving plan (can be replaced by autonomous driving)
    Kinematics kinematics;          // Kinematic model
    Localisation localisation;      // Simple localisation using initial position
                                    // and heading
    WallPerception wallPerception;  // LIDAR sensor

   private:
    unsigned int step_ = 0;
    bool bigLock_ = false;
};

}  // namespace mtrn4110

#endif  // TASK_CONTROL_HPP_