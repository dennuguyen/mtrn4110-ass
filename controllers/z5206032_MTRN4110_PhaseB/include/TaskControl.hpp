#ifndef TASK_CONTROL_HPP_
#define TASK_CONTROL_HPP_

#include <vector>
#include <webots/Robot.hpp>

#include "DrivePlan.hpp"
#include "Localisation.hpp"
#include "MotionControl.hpp"
#include "PathPlanner.hpp"
#include "Util.hpp"
#include "WallPerception.hpp"

namespace mtrn4110 {
class TaskControl {
   public:
    explicit TaskControl(webots::Robot &);
    explicit TaskControl(TaskControl const &) = delete;
    TaskControl(TaskControl &&) noexcept;
    ~TaskControl() = default;
    auto operator=(TaskControl const &) -> TaskControl & = delete;
    auto operator=(TaskControl &&) -> TaskControl & = delete;
    auto tick() noexcept -> void;
    auto acquireLock() noexcept -> void;
    auto releaseLock() noexcept -> void;
    auto isLockBusy() const noexcept -> bool;
    auto displayMessage() const noexcept -> void;
    auto initcsv() const -> void;
    auto writeMessage2csv() const -> void;

   private:
    auto getMessage() const noexcept -> std::vector<std::pair<std::string, std::string>>;

   public:
    static constexpr auto csvPath = "../../MotionExecution.csv";
    static constexpr auto mapPath = "../../Map.txt";
    static constexpr auto pathPlanPath = "../../PathPlan.txt";
    PathPlanner pathPlanner;        // Path planner.
    DrivePlan drivePlan;            // Driving plan (can be replaced by autonomous driving).
    MotionControl motionControl;    // Kinematic model.
    Localisation localisation;      // Simple localisation using initial position
                                    // and heading.
    WallPerception wallPerception;  // LIDAR sensor.

   private:
    unsigned int step_ = 0;
    bool bigLock_ = false;
};

}  // namespace mtrn4110

#endif  // TASK_CONTROL_HPP_