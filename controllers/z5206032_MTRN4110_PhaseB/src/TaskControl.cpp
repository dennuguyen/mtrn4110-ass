#include "TaskControl.hpp"

#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace mtrn4110 {

TaskControl::TaskControl(webots::Robot &robot)
    : pathPlanner(PathPlanner(mapPath)),
      drivePlan(DrivePlan(drivePlanPath)),
      motionControl(MotionControl(robot)),
      localisation(
          Localisation(robot, drivePlan.getInitialLocalisation(), drivePlan.getInitialHeading())),
      wallPerception(WallPerception(robot)) {
    initcsv();

    // Display the initial state.
    displayMessage();
    writeMessage2csv();
}

TaskControl::TaskControl(TaskControl &&taskControl) noexcept
    : pathPlanner(std::move(taskControl.pathPlanner)),
      drivePlan(std::move(taskControl.drivePlan)),
      motionControl(std::move(taskControl.motionControl)),
      localisation(std::move(taskControl.localisation)),
      wallPerception(std::move(taskControl.wallPerception)),
      step_(std::move(taskControl.step_)),
      bigLock_(std::move(taskControl.bigLock_)) {}

auto TaskControl::tick() noexcept -> void { step_++; }

auto TaskControl::acquireLock() noexcept -> void { bigLock_ = true; }

auto TaskControl::releaseLock() noexcept -> void { bigLock_ = false; }

auto const TaskControl::isLockBusy() const noexcept -> bool { return bigLock_; }

auto TaskControl::displayMessage() const noexcept -> void {
    auto ss = std::stringstream();
    auto const &msg = getMessage();
    for (auto const &cell : msg) {
        ss << cell.first << ": " << cell.second;
        if (&cell != &msg.back()) {
            ss << ", ";
        }
    }
    print(ss.str());
}

auto TaskControl::initcsv() const -> void {
    auto csv = std::ofstream(csvPath, std::ios::trunc);
    if (csv.good() == false) {
        throw std::runtime_error("Could not open file.");
    }

    auto const &msg = getMessage();

    // Overwrite file with headers.
    for (auto const &cell : msg) {
        csv << cell.first << ", ";
    }
    csv << std::endl;
    csv.close();
}

auto TaskControl::writeMessage2csv() const -> void {
    auto csv = std::ofstream(csvPath, std::ios::app);
    if (csv.good() == false) {
        throw std::runtime_error("Could not open file.");
    }

    auto const &msg = getMessage();

    // Append message to CSV.
    for (auto const &cell : msg) {
        csv << cell.second << ", ";
    }
    csv << std::endl;
    csv.close();
}

auto const TaskControl::getMessage() const noexcept
    -> std::vector<std::pair<std::string, std::string>> {
    auto msg = std::vector<std::pair<std::string, std::string>>();
    auto ss = std::stringstream();
    ss << std::setw(3) << std::setfill('0') << step_;
    msg.push_back({"Step", ss.str()});
    msg.push_back({"Row", std::string(1, localisation.getRow())});
    msg.push_back({"Column", std::string(1, localisation.getColumn())});
    msg.push_back({"Heading", std::string(1, localisation.getHeading())});
    msg.push_back({"Left Wall", std::string(1, wallPerception.getLeftWall())});
    msg.push_back({"Front Wall", std::string(1, wallPerception.getFrontWall())});
    msg.push_back({"Right Wall", std::string(1, wallPerception.getRightWall())});
    return msg;
}

}  // namespace mtrn4110
