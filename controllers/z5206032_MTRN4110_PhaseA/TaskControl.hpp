#ifndef TASK_CONTROL_HPP_
#define TASK_CONTROL_HPP_

#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <webots/Robot.hpp>

#include "DrivePlan.hpp"
#include "Kinematics.hpp"
#include "Localisation.hpp"
#include "Util.hpp"
#include "WallPerception.hpp"

class TaskControl {
   public:
    TaskControl(webots::Robot &robot)
        : drivePlan(DrivePlan(drivePlanPath)),
          kinematics(Kinematics(robot)),
          localisation(
              Localisation(drivePlan.getInitialLocalisation(), drivePlan.getInitialHeading())),
          wallPerception(WallPerception(robot)) {
        initcsv();

        // Display the initial state.
        displayMessage();
        writeMessage2csv();
    }

    auto tick() -> void { step++; }

    auto acquireLock() -> void { bigLock = true; }

    auto releaseLock() -> void { bigLock = false; }

    const auto isLockBusy() -> bool { return bigLock; }

    auto displayMessage() -> void {
        auto ss = std::stringstream();
        const auto &msg = getMessage();
        for (const auto &cell : msg) {
            ss << cell.first << ": " << cell.second;
            if (&cell != &msg.back()) {
                ss << ", ";
            }
        }
        printConsole(ss.str());
    }

    auto initcsv() -> void {
        auto csv = std::ofstream(csvPath, std::ios::trunc);
        const auto &msg = getMessage();

        // Overwrite file with headers.
        for (const auto &cell : msg) {
            csv << cell.first << ", ";
        }
        csv << std::endl;
        csv.close();
    }

    auto writeMessage2csv() -> void {
        auto csv = std::ofstream(csvPath, std::ios::app);
        const auto &msg = getMessage();

        // Append message to CSV.
        for (const auto &cell : msg) {
            csv << cell.second << ", ";
        }
        csv << std::endl;
        csv.close();
    }

   private:
    auto getMessage() -> std::vector<std::pair<std::string, std::string>> {
        auto msg = std::vector<std::pair<std::string, std::string>>();
        auto ss = std::stringstream();
        ss << std::setw(3) << std::setfill('0') << step;
        msg.push_back({"Step", ss.str()});
        msg.push_back({"Row", std::string(1, localisation.getRow())});
        msg.push_back({"Column", std::string(1, localisation.getColumn())});
        msg.push_back({"Heading", std::string(1, localisation.getHeading())});
        msg.push_back({"Left Wall", std::string(1, wallPerception.getLeftWall())});
        msg.push_back({"Front Wall", std::string(1, wallPerception.getFrontWall())});
        msg.push_back({"Right Wall", std::string(1, wallPerception.getRightWall())});
        return msg;
    }

   public:
    DrivePlan drivePlan;            // Driving plan (can be replaced by autonomous driving)
    Kinematics kinematics;          // Kinematic model
    Localisation localisation;      // Simple localisation using initial position and heading
    WallPerception wallPerception;  // LIDAR sensor

   private:
    unsigned int step = 0;
    bool bigLock = false;
    static constexpr auto drivePlanPath = "../../MotionPlan.txt";
    static constexpr auto csvPath = "../../MotionExecution.csv";
};

#endif  // TASK_CONTROL_HPP_