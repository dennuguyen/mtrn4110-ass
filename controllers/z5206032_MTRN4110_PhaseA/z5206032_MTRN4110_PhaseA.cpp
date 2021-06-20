// File:          z5206032_MTRN4110_PhaseA.cpp
// Date:          14/06/2021
// Description:   Controller of E-puck for Phase A - Driving and Perception
// Author:        Dan Nguyen (z5206032)
// Modifications:
// Platform:      Windows
// Notes:

#include <thread>
#include <webots/Robot.hpp>

#include "TaskControl.hpp"
#include "Util.hpp"

// Perform simulation steps until Webots is stopping the controller.
static auto simulationSteps(webots::Robot &robot) -> void {
    const auto timeStep = robot.getBasicTimeStep();
    while (robot.step(timeStep) != -1) {
    }
}

// Perform real-time steps.
static auto realtimeSteps(TaskControl &taskControl, Timer &timer) -> void {
    auto instr = 'E';

    // Wait a bit.
    timer.time(1);
    while (timer.expired() == false) {
    }

    // Enter control loop.
    while (1) {
        // Perform an action.
        if (taskControl.isLockBusy() == false) {
            taskControl.tick();

            instr = taskControl.drivePlan.nextSequence();
            if (instr == 'E') {
                return;
            }

            taskControl.kinematics.tick(instr, taskControl.localisation.getInitialPositions());

            // Block other actions.
            taskControl.acquireLock();
            timer.time(3);
        }

        // Wait for action to finish.
        if (timer.expired() == true) {
            taskControl.localisation.tick(instr);
            taskControl.wallPerception.tick();
            taskControl.displayMessage();
            taskControl.writeMessage2csv();

            // Ready to perform another action.
            taskControl.releaseLock();

            // Wait a bit. This is to make sense of timing of displaying
            // messages. Can be omitted.
            timer.time(1);
            while (timer.expired() == false) {
            }
        }
    }
}

auto main(int argc, char **argv) -> int {
    // Instantiate webots robot.
    auto robot = webots::Robot();
    Timer timer(robot);

    // Instantiate our task controller class.
    auto taskControl = TaskControl(robot);

    // Spin threads.
    auto t1 = std::thread(simulationSteps, std::ref(robot));
    auto t2 = std::thread(realtimeSteps, std::ref(taskControl), std::ref(timer));

    // Wait for threads to finish.
    t1.join();
    t2.join();

    return 0;
}
