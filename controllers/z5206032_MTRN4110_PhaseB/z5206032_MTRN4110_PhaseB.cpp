#include <thread>
#include <webots/Robot.hpp>

#include "TaskControl.hpp"
#include "Timer.hpp"
#include "Util.hpp"

// Perform simulation steps until Webots is stopping the controller.
static auto simulationSteps(webots::Robot &robot) -> void {
    auto const timeStep = robot.getBasicTimeStep();
    while (robot.step(timeStep) != -1) {
    }
}

// Perform real-time steps.
static auto realtimeSteps(mtrn4110::TaskControl &taskControl, mtrn4110::Timer &timer) -> void {
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

            taskControl.motionControl.tick(instr);

            // Block other actions.
            taskControl.acquireLock();
            timer.time(4);
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
    auto timer = mtrn4110::Timer(robot);

    // Instantiate our task controller class.
    auto taskControl = mtrn4110::TaskControl(robot);

    // Spin threads.
    auto t1 = std::thread(simulationSteps, std::ref(robot));
    auto t2 = std::thread(realtimeSteps, std::ref(taskControl), std::ref(timer));

    // Wait for threads to finish.
    t1.join();
    t2.join();

    return 0;
}
