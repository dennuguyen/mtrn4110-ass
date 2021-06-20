#ifndef KINEMATICS_HPP_
#define KINEMATICS_HPP_

#include <cmath>
#include <memory>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>

class Kinematics {
   public:
    Kinematics(webots::Robot &robot)
        : leftMotor(robot.getMotor("left wheel motor")),
          rightMotor(robot.getMotor("right wheel motor")),
          leftPositionSensor(robot.getPositionSensor("left wheel sensor")),
          rightPositionSensor(robot.getPositionSensor("right wheel sensor")) {
        // Initialise left motor.
        leftMotor->setPosition(0.0);
        leftMotor->setVelocity(0.0);
        leftMotor->setControlPID(10, 0, 0);

        // Initialise right motor.
        rightMotor->setPosition(0.0);
        rightMotor->setVelocity(0.0);
        rightMotor->setControlPID(10, 0, 0);

        // Initialise position sensors.
        const auto timeStep = robot.getBasicTimeStep();
        leftPositionSensor->enable(timeStep);
        rightPositionSensor->enable(timeStep);
    }

    auto tick(char instruction) -> void {
        switch (instruction) {
            case 'L':
                odometry(-idealSetPosition2Turn, idealSetPosition2Turn);
                break;
            case 'R':
                odometry(idealSetPosition2Turn, -idealSetPosition2Turn);
                break;
            case 'F':
                odometry(idealSetPosition2NextCell, idealSetPosition2NextCell);
                break;
            default:
                std::cerr << "WARNING: Invalid instruction in motion sequence." << std::endl;
        }
    }

   private:
    // Odometry-based localisation is accurate enough for 24-step motion plans.
    // + Easy to implement.
    // - Not very good with turns.
    // - No ideal velocity.
    auto odometry(double left, double right) -> void {
        auto leftInitial = leftPositionSensor->getValue();
        auto rightInitial = rightPositionSensor->getValue();
        leftMotor->setPosition(leftInitial + left);
        rightMotor->setPosition(rightInitial + right);
        leftMotor->setVelocity(2.2);
        rightMotor->setVelocity(2.2);
    }

    // Dead reckoning.
    auto dead_reckoning(double left, double right) -> void {}

   private:
    std::unique_ptr<webots::Motor> leftMotor;
    std::unique_ptr<webots::Motor> rightMotor;
    std::unique_ptr<webots::PositionSensor> leftPositionSensor;
    std::unique_ptr<webots::PositionSensor> rightPositionSensor;
    static constexpr auto maxMotorSpeed = 6.28;
    static constexpr auto wheelRadius = 0.02;
    static constexpr auto axleLength = 0.0566;
    static constexpr auto distanceBetweenCells = 0.165;
    static constexpr auto idealSetPosition2NextCell = distanceBetweenCells / wheelRadius;
    static constexpr auto idealSetPosition2Turn = axleLength * M_PI / 2.0 / wheelRadius / 2.0;
};

#endif  // KINEMATICS_HPP_