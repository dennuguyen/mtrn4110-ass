// TODO: Move all sensing code to Localisation.hpp

#ifndef KINEMATICS_HPP_
#define KINEMATICS_HPP_

#include <cmath>
#include <memory>
#include <webots/InertialUnit.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>

class Kinematics {
   public:
    Kinematics(webots::Robot &robot)
        : leftMotor_(robot.getMotor("left wheel motor")),
          rightMotor_(robot.getMotor("right wheel motor")),
          leftPositionSensor_(robot.getPositionSensor("left wheel sensor")),
          rightPositionSensor_(robot.getPositionSensor("right wheel sensor")),
          inertialUnit_(robot.getInertialUnit("inertial unit")) {
        // Initialise left motor.
        leftMotor_->setPosition(0.0);
        leftMotor_->setVelocity(0.0);
        leftMotor_->setControlPID(10, 0, 0);

        // Initialise right motor.
        rightMotor_->setPosition(0.0);
        rightMotor_->setVelocity(0.0);
        rightMotor_->setControlPID(10, 0, 0);

        // Initialise position sensors.
        const auto timeStep = robot.getBasicTimeStep();
        leftPositionSensor_->enable(timeStep);
        rightPositionSensor_->enable(timeStep);

        // Initialise inertial unit.
        inertialUnit_->enable(timeStep);
    }

    auto tick(char instruction) -> void {
        switch (instruction) {
            case 'L':
                odometry(-idealSetPosition2Turn_, idealSetPosition2Turn_);
                break;
            case 'R':
                odometry(idealSetPosition2Turn_, -idealSetPosition2Turn_);
                break;
            case 'F':
                odometry(idealSetPosition2NextCell_, idealSetPosition2NextCell_);
                break;
            default:
                std::cerr << "WARNING: Invalid instruction in motion sequence." << std::endl;
        }
    }

   private:
    auto odometry(double left, double right) -> void {
        auto leftInitial = leftPositionSensor_->getValue();
        auto rightInitial = rightPositionSensor_->getValue();
        leftMotor_->setPosition(leftInitial + left);
        rightMotor_->setPosition(rightInitial + right);
        leftMotor_->setVelocity(2.2);
        rightMotor_->setVelocity(2.2);
    }

   private:
    std::unique_ptr<webots::Motor> leftMotor_;
    std::unique_ptr<webots::Motor> rightMotor_;
    std::unique_ptr<webots::PositionSensor> leftPositionSensor_;
    std::unique_ptr<webots::PositionSensor> rightPositionSensor_;
    std::unique_ptr<webots::InertialUnit> inertialUnit_;
    static constexpr auto maxMotorSpeed_ = 6.28;
    static constexpr auto wheelRadius_ = 0.02;
    static constexpr auto axleLength_ = 0.0566;
    static constexpr auto distanceBetweenCells_ = 0.165;
    static constexpr auto idealSetPosition2NextCell_ = distanceBetweenCells_ / wheelRadius_;
    static constexpr auto idealSetPosition2Turn_ = axleLength_ * M_PI / 2.0 / wheelRadius_ / 2.0;
};

#endif  // KINEMATICS_HPP_