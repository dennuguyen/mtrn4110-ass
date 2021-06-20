#ifndef KINEMATICS_HPP_
#define KINEMATICS_HPP_

#include <cmath>
#include <memory>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

class Kinematics {
   public:
    Kinematics(webots::Robot &robot)
        : leftMotor_(robot.getMotor("left wheel motor")),
          rightMotor_(robot.getMotor("right wheel motor")) {
        // Initialise left motor.
        leftMotor_->setPosition(0.0);
        leftMotor_->setVelocity(0.0);
        leftMotor_->setControlPID(10, 0, 0);

        // Initialise right motor.
        rightMotor_->setPosition(0.0);
        rightMotor_->setVelocity(0.0);
        rightMotor_->setControlPID(10, 0, 0);
    }

    auto tick(char instruction, std::pair<double, double> initialPosition) -> void {
        switch (instruction) {
            case 'L':
                odometry(initialPosition.first - idealSetPosition2Turn_,
                         initialPosition.second + idealSetPosition2Turn_, 2.2, 2.2);
                break;
            case 'R':
                odometry(initialPosition.first + idealSetPosition2Turn_,
                         initialPosition.second - idealSetPosition2Turn_, 2.2, 2.2);
                break;
            case 'F':
                odometry(initialPosition.first + idealSetPosition2NextCell_,
                         initialPosition.second + idealSetPosition2NextCell_, 2.2, 2.2);
                break;
            default:
                std::cerr << "WARNING: Invalid instruction in motion sequence." << std::endl;
        }
    }

   private:
    auto odometry(double leftPose, double rightPose, double leftVel, double rightVel) -> void {
        leftMotor_->setPosition(leftPose);
        rightMotor_->setPosition(rightPose);
        leftMotor_->setVelocity(leftVel);
        rightMotor_->setVelocity(rightVel);
    }

   private:
    std::unique_ptr<webots::Motor> leftMotor_;
    std::unique_ptr<webots::Motor> rightMotor_;
    static constexpr auto maxMotorSpeed_ = 6.28;
    static constexpr auto wheelRadius_ = 0.02;
    static constexpr auto axleLength_ = 0.0566;
    static constexpr auto distanceBetweenCells_ = 0.165;
    static constexpr auto idealSetPosition2NextCell_ = distanceBetweenCells_ / wheelRadius_;
    static constexpr auto idealSetPosition2Turn_ = axleLength_ * M_PI / 2.0 / wheelRadius_ / 2.0;
};

#endif  // KINEMATICS_HPP_