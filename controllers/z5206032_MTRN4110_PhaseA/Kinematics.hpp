#ifndef KINEMATICS_HPP_
#define KINEMATICS_HPP_

#include <cmath>
#include <memory>
#include <tuple>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

#include "Util.hpp"

class Kinematics {
   public:
    Kinematics(webots::Robot &robot)
        : timer_(robot),
          leftMotor_(robot.getMotor("left wheel motor")),
          rightMotor_(robot.getMotor("right wheel motor")) {
        // Initialise motors.
        setGain({0, 0, 0}, {0, 0, 0});
        setPoint({0, 0}, {0, 0});
    }

    auto tick(char instruction, std::pair<double, double> initialPosition) -> void {
        switch (instruction) {
            case 'L':
                setGain({9.9, 0.012, 0.82}, {10.1, 0.002, 0.82});
                timer_.time(0.1);
                while (timer_.expired() == false) {
                }
                setPoint({initialPosition.first - idealSetPosition2Turn, 0.4 * maxMotorSpeed},
                         {initialPosition.second + idealSetPosition2Turn, 0.4 * maxMotorSpeed});
                break;
            case 'R':
                setGain({9.9, 0.012, 0.82}, {10.1, 0.002, 0.82});
                timer_.time(0.1);
                while (timer_.expired() == false) {
                }
                setPoint({initialPosition.first + idealSetPosition2Turn, 0.4 * maxMotorSpeed},
                         {initialPosition.second - idealSetPosition2Turn, 0.4 * maxMotorSpeed});
                break;
            case 'F':
                // setGain({1, 0, 0}, {1, 0, 0});
                // setGain({9.58, 0.003, 0.82}, {9.58, 0.002, 0.82});
                timer_.time(0.1);
                while (timer_.expired() == false) {
                }
                setPoint({initialPosition.first + idealSetPosition2NextCell, 0.6 * maxMotorSpeed},
                         {initialPosition.second + idealSetPosition2NextCell, 0.6 * maxMotorSpeed});
                break;
            default:
                std::cerr << "WARNING: Invalid instruction in motion sequence." << std::endl;
        }
    }

    auto setGain(std::tuple<double, double, double> left, std::tuple<double, double, double> right)
        -> void {
        leftMotor_->setControlPID(std::get<0>(left), std::get<1>(left), std::get<2>(left));
        rightMotor_->setControlPID(std::get<0>(right), std::get<1>(right), std::get<2>(right));
    }

    auto setPoint(std::tuple<double, double> left, std::tuple<double, double> right) -> void {
        leftMotor_->setPosition(std::get<0>(left));
        leftMotor_->setVelocity(std::get<1>(left));
        rightMotor_->setPosition(std::get<0>(right));
        rightMotor_->setVelocity(std::get<1>(right));
    }

   public:
    static constexpr auto maxMotorSpeed = 6.28;
    static constexpr auto wheelRadius = 0.02;
    static constexpr auto axleLength = 0.0566;
    static constexpr auto distanceBetweenCells = 0.165;
    static constexpr auto idealSetPosition2NextCell = distanceBetweenCells / wheelRadius;
    static constexpr auto idealSetPosition2Turn = axleLength * M_PI / 2.0 / wheelRadius / 2.0;

   private:
    Timer timer_;
    std::unique_ptr<webots::Motor> leftMotor_;
    std::unique_ptr<webots::Motor> rightMotor_;
};

#endif  // KINEMATICS_HPP_