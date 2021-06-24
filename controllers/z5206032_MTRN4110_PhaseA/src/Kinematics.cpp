#include "Kinematics.hpp"

#include "Util.hpp"

namespace mtrn4110 {

Kinematics::Kinematics(webots::Robot &robot)
    : timer_(robot),
      leftMotor_(robot.getMotor("left wheel motor")),
      rightMotor_(robot.getMotor("right wheel motor")),
      leftPositionSensor_(robot.getPositionSensor("left wheel sensor")),
      rightPositionSensor_(robot.getPositionSensor("right wheel sensor")) {
    // Initialise position sensors.
    auto const timeStep = robot.getBasicTimeStep();
    leftPositionSensor_->enable(timeStep);
    rightPositionSensor_->enable(timeStep);

    // Initialise motors.
    setGain({10, 0, 0}, {10, 0, 0});
    setPoint({0, 0}, {0, 0});
}

Kinematics::Kinematics(Kinematics &&kinematics) noexcept
    : timer_(std::move(kinematics.timer_)),
      leftMotor_(std::move(kinematics.leftMotor_)),
      rightMotor_(std::move(kinematics.rightMotor_)),
      leftPositionSensor_(std::move(kinematics.leftPositionSensor_)),
      rightPositionSensor_(std::move(kinematics.rightPositionSensor_)) {}

auto Kinematics::tick(char instruction) -> void {
    switch (instruction) {
        case 'L':
            // setGain({9.9, 0.012, 0.82}, {10.1, 0.002, 0.82});
            setPoint({-idealSetPosition2Turn, 0.4 * maxMotorSpeed},
                     {idealSetPosition2Turn, 0.4 * maxMotorSpeed});
            break;
        case 'R':
            // setGain({9.9, 0.012, 0.82}, {10.1, 0.002, 0.82});
            setPoint({idealSetPosition2Turn, 0.4 * maxMotorSpeed},
                     {-idealSetPosition2Turn, 0.4 * maxMotorSpeed});
            break;
        case 'F':
            // setGain({9.58, 0.003, 0.82}, {9.58, 0.002, 0.82});
            setPoint({idealSetPosition2NextCell, 0.4 * maxMotorSpeed},
                     {idealSetPosition2NextCell, 0.4 * maxMotorSpeed});
            break;
        default:
            std::cerr << "WARNING: Invalid instruction in motion sequence." << std::endl;
    }
}

auto Kinematics::setGain(std::tuple<double, double, double> left,
                         std::tuple<double, double, double> right) -> void {
    leftMotor_->setControlPID(std::get<0>(left), std::get<1>(left), std::get<2>(left));
    rightMotor_->setControlPID(std::get<0>(right), std::get<1>(right), std::get<2>(right));
}

auto Kinematics::setPoint(std::tuple<double, double> left, std::tuple<double, double> right)
    -> void {
    auto leftInitial = leftPositionSensor_->getValue();
    auto rightInitial = rightPositionSensor_->getValue();
    leftMotor_->setPosition(leftInitial + std::get<0>(left));
    leftMotor_->setVelocity(std::get<1>(left));
    rightMotor_->setPosition(rightInitial + std::get<0>(right));
    rightMotor_->setVelocity(std::get<1>(right));
}

}  // namespace mtrn4110
