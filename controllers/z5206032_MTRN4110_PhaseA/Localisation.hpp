#ifndef LOCALISATION_HPP_
#define LOCALISATION_HPP_

#include <array>
#include <webots/InertialUnit.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>

class Localisation {
   public:
    Localisation(webots::Robot &robot, std::pair<int, int> position, char heading)
        : leftPositionSensor_(robot.getPositionSensor("left wheel sensor")),
          rightPositionSensor_(robot.getPositionSensor("right wheel sensor")),
          inertialUnit_(robot.getInertialUnit("inertial unit")),
          position_(position) {
        // Initialise position sensors.
        const auto timeStep = 64;  // robot.getBasicTimeStep();
        leftPositionSensor_->enable(timeStep);
        rightPositionSensor_->enable(timeStep);

        // Initialise inertial unit.
        inertialUnit_->enable(timeStep);

        // Get heading index.
        switch (heading) {
            case 'N':
                headingIndex_ = 0;
                break;
            case 'E':
                headingIndex_ = 1;
                break;
            case 'S':
                headingIndex_ = 2;
                break;
            case 'W':
                headingIndex_ = 3;
                break;
            default:
                std::cerr << "WARNING: Invalid heading in motion plan." << std::endl;
        }
    }

    const auto getRow() const -> int { return position_.first; }

    const auto getColumn() const -> int { return position_.second; }

    const auto getHeading() const -> char { return cardinalPoints_[headingIndex_]; }

    auto tick(char instruction) -> void { updateHeadingByPlan(instruction); }

    const auto getInitialPositions() const -> std::pair<double, double> {
        return {leftPositionSensor_->getValue(), rightPositionSensor_->getValue()};
    }

    const auto getYaw() const -> double { return inertialUnit_->getRollPitchYaw()[2]; }

   private:
    // Updates the heading using information given by motion plan sequence.
    auto updateHeadingByPlan(char instruction) -> void {
        switch (instruction) {
            case 'F':
                updatePositionByPlan();
                break;
            case 'L':
                headingIndex_--;
                if (headingIndex_ < 0) {
                    headingIndex_ += 4;
                }
                break;
            case 'R':
                headingIndex_++;
                if (headingIndex_ > 3) {
                    headingIndex_ -= 4;
                }
                break;
            default:
                std::cerr << "WARNING: Invalid character in motion plan." << std::endl;
        }
    }

    // Updates the position using information given by motion plan sequence.
    auto updatePositionByPlan() -> void {
        switch (getHeading()) {
            case 'N':
                position_.first--;
                break;
            case 'E':
                position_.second++;
                break;
            case 'S':
                position_.first++;
                break;
            case 'W':
                position_.second--;
                break;
            default:
                std::cerr << "WARNING: Invalid character for cardinal direction." << std::endl;
        }
    }

    auto updatePositionByOdometry() -> void {}

    auto updateHeadingByIMU() -> void {}

   private:
    std::unique_ptr<webots::PositionSensor> leftPositionSensor_;
    std::unique_ptr<webots::PositionSensor> rightPositionSensor_;
    std::unique_ptr<webots::InertialUnit> inertialUnit_;
    const std::array<char, 4> cardinalPoints_ = {'N', 'E', 'S', 'W'};
    std::pair<int, int> position_;  // row, column
    int headingIndex_;
};

#endif  // LOCALISATION_HPP_