#include "Localisation.hpp"

namespace mtrn4110 {

Localisation::Localisation(webots::Robot &robot, std::pair<int, int> position, char heading)
    : inertialUnit_(robot.getInertialUnit("inertial unit")), position_(position) {
    // Initialise inertial unit.
    auto const timeStep = robot.getBasicTimeStep();
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

Localisation::Localisation(Localisation &&localisation) noexcept
    : inertialUnit_(std::move(localisation.inertialUnit_)),
      position_(std::move(localisation.position_)),
      headingIndex_(std::move(localisation.headingIndex_)) {}

auto const Localisation::getRow() const -> int { return position_.first; }

auto const Localisation::getColumn() const -> int { return position_.second; }

auto const Localisation::getHeading() const -> char { return cardinalPoints[headingIndex_]; }

auto Localisation::tick(char instruction) -> void { updateHeadingByPlan(instruction); }

auto const Localisation::getYaw() const -> double { return inertialUnit_->getRollPitchYaw()[2]; }

auto Localisation::updateHeadingByPlan(char instruction) -> void {
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

auto Localisation::updatePositionByPlan() -> void {
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

auto Localisation::updatePositionByOdometry() -> void {}

auto Localisation::updateHeadingByIMU() -> void {}

}  // namespace mtrn4110
