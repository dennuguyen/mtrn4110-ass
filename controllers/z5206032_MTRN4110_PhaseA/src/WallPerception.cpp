
#include "WallPerception.hpp"

#include <algorithm>
#include <cmath>

namespace mtrn4110 {

WallPerception::WallPerception(webots::Robot &robot) : lidar_(robot.getLidar("lidar")) {
    auto const timeStep = robot.getBasicTimeStep();
    lidar_->enable(timeStep);
    lidar_->enablePointCloud();

    // Let LIDAR initialise because poor simulation design.
    while (tick() == -1) {
        robot.step(timeStep);
    }
}

WallPerception::WallPerception(WallPerception &&wallPerception) noexcept
    : lidar_(std::move(wallPerception.lidar_)), walls_(std::move(wallPerception.walls_)) {}

auto const WallPerception::getLeftWall() const -> char { return walls_[left_]; }

auto const WallPerception::getFrontWall() const -> char { return walls_[front_]; }

auto const WallPerception::getRightWall() const -> char { return walls_[right_]; }

auto const WallPerception::tick() -> int {
    auto const pointCloud = lidar_->getPointCloud();
    if (pointCloud == nullptr) {
        return -1;
    }

    // Convert (x, y) into distances.
    auto const numberPoints = lidar_->getNumberOfPoints();
    auto pointDistances = std::vector<double>();
    pointDistances.reserve(numberPoints);
    for (int i = 0; i < numberPoints; i++) {
        pointDistances.emplace_back(
            std::sqrt(std::pow(pointCloud[i].x, 2) + std::pow(pointCloud[i].z, 2)));
    }

    // Convert point distances to a boolean if it is within wall distance
    // range.
    auto wallDetected = std::vector<bool>();
    wallDetected.reserve(numberPoints);
    for (auto const& point : pointDistances) {
        wallDetected.push_back(point < wallDistance);
    }

    // Sectorise.
    auto constexpr sectorWidth = static_cast<double>(20.0);
    auto constexpr startAngle = static_cast<double>(90.0 - 0.75 * sectorWidth);

    // Convert sector angles into point densities.
    auto const pointDensity = static_cast<double>(numberPoints / 360.0);
    auto const pointStart = static_cast<int>(pointDensity * startAngle);
    auto pointSpread = static_cast<int>(pointDensity * sectorWidth);
    auto pointOnset = pointStart;

    // Get number of distances for a section that is within wall distance.
    auto constexpr sensitivity = static_cast<double>(0.9);  // [0, 1]
    auto const pointThreshold = static_cast<int>(pointDensity * sensitivity);
    for (auto &wall : walls_) {
        wall = std::count(wallDetected.begin() + pointOnset,
                          wallDetected.begin() + pointOnset + pointSpread, true) > pointThreshold
                   ? 'Y'
                   : 'N';
        pointOnset += (pointStart + pointSpread);
    }

    return 0;
}

}  // namespace mtrn4110
