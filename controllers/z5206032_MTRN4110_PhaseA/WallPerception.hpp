#ifndef WALL_PERCEPTION_HPP_
#define WALL_PERCEPTION_HPP_

#include <array>
#include <cmath>
#include <memory>
#include <webots/Lidar.hpp>
#include <webots/Robot.hpp>

class WallPerception {
   public:
    WallPerception(webots::Robot &robot) : lidar(robot.getLidar("lidar")) {
        const auto timeStep = robot.getBasicTimeStep();
        lidar->enable(timeStep);
        lidar->enablePointCloud();

        // Let LIDAR initialise because poor simulation design.
        while (tick() == -1) {
            robot.step(timeStep);
        }
    }

    const auto getLeftWall() -> char { return walls[left]; }

    const auto getFrontWall() -> char { return walls[front]; }

    const auto getRightWall() -> char { return walls[right]; }

    auto tick() -> int {
        const auto pointCloud = lidar->getPointCloud();
        if (pointCloud == nullptr) {
            return -1;
        }

        // Convert (x, y) into distances.
        const auto numberPoints = lidar->getNumberOfPoints();
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
        for (const auto &point : pointDistances) {
            wallDetected.push_back(point < wallDistance);
        }

        // Sectorise.
        constexpr auto sectorWidth = static_cast<double>(20.0);
        constexpr auto startAngle = static_cast<double>(90.0 - 0.75 * sectorWidth);

        // Convert sector angles into point densities.
        const auto pointDensity = static_cast<double>(numberPoints / 360.0);
        const auto pointStart = static_cast<int>(pointDensity * startAngle);
        auto pointSpread = static_cast<int>(pointDensity * sectorWidth);
        auto pointOnset = pointStart;

        // Get number of distances for a section that is within wall distance.
        constexpr auto sensitivity = static_cast<double>(0.9);  // [0, 1]
        const auto pointThreshold = static_cast<int>(pointDensity * sensitivity);
        for (auto &wall : walls) {
            wall =
                std::count(wallDetected.begin() + pointOnset,
                           wallDetected.begin() + pointOnset + pointSpread, true) > pointThreshold
                    ? 'Y'
                    : 'N';
            pointOnset += (pointStart + pointSpread);
        }

        return 0;
    }

   private:
    static constexpr auto wallDistance = 0.085;  // Distance from centre of cell to wall.
    std::unique_ptr<webots::Lidar> lidar;        // 360 lidar.
    std::array<char, 3> walls;
    static constexpr auto left = 0;
    static constexpr auto front = 1;
    static constexpr auto right = 2;
};

#endif  // WALL_PERCEPTION_HPP_