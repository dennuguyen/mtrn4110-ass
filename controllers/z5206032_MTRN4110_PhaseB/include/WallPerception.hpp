#ifndef WALL_PERCEPTION_HPP_
#define WALL_PERCEPTION_HPP_

#include <array>
#include <memory>
#include <webots/Lidar.hpp>
#include <webots/Robot.hpp>

namespace mtrn4110 {

class WallPerception {
   public:
    explicit WallPerception(webots::Robot &);
    explicit WallPerception(WallPerception const &) = delete;
    WallPerception(WallPerception &&) noexcept;
    ~WallPerception() = default;
    auto const getLeftWall() const noexcept -> char;
    auto const getFrontWall() const noexcept -> char;
    auto const getRightWall() const noexcept -> char;
    auto const tick() noexcept -> int;

   public:
    static auto constexpr wallDistance = 0.085;  // Distance from centre of cell to wall.

   private:
    std::unique_ptr<webots::Lidar> lidar_;  // 360 lidar.
    std::array<char, 3> walls_;
    static auto constexpr left_ = 0;
    static auto constexpr front_ = 1;
    static auto constexpr right_ = 2;
};

}  // namespace mtrn4110

#endif  // WALL_PERCEPTION_HPP_