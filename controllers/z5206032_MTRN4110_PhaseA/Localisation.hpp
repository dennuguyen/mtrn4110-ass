#ifndef LOCALISATION_HPP_
#define LOCALISATION_HPP_

#include <array>
#include <memory>
#include <webots/InertialUnit.hpp>
#include <webots/Robot.hpp>

namespace mtrn4110 {

class Localisation {
   public:
    explicit Localisation(webots::Robot &, std::pair<int, int>, char);
    explicit Localisation(Localisation const &) = delete;
    Localisation(Localisation &&) noexcept;
    ~Localisation() = default;
    auto const getRow() const -> int;
    auto const getColumn() const -> int;
    auto const getHeading() const -> char;
    auto tick(char instruction) -> void;
    auto const getYaw() const -> double;

   private:
    // Updates the heading using information given by motion plan sequence.
    auto updateHeadingByPlan(char instruction) -> void;

    // Updates the position using information given by motion plan sequence.
    auto updatePositionByPlan() -> void;

    auto updatePositionByOdometry() -> void;

    auto updateHeadingByIMU() -> void;

   public:
    const std::array<char, 4> cardinalPoints = {'N', 'E', 'S', 'W'};

   private:
    std::unique_ptr<webots::InertialUnit> inertialUnit_;
    std::pair<int, int> position_;  // row, column
    int headingIndex_;
};

}  // namespace mtrn4110

#endif  // LOCALISATION_HPP_