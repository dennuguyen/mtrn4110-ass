#ifndef MOTION_CONTROL_HPP_
#define MOTION_CONTROL_HPP_

#include <cmath>
#include <memory>
#include <tuple>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>

#include "Timer.hpp"

namespace mtrn4110 {

class MotionControl {
   public:
    explicit MotionControl(webots::Robot &);
    explicit MotionControl(MotionControl const &) = delete;
    MotionControl(MotionControl &&) noexcept;
    ~MotionControl() = default;
    auto operator=(MotionControl const &) -> MotionControl & = delete;
    auto operator=(MotionControl &&) -> MotionControl & = delete;
    auto tick(char) -> void;
    auto setGain(std::tuple<double, double, double>, std::tuple<double, double, double>) noexcept
        -> void;
    auto setPoint(std::tuple<double, double> left, std::tuple<double, double> right) noexcept
        -> void;

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
    std::unique_ptr<webots::PositionSensor> leftPositionSensor_;
    std::unique_ptr<webots::PositionSensor> rightPositionSensor_;
};

}  // namespace mtrn4110

#endif  // MOTION_CONTROL_HPP_