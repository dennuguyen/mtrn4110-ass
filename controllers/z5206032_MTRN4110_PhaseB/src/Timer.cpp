#include "Timer.hpp"

namespace mtrn4110 {

Timer::Timer(webots::Robot& robot)
    : robot_(std::make_shared<webots::Robot>(robot)), ref_(), exp_() {}

auto Timer::time(double expiry) noexcept -> void {
    ref_ = robot_->getTime();
    exp_ = expiry;
}

auto const Timer::expired() const noexcept -> bool { return (robot_->getTime() - ref_) > exp_; }

auto operator<<(std::ostream& ss, Timer const& timer) noexcept -> std::ostream& {
    ss << "Time: " << timer.ref_ << std::endl;
    ss << "Expiry: " << timer.exp_ << std::endl;
    return ss;
}

}  // namespace mtrn4110