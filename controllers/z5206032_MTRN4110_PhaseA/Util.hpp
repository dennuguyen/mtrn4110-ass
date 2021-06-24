#ifndef UTIL_HPP_
#define UTIL_HPP_

#include <memory>
#include <webots/Robot.hpp>

auto printConsole(std::string str) -> void {
    std::cout << "[z5206032_MTRN4110_PhaseA] " << str << std::endl;
}

class Timer {
   public:
    Timer(webots::Robot &robot) : robot_(std::make_shared<webots::Robot>(robot)), ref_(), exp_() {}
    ~Timer() {}

    auto time(double expiry) -> void {
        ref_ = robot_->getTime();
        exp_ = expiry;
    }

    const auto expired() const -> bool { return (robot_->getTime() - ref_) > exp_; }

    auto const print() -> void {
        std::cout << "Time: " << ref_ << std::endl;
        std::cout << "Expiry: " << exp_ << std::endl;
    }

   private:
    std::shared_ptr<webots::Robot> robot_;
    double ref_;  // Time of reference
    double exp_;  // Expiry duration
};

#endif  // UTIL_HPP_