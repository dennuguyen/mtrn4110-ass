#ifndef TIMER_HPP_
#define TIMER_HPP_

#include <memory>
#include <ostream>
#include <webots/Robot.hpp>

namespace mtrn4110 {

class Timer {
   public:
    explicit Timer(webots::Robot &);
    explicit Timer(Timer const &) = delete;
    explicit Timer(Timer &&) = delete;
    ~Timer() = default;
    auto operator=(Timer const &) -> Timer & = delete;
    auto operator=(Timer &&) -> Timer & = delete;

    auto time(double expiry) noexcept -> void;
    auto const expired() const noexcept -> bool;
    friend auto operator<<(std::ostream &, Timer const &) noexcept -> std::ostream &;

   private:
    std::shared_ptr<webots::Robot> robot_;
    double ref_;  // Time of reference
    double exp_;  // Expiry duration
};

}  // namespace mtrn4110

#endif  // TIMER_HPP_