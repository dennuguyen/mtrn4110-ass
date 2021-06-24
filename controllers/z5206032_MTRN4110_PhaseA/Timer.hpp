#ifndef TIMER_HPP_
#define TIMER_HPP_

#include <memory>
#include <ostream>
#include <webots/Robot.hpp>

namespace mtrn4110 {

class Timer {
   public:
    explicit Timer(webots::Robot&);
    ~Timer() = default;

    auto time(double expiry) -> void;
    auto const expired() const -> bool;
    friend auto operator<<(std::ostream&, Timer const&) -> std::ostream&;

   private:
    std::shared_ptr<webots::Robot> robot_;
    double ref_;  // Time of reference
    double exp_;  // Expiry duration
};

}  // namespace mtrn4110

#endif  // TIMER_HPP_