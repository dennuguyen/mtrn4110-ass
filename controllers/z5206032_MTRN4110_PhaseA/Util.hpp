#ifndef UTIL_HPP_
#define UTIL_HPP_

#include <memory>
#include <webots/Robot.hpp>

auto printConsole(std::string str) -> void {
    std::cout << "[z5206032_MTRN4110_PhaseA] " << str << std::endl;
}

class Timer {
   public:
    Timer(webots::Robot &robot) : robot(std::make_unique<webots::Robot>(robot)), ref(), exp() {}
    ~Timer() {}

    void time(double expiry) {
        ref = robot->getTime();
        exp = expiry;
    }

    bool expired() { return (robot->getTime() - ref) > exp; }

    void print() {
        std::cout << "Time: " << ref << std::endl;
        std::cout << "Expiry: " << exp << std::endl;
    }

   private:
    std::unique_ptr<webots::Robot> robot;
    double ref;  // Time of reference
    double exp;  // Expiry duration
};

#endif  // UTIL_HPP_