#ifndef DRIVE_PLAN_HPP_
#define DRIVE_PLAN_HPP_

#include <deque>
#include <string>

namespace mtrn4110 {

class DrivePlan {
   public:
    explicit DrivePlan(std::string);
    explicit DrivePlan(DrivePlan const&) = delete;
    DrivePlan(DrivePlan&&) noexcept;
    ~DrivePlan();
    auto const getMotionPlan() const -> std::string;
    auto const nextSequence() -> char;
    auto const getInitialLocalisation() const -> std::pair<int, int>;
    auto const getInitialHeading() const -> char;
    auto displayMotionSequence() const -> void;

   private:
    std::string motionPlan_;
    std::deque<char> motionSequence_;
};

}  // namespace mtrn4110

#endif  // DRIVE_PLAN_HPP_