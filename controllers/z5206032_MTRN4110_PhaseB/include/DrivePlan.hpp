#ifndef DRIVE_PLAN_HPP_
#define DRIVE_PLAN_HPP_

#include <deque>
#include <string>

namespace mtrn4110 {

class DrivePlan {
   public:
    explicit DrivePlan(std::string const&);
    explicit DrivePlan(DrivePlan const&) = delete;
    DrivePlan(DrivePlan&&) noexcept;
    ~DrivePlan();
    auto getMotionPlan() const -> std::string const;
    auto nextSequence() -> char const;
    auto getInitialLocalisation() const -> std::pair<int, int> const;
    auto getInitialHeading() const -> char const;
    auto displayMotionSequence() const -> void;

   private:
    std::string motionPlan_;
    std::deque<char> motionSequence_;
};

}  // namespace mtrn4110

#endif  // DRIVE_PLAN_HPP_