#ifndef DRIVE_PLAN_HPP_
#define DRIVE_PLAN_HPP_

#include <deque>
#include <string>

namespace mtrn4110 {

class DrivePlan {
   public:
    explicit DrivePlan(std::string const &);
    explicit DrivePlan(DrivePlan const &) = delete;
    DrivePlan(DrivePlan &&) noexcept;
    ~DrivePlan();
    auto operator=(DrivePlan const &) -> DrivePlan & = delete;
    auto operator=(DrivePlan &&) -> DrivePlan & = delete;
    auto getMotionPlan() const noexcept -> std::string;
    auto nextSequence() noexcept -> char;
    auto getInitialLocalisation() const noexcept -> std::pair<int, int>;
    auto getInitialHeading() const noexcept -> char;
    auto displayMotionSequence() const noexcept -> void;

   private:
    std::string motionPlan_;
    std::deque<char> motionSequence_;
};

}  // namespace mtrn4110

#endif  // DRIVE_PLAN_HPP_