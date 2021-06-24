#include "DrivePlan.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>

#include "Util.hpp"

namespace mtrn4110 {

DrivePlan::DrivePlan(std::string fileName) {
    // Read motion plan file.
    printConsole("Reading in motion plan from " + fileName + "...");
    auto motionPlanFile = std::fstream(fileName.c_str(), std::fstream::in);
    if (motionPlanFile.good() == false) {
        throw std::runtime_error("ERROR: No such file.");
    }

    // Get motion plan assuming it is valid.
    motionPlanFile >> motionPlan_;
    printConsole("Motion Plan: " + motionPlan_);

    std::copy(motionPlan_.begin() + 3, motionPlan_.end(), std::back_inserter(motionSequence_));
    printConsole("Motion plan read in!");

    printConsole("Executing motion plan...");
}

DrivePlan::DrivePlan(DrivePlan&& drivePlan) noexcept
    : motionPlan_(std::move(drivePlan.motionPlan_)),
      motionSequence_(std::move(drivePlan.motionSequence_)) {}

DrivePlan::~DrivePlan() { printConsole("Motion plan executed!"); }

auto const DrivePlan::getMotionPlan() const -> std::string { return motionPlan_; }

auto const DrivePlan::nextSequence() -> char {
    if (motionSequence_.empty() == true) {
        return 'E';
    }

    auto sequence = motionSequence_.front();
    motionSequence_.pop_front();
    return sequence;
}

auto const DrivePlan::getInitialLocalisation() const -> std::pair<int, int> {
    return {static_cast<int>(motionPlan_[0]),   // row
            static_cast<int>(motionPlan_[1])};  // column
}

auto const DrivePlan::getInitialHeading() const -> char { return motionPlan_[2]; }

auto DrivePlan::displayMotionSequence() const -> void {
    auto ss = std::stringstream();
    for (auto& sequence : motionSequence_) {
        ss << sequence;
    }
    printConsole(ss.str());
}

}  // namespace mtrn4110
