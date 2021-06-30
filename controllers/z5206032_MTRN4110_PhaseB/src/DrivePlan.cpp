#include "DrivePlan.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "Util.hpp"

namespace mtrn4110 {

DrivePlan::DrivePlan(std::string const& fileName) {
    // Read motion plan file.
    // print("Reading in motion plan from " + fileName + "...");
    auto motionPlanFile = std::fstream(fileName.c_str(), std::fstream::in);
    if (motionPlanFile.good() == false) {
        throw std::runtime_error("Could not open file.");
    }

    // Get motion plan assuming it is valid.
    motionPlanFile >> motionPlan_;
    // print("Motion Plan: " + motionPlan_);

    std::copy(motionPlan_.begin() + 3, motionPlan_.end(), std::back_inserter(motionSequence_));
    // print("Motion plan read in!");

    // print("Executing motion plan...");

    if (motionPlanFile.bad() == true) {
        throw std::runtime_error("I/O error while reading.");
    }
    if (motionPlanFile.eof() == false) {
        throw std::runtime_error("Did not reach EOF.");
    }
}

DrivePlan::DrivePlan(DrivePlan&& drivePlan) noexcept
    : motionPlan_(std::move(drivePlan.motionPlan_)),
      motionSequence_(std::move(drivePlan.motionSequence_)) {}

DrivePlan::~DrivePlan() { print("Motion plan executed!"); }

auto DrivePlan::getMotionPlan() const noexcept -> std::string { return motionPlan_; }

auto DrivePlan::nextSequence() noexcept -> char {
    if (motionSequence_.empty() == true) {
        return 'E';
    }

    auto sequence = motionSequence_.front();
    motionSequence_.pop_front();
    return sequence;
}

auto DrivePlan::getInitialLocalisation() const noexcept -> std::pair<int, int> {
    return {static_cast<int>(motionPlan_[0]),   // row
            static_cast<int>(motionPlan_[1])};  // column
}

auto DrivePlan::getInitialHeading() const noexcept -> char { return motionPlan_[2]; }

auto DrivePlan::displayMotionSequence() const noexcept -> void {
    auto ss = std::stringstream();
    for (auto& sequence : motionSequence_) {
        ss << sequence;
    }
    print(ss.str());
}

}  // namespace mtrn4110
