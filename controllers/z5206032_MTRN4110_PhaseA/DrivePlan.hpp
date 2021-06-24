#ifndef DRIVE_PLAN_HPP_
#define DRIVE_PLAN_HPP_

#include <algorithm>
#include <deque>
#include <fstream>
#include <sstream>
#include <string>

#include "Util.hpp"

class DrivePlan {
   public:
    DrivePlan(std::string fileName) {
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

    ~DrivePlan() { printConsole("Motion plan executed!"); }

    const auto getMotionPlan() const -> std::string { return motionPlan_; }

    const auto nextSequence() -> char {
        if (motionSequence_.empty() == true) {
            return 'E';
        }

        auto sequence = motionSequence_.front();
        motionSequence_.pop_front();
        return sequence;
    }

    const auto getInitialLocalisation() const -> std::pair<int, int> {
        return {static_cast<int>(motionPlan_[0]),   // row
                static_cast<int>(motionPlan_[1])};  // column
    }

    const auto getInitialHeading() const -> char { return motionPlan_[2]; }

    auto displayMotionSequence() const -> void {
        auto ss = std::stringstream();
        for (auto &sequence : motionSequence_) {
            ss << sequence;
        }
        printConsole(ss.str());
    }

   private:
    std::string motionPlan_;
    std::deque<char> motionSequence_;
};

#endif  // DRIVE_PLAN_HPP_