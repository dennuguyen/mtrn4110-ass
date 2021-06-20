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
        motionPlanFile >> motionPlan;
        printConsole("Motion Plan: " + motionPlan);

        std::copy(motionPlan.begin() + 3, motionPlan.end(), std::back_inserter(motionSequence));
        printConsole("Motion plan read in!");

        printConsole("Executing motion plan...");
    }

    ~DrivePlan() { printConsole("Motion plan executed!"); }

    const auto getMotionPlan() -> std::string { return motionPlan; }

    const auto nextSequence() -> char {
        if (motionSequence.empty() == true) {
            return 'E';
        }

        auto sequence = motionSequence.front();
        motionSequence.pop_front();
        return sequence;
    }

    const auto getInitialLocalisation() -> std::pair<int, int> {
        return {static_cast<int>(motionPlan[0]),   // row
                static_cast<int>(motionPlan[1])};  // column
    }

    const auto getInitialHeading() -> char { return motionPlan[2]; }

    auto displayMotionSequence() -> void {
        auto ss = std::stringstream();
        for (auto &sequence : motionSequence) {
            ss << sequence;
        }
        printConsole(ss.str());
    }

   private:
    std::string motionPlan;
    std::deque<char> motionSequence;
};

#endif  // DRIVE_PLAN_HPP_