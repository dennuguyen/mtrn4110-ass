#ifndef LOCALISATION_HPP_
#define LOCALISATION_HPP_

#include <array>

class Localisation {
   public:
    Localisation(std::pair<int, int> position, char heading) : position_(position) {
        switch (heading) {
            case 'N':
                headingIndex_ = 0;
                break;
            case 'E':
                headingIndex_ = 1;
                break;
            case 'S':
                headingIndex_ = 2;
                break;
            case 'W':
                headingIndex_ = 3;
                break;
            default:
                std::cerr << "WARNING: Invalid heading in motion plan." << std::endl;
        }
    }

    const auto getRow() -> int { return position_.first; }

    const auto getColumn() -> int { return position_.second; }

    const auto getHeading() -> char { return cardinalPoints_[headingIndex_]; }

    auto tick(char instruction) -> void {
        switch (instruction) {
            case 'F':
                updateposition();
                break;
            case 'L':
                headingIndex_--;
                if (headingIndex_ < 0) {
                    headingIndex_ += 4;
                }
                break;
            case 'R':
                headingIndex_++;
                if (headingIndex_ > 3) {
                    headingIndex_ -= 4;
                }
                break;
            default:
                std::cerr << "WARNING: Invalid character in motion plan." << std::endl;
        }
    }

   private:
    auto updateposition() -> void {
        switch (getHeading()) {
            case 'N':
                position_.first--;
                break;
            case 'E':
                position_.second++;
                break;
            case 'S':
                position_.first++;
                break;
            case 'W':
                position_.second--;
                break;
            default:
                std::cerr << "WARNING: Invalid character for cardinal direction." << std::endl;
        }
    }

   private:
    const std::array<char, 4> cardinalPoints_ = {'N', 'E', 'S', 'W'};
    std::pair<int, int> position_;  // row, column
    int headingIndex_;
};

#endif  // LOCALISATION_HPP_