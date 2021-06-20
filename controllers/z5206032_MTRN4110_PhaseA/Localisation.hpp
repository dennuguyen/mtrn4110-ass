#ifndef LOCALISATION_HPP_
#define LOCALISATION_HPP_

#include <array>

class Localisation {
   public:
    Localisation(std::pair<int, int> position, char heading) : position(position) {
        switch (heading) {
            case 'N':
                headingIndex = 0;
                break;
            case 'E':
                headingIndex = 1;
                break;
            case 'S':
                headingIndex = 2;
                break;
            case 'W':
                headingIndex = 3;
                break;
            default:
                std::cerr << "WARNING: Invalid heading in motion plan." << std::endl;
        }
    }

    const auto getRow() -> int { return position.first; }

    const auto getColumn() -> int { return position.second; }

    const auto getHeading() -> char { return cardinalPoints[headingIndex]; }

    auto tick(char instruction) -> void {
        switch (instruction) {
            case 'F':
                updateposition();
                break;
            case 'L':
                headingIndex--;
                if (headingIndex < 0) {
                    headingIndex += 4;
                }
                break;
            case 'R':
                headingIndex++;
                if (headingIndex > 3) {
                    headingIndex -= 4;
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
                position.first--;
                break;
            case 'E':
                position.second++;
                break;
            case 'S':
                position.first++;
                break;
            case 'W':
                position.second--;
                break;
            default:
                std::cerr << "WARNING: Invalid character for cardinal direction." << std::endl;
        }
    }

   private:
    const std::array<char, 4> cardinalPoints = {'N', 'E', 'S', 'W'};
    std::pair<int, int> position;  // row, column
    int headingIndex;
};

#endif  // LOCALISATION_HPP_