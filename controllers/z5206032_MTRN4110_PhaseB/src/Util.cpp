#include "Util.hpp"

#include <iostream>

namespace mtrn4110 {

auto print(std::ostream& os, std::string const& t) -> void {
    os << "[z5206032_MTRN4110_PhaseB] " << t << std::endl;
}

auto print(std::string const& t) -> void {
    std::cout << "[z5206032_MTRN4110_PhaseC] " << t << std::endl;
}

}  // namespace mtrn4110