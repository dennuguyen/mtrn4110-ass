#ifndef UTIL_HPP_
#define UTIL_HPP_

#include <string>

namespace mtrn4110 {

auto print(std::ostream& os, std::string const& t) -> void;
auto print(std::string const&) -> void;

}  // namespace mtrn4110

#endif  // UTIL_HPP_