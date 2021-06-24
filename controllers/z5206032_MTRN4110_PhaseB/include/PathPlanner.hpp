#ifndef PATH_PLANNER_HPP_
#define PATH_PLANNER_HPP_

#include <string>

namespace mtrn4110 {

class PathPlanner {
   public:
    explicit PathPlanner(std::string);
    explicit PathPlanner(PathPlanner const &) = delete;
    PathPlanner(PathPlanner &&) noexcept;
    ~PathPlanner() = default;

   private:
};
}  // namespace mtrn4110

#endif  // PATH_PLANNER_HPP_