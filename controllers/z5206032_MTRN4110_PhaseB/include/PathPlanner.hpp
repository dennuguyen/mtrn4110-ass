#ifndef PATH_PLANNER_HPP_
#define PATH_PLANNER_HPP_

#include <map>
#include <string>
#include <vector>

namespace mtrn4110 {

using Coordinate = std::pair<int, int>;

class PathPlanner {
   public:
    explicit PathPlanner(std::string const &);
    explicit PathPlanner(PathPlanner const &) = delete;
    PathPlanner(PathPlanner &&) noexcept;
    ~PathPlanner() = default;

   private:
    auto readMapFile(std::string const &) -> void;
    auto buildGraph() -> void;
    auto buildDirectedGraph() -> void;
    auto searchPaths() -> void;
    auto writePathPlan() const -> void;

   private:
    std::map<Coordinate, std::pair<int, std::vector<Coordinate>>> directedGraph_;
    std::vector<std::vector<Coordinate>> paths_;
    Coordinate start_;
    Coordinate destination_;
};
}  // namespace mtrn4110

#endif  // PATH_PLANNER_HPP_