#ifndef PATH_PLANNER_HPP_
#define PATH_PLANNER_HPP_

#include <map>
#include <string>
#include <vector>

namespace mtrn4110 {

class PathPlanner {
   public:
    explicit PathPlanner(std::string const &);
    explicit PathPlanner(PathPlanner const &) = delete;
    PathPlanner(PathPlanner &&) noexcept;
    ~PathPlanner() = default;

   private:
    auto readMapFile(std::string const &) const -> std::vector<std::string> const;
    auto buildGraph(std::vector<std::string> const &map) -> void;
    auto buildDirectedGraph() -> void;
    auto searchPaths() -> void;
    auto writePathPlan() const -> void;

   public:
    static auto constexpr unvisited = -1;

   private:
    std::map<std::pair<int, int>, std::pair<int, std::vector<std::pair<int, int>>>> directedGraph_;
    std::vector<std::vector<std::pair<int, int>>> paths_;
    std::pair<int, int> start_;
    std::pair<int, int> end_;
};

}  // namespace mtrn4110

#endif  // PATH_PLANNER_HPP_