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
    auto readMapFile(std::string const &) -> void;
    auto buildGraph() -> void;
    auto buildDirectedGraph() noexcept -> void;
    auto searchPaths() noexcept -> void;
    auto printPaths() const noexcept -> void;
    auto printLeastTurnsPath() const noexcept -> void;
    auto writePathPlan() const noexcept -> void;

   public:
    static auto constexpr unvisited = -1;

   private:
    std::vector<std::string> map_;
    std::map<std::pair<int, int>, std::pair<int, std::vector<std::pair<int, int>>>>
        graph_;  // {point, (visitWeight, [point])}
    std::vector<std::pair<std::vector<std::pair<int, int>>, int>> paths_;  // [(path, numberTurns)]
    std::pair<int, int> start_;
    std::pair<int, int> end_;
    int heading_;  // N = 0, E = 1, S = 2, W = 3
};

}  // namespace mtrn4110

#endif  // PATH_PLANNER_HPP_