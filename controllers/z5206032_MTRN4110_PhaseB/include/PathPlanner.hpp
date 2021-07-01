#ifndef PATH_PLANNER_HPP_
#define PATH_PLANNER_HPP_

#include <array>
#include <map>
#include <ostream>
#include <string>
#include <vector>

namespace mtrn4110 {

class PathPlanner {
   public:
    explicit PathPlanner(std::string const &, std::string const &);
    explicit PathPlanner(PathPlanner const &) = delete;
    PathPlanner(PathPlanner &&) noexcept;
    ~PathPlanner() = default;

   private:
    auto readMapFile(std::string const &) -> void;
    auto buildGraph() -> void;
    auto buildDirectedGraph() noexcept -> void;
    auto searchPaths() noexcept -> void;
    auto searchLeastTurnsPath() noexcept -> void;
    auto printPath(std::vector<std::pair<int, int>> const &) const noexcept -> void;
    auto writePathPlan2txt(std::string const &) const -> void;
    auto getHeadingIndex(std::pair<int, int>, std::pair<int, int>) const -> int;
    auto getAction(int a, int b) const -> std::string;

   public:
    static auto constexpr unvisited = -1;
    const std::array<char, 4> cardinalPoints = {'N', 'E', 'S', 'W'};

   private:
    std::vector<std::string> map_;
    std::map<std::pair<int, int>, std::pair<int, std::vector<std::pair<int, int>>>>
        graph_;  // {point, (visitWeight, [point])}
    std::vector<std::pair<std::vector<std::pair<int, int>>, std::string>>
        paths_;  // [(path, pathPlan)]
    std::vector<std::pair<std::vector<std::pair<int, int>>, std::string>>::iterator leastTurnsPath_;
    std::pair<int, int> start_;
    std::pair<int, int> end_;
    int heading_;  // N = 0, E = 1, S = 2, W = 3
};

}  // namespace mtrn4110

#endif  // PATH_PLANNER_HPP_