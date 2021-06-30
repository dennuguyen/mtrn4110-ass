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
    auto writePathPlan() const noexcept -> void;

   public:
    static auto constexpr unvisited = -1;

   private:
    std::vector<std::string> map_;
    std::map<std::pair<int, int>, std::pair<int, std::vector<std::pair<int, int>>>> graph_;
    std::vector<std::vector<std::pair<int, int>>> paths_;
    std::pair<int, int> start_;
    std::pair<int, int> end_;
};

}  // namespace mtrn4110

#endif  // PATH_PLANNER_HPP_