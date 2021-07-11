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
    // Explicit constructor which calls the private members of PathPlanner. There is no need for a
    // PathPlanner object to exist after construction.
    explicit PathPlanner(std::string const &, std::string const &);

    // Copy constructor.
    explicit PathPlanner(PathPlanner const &) = delete;

    // Move constructor.
    explicit PathPlanner(PathPlanner &&) noexcept;

    // Default destructor.
    ~PathPlanner() = default;

    // Copy assignment.
    auto operator=(PathPlanner const &) -> PathPlanner & = delete;

    // Move assignment.
    auto operator=(PathPlanner &&) -> PathPlanner & = delete;

   private:
    // Reads a map from a file and stores it in map_.
    auto readMapFile(std::string const &) -> void;

    // Build a graph from the map.
    auto buildGraph() -> void;

    // Perform a BFS to get directedness of the graph from end_ to start_.
    auto buildDirectedGraph() noexcept -> void;

    // Perform a DFS to get the shortest possible paths from start_ to end_ and generate path plans
    // during DFS.
    auto searchPaths() noexcept -> void;

    // Searches the shortest possible paths for any path with the least number of turns.
    auto searchLeastTurnsPath() noexcept -> void;

    // Prints out the path on the map.
    auto printPath(std::vector<std::pair<int, int>> const &) const noexcept -> void;

    // Write least turns path to a file.
    auto writePathPlan2txt(std::string const &) const -> void;

    // Gets the heading when moving from point a to point b.
    auto getHeadingIndex(std::pair<int, int>, std::pair<int, int>) const -> int;

    // Gets the required actions from b to a.
    auto getAction(int a, int b) const -> std::string;

   public:
    static auto constexpr unvisited = -1;
    const std::array<char, 4> cardinalPoints = {'N', 'E', 'S', 'W'};

   private:
    // Map is represented as a vector of string for the ease of use of [][] access operators.
    std::vector<std::string> map_;

    // Graph is represented as an adjacency list; mapping a point to adjacent points and mapping a
    // point to an edge weighting. Pythonically: {point, (edgeWeight, [point])}
    std::map<std::pair<int, int>, std::pair<int, std::vector<std::pair<int, int>>>> graph_;

    // A vector of the shortest paths which is paired with their path plan sequence. Pythonically:
    // [(path, pathPlan)]
    std::vector<std::pair<std::vector<std::pair<int, int>>, std::string>> paths_;

    // An iterator to paths_ representing the shortest path with the least turns.
    std::vector<std::pair<std::vector<std::pair<int, int>>, std::string>>::iterator leastTurnsPath_;

    // Start position of path plan.
    std::pair<int, int> start_;

    // End position of path plan.
    std::pair<int, int> end_;

    // Initial heading represented as an index to cardinalPoints.
    int initialHeading_;  // N = 0, E = 1, S = 2, W = 3
};

}  // namespace mtrn4110

#endif  // PATH_PLANNER_HPP_