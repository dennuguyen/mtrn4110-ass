#include "PathPlanner.hpp"

namespace mtrn4110 {

PathPlanner::PathPlanner(std::string const& fileName) {
    readMapFile(fileName);
    buildGraph();
    buildDirectedGraph();
    searchPaths();
    writePathPlan();
}

PathPlanner::PathPlanner(PathPlanner&& pathPlanner) noexcept
    : directedGraph_(pathPlanner.directedGraph_),
      paths_(pathPlanner.paths_),
      start_(pathPlanner.start_),
      destination_(pathPlanner.destination_) {}

auto PathPlanner::readMapFile(std::string const& fileName) -> void {}

auto PathPlanner::buildGraph() -> void {}

auto PathPlanner::buildDirectedGraph() -> void {}

auto PathPlanner::searchPaths() -> void {}

auto PathPlanner::writePathPlan() const -> void {}

}  // namespace mtrn4110
