#include "PathPlanner.hpp"

#include <fstream>
#include <iostream>
#include <sstream>

#include "Util.hpp"

namespace mtrn4110 {

PathPlanner::PathPlanner(std::string const& fileName) {
    auto const& map = readMapFile(fileName);
    buildGraph(map);
    // buildDirectedGraph();
    // searchPaths();
    // writePathPlan();
}

PathPlanner::PathPlanner(PathPlanner&& pathPlanner) noexcept
    : directedGraph_(pathPlanner.directedGraph_),
      paths_(pathPlanner.paths_),
      start_(pathPlanner.start_),
      destination_(pathPlanner.destination_) {}

auto PathPlanner::readMapFile(std::string const& fileName) const -> std::string const {
    // Read map file.
    print("Reading in map from " + fileName + "...");
    auto mapFile = std::fstream(fileName.c_str(), std::fstream::in);
    if (mapFile.good() == false) {
        throw std::runtime_error("ERROR: No such file.");
    }

    // Get map assuming it is valid.
    auto map = std::stringstream();
    map << mapFile.rdbuf();

    print("Map read in!");

    return map.str();
}

auto PathPlanner::buildGraph(std::string const& map) -> void {
    // Parse map into graph.
    auto x = 0;
    auto y = 0;
    auto line = 0;
    auto col = 0;
    for (auto const& symbol : map) {
        if (symbol == '\n') {
            col = 0;
            x = 0;
            line++;
            if (line % 2 == 0) {
                y++;
            }
            continue;
        }

        // Iterate (x, y)
        if (0) {
        }

        // Check for horizontal wall.
        if ((col + 2) % 4 == 0 && line % 2 == 0) {
            if (symbol == ' ') {
                directedGraph_[{x, y}].second.emplace_back(x + 1, y);
                directedGraph_[{x + 1, y}].second.emplace_back(x, y);
            } else {
                directedGraph_[{x, y}].second.emplace_back();
                directedGraph_[{x + 1, y}].second.emplace_back();
            }
        }

        // Check for start and end position.
        if ((col + 2) % 4 == 0 && line % 2 != 0) {
            if (symbol == 'v') {
            }
            if (symbol == 'x') {
            }
        }

        // Check for vertical wall.
        if (col % 4 == 0) {
            if (symbol == ' ') {
                directedGraph_[{x, y}].second.emplace_back(x, y + 1);
                directedGraph_[{x, y + 1}].second.emplace_back(x, y);
            } else {
                directedGraph_[{x, y}].second.emplace_back();
                directedGraph_[{x, y + 1}].second.emplace_back();
            }
            x++;
        }

        // Add edge to graph.

        col++;
    }

    for (auto const& i : directedGraph_) {
        std::cout << "(" << i.first.first << ", " << i.first.second << ") : ";
        for (auto const& j : i.second.second) {
            std::cout << "(" << j.first << ", " << j.second << "), ";
        }
        std::cout << std::endl;
    }
}

auto PathPlanner::buildDirectedGraph() -> void {}

auto PathPlanner::searchPaths() -> void {}

auto PathPlanner::writePathPlan() const -> void {}

}  // namespace mtrn4110
