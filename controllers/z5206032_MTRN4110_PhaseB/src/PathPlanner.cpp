#include "PathPlanner.hpp"

#include <fstream>
#include <iostream>
#include <iterator>
#include <queue>
#include <sstream>
#include <stack>
#include <stdexcept>
#include <vector>

#include "Util.hpp"

namespace mtrn4110 {

PathPlanner::PathPlanner(std::string const& fileName) {
    auto const& map = readMapFile(fileName);
    buildGraph(map);
    buildDirectedGraph();
    searchPaths();
    writePathPlan();
}

PathPlanner::PathPlanner(PathPlanner&& pathPlanner) noexcept
    : directedGraph_(pathPlanner.directedGraph_),
      paths_(pathPlanner.paths_),
      start_(pathPlanner.start_),
      end_(pathPlanner.end_) {}

auto PathPlanner::readMapFile(std::string const& fileName) const -> std::vector<std::string> {
    print("Reading in map from " + fileName + "...");
    auto mapFile = std::ifstream(fileName.data());
    if (mapFile.good() == false) {
        throw std::runtime_error("Could not open file.");
    }

    auto line = std::string();
    auto map = std::vector<std::string>();
    while (std::getline(mapFile, line)) {
        map.push_back(line);
    }

    if (mapFile.bad() == true) {
        throw std::runtime_error("I/O error while reading.");
    }
    if (mapFile.eof() == false) {
        throw std::runtime_error("Did not reach EOF.");
    }

    print("Map read in!");

    return map;
}

auto PathPlanner::buildGraph(std::vector<std::string> const& map) -> void {
    if (map.empty() == true) {
        throw std::runtime_error("Cannot build graph from empty map.");
    }

    for (auto line = 0; line < static_cast<int>(map.size()); line++) {
        for (auto col = 0; col < static_cast<int>(map.at(0).size()); col++) {
            // At centre of tile.
            if ((col + 2) % 4 == 0 && line % 2 != 0) {
                auto x = (col - 2) / 4;
                auto y = (line - 1) / 2;

                // Check start position.
                if (map[line][col] == 'v') {
                    start_ = {x, y};
                }

                // Check end position.
                if (map[line][col] == 'x') {
                    end_ = {x, y};
                }

                // Check vertical wall to right of centre of tile.
                if (col + 2 < static_cast<int>(map.at(0).size())) {
                    if (map[line][col + 2] == ' ') {
                        directedGraph_[{x, y}].first = unvisited;
                        directedGraph_[{x, y}].second.emplace_back(x + 1, y);
                        directedGraph_[{x + 1, y}].first = unvisited;
                        directedGraph_[{x + 1, y}].second.emplace_back(x, y);
                    }
                }

                // Check horizontal wall below centre of tile.
                if (line + 1 < static_cast<int>(map.size())) {
                    if (map[line + 1][col] == ' ') {
                        directedGraph_[{x, y}].first = unvisited;
                        directedGraph_[{x, y}].second.emplace_back(x, y + 1);
                        directedGraph_[{x, y + 1}].first = unvisited;
                        directedGraph_[{x, y + 1}].second.emplace_back(x, y);
                    }
                }
            }
        }
    }
}

auto PathPlanner::buildDirectedGraph() noexcept -> void {
    directedGraph_.at(start_).first = 0;

    auto pathQueue = std::queue<std::pair<int, int>>();
    pathQueue.push(start_);

    while (pathQueue.empty() == false) {
        auto const currentPosition = pathQueue.front();
        pathQueue.pop();

        // Found the destination.
        if (currentPosition == end_) {
            break;
        }

        for (auto const& adjacentPosition : directedGraph_.at(currentPosition).second) {
            // Give unvisited words a distance from source.
            if (directedGraph_.at(adjacentPosition).first == unvisited) {
                directedGraph_.at(adjacentPosition).first =
                    directedGraph_.at(currentPosition).first + 1;
                pathQueue.push(adjacentPosition);
            }
        }
    }
}

auto PathPlanner::searchPaths() noexcept -> void {
    // Initialisepath.
    auto path = std::vector<std::pair<int, int>>();
    path.emplace_back(start_);

    // Initialise path stack.
    auto pathStack = std::stack<std::pair<std::pair<int, int>, std::vector<std::pair<int, int>>>>();
    pathStack.push(std::make_pair(start_, path));

    while (pathStack.empty() == false) {
        auto const pairValue = pathStack.top();
        auto const& currentPosition = pairValue.first;
        auto const& path = pairValue.second;
        pathStack.pop();

        // Found the destination.
        if (currentPosition == end_) {
            paths_.push_back(path);
        }

        for (auto const& adjacentPosition : directedGraph_.at(currentPosition).second) {
            // Check for direction of graph.
            if (directedGraph_.at(adjacentPosition).first >
                directedGraph_.at(currentPosition).first) {
                auto newPath = std::vector<std::pair<int, int>>(path);
                newPath.emplace_back(adjacentPosition);
                pathStack.push(std::make_pair(adjacentPosition, newPath));
            }
        }
    }
}

auto PathPlanner::writePathPlan() const noexcept -> void {
    for (auto const& i : paths_) {
        for (auto const& j : i) {
            std::cout << "(" << j.second << ", " << j.first << "), ";
        }
        std::cout << std::endl;
    }
}

}  // namespace mtrn4110
