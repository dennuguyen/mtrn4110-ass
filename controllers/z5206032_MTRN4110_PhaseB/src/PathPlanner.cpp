#include "PathPlanner.hpp"

#include <algorithm>
#include <fstream>
#include <iterator>
#include <queue>
#include <sstream>
#include <stack>
#include <stdexcept>
#include <tuple>
#include <vector>

#include "Util.hpp"

namespace mtrn4110 {

PathPlanner::PathPlanner(std::string const& fileName) {
    readMapFile(fileName);
    buildGraph();
    buildDirectedGraph();
    searchPaths();
    printPaths();
    printLeastTurnsPath();
}

PathPlanner::PathPlanner(PathPlanner&& pathPlanner) noexcept
    : graph_(pathPlanner.graph_),
      paths_(pathPlanner.paths_),
      start_(pathPlanner.start_),
      end_(pathPlanner.end_) {}

auto PathPlanner::readMapFile(std::string const& fileName) -> void {
    print("Reading in map from " + fileName + "...");
    auto mapFile = std::ifstream(fileName.data());
    if (mapFile.good() == false) {
        throw std::runtime_error("Could not open file.");
    }

    auto line = std::string();
    while (std::getline(mapFile, line)) {
        map_.push_back(line);
    }

    if (mapFile.bad() == true) {
        throw std::runtime_error("I/O error while reading.");
    }
    if (mapFile.eof() == false) {
        throw std::runtime_error("Did not reach EOF.");
    }

    print("Map read in!");
}

auto PathPlanner::buildGraph() -> void {
    if (map_.empty() == true) {
        throw std::runtime_error("Cannot build graph from empty map.");
    }

    auto const& maxLine = static_cast<int>(map_.size());
    auto const& maxColumn = static_cast<int>(map_.at(0).size());
    for (auto line = 0; line < maxLine; line++) {
        for (auto col = 0; col < maxColumn; col++) {
            // At centre of tile.
            if ((col + 2) % 4 == 0 && line % 2 != 0) {
                auto const x = (col - 2) / 4;
                auto const y = (line - 1) / 2;

                // Check start and end position.
                switch (map_[line][col]) {
                    case ' ':
                        break;
                    case 'x':
                        end_ = {x, y};
                        break;
                    case '^':
                        heading_ = 0;
                        start_ = {x, y};
                        break;
                    case '>':
                        heading_ = 1;
                        start_ = {x, y};
                        break;
                    case 'v':
                        heading_ = 2;
                        start_ = {x, y};
                        break;
                    case '<':
                        heading_ = 3;
                        start_ = {x, y};
                        break;
                    default:
                        throw std::runtime_error("Invalid map character");
                }

                // Check vertical wall to right of centre of tile.
                if (col + 2 < maxColumn) {
                    if (map_[line][col + 2] == ' ') {
                        graph_[{x, y}].first = unvisited;
                        graph_[{x, y}].second.emplace_back(x + 1, y);
                        graph_[{x + 1, y}].first = unvisited;
                        graph_[{x + 1, y}].second.emplace_back(x, y);
                    }
                }

                // Check horizontal wall below centre of tile.
                if (line + 1 < maxLine) {
                    if (map_[line + 1][col] == ' ') {
                        graph_[{x, y}].first = unvisited;
                        graph_[{x, y}].second.emplace_back(x, y + 1);
                        graph_[{x, y + 1}].first = unvisited;
                        graph_[{x, y + 1}].second.emplace_back(x, y);
                    }
                }
            }
        }
    }
}

auto PathPlanner::buildDirectedGraph() noexcept -> void {
    graph_.at(end_).first = 0;

    auto pathQueue = std::queue<std::pair<int, int>>();
    pathQueue.push(end_);

    while (pathQueue.empty() == false) {
        auto const currentPosition = pathQueue.front();
        pathQueue.pop();

        // Found the destination.
        if (currentPosition == start_) {
            break;
        }

        for (auto const& adjacentPosition : graph_.at(currentPosition).second) {
            // Give unvisited words a distance from source.
            if (graph_.at(adjacentPosition).first == unvisited) {
                graph_.at(adjacentPosition).first = graph_.at(currentPosition).first + 1;
                pathQueue.push(adjacentPosition);
            }
        }
    }
}

auto PathPlanner::searchPaths() noexcept -> void {
    auto path = std::vector<std::pair<int, int>>();
    path.emplace_back(start_);

    auto pathStack = std::stack<std::tuple<std::pair<int, int>, std::vector<std::pair<int, int>>,
                                           int>>();  // [(point, path, numberTurns)]
    pathStack.push({start_, path, 0});

    while (pathStack.empty() == false) {
        auto const retval = pathStack.top();
        auto const& currentPosition = std::get<0>(retval);
        auto const& path = std::get<1>(retval);
        auto const& numberTurns = std::get<2>(retval);
        pathStack.pop();

        // Found the destination.
        if (currentPosition == end_) {
            // Get heading from last two points and subtract it from the given initial heading to
            // get the number of turns at beginning of path plan.
            auto westEastHeading = path.rbegin()[1].first - path.rbegin()[0].first;
            auto northSouthHeading = path.rbegin()[1].second - path.rbegin()[0].second;
            auto heading = northSouthHeading == 0 ? westEastHeading + 2 : northSouthHeading + 1;
            paths_.emplace_back(path, numberTurns + std::abs((heading - heading_) % 2));
        }

        for (auto const& adjacentPosition : graph_.at(currentPosition).second) {
            // Check for direction of graph.
            if (graph_.at(adjacentPosition).first <= graph_.at(currentPosition).first) {
                auto newPath = std::vector<std::pair<int, int>>(path);
                newPath.emplace_back(adjacentPosition);

                // Given (i, j), the following is true for 3 points on an L-shaped path:
                //      (i-1) + (i) + (i+1) != 3*i   for all values of i
                //      (j-1) + (j) + (j+1) != 3*j   for all values of j
                auto newNumberTurns = numberTurns;
                if (newPath.size() > 2 &&
                    3 * newPath.rbegin()[1].first != newPath.rbegin()[0].first +
                                                         newPath.rbegin()[1].first +
                                                         newPath.rbegin()[2].first &&
                    3 * newPath.rbegin()[1].second != newPath.rbegin()[0].second +
                                                          newPath.rbegin()[1].second +
                                                          newPath.rbegin()[2].second) {
                    newNumberTurns++;
                }

                pathStack.push({adjacentPosition, newPath, newNumberTurns});
            }
        }
    }
}

auto PathPlanner::printPaths() const noexcept -> void {
    print("Finding shortest paths...");

    for (auto i = 0; i < static_cast<int>(paths_.size()); i++) {
        print("Path - " + std::to_string(i + 1) + ":");

        auto const& path = paths_.at(i).first;
        auto tempMap = map_;
        for (auto const& position : path) {
            auto const col = 4 * position.first + 2;
            auto const line = 2 * position.second + 1;

            // Do no write over this.
            if (position == start_) {
                continue;
            }

            // Write path weighting into map.
            auto const index = std::to_string(graph_.at(position).first);
            tempMap[line][col] = index[0];
            tempMap[line][col + 1] = index.size() > 1 ? index[1] : ' ';
        }

        for (auto const& line : tempMap) {
            print(line);
        }
    }
    print(std::to_string(paths_.size()) + " shortest paths found!");
}

auto PathPlanner::printLeastTurnsPath() const noexcept -> void {
    print("Finding shortest path with least turns...");

    auto const& leastTurnsPath =
        std::min_element(paths_.begin(), paths_.end(),
                         [](auto const& a, auto const& b) { return a.second < b.second; });
    auto tempMap = map_;
    for (auto const& position : leastTurnsPath->first) {
        auto const col = 4 * position.first + 2;
        auto const line = 2 * position.second + 1;

        if (position == start_) {
            continue;
        }

        auto const index = std::to_string(graph_.at(position).first);
        tempMap[line][col] = index[0];
        tempMap[line][col + 1] = index.size() > 1 ? index[1] : ' ';
    }

    for (auto const& line : tempMap) {
        print(line);
    }
    print("Shortest path with least turns found!");

    print("Path Plan (" + std::to_string(2) + "steps): ");
}

auto PathPlanner::writePathPlan() const noexcept -> void {}

}  // namespace mtrn4110
