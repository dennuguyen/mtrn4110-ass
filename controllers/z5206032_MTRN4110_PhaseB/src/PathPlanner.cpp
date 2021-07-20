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

PathPlanner::PathPlanner(std::string const& mapPath, std::string const& pathPlanPath) {
    // Read map.
    readMapFile(mapPath);

    // Build graph from map.
    buildGraph();

    // Build directed graph from graph.
    buildDirectedGraph();

    // Find all shortest paths in directed graph.
    searchPaths();

    // Find path with least number of turns from shortest paths.
    searchLeastTurnsPath();

    // Write out path plan.
    writePathPlan2txt(pathPlanPath);
}

PathPlanner::PathPlanner(PathPlanner&& pathPlanner) noexcept
    : map_(std::move(pathPlanner.map_)),
      graph_(std::move(pathPlanner.graph_)),
      paths_(std::move(pathPlanner.paths_)),
      leastTurnsPath_(std::move(pathPlanner.leastTurnsPath_)),
      start_(std::move(pathPlanner.start_)),
      end_(std::move(pathPlanner.end_)),
      initialHeading_(std::move(pathPlanner.initialHeading_)) {}

auto PathPlanner::readMapFile(std::string const& mapPath) -> void {
    auto mapFile = std::ifstream(mapPath.data());
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
                        initialHeading_ = 0;
                        start_ = {x, y};
                        break;
                    case '>':
                        initialHeading_ = 1;
                        start_ = {x, y};
                        break;
                    case 'v':
                        initialHeading_ = 2;
                        start_ = {x, y};
                        break;
                    case '<':
                        initialHeading_ = 3;
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
                                           std::string>>();  // [(point, path, pathPlan)]
    auto pathPlan = std::to_string(start_.second) + std::to_string(start_.first) +
                    cardinalPoints[initialHeading_];
    pathStack.push({start_, path, pathPlan});

    while (pathStack.empty() == false) {
        auto const retval = pathStack.top();
        auto const& currentPosition = std::get<0>(retval);
        auto const& path = std::get<1>(retval);
        auto const& pathPlan = std::get<2>(retval);
        pathStack.pop();

        // Found the destination.
        if (currentPosition == end_) {
            paths_.emplace_back(path, pathPlan);
        }

        for (auto const& adjacentPosition : graph_.at(currentPosition).second) {
            // Check for direction of graph.
            if (graph_.at(adjacentPosition).first <= graph_.at(currentPosition).first) {
                auto newPath = std::vector<std::pair<int, int>>(path);
                newPath.emplace_back(adjacentPosition);

                auto newPathPlan = pathPlan;
                if (newPath.size() > 1) {
                    auto const& a = newPath.rbegin()[2];
                    auto const& b = newPath.rbegin()[1];
                    auto const& c = newPath.rbegin()[0];
                    if (newPath.size() > 2) {
                        newPathPlan += getAction(getHeadingIndex(a, b), getHeadingIndex(b, c));
                    } else {
                        newPathPlan += getAction(initialHeading_, getHeadingIndex(b, c));
                    }
                }
                pathStack.push({adjacentPosition, newPath, newPathPlan});
            }
        }
    }
}

auto PathPlanner::searchLeastTurnsPath() noexcept -> void {
    auto checkTurn = [](auto const& c) { return c == 'L' || c == 'R'; };
    leastTurnsPath_ =
        std::min_element(paths_.begin(), paths_.end(), [checkTurn](auto const& a, auto const& b) {
            return std::count_if(a.second.begin(), a.second.end(), checkTurn) <
                   std::count_if(b.second.begin(), b.second.end(), checkTurn);
        });
}

auto PathPlanner::printPath(std::vector<std::pair<int, int>> const& path) const noexcept -> void {
    // Fill out a map with the given path.
    auto tempMap = map_;
    for (auto const& position : path) {
        auto const col = 4 * position.first + 2;
        auto const line = 2 * position.second + 1;

        // Do not write over this.
        if (position == start_) {
            continue;
        }

        // Write path weighting into map.
        auto const index = std::to_string(graph_.at(position).first);
        tempMap[line][col] = index[0];
        tempMap[line][col + 1] = index.size() > 1 ? index[1] : ' ';
    }

    // Print out map.
    for (auto const& line : tempMap) {
        print(line);
    }
}

auto PathPlanner::writePathPlan2txt(std::string const& pathPlanPath) const -> void {
    auto pathPlan = std::ofstream(pathPlanPath);
    if (pathPlan.good() == false) {
        throw std::runtime_error("Could not open file.");
    }

    pathPlan << leastTurnsPath_->second;

    if (pathPlan.bad() == true) {
        throw std::runtime_error("I/O error while reading.");
    }
}

auto PathPlanner::getHeadingIndex(std::pair<int, int> a, std::pair<int, int> b) const -> int {
    auto westEastHeading = a.first - b.first;      // W = 1, E = -1
    auto northSouthHeading = b.second - a.second;  // S = 1, N = -1
    auto heading = northSouthHeading == 0 ? westEastHeading + 2 : northSouthHeading + 1;
    return heading;
}

auto PathPlanner::getAction(int a, int b) const -> std::string {
    switch (a - b) {
        case 0:
            return "F";
        case -2:
        case 2:
            return "LL";
        case -1:
        case 3:
            return "RF";
        case 1:
        case -3:
            return "LF";
        default:
            throw std::runtime_error("Invalid heading index.");
    }
}

}  // namespace mtrn4110
