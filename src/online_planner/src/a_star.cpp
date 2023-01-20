#include "online_planner/a_star.hpp"
#include <algorithm>
#include <math.h>

using namespace std::placeholders;


AStar::Node::Node(GridPoint coordinates_, Node *parent_):parent(parent_), coordinates(coordinates_)
{
    G = H = 0;
}

AStar::uint AStar::Node::getScore()
{
    return G + H;
}

AStar::Generator::Generator()
{
    setDiagonalMovement(true);
    setHeuristic(&Heuristic::euclidean);

    direction = {GridPoint(0, 1), GridPoint(0, -1), GridPoint(1, 0), GridPoint(-1, 0),
        GridPoint(1, 1), GridPoint(1, -1), GridPoint(-1, 1), GridPoint(-1, -1)};
}

void AStar::Generator::setWorldSize(unsigned int world_size_x_, unsigned int world_size_y_)
{
    world_size_x = world_size_x_;
    world_size_y = world_size_y_;
}

void AStar::Generator::setDiagonalMovement(bool enable_)
{
    directions = (enable_ ? 8 : 4);
}

void AStar::Generator::setHeuristic(HeuristicFunction heuristic_)
{
    heuristic = std::bind(heuristic_, _1, _2);
}

// classified grid but be of form that valid points have value 0 and invalid points have value 1
void AStar::Generator::loadGrid(Grid<unsigned char> grid)
{   
    clearCollisions();
    setWorldSize(grid.size_x, grid.size_y);

    for(int x = 0; x < grid.size_x; ++x){
        for(int y = 0; y < grid.size_y; ++y){
            if(grid.get_value(x, y) == 1){
                addCollision(GridPoint(x, y));
            }
        }
    }
}

void AStar::Generator::addCollision(GridPoint coordinates_)
{
    walls.push_back(coordinates_);
}

void AStar::Generator::removeCollision(GridPoint coordinates_)
{
    auto it = std::find(walls.begin(), walls.end(), coordinates_);
    if (it != walls.end()) {
        walls.erase(it);
    }
}

void AStar::Generator::clearCollisions()
{
    walls.clear();
}

AStar::CoordinateList AStar::Generator::findPath(GridPoint source_, GridPoint target_)
{
    Node *current = nullptr;
    NodeSet openSet, closedSet;
    openSet.reserve(100);
    closedSet.reserve(100);
    openSet.push_back(new Node(source_));

    while (!openSet.empty()) {
        RCLCPP_INFO(logger, "while");
        auto current_it = openSet.begin();
        current = *current_it;

        for (auto it = openSet.begin(); it != openSet.end(); it++) {
            auto node = *it;
            if (node->getScore() <= current->getScore()) {
                current = node;
                current_it = it;
            }
        }

        if (current->coordinates == target_) {
            break;
        }

        closedSet.push_back(current);
        openSet.erase(current_it);

        for (uint i = 0; i < directions; ++i) {
            GridPoint newCoordinates(current->coordinates + direction[i]);
            if (detectCollision(newCoordinates) ||
                findNodeOnList(closedSet, newCoordinates)) {
                continue;
            }

            uint totalCost = current->G + ((i < 4) ? 10 : 14);

            Node *successor = findNodeOnList(openSet, newCoordinates);
            if (successor == nullptr) {
                successor = new Node(newCoordinates, current);
                successor->G = totalCost;
                successor->H = heuristic(successor->coordinates, target_);
                openSet.push_back(successor);
            }
            else if (totalCost < successor->G) {
                successor->parent = current;
                successor->G = totalCost;
            }
        }
    }

    CoordinateList path;
    while (current != nullptr) {
        path.push_back(current->coordinates);
        current = current->parent;
    }

    releaseNodes(openSet);
    releaseNodes(closedSet);

    return path;
}

AStar::Node* AStar::Generator::findNodeOnList(NodeSet& nodes_, GridPoint coordinates_)
{
    for (auto node : nodes_) {
        if (node->coordinates == coordinates_) {
            return node;
        }
    }
    return nullptr;
}

void AStar::Generator::releaseNodes(NodeSet& nodes_)
{
    for (auto it = nodes_.begin(); it != nodes_.end();) {
        delete *it;
        it = nodes_.erase(it);
    }
}

bool AStar::Generator::detectCollision(GridPoint coordinates_)
{
    if (coordinates_.x < 0 || coordinates_.x >= world_size_x ||
        coordinates_.y < 0 || coordinates_.y >= world_size_y ||
        std::find(walls.begin(), walls.end(), coordinates_) != walls.end()) {
        return true;
    }
    return false;
}


AStar::uint AStar::Heuristic::euclidean(GridPoint source_, GridPoint target_)
{
    return static_cast<uint>(10 * sqrt(pow(source_.x - target_.x, 2) + pow(source_.y - target_.y, 2)));
}

