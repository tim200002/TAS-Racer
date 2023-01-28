
# pragma once

#include <vector>
#include <functional>
#include <set>
#include "online_planner/point.hpp"
#include "online_planner/grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace AStar
{
    
    using uint = unsigned int;
    using HeuristicFunction = std::function<uint(GridPoint, GridPoint)>;
    using CoordinateList = std::vector<GridPoint>;

    struct Node
    {
        uint G, H;
        GridPoint coordinates;
        Node *parent;

        Node(GridPoint coord_, Node *parent_ = nullptr);
        uint getScore();
    };

    using NodeSet = std::vector<Node*>;

    class Generator
    {
        bool detectCollision(GridPoint coordinates_);
        Node* findNodeOnList(NodeSet& nodes_, GridPoint coordinates_);
        void releaseNodes(NodeSet& nodes_);

    public:
        Generator();
        void loadGrid(Grid<unsigned char> grid);
        void setDiagonalMovement(bool enable_);
        void setHeuristic(HeuristicFunction heuristic_);
        CoordinateList findPath(GridPoint source_, GridPoint target_);
        

    private:
        void setWorldSize(unsigned int world_size_x, unsigned int world_size_y);
        void addCollision(GridPoint coordinates_);
        void removeCollision(GridPoint coordinates_);
        void clearCollisions();
        HeuristicFunction heuristic;
        CoordinateList direction, walls;
        unsigned int world_size_x;
        unsigned int world_size_y;
        uint directions;
        rclcpp::Logger logger = rclcpp::get_logger("A_star");

    };

    class Heuristic
    {

    public:
        static uint euclidean(GridPoint source_, GridPoint target_);
    };
}