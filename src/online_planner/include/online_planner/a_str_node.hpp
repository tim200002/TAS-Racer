using namespace std;

#include "online_planner/a_star_efficient.hpp" // See header for copyright and usage information

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <online_planner/grid.hpp>
#include "rclcpp/rclcpp.hpp"

#define DEBUG_LISTS 0
#define DEBUG_LIST_LENGTHS_ONLY 0

class MapSearchNode
{
public:
    int x; // the (x,y) positions of the node
    int y;
    int desired_threshold_pixels = 15;
    // int min_trehshold_pixels = 10;

    unsigned size_x, size_y;
    double *grid;

    MapSearchNode()
    {
        x = y = 0;
        size_x = size_y = 0;
        grid = nullptr;
    }
    MapSearchNode(int px, int py, int size_x_, int size_y_, double *grid_)
    {
        x = px;
        y = py;
        size_x = size_x_;
        size_y = size_y_;
        grid = grid_;
    }

    float GoalDistanceEstimate(MapSearchNode &nodeGoal);
    bool IsGoal(MapSearchNode &nodeGoal);
    bool GetSuccessors(AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node);
    float GetCost(MapSearchNode &successor);
    bool IsSameState(MapSearchNode &rhs);

    inline bool checkMove(int x, int y, int thresh);

    void PrintNodeInfo();
    rclcpp::Logger logger = rclcpp::get_logger("node");
};

bool MapSearchNode::IsSameState(MapSearchNode &rhs)
{

    // same state in a maze search is simply when (x,y) are the same
    if ((x == rhs.x) &&
        (y == rhs.y))
    {
        return true;
    }
    else
    {
        return false;
    }
}

void MapSearchNode::PrintNodeInfo()
{
    char str[100];
    sprintf(str, "Node position : (%d,%d)\n", x, y);

    cout << str;
}

// Here's the heuristic function that estimates the distance from a Node
// to the Goal.

float MapSearchNode::GoalDistanceEstimate(MapSearchNode &nodeGoal)
{
    return abs(x - nodeGoal.x) + abs(y - nodeGoal.y);
}

bool MapSearchNode::IsGoal(MapSearchNode &nodeGoal)
{
    if (sqrt( pow(x-nodeGoal.x,2) + pow(y-nodeGoal.y,2)) <=desired_threshold_pixels){
        return true;
    }
    return false;
    if ((x == nodeGoal.x) &&
        (y == nodeGoal.y))
    {
        return true;
    }

    return false;
}

// This generates the successors to the given Node. It uses a helper function called
// AddSuccessor to give the successors to the AStar class. The A* specific initialisation
// is done for each node internally, so here you just set the state information that
// is specific to the application
bool MapSearchNode::GetSuccessors(AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node)
{

    int parent_x = -1;
    int parent_y = -1;

    if (parent_node)
    {
        parent_x = parent_node->x;
        parent_y = parent_node->y;
    }

    MapSearchNode NewNode;

    int no_of_succeses = 0;
    int tmep_threshold = desired_threshold_pixels;
    do
    {
        if (tmep_threshold != desired_threshold_pixels)
        {
            RCLCPP_INFO(logger, "mpde iteration wiht thresh %d", tmep_threshold);
        }

        if (checkMove(x - 1, y, tmep_threshold))
        {
            NewNode = MapSearchNode(x - 1, y, size_x, size_y, grid);
            astarsearch->AddSuccessor(NewNode);
            no_of_succeses++;
        }

        if (checkMove(x + 1, y, tmep_threshold))
        {
            NewNode = MapSearchNode(x + 1, y, size_x, size_y, grid);
            astarsearch->AddSuccessor(NewNode);
            no_of_succeses++;
        }

        if (checkMove(x, y - 1, tmep_threshold))
        {
            NewNode = MapSearchNode(x, y - 1, size_x, size_y, grid);
            astarsearch->AddSuccessor(NewNode);
            no_of_succeses++;
        }

        if (checkMove(x, y + 1, tmep_threshold))
        {
            NewNode = MapSearchNode(x, y + 1, size_x, size_y, grid);
            astarsearch->AddSuccessor(NewNode);
            no_of_succeses++;
        }

        if (checkMove(x - 1, y - 1, tmep_threshold))
        {
            NewNode = MapSearchNode(x - 1, y - 1, size_x, size_y, grid);
            astarsearch->AddSuccessor(NewNode);
            no_of_succeses++;
        }

        if (checkMove(x - 1, y + 1, tmep_threshold))
        {
            NewNode = MapSearchNode(x - 1, y + 1, size_x, size_y, grid);
            astarsearch->AddSuccessor(NewNode);
            no_of_succeses++;
        }

        if (checkMove(x + 1, y - 1, tmep_threshold))
        {
            NewNode = MapSearchNode(x + 1, y - 1, size_x, size_y, grid);
            astarsearch->AddSuccessor(NewNode);
            no_of_succeses++;
        }

        if (checkMove(x + 1, y + 1, tmep_threshold))
        {
            NewNode = MapSearchNode(x + 1, y + 1, size_x, size_y, grid);
            astarsearch->AddSuccessor(NewNode);
            no_of_succeses++;
        }
        //RCLCPP_INFO(logger, "No of succeses %d", no_of_succeses);
        tmep_threshold--;
    } while (no_of_succeses == 0);

    return true;
}

// given this node, what does it cost to move to successor. In the case
// of our map the answer is the map terrain value at this node since that is
// conceptually where we're moving

float MapSearchNode::GetCost(MapSearchNode &successor)
{
    return (float)sqrt(pow(x - successor.x, 2) + pow(y - successor.y, 2));
}

inline bool MapSearchNode::checkMove(int x, int y, int thresh)
{
    if (x >= size_x || y >= size_y)
    {
        return false;
    }
    return grid[y * size_x + x] > thresh;
}
