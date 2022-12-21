#include <costmap_2d/costmap_2d_ros.h>

using NodeSet = std::vector<Node *>;

struct Point
{
    int x, y;

    bool operator==(const Vec2i &coordinates_)
    {
        return (x == coordinates_.x && y == coordinates_.y);
    }

    Point operator+(const Point &left_, const Point &right_)
    {
        return {left_.x + right_.x, left_.y + right_.y};
    }
};

struct Node
{
    unsigned int G, H;

    Point coordinate;
    Node *parent;

    Node(Point coordinate, Node *parent) : (this.coordinate = coordinate, this.parent = parent, this.G = 0, this.H = 0);

    unsigned int getScore()
    {
        return G + H;
    }
};

// return path of costmap Grid coordinates
std::vector<Point> findPath(Point start, Point goal, Costmap2D *const costmap)
{
    Node *current = nullptr;
    NodeSet openSet, closedSet;
    openSet.reserve(100);
    closedSet.reserve(100);

    // initialize first node
    openSet.push_back(new Node(start, nullptr));

    while (!openSet.empty())
    {
        auto current_it = openSet.begin();
        current = *current_it;

        for (auto it = openSet.begin(); it != openSet.end(); it++)
        {
            auto node = *it;
            if (node->getScore() <= current->getScore())
            {
                current = node;
                current_it = it;
            }
        }

        if (current->coordinates == target_)
        {
            break;
        }

        closedSet.push_back(current);
        openSet.erase(current_it);

        for (uint i = 0; i < directions; ++i)
        {
            Point newCoordinates(current->coordinates + direction[i]);
            if (detectCollision(newCoordinates) ||
                findNodeOnList(closedSet, newCoordinates))
            {
                continue;
            }

            uint totalCost = current->G + ((i < 4) ? 10 : 14);

            Node *successor = findNodeOnList(openSet, newCoordinates);
            if (successor == nullptr)
            {
                successor = new Node(newCoordinates, current);
                successor->G = totalCost;
                successor->H = heuristic(successor->coordinates, target_);
                openSet.push_back(successor);
            }
            else if (totalCost < successor->G)
            {
                successor->parent = current;
                successor->G = totalCost;
            }
        }
    }
}