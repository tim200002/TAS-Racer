#include <vector>
#include <costmap_2d/costmap_2d_ros.h>

namespace AStar
{
    using NodeSet = std::vector<Node *>;
    using Path = std::vector<Point>

        struct Point
    {
        int x, y;

        bool operator==(const Vec2i &coordinatess_)
        {
            return (x == coordinatess_.x && y == coordinatess_.y);
        }

        Point operator+(const Point &left_, const Point &right_)
        {
            return {left_.x + right_.x, left_.y + right_.y};
        }
    };

    struct Node
    {
        unsigned int G, H;

        Point coordinates;
        Node *parent;

        Node(Point coordinates, Node *parent) : (this.coordinates = coordinates, this.parent = parent, this.G = 0, this.H = 0);

        unsigned int getScore()
        {
            return G + H;
        }
    };

    class AStar
    {
    public:
        Path findPath(Point start, Point goal, Costmap2D *const costmap);

    private:
        // uitlity functions
        bool detectCollision(const Point &coordinates, Costmap2D *const costmap)
        {
            if (coordinates.x < 0 || coordinates.x >= costmap->getSizeInCellsX())
                return true;
            if (coordinates.y < 0 || coordinates.y >= costmap->getSizeInCellsY())
                return true;
            if (costmap->getCost(coordinates.x, coordinates.y) > lethalThreshold)
                return true;

            return false;
        }

        Node *findNodeInSet(const NodeSet &nodes_, Point coordinates)
        {
            for (const auto &node : nodes_)
            {
                if (node->coordinates == coordinates)
                {
                    return node;
                }
            }
            return nullptr;
        }

        double heuristic(Point a, Point b)
        {
            sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2));
        }

        unsigned int lethalThreshold = 20;
        std::array<Point, 8> directions = {Point(0, 1), Point(1, 0), Point(0, -1), Point(-1, 0), Point(-1, -1), Point(1, 1), Point(-1, 1), Point(1, -1)};
    }
} // namespace AStar
