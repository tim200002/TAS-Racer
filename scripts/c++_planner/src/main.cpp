#include <iostream>
#include <istream>
#include <fstream>
#include <cmath>

#include <xtensor/xarray.hpp>
#include <xtensor/xio.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xnpy.hpp>

typedef std::tuple<int8_t, int8_t> Point;

std::string printPoint(const Point point)
{
    return "(" + std::to_string(+std::get<0>(point)) + "," + std::to_string(+std::get<1>(point)) + ")";
}

std::vector<Point> a_star(Point start, Point goal, xt::xarray<int8_t> binaryOccupancyMap)
{
    start = {std::get<1>(start), std::get<0>(start)};
    goal = {std::get<1>(goal), std::get<0>(goal)};
}

double heuristic(Point a, Point b)
{
    sqrt(pow(std::get<0>(b) - std::get<0>(a), 2) + pow(std::get<1>(b) - std::get<1>(a), 2));
}

std::vector<Point> backSolvePath(const Point end, const std::vector<Point> came_from)
{
    std::vector<Point> path;
    Point current = end;

    while ()
}

int main(int argc, char *argv[])
{
    auto data = xt::load_npy<int8_t>("/home/parallels/Desktop/Parallels_Shared/Home/ros2_tas_project/scripts/c++_planner/src/occupancy_grid.npy");

    Point start = {1, -1};
    Point goal = {1, -1};

    a_star(start, goal, data);

    std::cout
        << data << std::endl;

    return 0;
}