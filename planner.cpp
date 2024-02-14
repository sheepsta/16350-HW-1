#include "planner.h"
#include <cmath>
#include <cstdio>
#include <queue>
#include <vector>
#include <iostream>
#include <unordered_set>
#include <unordered_map>
#include <chrono>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y - 1) * XSIZE + (X - 1))

#if !defined(MAX)
#define MAX(A, B) ((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define MIN(A, B) ((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

struct map_node
{
    int x;
    int y;
    int origin_node_idx;
    int g;
    int h;
    int steps;
    int prev_x = -1;
    int prev_y = -1;
    map_node(int x, int y, int prev_x, int prev_y, int origin_node_idx, int g, int h, int steps)
        : x(x), y(y), prev_x(prev_x), prev_y(prev_y), origin_node_idx(origin_node_idx), g(g), h(h), steps(steps) {}
};

bool operator==(const map_node &lhs, const map_node &rhs)
{
    return lhs.x == rhs.x && lhs.y == rhs.y;
}

void printMapNode(const map_node &node)
{
    std::cout << "Map Node Contents:\n"
              << "Location: (" << node.x << ", " << node.y << ")\n"
              << "Previous Location: (" << node.prev_x << ", " << node.prev_y << ")\n"
              << "Origin Node Index: " << node.origin_node_idx << "\n"
              << "G (cost from start): " << node.g << "\n"
              << "H (estimated cost to goal): " << node.h << "\n"
              << "F (total cost): " << node.g + node.h << "\n"
              << "Steps from start: " << node.steps << std::endl;
}

struct map_node_hash
{
    size_t operator()(const map_node &node) const
    {
        return std::hash<int>()(node.x) ^ std::hash<int>()(node.y);
    }
};

using map_node_set = std::unordered_set<map_node, map_node_hash>;

struct CompareMapNode
{
    bool operator()(const map_node &a, const map_node &b) const
    {
        if ((a.g + a.h) == (b.g + b.h))                   // First, check if the total costs are equal
            return a.origin_node_idx < b.origin_node_idx; // Then, the node with the greater origin_node_idx is considered "less" for priority queue purposes
        else
            return (a.g + 34 * a.h) > (b.g + 34 * b.h); // Otherwise, prioritize based on the sum of g and h
    }
};

using MinHeap = std::priority_queue<map_node, std::vector<map_node>, CompareMapNode>;

int heuristic(int x1, int y1, int x2, int y2)
{
    return std::abs(x1 - x2) + std::abs(y1 - y2);
}

bool valid_pos(int x, int y, int x_size, int y_size, int *map, int collision_thresh)
{
    return (!(x < 1 || x > x_size || y < 1 || y > y_size) && map[GETMAPINDEX(x, y, x_size, y_size)] < collision_thresh);
}

void planner(
    int *map,
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int target_steps,
    int *target_traj,
    int targetposeX,
    int targetposeY,
    int curr_time,
    int *action_ptr)
{
    auto start = std::chrono::steady_clock::now();
    action_ptr[0] = robotposeX;
    action_ptr[1] = robotposeY;
    // Define movement directions (8-connected grid)
    int dX[NUMOFDIRS] = {-1, -1, -1, 0, 0, 1, 1, 1};
    int dY[NUMOFDIRS] = {-1, 0, 1, -1, 1, -1, 0, 1};

    MinHeap minHeap;
    map_node_set visitedNodes;

    for (int i = curr_time; i < target_steps; i++)
    {
        if (target_traj[i] == robotposeX && target_traj[i + target_steps] == robotposeY)
        {
            // printf("we got it!\n");
            action_ptr[0] = robotposeX;
            action_ptr[1] = robotposeY;
            return;
        }
    }

    // Insert all reachable targets in the trajectory as starting states
    for (int i = curr_time; i < target_steps; i++)
    {
        int targetX = target_traj[i];
        int targetY = target_traj[i + target_steps];
        if (valid_pos(targetX, targetY, x_size, y_size, map, collision_thresh) && map[(targetX - 1) * x_size + targetY - 1] < collision_thresh)
        {
            int h = heuristic(robotposeX, robotposeY, targetX, targetY);
            map_node startNode(targetX, targetY, -1, -1, i, 0, h, 0);
            minHeap.push(startNode);
            visitedNodes.insert(startNode);
        }
    }

    while (!minHeap.empty())
    {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);
        if (elapsed.count() >= 800)
        {
            break;
        }
        map_node current = minHeap.top();
        // printMapNode(current);
        minHeap.pop();
        // printf("%d\n", minHeap.size());

        for (int dir = 0; dir < NUMOFDIRS; dir++)
        {
            int newX = current.x + dX[dir];
            int newY = current.y + dY[dir];

            if (valid_pos(newX, newY, x_size, y_size, map, collision_thresh))
            {
                if (newX == robotposeX && newY == robotposeY && valid_pos(current.x, current.y, x_size, y_size, map, collision_thresh))
                {
                    printf("changing action pointer to %d and %d\n", current.x, current.y);
                    action_ptr[0] = current.x;
                    action_ptr[1] = current.y;

                    return;
                }
                map_node newNode(newX, newY, current.x, current.y, current.origin_node_idx, current.g + map[GETMAPINDEX(newX, newY, x_size, y_size)], heuristic(robotposeX, robotposeY, newX, newY), current.steps + 1);
                if (visitedNodes.find(newNode) == visitedNodes.end())
                {
                    minHeap.push(newNode);
                    visitedNodes.insert(newNode);
                }
            }
        }
    }

    // If no path is found, keep the robot stationary
}