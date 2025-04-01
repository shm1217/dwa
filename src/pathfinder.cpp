#include "pathfinder.h"
#include <algorithm>
#include <queue>
#include <set>

PathNode::PathNode(double _x, double _y) : x(_x), y(_y), f(0), g(0), h(0) {}

bool PathNode::operator>(const PathNode &other) const { return f > other.f; }
bool PathNode::operator==(const PathNode &other) const { return x == other.x && y == other.y; }

std::vector<PathNode> FindPath(const std::vector<std::vector<int>> &graph, const PathNode &start, const PathNode &goal)
{
    const int dx[] = { -1, 0, 1, 0 };
    const int dy[] = { 0, 1, 0, -1 };

    std::priority_queue<PathNode, std::vector<PathNode>, std::greater<PathNode>> openList;
    std::map<std::pair<int, int>, std::pair<int, int>> cameFrom; // parentNode 역할
    std::vector<std::vector<bool>> closedList(graph.size(), std::vector<bool>(graph[0].size(), false));

    openList.push(start);

    while (!openList.empty())
    {
        PathNode current = openList.top();
        openList.pop();
        int current_x = static_cast<int>(current.x);
        int current_y = static_cast<int>(current.y);

        if (current_x == goal.x && current_y == goal.y)
        {
            std::vector<PathNode> path;
            std::pair<int, int> pos = { current_x, current_y };

            while (cameFrom.find(pos) != cameFrom.end())
            {
                path.push_back(PathNode(pos.first, pos.second));
                pos = cameFrom[pos];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }

        closedList[current_x][current_y] = true;

        for (int i = 0; i < 4; ++i)
        {
            int nx = current_x + dx[i];
            int ny = current_y + dy[i];

            if (nx >= 0 && nx < graph.size() && ny >= 0 && ny < graph[0].size() && graph[nx][ny] == 0 && !closedList[nx][ny])
            {
                PathNode neighbor(nx, ny);
                neighbor.g = current.g + 1;
                neighbor.h = abs(nx - goal.x) + abs(ny - goal.y);
                neighbor.f = neighbor.g + neighbor.h;

                cameFrom[{ nx, ny }] = { current_x, current_y };
                openList.push(neighbor);
            }
        }
    }
    return {};
}
std::vector<std::vector<int>> createGrid(double width, double height, const std::set<std::pair<int, int>> &obstacles)
{
    double cell_size = 0.1; // 셀 크기 (0.01m 단위)

    int grid_width = static_cast<int>(width / cell_size);
    int grid_height = static_cast<int>(height / cell_size);

    std::vector<std::vector<int>> grid(grid_width, std::vector<int>(grid_height, 0));
    const int dx[] = { -1, 0, 1, 0, -1, -1, 1, 1 };
    const int dy[] = { 0, 1, 0, -1, -1, 1, -1, 1 };

    for (const auto &obstacle : obstacles)
    {
        int x_idx = static_cast<int>(obstacle.first);
        int y_idx = static_cast<int>(obstacle.second);

        if (x_idx >= 0 && x_idx < grid_width && y_idx >= 0 && y_idx < grid_height)
        {
            double robot_radius = 0.2;                                     // 로봇 반지름 (m)
            int expand_cells = static_cast<int>(robot_radius / cell_size); // 장애물 확장 셀 개수

            for (int i = 0; i < 8; i++) // 8방향
            {
                for (int dist = 1; dist <= expand_cells; dist++) // 확장 거리
                {
                    int nx = x_idx + dx[i] * dist;
                    int ny = y_idx + dy[i] * dist;

                    if (nx >= 0 && nx < grid_width && ny >= 0 && ny < grid_height)
                    {
                        grid[nx][ny] = 1; // 8방향으로 장애물 확장
                    }
                }
            }
        }
    }

    return grid;
}