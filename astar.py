import heapq
import cv2
from config import dir_init

dirs = {"right": (0, 1), "left": (0, -1), "down": (1, 0), "up": (-1, 0)}


def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def direction_change_penalty(prev_direction, new_direction):
    return 0 if prev_direction == new_direction else 15


def astar_pixels_DEBUG(
    maze: cv2.typing.MatLike,
    start: tuple[int, int],
    end: tuple[int, int],
    obstacle_threshold: int = 255,
) -> list[tuple[int]] | None:
    if (
        maze[start[0], start[1]] == obstacle_threshold
        or maze[end[0], end[1]] == obstacle_threshold
    ):
        return None
    open_list = []
    closed_list = set()
    g_costs = {start: 0}
    f_costs = {start: heuristic(start, end)}
    heapq.heappush(open_list, (f_costs[start], start, dir_init))
    came_from = {}
    direction_from = {}
    while open_list:
        _, current, prev_direction = heapq.heappop(open_list)
        if current == end:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]
        closed_list.add(current)
        for direction in dirs:
            neighbor = (current[0] + direction[0], current[1] + direction[1])
            if 0 <= neighbor[0] < maze.shape[0] and 0 <= neighbor[1] < maze.shape[1]:
                if (
                    maze[neighbor[0], neighbor[1]] == obstacle_threshold
                    or neighbor in closed_list
                ):
                    continue
                tentative_g_cost = g_costs[current] + 1
                penalty = direction_change_penalty(prev_direction, direction)
                tentative_g_cost += penalty
                if neighbor not in g_costs or tentative_g_cost < g_costs[neighbor]:
                    g_costs[neighbor] = tentative_g_cost
                    f_costs[neighbor] = tentative_g_cost + heuristic(neighbor, end)
                    came_from[neighbor] = current
                    direction_from[neighbor] = direction
                    heapq.heappush(open_list, (f_costs[neighbor], neighbor, direction))
    return None


def astar_dirs(
    maze: cv2.typing.MatLike,
    start: tuple[int, int],
    end: tuple[int, int],
    dir_init: str,
    obstacle_threshold: int = 255
) -> list[tuple[int]] | None:
    if (maze[start[0], start[1]] == obstacle_threshold or maze[end[0], end[1]] == obstacle_threshold):
        print('invalid start or end point')
        return None
    open_list = []
    closed_list = set()
    g_costs = {start: 0}
    f_costs = {start: heuristic(start, end)}
    heapq.heappush(open_list, (f_costs[start], start, dir_init))
    came_from = {}
    direction_from = {}
    while open_list:
        _, current, prev_direction = heapq.heappop(open_list)
        if current == end:
            path = []
            while current in direction_from:
                if path and path[-1][0] == direction_from[current]:
                    path[-1][1] += 1
                else:
                    path.append([direction_from[current], 1])
                current = came_from[current]
            return path[::-1]
        closed_list.add(current)
        for direction in dirs:
            neighbor = (current[0] + dirs[direction][0], current[1] + dirs[direction][1])
            if 0 <= neighbor[0] < maze.shape[0] and 0 <= neighbor[1] < maze.shape[1]:
                if (
                    maze[neighbor[0], neighbor[1]] == obstacle_threshold
                    or neighbor in closed_list
                ):
                    continue
                tentative_g_cost = g_costs[current] + 1
                penalty = direction_change_penalty(prev_direction,direction)
                tentative_g_cost += penalty
                if neighbor not in g_costs or tentative_g_cost < g_costs[neighbor]:
                    g_costs[neighbor] = tentative_g_cost
                    f_costs[neighbor] = tentative_g_cost + heuristic(neighbor, end)
                    came_from[neighbor] = current
                    direction_from[neighbor] = direction
                    heapq.heappush(open_list, (f_costs[neighbor], neighbor, direction))
    return None
