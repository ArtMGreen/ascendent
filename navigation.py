import math
from typing import Dict, List, Optional, Tuple
import cv2
from ultralytics import YOLO
from astar import astar_dirs
from config import dir_init, team
from image_transforms import make_grid, match_template, transform_coordinates, unfish
import models


def get_angle(dir_1: str, dir_2: str) -> int:
    match dir_1:
        case "left":
            match dir_2:
                case "left": return 0                  
                case "right": return 180
                case "up": return 90
                case "down": return -90
        case "right":
            match dir_2:
                case "left": return 180                   
                case "right": return 0
                case "up": return -90
                case "down": return 90
        case "up":
            match dir_2:
                case "left": return -90                   
                case "right": return 90
                case "up": return 0
                case "down": return 180
        case "down":
            match dir_2:
                case "left": return 90                
                case "right": return -90
                case "up": return 180
                case "down": return 0
    

def add_turn(commands: list, dir_1, dir_2) -> None:
    angle = get_angle(dir_1, dir_2)
    if angle > 0:
        commands.append(('right', angle))
    elif angle < 0:
        commands.append(('left', -angle))


def get_commands(
    route,
    dir_init: tuple[int, int]
) -> list[str, int]:
    if route is None:
        return None
    commands = []
    add_turn(commands, dir_init, route[0][0])
    for i in range(len(route) - 1):
        dir_1 = route[i][0]
        dir_2 = route[i+1][0]
        route[i][0] = 'forward'
        route[i][1] /= 280
        commands.append(route[i])
        add_turn(commands, dir_1, dir_2)
    route[-1][0] = 'forward'    
    commands.append(route[-1])
    return commands


def euclidean_distance(p1: Tuple[int, int], p2: Tuple[int, int]) -> float:
    return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)


def choose_goal(
        goals: Dict[str, List[Tuple[int, int]]],
        me: Tuple[int, int],
        other_robot: Tuple[int, int],
        preferred_goal: Optional[str] = None
) -> Tuple[int, int]:
    best_goal = None
    min_distance = float('inf')

    if preferred_goal:
        for pos in goals[preferred_goal]:
            dist = euclidean_distance(me, pos)
            if dist < min_distance:
                min_distance = dist
                best_goal = pos

        return best_goal

    for goal, positions in goals.items():
        if goal in ['base_r', 'base_g', 'robot_r']:
            continue

        if goal == 'buttons':
            for pos in positions:
                dist = euclidean_distance(me, pos)
                if dist < min_distance:
                    min_distance = dist
                    best_goal = (goal, pos)

        elif goal == 'cube':
            for pos in positions:
                dist_to_me = euclidean_distance(me, pos)
                dist_to_other_robot = euclidean_distance(other_robot, pos)

                if dist_to_me < min_distance < dist_to_other_robot:
                    min_distance = dist_to_me
                    best_goal = pos

    return best_goal


def find_robot(team: str, goals: dict[str, list[tuple[int, int]]]) -> tuple[int, int] | None:
    if team == 'red' and 'robot_r' in goals:
        robot = goals['robot_r'][0]
    elif team == 'green' and 'robot_g' in goals:
        robot = goals['robot_g'][0]
    else: return None
    return robot
    

def find_base(team: str, goals: dict[str, list[tuple[int, int]]]) -> tuple[int, int] | None:
    if team == 'red' and 'base_r' in goals:
        robot = goals['base_r'][0]
    elif team == 'green' and 'base_g' in goals:
        robot = goals['base_g'][0]
    else: return None
    return robot

def extract_goals(image: cv2.typing.MatLike) -> dict[str, list[tuple[int, int]]]:
    pred = models.model_field.predict(source=image, conf=0.15)[0]
    goals = {}
    for box in pred.boxes:
        x1, y1, x2, y2 = box.xyxy.tolist()[0]
        point_x = (x2 + x1) / 2
        point_y = (y2 + y1) / 2
        point = transform_coordinates(point_x, point_y)
        goal = models.labels_field[int(box.cls.item())]
        if goal not in goals:
            goals[goal] = []
        goals[models.labels_field[int(box.cls.item())]].append(point)
    return goals

# def get_instructions(image: cv2.typing.MatLike, robot):
    # for _ in range(10):
    #     goals = extract_goals(image)
    #     goal = choose_goal(goals, robot, 'green' if team == 'red' else 'red', 'buttons')
    #     if robot is not None and goal is not None:
    #         robot = (robot[1], robot[0])
    #         goal = (goal[1], goal[0])
    #         route = astar_dirs(grid, robot, goal, dir_init)
    #         commands = get_commands(route, dir_init)
    #         if not commands:
    #             commands = []
    #         return commands


if __name__ == "__main__":
    image = cv2.imread('res/images/field/left0.png')

    grid = match_template(make_grid(image))
    print(get_instructions(image))
