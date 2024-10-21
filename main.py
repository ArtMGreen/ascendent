import numpy as np
from astar import astar_dirs
from client import SocketClient
from config import dir_init
from models import find
import navigation
from queue import Queue

team = None
goals = {}
socket = SocketClient('192.168.212.111', 9998)
instructions_queue = Queue()
grid = None
# current_goal = None
current_goal = 'cube'
cubes = 0
buttons = 0
balls = 0

def update_dir(instruction: tuple[str, int]):
    if instruction[0] == 'right':
        if dir_init == 'up': dir_init = 'right'
        elif dir_init == 'right': dir_init = 'down'
        elif dir_init == 'down': dir_init = 'left'
        elif dir_init == 'left': dir_init = 'up'
    if instruction[0] == 'left':
        if dir_init == 'up': dir_init = 'left'
        elif dir_init == 'left': dir_init = 'down'
        elif dir_init == 'down': dir_init = 'right'
        elif dir_init == 'right': dir_init = 'up'
    
def bytes_to_numpy(serialized_arr: str) -> np.array:
    sep = '|'.encode('utf-8')
    i_0 = serialized_arr.find(sep)
    i_1 = serialized_arr.find(sep, i_0 + 1)
    arr_dtype = serialized_arr[:i_0].decode('utf-8')
    arr_shape = tuple([int(a) for a in serialized_arr[i_0 + 1:i_1].decode('utf-8').split(',')])
    arr_str = serialized_arr[i_1 + 1:]
    arr = np.frombuffer(arr_str, dtype = arr_dtype).reshape(arr_shape)
    return arr

def send_from_queue() -> bool:
    if not instructions_queue.empty():
        instruction = instructions_queue.get(False)
        update_dir(instruction)
        socket.send(f'move {instruction[0]} {instruction[1]}')
        return True
    return False

def choose_preference() -> str:
    if buttons < 2:
        return 'buttons'
    if cubes < 2:
        return 'cube'
    if balls < 1:
        return 'ball'
    return 'base_r' if team == 'red' else 'base_g'

print('started main.py')
while True:
    image_robot, image_field, cmd = socket.receive().split('!')
    match cmd:
        case '': continue
        case 'start | green', 'start | red':
            team = cmd.split(' | ')[1]
            # image_field = bytes_to_numpy(image_field)
            # goals = navigation.extract_goals(image_field)
            # grid = match_template(make_grid(image_field))
            # robot = navigation.find_robot(team, goals)
            # base = navigation.find_base(team, goals)
            # if robot is not None and base is not None:
            #     print('init success!')
            socket.send('ok')
            print('socket get: start')
            print('socket send: ok')
        case 'what':
            print('socket get: what')
            if send_from_queue(): continue
            match current_goal:
                case None:
                    # goals = navigation.extract_goals(bytes_to_numpy(image_field))
                    # robot = navigation.find_robot(team, goals)
                    # current_goal = choose_preference()
                    # goal = navigation.choose_goal(goals, robot, 'green' if team == 'red' else 'red', current_goal)
                    # if robot is not None and goal is not None:
                    #     robot = (robot[1], robot[0])
                    #     goal = (goal[1], goal[0])
                    #     route = astar_dirs(grid, robot, goal, dir_init)
                    #     commands = navigation.get_commands(route, dir_init)
                    #     for command in commands:
                    #         instructions_queue.put(command)
                    #     send_from_queue()
                    pass
                case 'buttons':
                    socket.send('press')
                    buttons += 1
                    print('socket send: press')
                case 'cube':
                    socket.send('take')
                    cubes += 1
                    print('socket send: take')
                case 'ball':
                    socket.send('take')
                    balls += 1
                    print('socket send: take')
                case 'base_r', 'base_g':
                    socket.send('drop')
                    print('socket send: drop')
        case 'fail':
            print('socket get: fail')
            goals = navigation.extract_goals(bytes_to_numpy(image_field))
            robot = navigation.find_robot(team, goals)
            preference = choose_preference()
            goal = navigation.choose_goal(goals, robot, 'green' if team == 'red' else 'red', preference)
            if robot is not None and goal is not None:
                robot = (robot[1], robot[0])
                goal = (goal[1], goal[0])
                route = astar_dirs(grid, robot, goal, dir_init)
                commands = navigation.get_commands(route, dir_init)
                for command in commands:
                    instructions_queue.put(command)
                send_from_queue()
        case 'find':
            print('socket get: find')
            x, y, z = find(current_goal, bytes_to_numpy(image_robot))
            match current_goal:
                case 'base_r', 'base_g', 'buttons':
                    current_goal = None
                    socket.send(f'{x} {y} {z}')
                    print(f'socket send: {x} {y} {z}')
                case 'cube', 'ball':
                    current_goal = 'base_r' if team == 'red' else 'base_g'
                    socket.send(f'{x} {y} {z}')
                    print(f'socket send: {x} {y} {z}')
                    
