import cv2
from ultralytics import YOLO


model_field = YOLO('res\\models\\field_camera.pt')
labels_field = ['ball', 'base_g', 'base_r', 'buttons', 'cube', 'robot_g', 'robot_r']

model_robot = YOLO('res\\models\\robot_camera.pt')
labels_robot = ['ball', 'base_g', 'base_r', 'bin', 'button_b', 'button_g', 'button_p', 'button_r', 'cube', 'robot']

focal_length = 470
cx = 295.79  # Optical center x
cy = 242.29
sizes = {'cube': 4.5, 'ball': 5, 'buttons': 100, 'base_r': 200, 'base_g': 200}

# object_width = ...         # choose above
# object_width_pixels = ...  # find bounding box width
# distance = (object_width * focal_length) / object_width_pixels

def get_coords(x, y, width_real, width_pix) -> tuple[float, float, float]:
    Z = (width_real * focal_length) / width_pix
    X = (x - cx) * Z / focal_length
    Y = (y - cy) * Z / focal_length
    return (X, Y, Z)

def find(goal: str, image: cv2.typing.MatLike) -> tuple[int, int, int] | None:
    pred = model_robot.predict(source=image)
    for box in pred[0].boxes:
        label = labels_robot[int(box.cls.item())]
        if label == goal:
            xywh = box.xywh.tolist()
            break
        elif (goal == 'base_r' or goal == 'base_g') and label == 'bin':
            xywh = box.xywh.tolist()
            break
        elif goal == 'buttons' and (label == 'button_b' or label == 'button_g'):
            xywh = box.xywh.tolist()
            break
    else: return None
    if xywh is None: return None
    object_width = sizes[goal]   
    return get_coords(xywh[0], xywh[1], object_width, xywh[2])
    
            