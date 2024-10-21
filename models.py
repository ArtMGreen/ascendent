import cv2
from ultralytics import YOLO


model_field = YOLO('res\\models\\field_camera.pt')
labels_field = ['ball', 'base_g', 'base_r', 'buttons', 'cube', 'robot_g', 'robot_r']

model_robot = YOLO('res\\models\\robot_camera.pt')
labels_robot = ['ball', 'base_g', 'base_r', 'bin', 'button_b', 'button_g', 'button_p', 'button_r', 'cube', 'robot']

def image_to_objects(robot_image: cv2.typing.MatLike) ->dict[str, list[list[int, int, int, int]]] :
    pred = model_robot.predict(source=robot_image)
    objects = {}
    for box in pred[0].boxes:
        label = labels_robot[int(box.cls.item())]
        if label not in objects:
            objects[label] = []
        objects[label].append(box.xyxy.tolist()[0])
