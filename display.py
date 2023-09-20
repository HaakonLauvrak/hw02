from Map import Map_Obj
import cv2
import numpy as np


map = Map_Obj(1)
frames = []

for i in range(0, 17):
    pos = map.get_current_pos()
    move = [pos[0], pos[1]+1]
    map.move_current_pos(move)
    pil_image = map.show_map()  # assuming this returns a PIL Image
    frame = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)
    frames.append(frame)

# Set video properties
frame_height, frame_width = frames[0].shape[:2]
size = (frame_width, frame_height)
out = cv2.VideoWriter('video.avi', cv2.VideoWriter_fourcc(*'DIVX'), 1, size)

for i in range(len(frames)):
    out.write(frames[i])

out.release()