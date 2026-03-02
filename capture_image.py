import mujoco
import mujoco.viewer
import numpy as np
import cv2
import time

model = mujoco.MjModel.from_xml_path(
    "mujoco_menagerie/hello_robot_stretch/stretch.xml"
)
data = mujoco.MjData(model)

camera_name = "camera_rgb"
renderer = mujoco.Renderer(model, height=480, width=640)

with mujoco.viewer.launch_passive(model, data) as viewer:
    for _ in range(100):
        mujoco.mj_step(model, data)
        viewer.sync()

    renderer.update_scene(data, camera=camera_name)
    img = renderer.render()

    img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    cv2.imwrite("strawberry_view.png", img_bgr)
    print("✅ Image saved as strawberry_view.png")

    time.sleep(2)  # keep viewer alive a bit

print("Done cleanly")
