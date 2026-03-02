import mujoco
import mujoco.viewer
import time
import cv2
import numpy as np

model = mujoco.MjModel.from_xml_path("scene_with_stretch.xml")

data = mujoco.MjData(model)

# Actuator indices
FORWARD = 0
TURN = 1
LIFT = 2
ARM_EXTEND = 3
GRIP = 5

camera_name = "main_cam"

renderer = mujoco.Renderer(model, height=480, width=640)

berry_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "strawberry_1")

with mujoco.viewer.launch_passive(model, data) as viewer:

    print("🚜 Moving toward strawberries...")
    for _ in range(200):
        data.ctrl[:] = 0
        data.ctrl[FORWARD] = 0.5
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)

    print("🤖 Moving arm...")
    for _ in range(200):
        data.ctrl[:] = 0
        data.ctrl[LIFT] = 0.5
        data.ctrl[ARM_EXTEND] = 0.5
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)

    print("✋ Closing gripper...")
    for _ in range(100):
        data.ctrl[GRIP] = 1.0
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)

    print("🍓 Picking strawberry...")
    model.body_pos[berry_id] = [0, 0, -10]

    for _ in range(100):
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)

    print("📸 Capturing image...")
    renderer.update_scene(data, camera="main_cam")
    img = renderer.render()

    print("Mean pixel value:", img.mean())

    img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    cv2.imwrite("mission_result.png", img_bgr)

    print("✅ Mission complete. Image saved as mission_result.png")
    print("Press Q in the MuJoCo window to quit.")

    # Keep window alive so you can press ] to change camera
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)


