import mujoco
import mujoco.viewer
import time

model = mujoco.MjModel.from_xml_path(
    "mujoco_menagerie/hello_robot_stretch/stretch.xml"
)
data = mujoco.MjData(model)

# Actuator indices from your output
LIFT = 2
ARM_EXTEND = 3
GRIP = 5

# Get strawberry body id
berry_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "strawberry_1")

with mujoco.viewer.launch_passive(model, data) as viewer:
    print("➡️ Moving arm up and forward...")

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

    # keep viewer alive
    for _ in range(300):
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)

print("Finished cleanly")

