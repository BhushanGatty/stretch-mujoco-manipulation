import mujoco
import mujoco.viewer
import time

model = mujoco.MjModel.from_xml_path("ridge_plantation.xml")
data = mujoco.MjData(model)

# control indices
x_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "x")
y_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "y")

with mujoco.viewer.launch_passive(model, data) as viewer:
    for _ in range(300):
        data.ctrl[x_id] = 0.5
        data.ctrl[y_id] = 0.0
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)

    for _ in range(300):
        data.ctrl[x_id] = 0.0
        data.ctrl[y_id] = 0.5
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)
