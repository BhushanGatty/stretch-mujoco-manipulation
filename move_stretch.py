import mujoco
import mujoco.viewer
import time

model = mujoco.MjModel.from_xml_path(
    "mujoco_menagerie/hello_robot_stretch/stretch.xml"
)
data = mujoco.MjData(model)

# print actuator names
for i in range(model.nu):
    print(i, mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i))

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # try small forward motion
        data.ctrl[:] = 0
        data.ctrl[0] = 0.5   # left wheel
        data.ctrl[1] = 0.5   # right wheel

        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)
