import mujoco

model = mujoco.MjModel.from_xml_path(
    "mujoco_menagerie/hello_robot_stretch/stretch.xml"
)

print("ACTUATORS:")
for i in range(model.nu):
    print(i, mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i))
