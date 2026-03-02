import mujoco

model = mujoco.MjModel.from_xml_path("scene_with_stretch.xml")

print("Cameras in model:")
for i in range(model.ncam):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_CAMERA, i)
    print(i, name)
