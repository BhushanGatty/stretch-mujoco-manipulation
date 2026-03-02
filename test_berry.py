"""
Quick test script to verify berries settle correctly on ground
"""
import mujoco
import mujoco.viewer
import time
import numpy as np

MODEL_XML = "scene_with_stretch.xml"

print("Loading model...")
model = mujoco.MjModel.from_xml_path(MODEL_XML)
data = mujoco.MjData(model)

# Force initial positions (in case XML isn't setting them)
print("\n🔧 Setting initial berry positions manually...")
for i in range(model.nbody):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
    if name and name.startswith('berry'):
        # Find the freejoint for this body
        jnt_adr = model.body_jntadr[i]
        if jnt_adr >= 0:
            # Get qpos address for this joint
            qpos_adr = model.jnt_qposadr[jnt_adr]
            
            # Set position based on berry name
            if name == "berry1":
                data.qpos[qpos_adr:qpos_adr+3] = [-1.5, 1.0, 0.5]
            elif name == "berry2":
                data.qpos[qpos_adr:qpos_adr+3] = [0.0, 1.2, 0.5]
            elif name == "berry3":
                data.qpos[qpos_adr:qpos_adr+3] = [1.5, 1.0, 0.5]
            elif name == "berry4":
                data.qpos[qpos_adr:qpos_adr+3] = [-0.75, 0.8, 0.5]
            elif name == "berry5":
                data.qpos[qpos_adr:qpos_adr+3] = [0.75, 1.2, 0.5]
            
            # Set orientation to identity quaternion [w, x, y, z]
            data.qpos[qpos_adr+3:qpos_adr+7] = [1, 0, 0, 0]

# Forward kinematics to update positions
mujoco.mj_forward(model, data)

print("\nInitial berry positions (after manual setup):")
for i in range(model.nbody):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
    if name and name.startswith('berry'):
        pos = data.xpos[i]
        print(f"  {name}: x={pos[0]:.2f}, y={pos[1]:.2f}, z={pos[2]:.2f}")

print("\n⏳ Simulating physics for 5 seconds...")

with mujoco.viewer.launch_passive(model, data) as viewer:
    start_time = time.time()
    
    # Simulation loop
    step_count = 0
    while viewer.is_running() and (time.time() - start_time) < 5:
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)
        step_count += 1
    
    print(f"\n📍 Settled berry positions after {step_count} steps (~{step_count*0.005:.1f}s):")
    
    all_settled = True
    
    for i in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        if name and name.startswith('berry'):
            pos = data.xpos[i]
            
            # Get velocity for this body
            jnt_adr = model.body_jntadr[i]
            if jnt_adr >= 0:
                # Get velocity address
                qvel_adr = model.jnt_dofadr[jnt_adr]
                vel = data.qvel[qvel_adr:qvel_adr+3]
                speed = np.linalg.norm(vel)
            else:
                vel = np.array([0, 0, 0])
                speed = 0
            
            print(f"  {name}: pos=[{pos[0]:6.2f}, {pos[1]:6.2f}, {pos[2]:6.2f}], speed={speed:.4f} m/s")
            
            # Check status
            if pos[2] < 0.1:
                print(f"    ❌ CRITICAL: {name} fell through ground!")
                all_settled = False
            elif pos[2] < 0.25:
                print(f"    ⚠️  WARNING: {name} is below expected height (should be ~0.34m)")
                all_settled = False
            elif speed > 0.01:
                print(f"    ⚠️  {name} still moving (speed={speed:.4f} m/s)")
                all_settled = False
            else:
                print(f"    ✅ {name} settled correctly")
    
    if all_settled:
        print("\n✅ All berries settled successfully!")
    else:
        print("\n❌ Some berries have issues - see warnings above")
        print("\nPossible fixes:")
        print("1. Increase contact impedance in XML: solimp='0.95 0.99 0.0001'")
        print("2. Start berries higher: pos='... ... 0.8' instead of 0.5")
        print("3. Add explicit contact pairs between floor and berry geoms")
    
    print("\nKeeping viewer open for inspection...")
    print("Press Ctrl+C or close window to exit.")
    
    try:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\nShutting down...")

print("\n✓ Test complete")