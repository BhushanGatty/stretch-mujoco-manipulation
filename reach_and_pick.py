import mujoco
import mujoco.viewer
import numpy as np
import time
import imageio
from pathlib import Path

MODEL_XML = "scene_with_stretch.xml"

# Actuator indices (Stretch 2 specifications)
ACT = {
    "forward": 0,
    "turn": 1,
    "lift": 2,
    "arm_extend": 3,
    "wrist_yaw": 4,
    "grip": 5,
    "head_pan": 6,
    "head_tilt": 7,
}


class StretchBerryPicker:
    """Strawberry picking controller with physics-based magnet pick."""
    
    def __init__(self, model_path):
        print(f"Loading model: {model_path}")
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        
        # Find all berries and their magnet constraints
        self.berries = {}
        for i in range(self.model.nbody):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, i)
            if name and name.startswith('berry'):
                magnet_name = f"magnet_{name}"
                magnet_id = None
                
                # Find corresponding magnet constraint
                for j in range(self.model.neq):
                    eq_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_EQUALITY, j)
                    if eq_name == magnet_name:
                        magnet_id = j
                        break
                
                self.berries[name] = {
                    'body_id': i,
                    'magnet_id': magnet_id
                }
                print(f"Found {name}: body_id={i}, magnet_id={magnet_id}")
        
        # Find gripper body
        self.gripper_body_id = self.model.body("link_gripper_finger_right").id
        
        print(f"Initialized with {len(self.berries)} berries")
    
    def settle_physics(self, viewer, steps=500):
        """Let berries fall and settle on ground."""
        print("⏳ Letting berries settle on ground...")
        for i in range(steps):
            mujoco.mj_step(self.model, self.data)
            if viewer and i % 10 == 0:
                viewer.sync()
            time.sleep(0.002)
        
        # Report settled positions
        print("\n📍 Settled berry positions:")
        for berry_name in sorted(self.berries.keys()):
            pos = self.get_berry_position(berry_name)
            print(f"  {berry_name}: x={pos[0]:.2f}, y={pos[1]:.2f}, z={pos[2]:.2f}")
        print()
    
    def get_body_position(self, body_id):
        """Get world position of a body."""
        return self.data.xpos[body_id].copy()
    
    def get_berry_position(self, berry_name):
        """Get position of a specific berry."""
        return self.get_body_position(self.berries[berry_name]['body_id'])
    
    def get_gripper_position(self):
        """Get gripper position."""
        return self.get_body_position(self.gripper_body_id)
    
    def step(self, viewer, n_steps=100):
        """Advance simulation by n steps."""
        for _ in range(n_steps):
            mujoco.mj_step(self.model, self.data)
            if viewer:
                viewer.sync()
            time.sleep(0.01)
    
    def navigate_to(self, target_x, target_y, viewer, tolerance=0.15):
        """Navigate base to target position."""
        print(f"🚜 Navigating to ({target_x:.2f}, {target_y:.2f})...")
        
        for iteration in range(500):
            # Get current base position
            current_x = self.data.qpos[0]
            current_y = self.data.qpos[1]
            current_theta = self.data.qpos[2]
            
            # Calculate error
            dx = target_x - current_x
            dy = target_y - current_y
            distance = np.sqrt(dx**2 + dy**2)
            
            if distance < tolerance:
                print(f"  ✓ Reached position in {iteration} steps (distance: {distance:.3f}m)")
                self.data.ctrl[ACT["forward"]] = 0
                self.data.ctrl[ACT["turn"]] = 0
                break
            
            # Calculate desired heading
            target_theta = np.arctan2(dy, dx)
            theta_error = target_theta - current_theta
            
            # Normalize to [-pi, pi]
            theta_error = np.arctan2(np.sin(theta_error), np.cos(theta_error))
            
            # Control: rotate first, then move
            if abs(theta_error) > 0.1:
                self.data.ctrl[ACT["turn"]] = np.clip(theta_error * 1.5, -0.5, 0.5)
                self.data.ctrl[ACT["forward"]] = 0
            else:
                self.data.ctrl[ACT["turn"]] = 0
                self.data.ctrl[ACT["forward"]] = np.clip(distance * 0.8, 0, 0.5)
            
            mujoco.mj_step(self.model, self.data)
            if viewer:
                viewer.sync()
            time.sleep(0.01)
        
        self.data.ctrl[ACT["forward"]] = 0
        self.data.ctrl[ACT["turn"]] = 0
    
    def align_arm_to_berry(self, berry_position, viewer):
        """Align arm to reach berry position."""
        print(f"🦾 Aligning arm to berry at [{berry_position[0]:.2f}, {berry_position[1]:.2f}, {berry_position[2]:.2f}]...")
        
        # Point camera down to observe
        self.data.ctrl[ACT["head_pan"]] = 0
        self.data.ctrl[ACT["head_tilt"]] = -0.7
        self.step(viewer, 50)
        
        # Get base position
        base_x = self.data.qpos[0]
        base_y = self.data.qpos[1]
        
        # Calculate required lift and extension
        target_height = berry_position[2]
        target_radius = np.sqrt(
            (berry_position[0] - base_x)**2 + 
            (berry_position[1] - base_y)**2
        )
        
        print(f"  Target height: {target_height:.2f}m, radius: {target_radius:.2f}m")
        
        # Move arm in stages
        for iteration in range(400):
            gripper_pos = self.get_gripper_position()
            
            # Height control
            height_error = target_height - gripper_pos[2]
            self.data.ctrl[ACT["lift"]] = np.clip(height_error * 3.0, -0.05, 0.05)
            
            # Extension control
            current_radius = np.sqrt(
                (gripper_pos[0] - base_x)**2 + 
                (gripper_pos[1] - base_y)**2
            )
            radius_error = target_radius - current_radius
            self.data.ctrl[ACT["arm_extend"]] = np.clip(radius_error * 2.0, -0.5, 0.5)
            
            mujoco.mj_step(self.model, self.data)
            if viewer:
                viewer.sync()
            time.sleep(0.01)
            
            # Check convergence
            distance_to_target = np.linalg.norm(gripper_pos - berry_position)
            if distance_to_target < 0.08:
                print(f"  ✓ Aligned in {iteration} steps (distance: {distance_to_target:.3f}m)")
                break
        
        self.data.ctrl[ACT["lift"]] = 0
        self.data.ctrl[ACT["arm_extend"]] = 0
    
    def magnet_pick(self, berry_name, viewer, threshold=0.08):
        """Activate magnet to pick berry."""
        berry_info = self.berries[berry_name]
        berry_pos = self.get_berry_position(berry_name)
        gripper_pos = self.get_gripper_position()
        
        distance = np.linalg.norm(gripper_pos - berry_pos)
        
        print(f"🧲 Attempting magnet pick...")
        print(f"  Berry pos: [{berry_pos[0]:.2f}, {berry_pos[1]:.2f}, {berry_pos[2]:.2f}]")
        print(f"  Gripper pos: [{gripper_pos[0]:.2f}, {gripper_pos[1]:.2f}, {gripper_pos[2]:.2f}]")
        print(f"  Distance: {distance:.3f}m (threshold: {threshold}m)")
        
        if distance < threshold:
            if berry_info['magnet_id'] is not None:
                # Activate weld constraint
                self.model.eq_active[berry_info['magnet_id']] = 1
                print(f"  ✓ Magnet activated for {berry_name}")
                
                # Let physics settle
                self.step(viewer, 100)
                return True
            else:
                print(f"  ✗ No magnet constraint found for {berry_name}")
                return False
        else:
            print(f"  ✗ Too far from berry (need <{threshold}m)")
            return False
    
    def retract_arm(self, viewer):
        """Retract arm after picking."""
        print("⬆️ Retracting arm with berry...")
        self.data.ctrl[ACT["lift"]] = 0.05
        self.data.ctrl[ACT["arm_extend"]] = -0.4
        self.step(viewer, 200)
        
        self.data.ctrl[ACT["lift"]] = 0
        self.data.ctrl[ACT["arm_extend"]] = 0
    
    def capture_image(self, camera_name="main_cam", filename="mission_complete.png"):
        """Capture image from specified camera."""
        print(f"📸 Capturing image from {camera_name}...")
        
        renderer = mujoco.Renderer(self.model, width=640, height=480)
        
        # Update scene
        mujoco.mj_forward(self.model, self.data)
        
        # Render from camera
        camera_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name
        )
        renderer.update_scene(self.data, camera=camera_id)
        img = renderer.render()
        
        imageio.imwrite(filename, img)
        print(f"  ✓ Saved: {filename}")
        
        renderer.close()
        return filename
    
    def harvest_berry(self, berry_name, screenshot_path="harvest_result.png"):
        """Complete harvest sequence for a specific berry."""
        print("=" * 70)
        print(f"🍓 KISAN-DRISHTI HARVEST MISSION: {berry_name}")
        print("=" * 70)
        
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            # CRITICAL: Let physics settle first!
            self.settle_physics(viewer, steps=500)
            
            # Get settled berry position
            berry_pos = self.get_berry_position(berry_name)
            
            # Check if berry fell through ground (z < 0)
            if berry_pos[2] < 0.1:
                print(f"⚠️ WARNING: {berry_name} appears to have fallen through ground!")
                print(f"   Position: {berry_pos}")
                print("   Aborting mission.")
                return False, None
            
            time.sleep(1)
            
            # Step 1: Navigate to berry
            nav_x = berry_pos[0] - 0.4  # Stop 40cm before berry
            nav_y = berry_pos[1] - 0.4
            self.navigate_to(nav_x, nav_y, viewer)
            
            # Step 2: Align arm
            self.align_arm_to_berry(berry_pos, viewer)
            
            # Step 3: Magnet pick
            success = self.magnet_pick(berry_name, viewer)
            
            # Step 4: Retract if successful
            if success:
                self.retract_arm(viewer)
            
            # Step 5: Capture screenshot
            screenshot = self.capture_image("main_cam", screenshot_path)
            
            print("=" * 70)
            print(f"Mission Status: {'✓ SUCCESS' if success else '✗ FAILED'}")
            print(f"Screenshot: {screenshot}")
            print("=" * 70)
            
            # Keep viewer open
            print("\nViewer will stay open. Press Ctrl+C or close window to exit.")
            try:
                while viewer.is_running():
                    mujoco.mj_step(self.model, self.data)
                    viewer.sync()
                    time.sleep(0.02)
            except KeyboardInterrupt:
                print("\nShutting down...")
        
        return success, screenshot


# ==================== MAIN EXECUTION ====================

if __name__ == "__main__":
    print("\n" + "=" * 70)
    print("KISAN-DRISHTI: Strawberry Harvesting Robot")
    print("=" * 70 + "\n")
    
    # Initialize picker
    picker = StretchBerryPicker(MODEL_XML)
    
    # Define which berry to harvest
    TARGET_BERRY = "berry2"  # Change to berry1, berry3, berry4, or berry5
    
    # Execute mission
    success, screenshot = picker.harvest_berry(
        berry_name=TARGET_BERRY,
        screenshot_path=f"{TARGET_BERRY}_harvest.png"
    )
    
    print(f"\n{'✓' if success else '✗'} Mission completed!")