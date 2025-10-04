import mujoco
import numpy as np
import time
import sys
import os
import json
import threading
import asyncio
import base64

# Import checks (these are safe to do at module level)
USE_BUILTIN_VIEWER = False
viewer_module = None

try:
    import mujoco.viewer as viewer_module

    USE_BUILTIN_VIEWER = True
    print("Using built-in mujoco.viewer")
except (ImportError, AttributeError):
    try:
        import mujoco_viewer

        print("Using mujoco_viewer package")
    except ImportError:
        print("ERROR: No viewer available. Install with: pip install mujoco-viewer")
        sys.exit(1)

IK_AVAILABLE = False
try:
    from robot_arm_ik import G1_29_ArmIK
    import pinocchio as pin

    IK_AVAILABLE = True
    print("IK solver loaded successfully!")
except ImportError as e:
    print(f"IK solver not available: {e}")

VOICE_CONTROL_AVAILABLE = False
try:
    import websockets
    import pyaudio
    from dotenv import load_dotenv

    load_dotenv()
    VOICE_CONTROL_AVAILABLE = True
    print("Voice control dependencies available!")
except ImportError as e:
    print(f"Voice control not available: {e}")
    print("Install with: pip install websockets pyaudio python-dotenv")

# Global variables for target control
target_lock = threading.Lock()
target_position = np.array([0.25, -0.25, 0.1])
target_rotation = np.array([0.0, 0.0, 0.0])
right_target_pose = None

position_step = 0.01
rotation_step = 0.05
voice_position_step = 0.05
voice_rotation_step = 0.2


def create_target_pose(x, y, z, roll=0, pitch=0, yaw=0):
    """Create a 4x4 SE(3) transformation matrix from position and Euler angles"""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    R = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]
    ])

    pose = np.eye(4)
    pose[:3, :3] = R
    pose[:3, 3] = [x, y, z]
    return pose


def move_hand(direction, amount="normal"):
    """Move the hand in a specified direction"""
    global target_position, right_target_pose

    step = voice_position_step if amount == "normal" else voice_position_step * 2

    with target_lock:
        if direction == "forward":
            target_position[0] += step
        elif direction == "backward":
            target_position[0] -= step
        elif direction == "left":
            target_position[1] += step
        elif direction == "right":
            target_position[1] -= step
        elif direction == "up":
            target_position[2] += step
        elif direction == "down":
            target_position[2] -= step

        right_target_pose = create_target_pose(
            target_position[0], target_position[1], target_position[2],
            target_rotation[0], target_rotation[1], target_rotation[2]
        )

    print(f"Moved hand {direction} ({amount})")
    return f"Hand moved {direction}"


def rotate_hand(axis, amount="normal"):
    """Rotate the hand around a specified axis"""
    global target_rotation, right_target_pose

    step = voice_rotation_step if amount == "normal" else voice_rotation_step * 2

    with target_lock:
        if axis == "roll":
            target_rotation[0] += step
        elif axis == "pitch":
            target_rotation[1] += step
        elif axis == "yaw":
            target_rotation[2] += step

        right_target_pose = create_target_pose(
            target_position[0], target_position[1], target_position[2],
            target_rotation[0], target_rotation[1], target_rotation[2]
        )

    print(f"Rotated hand {axis} ({amount})")
    return f"Hand rotated on {axis} axis"


def reset_hand():
    """Reset hand to initial position"""
    global target_position, target_rotation, right_target_pose

    with target_lock:
        target_position = np.array([0.25, -0.25, 0.1])
        target_rotation = np.array([0.0, 0.0, 0.0])
        right_target_pose = create_target_pose(
            target_position[0], target_position[1], target_position[2],
            target_rotation[0], target_rotation[1], target_rotation[2]
        )

    print("Hand reset to initial position")
    return "Hand reset to initial position"


class VoiceController:
    def __init__(self, api_key):
        self.api_key = api_key
        self.ws = None
        self.running = False

    async def connect(self):
        """Connect to OpenAI Realtime API"""
        url = "wss://api.openai.com/v1/realtime?model=gpt-4o-realtime-preview-2024-10-01"

        self.ws = await websockets.connect(
            url,
            additional_headers={
                "Authorization": f"Bearer {self.api_key}",
                "OpenAI-Beta": "realtime=v1"
            }
        )
        print("Connected to OpenAI Realtime API")

        await self.ws.send(json.dumps({
            "type": "session.update",
            "session": {
                "modalities": ["text", "audio"],
                "instructions": "You are controlling a robot arm. When the user asks to move or rotate the hand, call the appropriate function. Be concise and confirm the action.",
                "voice": "alloy",
                "input_audio_format": "pcm16",
                "output_audio_format": "pcm16",
                "tools": [
                    {
                        "type": "function",
                        "name": "move_hand",
                        "description": "Move the robot hand in a specified direction (forward, backward, left, right, up, down)",
                        "parameters": {
                            "type": "object",
                            "properties": {
                                "direction": {
                                    "type": "string",
                                    "enum": ["forward", "backward", "left", "right", "up", "down"],
                                    "description": "The direction to move the hand"
                                },
                                "amount": {
                                    "type": "string",
                                    "enum": ["normal", "large"],
                                    "description": "How much to move (normal or large step)",
                                    "default": "normal"
                                }
                            },
                            "required": ["direction"]
                        }
                    },
                    {
                        "type": "function",
                        "name": "rotate_hand",
                        "description": "Rotate the robot hand around an axis (roll, pitch, or yaw)",
                        "parameters": {
                            "type": "object",
                            "properties": {
                                "axis": {
                                    "type": "string",
                                    "enum": ["roll", "pitch", "yaw"],
                                    "description": "The axis to rotate around"
                                },
                                "amount": {
                                    "type": "string",
                                    "enum": ["normal", "large"],
                                    "description": "How much to rotate",
                                    "default": "normal"
                                }
                            },
                            "required": ["axis"]
                        }
                    },
                    {
                        "type": "function",
                        "name": "reset_hand",
                        "description": "Reset the hand to its initial position",
                        "parameters": {
                            "type": "object",
                            "properties": {}
                        }
                    }
                ],
                "tool_choice": "auto"
            }
        }))

    async def send_audio(self, audio_data):
        """Send audio to the API"""
        if self.ws:
            audio_b64 = base64.b64encode(audio_data).decode()
            await self.ws.send(json.dumps({
                "type": "input_audio_buffer.append",
                "audio": audio_b64
            }))

    async def handle_messages(self):
        """Handle incoming messages from the API"""
        async for message in self.ws:
            data = json.loads(message)
            msg_type = data.get("type")

            if msg_type == "response.function_call_arguments.done":
                function_name = data.get("name")
                arguments = json.loads(data.get("arguments", "{}"))
                call_id = data.get("call_id")

                print(f"Function call: {function_name}({arguments})")

                result = ""
                if function_name == "move_hand":
                    result = move_hand(**arguments)
                elif function_name == "rotate_hand":
                    result = rotate_hand(**arguments)
                elif function_name == "reset_hand":
                    result = reset_hand()

                await self.ws.send(json.dumps({
                    "type": "conversation.item.create",
                    "item": {
                        "type": "function_call_output",
                        "call_id": call_id,
                        "output": result
                    }
                }))

            elif msg_type == "response.done":
                print("Response completed")
            elif msg_type == "error":
                print(f"Error: {data}")

    async def audio_input_loop(self):
        """Capture audio from microphone and send to API"""
        p = pyaudio.PyAudio()
        stream = p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=24000,
            input=True,
            frames_per_buffer=4096
        )

        print("Listening for voice commands...")

        try:
            while self.running:
                audio_data = stream.read(4096, exception_on_overflow=False)
                await self.send_audio(audio_data)
                await asyncio.sleep(0.01)
        finally:
            stream.stop_stream()
            stream.close()
            p.terminate()

    async def run(self):
        """Main run loop"""
        self.running = True
        await self.connect()
        await asyncio.gather(
            self.audio_input_loop(),
            self.handle_messages()
        )


def start_voice_control():
    """Start voice control in a separate thread"""
    OPENAI_API_KEY = os.getenv("OPENAI_API_KEY", "")

    if not VOICE_CONTROL_AVAILABLE:
        print("Voice control not available - missing dependencies")
        return

    if not OPENAI_API_KEY:
        print("WARNING: OPENAI_API_KEY not found in .env file")
        return

    controller = VoiceController(OPENAI_API_KEY)

    async def run_async():
        try:
            await controller.run()
        except Exception as e:
            print(f"Voice control error: {e}")

    def run_in_thread():
        asyncio.run(run_async())

    thread = threading.Thread(target=run_in_thread, daemon=True)
    thread.start()
    print("Voice control started!")


def keyboard_listener():
    """Thread to handle keyboard input"""
    global target_position, target_rotation, right_target_pose

    try:
        import keyboard

        print("\nKeyboard controls active!")
        print("  ↑/↓: Forward/Backward")
        print("  ←/→: Left/Right")
        print("  PgUp/PgDn: Up/Down")
        print("  SHIFT+arrows/pgup/pgdn: Rotation")
        print("  ESC: Exit\n")

        while True:
            changed = False
            shift_held = keyboard.is_pressed('shift')

            if not shift_held:
                if keyboard.is_pressed('up'): target_position[0] += position_step; changed = True
                if keyboard.is_pressed('down'): target_position[0] -= position_step; changed = True
                if keyboard.is_pressed('left'): target_position[1] += position_step; changed = True
                if keyboard.is_pressed('right'): target_position[1] -= position_step; changed = True
                if keyboard.is_pressed('page up'): target_position[2] += position_step; changed = True
                if keyboard.is_pressed('page down'): target_position[2] -= position_step; changed = True
            else:
                if keyboard.is_pressed('up'): target_rotation[1] += rotation_step; changed = True
                if keyboard.is_pressed('down'): target_rotation[1] -= rotation_step; changed = True
                if keyboard.is_pressed('left'): target_rotation[2] += rotation_step; changed = True
                if keyboard.is_pressed('right'): target_rotation[2] -= rotation_step; changed = True
                if keyboard.is_pressed('page up'): target_rotation[0] += rotation_step; changed = True
                if keyboard.is_pressed('page down'): target_rotation[0] -= rotation_step; changed = True

            if keyboard.is_pressed('esc'):
                os._exit(0)

            if changed:
                with target_lock:
                    right_target_pose = create_target_pose(
                        target_position[0], target_position[1], target_position[2],
                        target_rotation[0], target_rotation[1], target_rotation[2]
                    )

            time.sleep(0.05)

    except ImportError:
        print("Keyboard control not available")


def main():
    """Main simulation function"""
    global right_target_pose

    print("\n" + "=" * 60)
    print("Loading G1 model...")
    model = mujoco.MjModel.from_xml_path("assets/g1_29dof_rev_1_0.xml")
    data = mujoco.MjData(model)
    print("Model loaded successfully")

    print(f"Creating viewer (USE_BUILTIN_VIEWER={USE_BUILTIN_VIEWER})...")
    if USE_BUILTIN_VIEWER:
        viewer = viewer_module.launch_passive(model, data)
        print("Built-in viewer created")
    else:
        viewer = mujoco_viewer.MujocoViewer(model, data)
        print("mujoco_viewer created")

    # Define joints
    right_arm_joint_names = [
        'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint',
        'right_elbow_joint', 'right_wrist_roll_joint', 'right_wrist_pitch_joint', 'right_wrist_yaw_joint'
    ]

    left_arm_joint_names = [
        'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint',
        'left_elbow_joint', 'left_wrist_roll_joint', 'left_wrist_pitch_joint', 'left_wrist_yaw_joint'
    ]

    fixed_joint_names = [
        'left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint',
        'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
        'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint',
        'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint',
        'waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint'
    ]

    # Get joint addresses
    right_arm_qpos_addrs = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)] for name in
                            right_arm_joint_names]
    left_arm_qpos_addrs = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)] for name in
                           left_arm_joint_names]
    fixed_joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in fixed_joint_names]
    fixed_qpos_addrs = [model.jnt_qposadr[jid] for jid in fixed_joint_ids]

    # Initialize IK
    arm_ik = None
    if IK_AVAILABLE:
        try:
            arm_ik = G1_29_ArmIK(Unit_Test=False, Visualization=False)
            print("IK solver initialized!")
        except Exception as e:
            print(f"Failed to initialize IK solver: {e}")

    # Set initial configuration
    data.qpos[0:7] = [0.0, 0.0, 0.8, 1.0, 0.0, 0.0, 0.0]
    left_arm_initial = np.zeros(7)
    for i, qpos_adr in enumerate(left_arm_qpos_addrs):
        data.qpos[qpos_adr] = left_arm_initial[i]
    for qpos_adr in fixed_qpos_addrs:
        data.qpos[qpos_adr] = 0.0
    mujoco.mj_forward(model, data)

    # Initialize targets
    right_target_pose = create_target_pose(0.25, -0.25, 0.1)
    left_target_pose = create_target_pose(0.25, 0.25, 0.1)

    # Start control threads
    keyboard_thread = threading.Thread(target=keyboard_listener, daemon=True)
    keyboard_thread.start()

    start_voice_control()

    print("=" * 60)
    print("MuJoCo G1 Arm Control with Voice & IK")
    print("=" * 60)
    print("Voice: Say 'move forward', 'go up', 'rotate left', etc.")
    print("Keyboard: Arrow keys + PgUp/PgDn for manual control")
    print("=" * 60 + "\n")

    # Main simulation loop
    dt = model.opt.timestep

    try:
        running = True
        while running:
            step_start = time.time()

            if USE_BUILTIN_VIEWER:
                running = viewer.is_running()
            else:
                running = viewer.is_alive

            if not running:
                break

            if arm_ik is not None:
                with target_lock:
                    current_right_target = right_target_pose.copy()

                current_left_arm_q = np.array([data.qpos[addr] for addr in left_arm_qpos_addrs])
                current_right_arm_q = np.array([data.qpos[addr] for addr in right_arm_qpos_addrs])
                current_lr_arm_q = np.concatenate([current_left_arm_q, current_right_arm_q])

                sol_q, sol_tauff = arm_ik.solve_ik(left_target_pose, current_right_target, current_lr_arm_q, None)

                for i, qpos_adr in enumerate(left_arm_qpos_addrs):
                    data.qpos[qpos_adr] = left_arm_initial[i]

                for i, qpos_adr in enumerate(right_arm_qpos_addrs):
                    data.qpos[qpos_adr] = sol_q[7 + i]

            for qpos_adr in fixed_qpos_addrs:
                data.qpos[qpos_adr] = 0.0

            data.qpos[0:7] = [0.0, 0.0, 0.8, 1.0, 0.0, 0.0, 0.0]
            data.qvel[0:6] = 0.0

            for joint_id in fixed_joint_ids:
                qvel_adr = model.jnt_dofadr[joint_id]
                data.qvel[qvel_adr] = 0.0

            mujoco.mj_step(model, data)

            if USE_BUILTIN_VIEWER:
                viewer.sync()
            else:
                viewer.render()

            time_until_next_step = dt - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    except KeyboardInterrupt:
        print("\nSimulation stopped")

    if USE_BUILTIN_VIEWER:
        viewer.close()
    else:
        viewer.close()

    print("Simulation ended")


if __name__ == "__main__":
    main()