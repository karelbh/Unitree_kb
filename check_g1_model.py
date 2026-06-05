from pathlib import Path
import mujoco

model_path = Path(r"F:\Unitree\unitree_mujoco\unitree_robots\g1\scene_23dof.xml")

print("Loading:", model_path)

if not model_path.exists():
    raise FileNotFoundError(model_path)

model = mujoco.MjModel.from_xml_path(str(model_path))
data = mujoco.MjData(model)

print("Model loaded OK")
print("Bodies:", model.nbody)
print("Joints:", model.njnt)
print("Actuators:", model.nu)

print()
print("Actuator list:")
for i in range(model.nu):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
    print(i, name)

for _ in range(100):
    mujoco.mj_step(model, data)

print()
print("Simulation step OK")