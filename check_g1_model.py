import mujoco

from unitree_paths import load_g1_model

model = load_g1_model()
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