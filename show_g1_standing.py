import mujoco
import mujoco.viewer
import time

from unitree_paths import load_g1_model

model = load_g1_model()
data = mujoco.MjData(model)

# Vypnutí gravitace jen pro prohlížení modelu
model.opt.gravity[:] = [0, 0, 0]

# Pokud model obsahuje keyframe, použijeme první uloženou pózu
if model.nkey > 0:
    print("Using keyframe 0")
    data.qpos[:] = model.key_qpos[0]
    data.qvel[:] = 0
else:
    print("No keyframe found, using default qpos")
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0

mujoco.mj_forward(model, data)

with mujoco.viewer.launch_passive(model, data) as viewer:
    print("Viewer running. Close window to exit.")
    while viewer.is_running():
        mujoco.mj_forward(model, data)
        viewer.sync()
        time.sleep(0.01)