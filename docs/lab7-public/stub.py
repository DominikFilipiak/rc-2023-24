"""
Stub for homework 2
"""
import time
import random
import numpy as np
import mujoco
from mujoco import viewer


model = mujoco.MjModel.from_xml_path("lab7-public/car.xml")
renderer = mujoco.Renderer(model, height=480, width=640)
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
viewer = viewer.launch_passive(model, data)


def sim_step(forward, turn, steps=1000, view=False):
    data.actuator("forward").ctrl = forward
    data.actuator("turn").ctrl = turn
    for _ in range(steps):
        step_start = time.time()
        mujoco.mj_step(model, data)
        if view:
            viewer.sync()
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step / 10)

    renderer.update_scene(data, camera="camera1")
    img = renderer.render()
    return img


def task_1_step(turn):
    return sim_step(0.1, turn, steps=200, view=True)


def task_1():
    steps = random.randint(0, 2000)
    img = sim_step(0, 0.1, steps, view=False)

    # TODO: change the lines below,
    # for car control, you should use task_1_step(turn) function
    for i in range(100):
        print(data.body("car").xpos)
        print(data.body("target-ball").xpos)
        img = task_1_step(0)

    # at the end, your car should be close to the red ball (0.2 distance is fine)
    # data.body("car").xpos) is the position of the car


def task_2():
    sim_step(0.5, 0, 1000, view=True)
    speed = random.uniform(0.3, 0.5)
    turn = random.uniform(-0.2, 0.2)
    img = sim_step(speed, turn, 1000, view=True)
    # TODO: change the lines below,
    # you should use sim_step(forward, turn) function
    # you can change the speed and turn as you want
    # do not change the number of steps (1000)

    # at the end, red ball should be close to the green box (0.25 distance is fine)


drift = 0


def task3_step(forward, turn, steps=1000, view=False):
    sim_step(forward, turn + drift, steps=steps, view=view)


def task_3():
    global drift
    drift = np.random.uniform(-0.1, 0.1)
    # TODO: change the lines below,
    # you should use task3_step(forward, turn, steps) function

    # at the end, car should be between the two boxes
