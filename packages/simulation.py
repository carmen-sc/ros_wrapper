#!/usr/bin/env python3
import io
import numpy as np

import gym_duckietown
from gym_duckietown.simulator import Simulator

from wrapper_node import WrapperNode

wrapper = WrapperNode('wrapper_node')

env = Simulator(
        seed=123, # random seed
        map_name="loop_empty",
        max_steps=500001, # we don't want the gym to reset itself
        domain_rand=0,
        camera_width=640,
        camera_height=480,
        accept_start_angle_deg=4, # start close to straight
        full_transparency=True,
        distortion=True,
    )   

while True:
    # get action from wrapper
    action = wrapper.action

    observation, reward, done, misc = env.step(action)

    wrapper.pub_img(observation)
    wrapper.pub_cam_info()

    env.render()
    if done:
        env.reset()


