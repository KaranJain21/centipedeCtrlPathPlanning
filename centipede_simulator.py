from gym.envs.mujoco import *
from centipede_env import CentipedeEnv
import numpy as np
import gym
import moviepy.editor as mpy
from rot_utils import *
import matplotlib.pyplot as plt
import controlpy

def linearize_dynamics(f, x_ref, u_ref, my_eps):
	dx, du = x_ref.shape[0], u_ref.shape[0]
	A, B = np.zeros((dx, dx)), np.zeros((dx, du))

	identityDX=np.eye(dx)
	identityDU=np.eye(du)
	for i in range(dx):
		A[:,i]=(f(x_ref+my_eps*identityDX[:,i],u_ref)-f(x_ref-my_eps*identityDX[:,i],u_ref))/(2*my_eps)
	for i in range(du):
		B[:,i]=(f(x_ref,u_ref+my_eps*identityDU[:,i])-f(x_ref,u_ref-my_eps*identityDU[:,i]))/(2*my_eps)
	
	if len(B.shape) == 1:
		return A, B.reshape(-1, 1)
	return A, B

env = CentipedeEnv()
'''
x_ref = np.zeros((2*env.init_qvel.shape[0],))
u_ref = -np.ones(env.action_space.shape)
eps_lin = 0.01
A, B = linearize_dynamics(env.f_sim, x_ref, u_ref, eps_lin)
dx, du = B.shape
Q = np.eye(dx)
R = np.eye(du)

#K_inf, P_inf = lqr_infinite_horizon(A, B, Q, R)
K, P, _ = controlpy.synthesis.controller_lqr_discrete_time(A, B, Q, R)
controlParams = {
	'dlqrK': K,
	'x_ref': x_ref,
	'u_ref': u_ref,
}
'''
timeSteps=500	# Control time steps, 25 Hz
stateArr = np.zeros((2*env.init_qvel.shape[0],timeSteps+1))
stateArr[:,0] = env.reset()
for t in range(timeSteps):
	action = env.get_action(t, 1)
	stateArr[:,t+1], _, _, _ = env.step(action)
	env.render()
#np.savetxt('test.csv', stateArr, delimiter=',', fmt='%s')
'''
fig, ax = plt.subplots(1,1)
ax.plot(range(timeSteps+1),stateArr.T)
plt.show()
'''