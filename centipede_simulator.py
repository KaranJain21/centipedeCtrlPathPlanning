from gym.envs.mujoco import *
from centipede_env import CentipedeEnv
import numpy as np
import gym
import moviepy.editor as mpy
from rot_utils import *
import matplotlib.pyplot as plt
import controlpy
from ModifyPlotForPublication import *
ModifyPlotForPublication()

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
genContPts=1
saveParams=[]
headPosVelArr=np.zeros((timeSteps,6))

for t in range(timeSteps):
	action, genContPts, saveParams = \
	env.get_action(t, genContPts, saveParams)
	headPosVelArr[t,:]=saveParams[-1]
	stateArr[:,t+1], _, _, _ = env.step(action)
	env.render()

avgSpeed=(headPosVelArr[-1,0]-headPosVelArr[0,0])/(timeSteps/25)
print(avgSpeed)

n = 2	# Number of subplots
ieeeColumnWidth = 3.46457  # corresponds to 88mm
figureHeight = 1.0 * ieeeColumnWidth
fig, ax = plt.subplots(n,1, figsize=[ieeeColumnWidth, figureHeight], sharex=True)
ax[0].plot(0.04*np.arange(timeSteps), headPosVelArr[:,0], color='b', label='X-position')
ax[0].set_ylabel('Head X-position [m]')

ax[1].plot(0.04*np.arange(timeSteps), headPosVelArr[:,3], color='b', label='Actual speed')
ax[1].plot([0, timeSteps/25], [avgSpeed, avgSpeed], 'r-', dashes=[5, 3], label='Average speed')
ax[1].set_ylabel('Head X-velocity [m/s]')
ax[1].legend()

ax[-1].set_xlabel('Time [sec]')
ax[-1].set_xlim([0,timeSteps/25])

#fig.tight_layout(pad=0.1)
#fig.savefig('headPosVel.pdf')
plt.show()