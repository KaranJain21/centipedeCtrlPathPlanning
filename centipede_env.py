import numpy as np
from gym import utils
from gym.envs.mujoco import mujoco_env
import copy
import os

class CentipedeEnv(mujoco_env.MujocoEnv, utils.EzPickle):
	def __init__(self):
		mujoco_env.MujocoEnv.__init__(self, 'centipede.xml', 4) # Last argument is the number of frames before updating control
		utils.EzPickle.__init__(self)

	def all_position_attitude(self):
		# Returns position (nBody x 3) and attitude (nBody x 9)
		# rotation matrix in r,p,y configuration
		return self.sim.data.xipos[1:,:], self.sim.data.ximat[1:,:]

	def all_velocity(self):
		# Returns rotational speed and translational speed, (nBody x 6)
		# angular speeds are in roll, pitch, yaw order
		return self.sim.data.cvel[1:,:]

	def link_position_attitude(self):
		return self.sim.data.xipos[1::7,:], self.sim.data.ximat[1::7,:]
	
	def link_velocity(self):
		return self.sim.data.cvel[1::7,:]

	def get_mass_inertia(self):
		mass = self.model.body_mass[1:]	# Returns masses of all rigid bodies
		inertia = self.model.body_inertia[1:]	# Returns masses of all rigid bodies
		return mass, inertia

	def leg_angles_velocities(self):	# all relative to the previous joint
		# Returns the angles and angular velocities of each joint
		# Order Link->(Left then right)->(body to toe); first half angles
		nu=self.action_space.shape[0]
		nv=self.model.nv
		N=int(nu/6)	# Pairs of legs
		state=self._get_obs()
		deleteIndex=np.array(range(3))
		for i in range(N):
			deleteIndex=np.append(deleteIndex,np.array(range(3,6))+9*i)
		angles = np.delete(state[:nv], deleteIndex)
		angVels = np.delete(state[nv:], deleteIndex)
		return angles, angVels

	def step(self, a):
		posbefore = self.sim.data.qpos[0]
		self.do_simulation(a, self.frame_skip)
		posafter, height, ang = self.sim.data.qpos[0:3]
		alive_bonus = 1.0
		reward = (posafter - posbefore) / self.dt
		reward += alive_bonus
		reward -= 1e-3 * np.square(a).sum()
		s = self.state_vector()
		done = False
		ob = self._get_obs()
		return ob, reward, done, {}

	def f_sim(self, x0, u):
		nq, nv = self.model.nq, self.model.nv
		self.sim.reset()
		qpos = copy.deepcopy(self.init_qpos)
		qvel = copy.deepcopy(self.init_qvel)

		qpos[1:] = x0[:nq-1]
		qvel[0:] = x0[nq-1:]

		self.set_state(qpos, qvel)
		self.step(u)
		return np.concatenate([
			self.sim.data.qpos.flat[1:],
			self.sim.data.qvel.flat[0:]
		])

	def get_action(self, time, params):
		nu=self.action_space.shape[0]
		action=np.zeros((nu,))
		positionArr, attitudeArr = self.link_position_attitude()
		velocityArr = self.link_velocity()
		angleArr, angVelArr = self.leg_angles_velocities()
		controlSaturation=0.05

		'''
		K=params['dlqrK']
		x_ref=params['x_ref']
		u_ref=params['u_ref']
		action = K @ (state - x_ref) + u_ref
		'''
		segmentPeriodicity=3	# Gait repeats every n segments
		for i in range(int(nu/6)):
			action[6*i+0]=controlSaturation*np.sin(time/20+i*2*np.pi/segmentPeriodicity)
			action[6*i+1]=-1
			action[6*i+2]=controlSaturation*np.sign(np.sin(time/20+i*2*np.pi/segmentPeriodicity-np.pi/2))
			action[6*i+3]=action[6*i+0]
			action[6*i+4]=action[6*i+1]
			action[6*i+5]=action[6*i+2]
			# action[6]=0.01*np.sin(time/10)
			# action[9]=-action[6]
		# action=self.action_space.sample()
		return action

	def _get_obs(self):
		return np.concatenate([
			self.sim.data.qpos.flat[1:],
			np.clip(self.sim.data.qvel.flat, -10, 10)
		])

	def reset_model(self):  # Uncomment lines below to add noise
		qpos = self.init_qpos# + self.np_random.uniform(low=-.005, high=.005, size=self.model.nq)
		qvel = self.init_qvel# + self.np_random.uniform(low=-.005, high=.005, size=self.model.nv)
		self.set_state(qpos, qvel)
		return self._get_obs()

	def viewer_setup(self):
		self.viewer.cam.trackbodyid = 2
		self.viewer.cam.distance = self.model.stat.extent * 0.75
		self.viewer.cam.lookat[2] += .8
		self.viewer.cam.elevation = -20
