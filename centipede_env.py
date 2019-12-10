import numpy as np
from gym import utils
from gym.envs.mujoco import mujoco_env
import copy
import os
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares, minimize

class CentipedeEnv(mujoco_env.MujocoEnv, utils.EzPickle):
	def __init__(self):
		mujoco_env.MujocoEnv.__init__(self, 'centipede.xml', 4) # Last argument is the number of frames before updating control
		utils.EzPickle.__init__(self)

	def all_position_attitude(self):
		# Returns position (nBody x 3) and attitude (nBody x 4)
		# rotation matrix in r,p,y configuration
		return self.sim.data.xipos[1:,:], self.sim.data.body_xquat[1:,:]

	def all_velocity(self):
		# Returns rotational speed and translational speed, (nBody x 6)
		# angular speeds are in roll, pitch, yaw order
		return self.sim.data.cvel[1:,:]

	def link_position_attitude(self):
		return self.sim.data.xipos[1::7,:], self.sim.data.body_xquat[1::7,:]
	
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

	def get_leg_tip_pos(self):
		legLength=0.3	# From centipede_xml_generator.py
		leftLegCMPosArr=self.sim.data.xipos[4::7,:]
		leftLegQuatArr=self.sim.data.body_xquat[4::7,:]
		rightLegCMPosArr=self.sim.data.xipos[7::7,:]
		rightLegQuatArr=self.sim.data.body_xquat[7::7,:]
		N=leftLegCMPosArr.shape[0]
		legTipPosArr=np.zeros((2*N,leftLegCMPosArr.shape[1]))
		for i in range(N):
			rLeft = R.from_euler('x',180,degrees=True)*R.from_quat(leftLegQuatArr[i,:])	# r is in z,y,x order
			rRight = R.from_euler('x',180,degrees=True)*R.from_quat(rightLegQuatArr[i,:])
			legTipPosArr[2*i,:]=leftLegCMPosArr[i,:]+legLength/2*(rLeft.apply([0,1,0]))[::-1]		# left Leg tip
			legTipPosArr[2*i+1,:]=rightLegCMPosArr[i,:]+legLength/2*(rRight.apply([0,-1,0]))[::-1]	# right Leg tip
		return legTipPosArr

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

	def get_tip_pos_BF(self, angles, LorR):
		torsoLength=0.2
		appendageLength=0.1
		thighLength=0.2
		legLength=0.3
		legDiameter=0.03
		xTipBF=np.sin(angles[0])*(appendageLength+thighLength*np.cos(angles[1])+legLength*np.cos(angles[1]+angles[2]))
		zTipBF=-thighLength*np.sin(angles[1])-legLength*np.sin(angles[1]+angles[2])
		yTipBF=(1-2*LorR)*(torsoLength/2+np.cos(angles[0])*(appendageLength+\
			thighLength*np.cos(angles[1])+legLength*np.cos(angles[1]+angles[2])))
		return np.array([xTipBF,yTipBF,zTipBF])

	def tip_pos_BF2EF(self, posBF, linkPos, rLink):
		return rLink.apply(posBF[::-1])[::-1]+linkPos

	def generateContactPts(self, prevContactPts):
		nLegs=prevContactPts.shape[0]
		return prevContactPts+0.1*np.array([np.ones((nLegs,)),np.zeros((nLegs,))]).T

	def desJointAngles_fromDesContactPt(self, p, *params):	# Scipy solver
		th1,th2,th3=p
		params=np.array((params)).reshape((-1,))
		lt, l1, l2, l3, linkCOM, linkRot, LorR, desContact = params
		BFvector=linkRot.inv().apply((desContact-linkCOM)[::-1])[::-1]-[0,(1-2*LorR)*lt/2,0]
		return [np.sin(th1)*(l1+l2*np.cos(th2)+l3*np.cos(th2+th3)),\
			(1-2*LorR)*(np.cos(th1)*(l1+l2*np.cos(th2)+l3*np.cos(th2+th3))),\
			-l2*np.sin(th2)-l3*np.sin(th2+th3)]-BFvector

	def contactPtBoundsSolver(self, p, *params):	# Scipy solver
		th1,th2,th3=p
		params=np.array(params).reshape((-1,))
		lt, l1, l2, l3, linkCOM, linkRot, LorR = params[0]
		case=params[1]
		BFvector=[np.sin(th1)*(l1+l2*np.cos(th2)+l3*np.cos(th2+th3)),\
					(1-2*LorR)*np.cos(th1)*(l1+l2*np.cos(th2)+l3*np.cos(th2+th3)),\
					-l2*np.sin(th2)-l3*np.sin(th2+th3)]
		rTip=linkRot.apply(BFvector[::-1])[::-1]+linkCOM
		if case==0:
			return rTip[0]+20*(rTip[2]-0.03)**2	# xmin
		if case==1:
			return rTip[1]+30*(rTip[2]-0.03)**2	# ymin
		if case==2:
			return -rTip[0]+20*(rTip[2]-0.03)**2	# xmax
		if case==3:
			return -rTip[1]+30*(rTip[2]-0.03)**2	# ymax

	def contactPtBounds(self, params):
		appendageAngleMin=-20
		appendageAngleMax=-appendageAngleMin
		thighAngleMin=-30
		thighAngleMax=-thighAngleMin
		legAngleMin=20
		legAngleMax=90
		boundsSolverParams=np.array([params,0])
		'''
		anglesMinX=minimize(self.contactPtBoundsSolver, (0,0,np.deg2rad(30)), args=boundsSolverParams, \
			bounds=(np.deg2rad([appendageAngleMin,appendageAngleMax]),np.deg2rad([thighAngleMin,thighAngleMax]), \
			np.deg2rad([legAngleMin,legAngleMax]))).x
		boundsSolverParams=np.array([params,1])
		anglesMinY=minimize(self.contactPtBoundsSolver, (0,0,np.deg2rad(30)), args=boundsSolverParams, \
			bounds=(np.deg2rad([appendageAngleMin,appendageAngleMax]),np.deg2rad([thighAngleMin,thighAngleMax]), \
			np.deg2rad([legAngleMin,legAngleMax]))).x
		boundsSolverParams=np.array([params,2])
		anglesMaxX=minimize(self.contactPtBoundsSolver, (0,0,np.deg2rad(30)), args=boundsSolverParams, \
			bounds=(np.deg2rad([appendageAngleMin,appendageAngleMax]),np.deg2rad([thighAngleMin,thighAngleMax]), \
			np.deg2rad([legAngleMin,legAngleMax]))).x
		boundsSolverParams=np.array([params,3])
		anglesMaxY=minimize(self.contactPtBoundsSolver, (0,0,np.deg2rad(30)), args=boundsSolverParams, \
			bounds=(np.deg2rad([appendageAngleMin,appendageAngleMax]),np.deg2rad([thighAngleMin,thighAngleMax]), \
			np.deg2rad([legAngleMin,legAngleMax]))).x
		tipPosMinXEF=self.tip_pos_BF2EF(self.get_tip_pos_BF(anglesMinX, params[-1]),params[-3:-1])
		tipPosMinYEF=self.tip_pos_BF2EF(self.get_tip_pos_BF(anglesMinY, params[-1]),params[-3:-1])
		tipPosMaxXEF=self.tip_pos_BF2EF(self.get_tip_pos_BF(anglesMaxX, params[-1]),params[-3:-1])
		tipPosMaxYEF=self.tip_pos_BF2EF(self.get_tip_pos_BF(anglesMaxY, params[-1]),params[-3:-1])
		return tipPosMinXEF[0], tipPosMinYEF[1], tipPosMaxXEF[0], tipPosMaxYEF[1]
		'''
		return -0.2, 0.48, 0.2, 0.6

	def get_action(self, time, genContactPts, savedParams):
		nu=self.action_space.shape[0]
		N=int(nu/6)
		action=np.zeros((nu,))
		linkPosArr, linkQuatArr = self.link_position_attitude()
		linkVelArr = self.link_velocity()
		angleArr, angVelArr = self.leg_angles_velocities()
		controlSat = 0.05	# control saturation
		legTipPosArr = self.get_leg_tip_pos()

		# Extract saved parameters
		if len(savedParams):
			desContactPointArr, desJointAngleArr, t0 = savedParams
		else:
			desContactPointArr, desJointAngleArr, t0 = [],[],0

		# From centipede_xml_generator.py
		torsoLength=0.2
		appendageLength=0.1
		thighLength=0.2
		legLength=0.3
		legDiameter=0.03
		appendageAngleMin=-20
		appendageAngleMax=-appendageAngleMin
		thighAngleMin=-30
		thighAngleMax=-thighAngleMin
		legAngleMin=20
		legAngleMax=90

		if (all(legTipPosArr[:,2]<0.05) and genContactPts and (time-t0>5)):
			desContactPointArr=np.append(self.generateContactPts(legTipPosArr[:,:-1]),0.03*np.ones((2*N,1)),1)
			genContactPts=0
			desJointAngleArr=np.zeros(desContactPointArr.shape)
			for i in range(N):
				rLink=R.from_euler('x',180,degrees=True)*R.from_quat(linkQuatArr[i,:])
				solverParams=[torsoLength, appendageLength, thighLength, legLength, linkPosArr[i,:], rLink, 0]
				#[xmin, ymin, xmax, ymax]=self.contactPtBounds(solverParams)	# Square
				solverParams.append(desContactPointArr[2*i,:])
				desJointAngleArr[2*i,:]=least_squares(self.desJointAngles_fromDesContactPt, (0,0,np.deg2rad(30)), args=solverParams, \
					bounds=(np.deg2rad([appendageAngleMin,thighAngleMin,legAngleMin]), \
					np.deg2rad([appendageAngleMax,thighAngleMax,legAngleMax]))).x

				solverParams=[torsoLength, appendageLength, thighLength, legLength, linkPosArr[i,:], rLink, 1]
				solverParams.append(desContactPointArr[2*i+1,:])
				desJointAngleArr[2*i+1,:]=least_squares(self.desJointAngles_fromDesContactPt, (0,0,np.deg2rad(30)), args=solverParams, \
					bounds=(np.deg2rad([appendageAngleMin,thighAngleMin,legAngleMin]), \
					np.deg2rad([appendageAngleMax,thighAngleMax,legAngleMax]))).x
			desJointAngleArr=desJointAngleArr.flatten()
			t0=time	# Time of generation of contact points

		segmentPeriodicity=3	# Gait repeats every n segments
		Kp=0.3
		Kd=0.12
		for i in range(N):
			action[6*i+0]=controlSat*np.sin(time/20+i*2*np.pi/segmentPeriodicity)
			action[6*i+1]=-controlSat
			action[6*i+2]=controlSat*np.sign(np.sin(time/20+i*2*np.pi/segmentPeriodicity-np.pi/2))
			#action[6*i+2]=controlSat
			action[6*i+3]=action[6*i+0]
			action[6*i+4]=action[6*i+1]
			action[6*i+5]=action[6*i+2]
		#action[0]=controlSat
		charTime=12	# Some characteristic time
		desAppendageAngleEnd=np.deg2rad(-5)
		t1=t0+2*charTime
		t2=t1+2*charTime
		if (time > t0 + 6*charTime) and (not genContactPts):
			genContactPts=1
		print(linkPosArr[0,:])
		'''
		if len(desContactPointArr):
			for i in range(N):
				if i%3==0:
					if (time-t0)<1*charTime:
						action[6*i+1]=-controlSat
						action[6*i+2]=-controlSat
						action[6*i+4]=action[6*i+1]
						action[6*i+5]=action[6*i+2]
					if (time-t0)>=1*charTime:
						action[6*i:6*(i+1)]=Kp*(desJointAngleArr[6*i:6*(i+1)]-angleArr[6*i:6*(i+1)])-Kd*angVelArr[6*i:6*(i+1)]
					if (time-t0)>=2*charTime:
						desJointAngleArr[6*i]=desAppendageAngleEnd
						desJointAngleArr[6*i+3]=desAppendageAngleEnd
				if i%3==1:
					if (time-t1)<1*charTime and (time-t0)>=2*charTime:
						action[6*i+1]=-controlSat
						action[6*i+2]=-controlSat
						action[6*i+4]=action[6*i+1]
						action[6*i+5]=action[6*i+2]
					if (time-t1)>=1*charTime:
						action[6*i:6*(i+1)]=Kp*(desJointAngleArr[6*i:6*(i+1)]-angleArr[6*i:6*(i+1)])-Kd*angVelArr[6*i:6*(i+1)]
					if (time-t1)>=2*charTime:
						desJointAngleArr[6*i]=desAppendageAngleEnd
						desJointAngleArr[6*i+3]=desAppendageAngleEnd
				if i%3==2:
					if (time-t2)<1*charTime and (time-t1)>=2*charTime:
						action[6*i+1]=-controlSat
						action[6*i+2]=-controlSat
						action[6*i+4]=action[6*i+1]
						action[6*i+5]=action[6*i+2]
					if (time-t2)>=1*charTime:
						action[6*i:6*(i+1)]=Kp*(desJointAngleArr[6*i:6*(i+1)]-angleArr[6*i:6*(i+1)])-Kd*angVelArr[6*i:6*(i+1)]
					if (time-t2)>=2*charTime:
						desJointAngleArr[6*i]=desAppendageAngleEnd
						desJointAngleArr[6*i+3]=desAppendageAngleEnd
		'''
		# action=self.action_space.sample()
		return action, genContactPts, (desContactPointArr, desJointAngleArr, t0)

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
