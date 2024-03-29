Nikhil.P.S and Karan.P.Jain {nikhilps/karanjain}@berkeley.edu

This folder contains a set of .m files that have been used to implement 
the force optimization method present in our document.
The 'Matlab' folder should contain the following files and folders.

1.Starter.m : 
This .m file is used to obtain the dynamics of a centipede like robot with 2 links.
It only requires minor extensions to make it modular and extend it to a 'N' links.
The dynamics have been generated using Lagrangian dynamics using Matlab's inbuilt
Symbolic toolbox.
Different system parameters have been introduced at the start of the file.
These values can be changed and the file can be run to obtain the dynamics 
for different system parameters.
This file genrates a bunch of utility functions like jacobains and foot positions
that are stored in the folder 'gen'. Make sure that the folder 'gen' is added to 
the path in the Starter.m file.
One just needs to change the system parameters in this file and run the code to
generate various states of the centipede as functions of time.
The gains in Controller.m can be changed to 

2.Controller.m:
This file contains the control strategy which uses force optimization to optimize
the internal force and the contact forces which is an extension of the referecnce 
presented in the document.
The gains can be changed to vary the controller's performance.
One can also uncomment the final section to implement an optimization approach to
find the optimal internal and contact forces at every instance instead of a
pseudo inverse approach.

3.LagrangianDynamics.m
This file generates the dynamics of the centipede.
This file is called by Starter.m and takes in Kinetic energy,potential energy,
generalized coordinates,generalized velocities and the actuated generalized
coordinates to output the dynamics of the centipede when its free to move in 
the air with no constraints on the feet.

4.centipede_dynamics.m
This file generates the dynamics of the centipede with its feet touching the 
ground. The dynamics is represented in state space form to be directly used 
by an ode solver.

5.gen:
This folder contains the utility functions generated by Starter.m


 

