import numpy as np

# All variables needed to define the centipede. Torso and legs are all capsule shaped.
# Not sure what the dimensions are in. TODO: Find it out
# Initially x-axis is forward for the centipede
torsoLength=0.2		# Length of one unit/link of the torso
torsoDiameter=0.06	# Diameter of one unit/link of the torso
appendageLength=0.1
appendageDiameter=0.03
thighLength=0.2
thighDiameter=appendageDiameter
legLength=0.3
legDiameter=appendageDiameter
torsoDistance=0.2	# Distance between 2 links of torso

# Angular freedom for various links (everything in degrees)
appendageAngleMin=-20
appendageAngleMax=-appendageAngleMin
thighAngleMin=-30
thighAngleMax=-thighAngleMin
legAngleMin=20
legAngleMax=90
torsoAngleXMin=-15
torsoAngleXMax=-torsoAngleXMin
torsoAngleYMin=-15
torsoAngleYMax=-torsoAngleYMin
torsoAngleZMin=-15
torsoAngleZMax=-torsoAngleZMin

N=5	# Number of pairs of legs for the centipede (1 = sad, 5 = happy, 10 = chad, 50 = shai hulud)
torsoArr=["" for x in range(N)]
leftLegArr=["" for x in range(N)]
rightLegArr=["" for x in range(N)]
leftActuatorArr=["" for x in range(N)]
rightActuatorArr=["" for x in range(N)]

# Model name, textures, inertia from geometry, numerical solver and other setup
Initializer="<!-- Model for a "+str(2*N)+"-legged centipede -->\n\
<mujoco model=\"centipede\">\n\
  <compiler angle=\"degree\" coordinate=\"local\" inertiafromgeom=\"true\"/>\n\
  <option integrator=\"RK4\" timestep=\"0.01\"/>\n\
  <default>\n\
    <joint armature=\"1\" damping=\"1\" limited=\"true\"/>\n\
    <geom conaffinity=\"0\" condim=\"3\" density=\"5.0\" friction=\"1 0.5 0.5\" margin=\"0.01\" rgba=\"0.8 0.6 0.4 1\"/>\n\
  </default>\n\
  <asset>\n\
    <texture builtin=\"gradient\" height=\"100\" rgb1=\"1 1 1\" rgb2=\"0 0 0\" type=\"skybox\" width=\"100\"/>\n\
    <texture builtin=\"flat\" height=\"1278\" mark=\"cross\" markrgb=\"1 1 1\" name=\"texgeom\" random=\"0.01\" rgb1=\"0.8 0.6 0.4\" rgb2=\"0.8 0.6 0.4\" type=\"cube\" width=\"127\"/>\n\
    <texture builtin=\"checker\" height=\"100\" name=\"texplane\" rgb1=\"0 0 0\" rgb2=\"0.8 0.8 0.8\" type=\"2d\" width=\"100\"/>\n\
    <material name=\"MatPlane\" reflectance=\"0.5\" shininess=\"1\" specular=\"1\" texrepeat=\"60 60\" texture=\"texplane\"/>\n\
    <material name=\"geom\" texture=\"texgeom\" texuniform=\"true\"/>\n\
  </asset>"

BeginBody="\n\
  <worldbody>\n\
    <light cutoff=\"100\" diffuse=\"1 1 1\" dir=\"-0 0 -1.3\" directional=\"true\" exponent=\"1\" pos=\"0 0 1.3\" specular=\".1 .1 .1\"/>\n\
    <geom conaffinity=\"1\" condim=\"3\" material=\"MatPlane\" name=\"floor\" pos=\"0 0 0\" rgba=\"0.8 0.9 0.8 1\" size=\"40 40 40\" type=\"plane\"/>"

torsoArr[0]="\n\
    <body name=\"torso00\" pos=\"0 0 0.75\">\n\
      <camera name=\"track\" mode=\"trackcom\" pos=\"0 -3 0.3\" xyaxes=\"1 0 0 0 0 1\"/>\n\
      <geom fromto=\"0 "+str(-torsoLength/2)+" 0 0 "+str(torsoLength/2)+" 0\" name=\"torsoGeom00\" size=\""+str(torsoDiameter)+"\" type=\"capsule\"/>\n\
      <joint armature=\"0\" damping=\"0\" limited=\"false\" margin=\"0.01\" name=\"root\" pos=\"0 0 0\" type=\"free\"/>"

for i in range(1,N):
	torsoArr[i]="\n"+str(i*"  ")+"\
    <body name=\"torso"+str("%02d"%i)+"\" pos=\""+str(-torsoDistance)+" 0 0\">\n"+str(i*"  ")+"\
      <geom fromto=\"0 "+str(-torsoLength/2)+" 0 0 "+str(torsoLength/2)+" 0\" name=\"torsoGeom"+str("%02d"%i)+"\" size=\""+str(torsoDiameter)+"\" type=\"capsule\"/>\n"+str(i*"  ")+"\
      <joint armature=\"0.02\" axis=\"1 0 0\" damping=\"5\" name=\"torsoX"+str("%02d"%i)+"\" pos=\"0 0 0\" range=\""+str(torsoAngleXMin)+" "+str(torsoAngleXMax)+"\" stiffness=\"10\" type=\"hinge\"/>\n"+str(i*"  ")+"\
      <joint armature=\"0.02\" axis=\"0 1 0\" damping=\"5\" name=\"torsoY"+str("%02d"%i)+"\" pos=\""+str(-torsoDistance/2)+" 0 0\" range=\""+str(torsoAngleYMin)+" "+str(torsoAngleYMax)+"\" stiffness=\"10\" type=\"hinge\"/>\n"+str(i*"  ")+"\
      <joint armature=\"0.02\" axis=\"0 0 1\" damping=\"5\" name=\"torsoZ"+str("%02d"%i)+"\" pos=\"0 0 0\" range=\""+str(torsoAngleZMin)+" "+str(torsoAngleZMax)+"\" stiffness=\"10\" type=\"hinge\"/>"

for i in range(N):
	leftLegArr[i]="\n"+str(i*"  ")+"\
      <body name=\"leftLeg"+str("%02d"%i)+"\" pos=\"0 "+str(torsoLength/2)+" 0\">\n"+str(i*"  ")+"\
        <joint axis=\"0 0 1\" name=\"leftHipZ"+str("%02d"%i)+"\" pos=\"0 0 0\" range=\""+str(appendageAngleMin)+" "+str(appendageAngleMax)+"\" type=\"hinge\"/>\n"+str(i*"  ")+"\
        <geom fromto=\"0 0 0 0 "+str(appendageLength)+" 0\" size=\""+str(appendageDiameter)+"\" type=\"capsule\"/>\n"+str(i*"  ")+"\
        <body pos=\"0 "+str(appendageLength)+" 0\">\n"+str(i*"  ")+"\
          <joint axis=\"-1 0 0\" name=\"leftHipX"+str("%02d"%i)+"\" pos=\"0 0 0\" range=\""+str(thighAngleMin)+" "+str(thighAngleMax)+"\" type=\"hinge\"/>\n"+str(i*"  ")+"\
          <geom fromto=\"0 0 0 0 "+str(thighLength)+" 0\" size=\""+str(thighDiameter)+"\" type=\"capsule\"/>\n"+str(i*"  ")+"\
          <body pos=\"0 "+str(thighLength)+" 0\">\n"+str(i*"  ")+"\
            <joint axis=\"-1 0 0\" name=\"leftKnee"+str("%02d"%i)+"\" pos=\"0 0 0\" range=\""+str(legAngleMin)+" "+str(legAngleMax)+"\" type=\"hinge\"/>\n"+str(i*"  ")+"\
            <geom fromto=\"0 0 0 0 "+str(legLength)+" 0\" size=\""+str(legDiameter)+"\" type=\"capsule\"/>\n"+str(i*"  ")+"\
          </body>\n"+str(i*"  ")+"\
        </body>\n"+str(i*"  ")+"\
      </body>"
	rightLegArr[i]="\n"+str(i*"  ")+"\
      <body name=\"rightLeg"+str("%02d"%i)+"\" pos=\"0 "+str(-torsoLength/2)+" 0\">\n"+str(i*"  ")+"\
        <joint axis=\"0 0 1\" name=\"rightHipZ"+str("%02d"%i)+"\" pos=\"0 0 0\" range=\""+str(appendageAngleMin)+" "+str(appendageAngleMax)+"\" type=\"hinge\"/>\n"+str(i*"  ")+"\
        <geom fromto=\"0 0 0 0 "+str(-appendageLength)+" 0\" size=\""+str(appendageDiameter)+"\" type=\"capsule\"/>\n"+str(i*"  ")+"\
        <body pos=\"0 "+str(-appendageLength)+" 0\">\n"+str(i*"  ")+"\
          <joint axis=\"1 0 0\" name=\"rightHipX"+str("%02d"%i)+"\" pos=\"0 0 0\" range=\""+str(thighAngleMin)+" "+str(thighAngleMax)+"\" type=\"hinge\"/>\n"+str(i*"  ")+"\
          <geom fromto=\"0 0 0 0 "+str(-thighLength)+" 0\" size=\""+str(thighDiameter)+"\" type=\"capsule\"/>\n"+str(i*"  ")+"\
          <body pos=\"0 "+str(-thighLength)+" 0\">\n"+str(i*"  ")+"\
            <joint axis=\"1 0 0\" name=\"rightKnee"+str("%02d"%i)+"\" pos=\"0 0 0\" range=\""+str(legAngleMin)+" "+str(legAngleMax)+"\" type=\"hinge\"/>\n"+str(i*"  ")+"\
            <geom fromto=\"0 0 0 0 "+str(-legLength)+" 0\" size=\""+str(legDiameter)+"\" type=\"capsule\"/>\n"+str(i*"  ")+"\
          </body>\n"+str(i*"  ")+"\
        </body>\n"+str(i*"  ")+"\
      </body>"
	leftActuatorArr[i]="\
    <motor name=\"leftHipZ"+str("%02d"%i)+"\" ctrllimited=\"true\" ctrlrange=\"-1.0 1.0\" joint=\"leftHipZ"+str("%02d"%i)+"\" gear=\"150\"/>\n\
    <motor name=\"leftHipX"+str("%02d"%i)+"\" ctrllimited=\"true\" ctrlrange=\"-1.0 1.0\" joint=\"leftHipX"+str("%02d"%i)+"\" gear=\"150\"/>\n\
    <motor name=\"leftKnee"+str("%02d"%i)+"\" ctrllimited=\"true\" ctrlrange=\"-1.0 1.0\" joint=\"leftKnee"+str("%02d"%i)+"\" gear=\"150\"/>\n"
	rightActuatorArr[i]="\
    <motor name=\"rightHipZ"+str("%02d"%i)+"\" ctrllimited=\"true\" ctrlrange=\"-1.0 1.0\" joint=\"rightHipZ"+str("%02d"%i)+"\" gear=\"150\"/>\n\
    <motor name=\"rightHipX"+str("%02d"%i)+"\" ctrllimited=\"true\" ctrlrange=\"-1.0 1.0\" joint=\"rightHipX"+str("%02d"%i)+"\" gear=\"150\"/>\n\
    <motor name=\"rightKnee"+str("%02d"%i)+"\" ctrllimited=\"true\" ctrlrange=\"-1.0 1.0\" joint=\"rightKnee"+str("%02d"%i)+"\" gear=\"150\"/>\n"
    

# Remove some legs if you wish
leftLegRemoveIndices=[]
rightLegRemoveIndices=[]
for i in leftLegRemoveIndices:
	leftLegArr[i]=""
	leftActuatorArr[i]=""
for i in rightLegRemoveIndices:
	rightLegArr[i]=""
	rightActuatorArr[i]=""

EndBody="\n"
for i in range(N):
	EndBody=EndBody+str("  "*(N+1-i))+"</body>\n"
EndBody=EndBody+"  </worldbody>"

Actuators="\n  <actuator>\n"
for i in range(N):
	Actuators=Actuators+leftActuatorArr[i]+rightActuatorArr[i]
Actuators=Actuators+"  </actuator>"

# Create the file and write everything in it
file1 = open("centipede.xml","w")
file1.write(Initializer)
file1.write(BeginBody)
for i in range(N):
	file1.write(torsoArr[i])
	file1.write(leftLegArr[i])
	file1.write(rightLegArr[i])
file1.write(EndBody)
file1.write(Actuators)
file1.write("\n</mujoco>")
file1.close()