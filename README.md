# Control and path planning for a centipede-like robot

This repository contains the following:
- centipede_xml_generator.py : MuJoCo model generator for a centipede with 'N' pairs of legs. 'N' and other physical parameters can be modified here as desired. Run this script to generate an '.xml' model file for MuJoCo.
- centipede_env.py : MuJoCo environment for the centipede. The controller can be modified here in the function 'get_action'.
- centipede_simulator.py : Run this script to simulate the centipede.

Other requirements:
- The MuJoCo key file (mjkey.txt) needs to be placed in this folder in order to run the MuJoCo environment. If you do not have a key, please download it from [this webpage](https://www.roboti.us/license.html).
