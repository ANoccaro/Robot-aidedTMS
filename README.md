# Robot-aidedTMS

** FlangiaPanda_coilAlpha40_partA, FlangiaPanda_coilAlpha40_partB : 3D model of the flange used to connect the alpha coil 40 mm to the panda robot end-effector. The two parts can be printed using a 3D printer. Suggested settings for the printer: PLA material, 70% filling density, grid filling.

** TMS_pandaNeuronavigator: Qt c++ project to control a robot-aided TMS platform composed of a Panda robot and a generic neuronavigator that streams to thirdy-parts software the pose of the coil, head and hot-spot.
The project is composed of:
  - a main file that runs the application
  - a mainwindow file that implements the function and the connections
  - a ui file that defines the GUI settings
  - a library to control the panda robot
  - a library to receive data through UDP (e.g. the camera/neuronavigator data)
