# Robot-aidedTMS

** FlangiaPanda_coilAlpha40_partA, FlangiaPanda_coilAlpha40_partB : 3D model of the flange used to connect the alpha coil 40 mm to the panda robot end-effector. The two parts can be printed using a 3D printer. Suggested settings for the printer: PLA material, 70% filling density, grid filling.

** TMS_pandaNeuronavigator: Qt c++ project to control a robot-aided TMS platform composed of a Panda robot and a generic neuronavigator that streams to thirdy-parts software the pose of the coil, head and hot-spot.
The project is composed of:
  - a main file that runs the application
  - a mainwindow file that implements the function and the connections
  - a ui file that defines the GUI settings
  - a library to control the panda robot
  - a library to receive data through UDP (e.g. the camera/neuronavigator data)

** User Manual: a guide that describes step by step how to setup the devices and run a TMS session using the main application. The guide includes a description of the user interface and the istructions to use it correctly.

** Robot_TMS_API DOC: documentation of the methods implemented and used in the code.

** Robot_TMS_UML_code: block diagram that describes the structure of the code, referring also to the user interface

** SupplementaryMaterials: supplementary materials of the paper: A. Noccaro et al. “Development and validation of a novel calibration methodology and control approach for robot-aided transcranial magnetic stimulation (TMS),” 2020, under review in IEEE Transaction on Biomedical Engineering.
