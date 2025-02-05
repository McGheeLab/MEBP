# Microscope Enabled 3D Bioprinter (ME3B)


# Device Control Code
## Stage control
This class contains all code needed to communicate with the microscope stage using the ProScan III controller.

## To Do list
  - [x] Initalize 
  - [x] Send Data
  - [x] Recieve Data
  - [x] Get Properties
  - [x] Send Move Command
  - [x] Send Jog Command
  
## Printerhead control
This class contains all code needed to communicate with the Printer head via Marlin G-Code 

## To Do list
  - [x] Initalize 
  - [x] Send Data
  - [x] Recieve Data
  - [x] Get Properties
  - [x] Send Move Command

# Image based path planner class
## Description
This class takes in a stack of images each with 3 colors R,B,G corresponding to the 3 pumps R = Pump 1, B = Pump 2, G = Pump 3

## Tasks todo
  - [x] Initalize
  - [x] Read Image Stack
  - [x] Set microns per pixel
  - [ ] Set volume per pixel based on color intensity and Z spacing
  - [x] Generate toolpath
  - [x] Merge toolpath layers
  - [x] save waypoints to file
  - [x] return waypoints directly
  - [x] plot toolpath

# Print setup 
## Description
This class handels all of the planning needed to move the needle to each well, print into the well, pickup ink, wash etc

## Tasks todo
- [ ] Initalize
  - [ ] manditory * well type, needle type, syringe setup
- [ ] Store each well print plan into an array of objects
- [ ] identify the amount of bio-ink needed for each part of the print and go pick it up 
  - [ ] limit the amount so nothing goes into the syringe line based on which needle is attached
- [ ] create a Well plate based on an xml file that describes the well spacing and offsets
- [ ] using a camera find the xyz location of the needle tip and set it as 0,0,0
- [ ] calculate the print offset needed
- [ ] handle the ink locations and types
- [ ] 


# Object based path planner class
## Description 
This class inferences object types from a folder of object types. ie disk, cylinder, sphere etc which the user can place into the microwell at any location. The class should keep track of all objects placed in the environment and build a sequential path each time a new object is placed in. 

## Tasks todo
- [ ] Initalize
- [ ] Import object Library
- [ ] Simulate the 3D microwell environment with all objects 

# Single cell identification and extraction 
## Description 
This class stores the 2D or 3D location of cell features of interest then moves the needle to the correct X,Y,Z location for extraction. It should then tag each cell with the meta data and bin it into some pre determined well location. 

## Tasks todo
- [ ] Initalize
- [ ] Find cells by some feature set 
- [ ] determine XYZ location
- [ ] Move to location
- [ ] Offset needle based on which needle bore

# GUI to setup and monitor the print progress 
## Description
This set of functions will setup the print process and display all relavent information such as video feeds, syringe pump positions, xy stage position, cell types, print layout etc. Any class created above can be instanced. 

