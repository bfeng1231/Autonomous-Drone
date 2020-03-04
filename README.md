# Autonomous-Drone
 CMPEN 473 project to confine a drone in an area

This project was done in C on a Raspberry pi 3. A drone and camera were provided. The goal of this project was to make the drone be controlled manually using keyboard inputs and create a restricted border control with the help of a camera and LEDs. If the drone flew out of the view of the camera, the drone would automataclly fly back into the view of the camera.
Other features included were a map to display the position of the drone, the direction and speed the drone was moving, and the
direction the drone was facing. Image processing was a big part of this project. It was used to find the location of the drone and track it's movements.
The program was multi-threaded in order to manage the camera and the drone. The work that I did is located in the hw10feng.c file. The other files are libraries provided to help us design our program.
