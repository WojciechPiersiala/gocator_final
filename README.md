# robot_40_gocator
This repo contains source code used in our diploma work:"Select Determining the nature of measurement errors when scanning objects with an LMI scanner Determining the nature of measurement errors when scanning objects with an LMI scanner"

## Start simulation
To start simulated robot enter : roslaunch gocator_sim sim.launch 
To start simulation with real sensor enter: roslaunch gocator_sim sim.launch use_bagfile:=false

## Start real robot
To start real robot enter : roslaunch gocator_sim real.launch
To use real data recorded from bagfile use roslaunch gocator_sim real.launch use_bagrile:=true
