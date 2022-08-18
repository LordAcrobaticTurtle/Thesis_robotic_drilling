# Controller Setup instructions
1. https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers/issues/15
2. Open RQT and get TF frame tree. 
3. Create new .yaml file for cartesian compliance controller. 
   1. Follow example files
   2. Sensor link should be on the "flange" frame
4. Create .launch file
   1. This launch file loadds the controller configuration to the parameter server  
5. Confirm the changes have been loaded by using rosparam get --parameter_name--



Confirm it is subscribed to the right /wrench topic
Using the remap keyword inside the launch file, change the compliance controller to listen to the wrench topic.


Confirm it is publishing to the right pose topic. 

Be wary of the error_scale value. Larger values will cause angry vibrations


