Project 2, deliverable 2 - Alexandria Leto 

To run the code, unpack the D1_alex_leto.tar file in your catkin_ws/src.

The q_table that resulted from the training of the robot was saved as a
pickle. The code that executes the robot's actions loads the table from the
pickle. Therefore, this file, which is named "q_table_pickle" and is
currently located in the stingray-simulation package, must be moved to the
/home/[username]/.ros directory. 

Then, run the following commands:

      	catkin_make
	source /opt/ros/melodic/setup.bash
	source ~/Stingray-Simulation/catkin_ws/devel/setup.bash
	source ~/Stingray-Simulation/stingray_setup.py
	roslaunch stingray-sim run_sim.launch

Because of countless technical difficulties and having to start over on this
deliverable because my VM crashed, I was unable to come up with a solution
that was even close to correct. When I ran the program that is supposed to
execute actions from the q table, the robot simply spun in place. Because
this would make an extremely underwhelming video, I haven't included it. 

However, my training and execution codes ("train.py" and "run_sim.py") are
included in the stingray-sim/src directory. The launch files for each are
included in the stingray-sim/launch directory. 

It seemed that the issue with my training was that not a lot of the q table
was filled in. Given more time, I may have tried to set the robot's position
to random in order to encourage encountering more states. Further, the lab
write-up mentioned that the stop/start_physics services may be helpful. I may
have figured out how. 
