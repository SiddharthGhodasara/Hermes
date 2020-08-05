# Hermes (An Autonomous Mobile Robot)

Hermes (named after the eponymous Greek God of travel and navigation) is a differential drive robot that uses SLAM (Simultaneous Localization And Mapping) to create a map of its surroundings and navigate within said map. The map is created using a LIDAR (Light Detection And Ranging) and encoders built into the motors help it to localize in the map. The robot can plan a path to a given destination and drive to it while avoiding obstacles. In addition to this, it has an exploration feature, that detects frontiers and sends goals to the robot until it has completely explored an area and created a comprehensive map of its surroundings. 

The robot uses a micro-controller to read encoder values and calculate raw velocities of the robot which is then sent to an on board computer through a serial interface where the robot's odometry is calculated by processing the raw velocities. This is then sent to remote desktop over Wi-Fi which then carries out map building and navigation. The desktop in turn sends velocity commands which are sent to the micro-controller through the onboard computer.    

------

#### Hardware Requirements

- Odroid XU4 with ROS installed (or any other single board computer with at least 1 GB memory)
- RPLidar (or any other laser scanner which can be interfaced with ROS)
- Teensy/Arduino (or any other equivalent micro-controller)
- Geared 12V DC Motors with Encoders 
- Cytron MDD3A motor driver (or any other motor driver)
- Power source (Lipo battery for motors and DC rectification unit for AC-DC conversion)

It's better to have separate power sources for the motors and the single board computer to ensure a clean supply to the latter. 

#### Software Requirements

- Robot Operating System 
- Arduino IDE (or any other equivalent IDE)

------

#### Mapping and Path Planning

In its current state the robot uses Gmapping to create a 2D map. As in any autonomous robot, there are two path planners: A global planner and a local planner. The Global planner is responsible for finding the main path to reach the desired destination, while the local planner is responsible for modifying the global path determined by the global planner, to account for any environmental changes in the vicinity of the robot. The global planner used here is Navfn and Trajectory Planner is used as the local planner. To get the robot to maneuver reasonably well around obstacles and in tight spaces I had to spend most of my time tweaking the move_base parameters. 

The following video shows the robot's dynamic obstacle avoidance. 
<video src="videos/DynamicObstacleAvoidance.mp4" width="720" height="480" controls preload></video>



Take a look at the following video to see the robot navigating to reach a desired goal:-
<video src="videos/Navigation.mp4" width="720" height="480" controls preload></video>



------

#### Autonomous Map Exploration

As mentioned earlier, the robot can autonomously explore its surroundings. This is particularly helpful when the map to be explored is large and manual teleoperation of the robot to create a map becomes arduous. 
<video src="videos/Frontier Exploration.mp4" width="720" height="480" controls preload></video>

​    

------

#### Applications  

AMRs are being widely used in warehouse or manufacturing operation where traditional material handling automation has not yet proliferated. 

AMRs have a great scope to be used in the mining industry as well. Mining laws require that operational mines conduct regular service inspections. These inspections ensure safe ventilation and operations. After a mine’s life is over, many laws demand rehabilitation, but inspections and duties associated with rehabilitation often are overlooked. Abandoned mines grow more hazardous as time passes for personnel to enter and conduct surveys. Navigating confined spaces with limited visibility is a better job for a mining robot. 

Agriculture is a $5 trillion industry, and it’s ripe for automation. An AMR that explores crop fields, monitors crop health by analyzing data (through the use of cameras), disperses pesticide and takes respective action on them is hugely desirable and would play an instrumental role in the agriculture sector.

------

#### Future work

- I am currently working on fusing an IMU along with encoders to improve odometry of the robot. ROS has a standardized way of doing this through Kalman filters.

- I will be re-structuring the current robot to better navigate the rough terrain and make it more robust by adding a 3D map building capability using stereo cameras.    

------

#### Current limitations

One of the biggest limitations with AMRs today is that the technology is still in its adolescent stages. As a result, if an AMR gets stuck in a scenario or while performing an operation, customers are likely to feel that their workload has been increased as a result of babysitting the robot, rather than simplifying things as was originally intended. While this is certainly true, I feel that this would certainly be a driving force for developers to refine their products and there will soon come a time when we have products that perform the job assigned to them remarkably well. I personally had to babysit my AMR until it was smart enough not to crash. I have put in a good amount of effort in building this robot and have spent lot of time in fine tuning the move_base parameters. Even though this is not ready for the industry yet, I am confident that soon it will conform with the desired industry standards.  

------

#### My journey so far..

I originally set out to build an AMR using a standalone micro-controller, like the Arduino, but quickly realized that a micro-controller was just a tool, and what I needed was a system that was able to provide services like communication between nodes, carry out complex calculations like path planning, SLAM and the ability to integrate many other packages with it. The micro-controller simply does not have the computational power and memory to do this. Even if it had the computational ability, it would be a time consuming task to make such libraries exclusively for the micro-controller. It would also mean reinventing the wheel as not all code is compatible with every single micro-controller out there. And so after a quick but exhaustive search, I stumbled upon ROS (Robot Operating System). There were a few other alternatives to ROS:-

- Mobile Robot Programming Toolkit (specific to mobile robotics) (https://www.mrpt.org/)
- Microsoft Robotics Developer Studio (https://www.microsoft.com/en-in/download/details.aspx?id=29081)
- ZeroMQ (https://zeromq.org/)

But ROS seemed like the goto choice specifically because of its massive online community. I have been learning ROS, ever since. This project is the sole reason I first came across ROS, and therefore is precious to me. 

  

