# Implementing a Particle Filter Algorithm in Ros2
### Computational Robotics 2025, Robot Localization Project
Brooke Wager and Kelsey McClung

## Project Goal
The goal of this project is to localize a robot within a known map space by implementing a particle filter algorithm. The particle filter uses the robot’s odometry and lidar data to estimate the robot’s pose as it moves within a space. Particles act as hypothesis positions of the robot, and are updated to move as the robot does, relative to their starting pose. After each movement, the lidar measurements from the robot in the physical space and the particles in the map space are compared to measure the confidence of each particle. High confidence particles are resampled and continue to move with the robot. Low confidence particles are dropped, until particles eventually converge at the robot’s most probable location. Given skeleton code, we completed implementation of the particle filtering algorithm and adjusted parameters to achieve consistent localization accuracy. This report summarizes our method of implementation, and the challenges and learnings that arose during this project.

## Implementing the Particle Filter
To implement our particle filter, we follow these broad steps:
### Generate Particles:
To initialize our particle cloud, we generated 300 particles around a given starting location. The x, y, and theta values making up the pose of each particle are randomized on a Gaussian scale centered around the given location.
### Weigh Particles:
In order to weigh our particles by confidence, we compare the shortest distance between the robot and an object in the (simulated) physical space, with the shortest distance between each particle and an object in the map space. We cast a lidar measurement out from each particle in the map frame to find its shortest distance to an obstacle. 

Initially, we had planned to compare not only the shortest distance to an obstacle between robot and particles, but also the bearing to that obstacle on the lidars 360 degree range. However, we found by testing the algorithm using only the shortest distance, that we received acceptable results. This surprised us, but we realized that by updating the particle's position based on the robot’s movement, particles with inaccurate headings would be quickly dropped, as they would be moving in the wrong direction.

The weight of each particle is determined by:

![](https://github.com/b-wager/robot_localization/blob/main/images/Screenshot%20from%202025-10-22%2020-57-43.png)

Particle weights are then normalized so they sum to 1. Particles with higher weights are more likely to live on when we resample all our particles.
### Update Particle Positions
Each time the robot moves, particle positions are updated to match that movement relative to the initial heading of the particle. Noise on a gaussian distribution is added to both the x, y, and theta transformation of particles to more accurately capture the uncertainty in the robot’s odometry measurements. By updating our particles with the robot’s movement, we can continue resampling particles with a high confidence weight, and re-weigh them as the robot’s position changes.

![](https://github.com/b-wager/robot_localization/blob/main/images/Movement_update.png)

The figure above represents how a movement update from the robot (blue) causes a movement update in a particle (red). The particle follows the same transformation as the robot, with the linear and angular movement relative to the particle’s initial heading. This means that particles with different theta values than the robot will move in a different direction than the robot in the global frame, allowing these particles with inaccurate thetas to be filtered out when the map data starts to differ from the laser data.

### Resample Particles
Particles are randomly sampled with `numpy.random.choice()`, where particles with higher weight are more likely to be resampled. This means that as the robot continues to move, and improbable particles are updated to have lesser weights, they will be dropped, while particles that keep high weights each time they are sampled will continue to be resampled. Thus, as the robot continues moving, particles converge and give more accurate estimations.

We also resample 30% of our particle cloud with completely random particles. These particles are generated the same way as the particles we originally generated, except these are centered around the current odometry of the robot.

## Design Decisions
The first design decision we made was how to approach this project. We decided to use the entirety of the reference framework provided for this project to write our code. This is because at the beginning, we really did not understand the particle filter and were daunted by the project. We decided to focus first on achieving a solid conceptual understanding, and the template provided a good starting direction. The template and helper files also allowed us to focus on particle filter implementation rather than trying to figure out how to access map, laser, and odometry data, since that was already done for us. By using the framework, we were able to write out and understand fully each step of the particle filter.

One of the more significant design decisions we made while coding our particle filter was how to calculate particle weights. We asked Victoria about the recommended ways to do this, and we ended up choosing the most computationally basic option of comparing the shortest distance to an obstacle between each particle and the robot (with the robot using lidar data on it’s surroundings, and particles using simulated lidar data within the map space). However, we liked the idea of also considering bearing in the particle calculation to provide a more accurate theta estimate. The third option, which we found to be much more computationally intensive, is comparing multiple distances from several lidar vectors between robot and particles. This might provide increased localization accuracy, but we knew from previous projects that it is not necessary for a successful particle filter. In the end, we choose the first method, using shortest distance only, for its simplicity.

Another design decision we made regarded how to choose the estimated robot position based on particle data. Originally, we thought the particle with the highest weight would make the best estimate, but we quickly realized that may not be the case due to lidar error and new, randomly generated particles included in each resample. We switched to using the weighted average of the resampled particles, where the weights affected how much each particle influenced the estimated position. Only resampled particles contributed to the weighted average to remove bias from any lucky new particles. We found the estimated robot position ended up being suitably accurate using this method.

## Challenges
Our biggest challenge was running the bag files to test our particle filter in the MAC first floor map. For a while we were not able to get the bag files to run, due to a typo in our command text. Before fixing this issue (thank you, Vivian!), we were only able to test our particle filter in the Neato Gazebo guantlet_final.world map using teleops. This limited our understanding of the particle filter’s accuracy, as we could only test it in a simple digital environment, so we were delighted when we were finally able to run our filter on the `mac_1st_floor_9_23.yaml` map using the bag files.

Once this error was solved, we ran into another treacherous issue. We hadn’t written any code to get map boundary data from the map files. We had hardcoded in map boundaries at (0,0), where our particles can only exist at positive x and y values. When we ran our filter on the MAC map, it was as if an invisible wall blocked our particles from moving past the x=0 and y=0 lines. Once we understood the problem, this was an easy fix. It did however make a good example of the consequences of assumptions about our skeleton code. 
	
## Future Work
Our biggest gripe with our final particle filter is that the position estimation is pretty accurate, but our theta estimate is not. This is most likely because we are not validating our theta value with LIDAR data the same way we are the x and y. One way to possibly fix this is to assume the robot is always traveling forward. This means that whenever positional odometry updates, we set the robot to be facing the same direction it moved. However, this strategy is assuming the robot only moves forward, which is not always the case. Another option is to include bearing in our weight calculation as described below.

While we are happy with the results of our particle filter using only shortest distance to an obstacle to determine particle weight, we would like to see how much of a difference including bearing in the particle weight calculation would make. It would be interesting to see how this change could affect the accuracy of the algorithm, and to play around with how to balance the comparisons of linear distance lidar angle in our particle weight equation. We would try something like the equation below, where  and  are parameters to adjust the weight of the linear distance and angle. 

![](https://github.com/b-wager/robot_localization/blob/main/images/Screenshot%20from%202025-10-22%2020-58-09.png)

## Lessons Learned
This assignment gave us an opportunity to write code to add to an existing, unfinished program. We were able to add missing pieces, and reconfigure or delete parts of the existing code to fit our needs. This presented an exciting challenge, as we needed to really understand what we were working with to avoid errors and wasted time spent re-doing existing work. We learned to be careful with our assumptions about data types and the input and output of functions. Our mistake of not subscribing to the map really drove this lesson home for us. This lesson prepares us for future collaborations where we may join in ongoing work with existing code, and even personal projects where we might use existing code from open sources to build our own work off of.
We also learned an appreciation of the power of simple computations repeated to achieve a complex goal. We found this particle filter to be quite a challenge, but the computation to weigh particles is very simple. We only use one metric to compare between the actual robot and each particle. Given this, we were impressed to see our particle filter run as well as it did, though it is certainly not flawless.

![](https://github.com/b-wager/robot_localization/blob/main/images/demo.gif)

The above video is an example of our particle filter working. What is interesting is that it is able to very quickly converge and is in a fairly accurate position until it reaches the big nook, in which case some of the particles diverge and two main clusters appear instead of 1. However, when the robot is relatively close to a wall again, the particle filter re-converges and returns to a fairly accurate position.
