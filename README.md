# Particle Filter

Video of particle filter demo: https://www.youtube.com/watch?v=pyXoEt8-hL4&feature=youtu.be

Particle filter used for tracking targets using sound observations containing azimuth and elevation bearing measurements. Parameters have not been tuned for 3D target tracking, however, these parameters can be determined experimentally and changed in particle_filter_main.cc. The sound observation messages used in this implementation is proprietry software and therefore cannot be uploaded. However, this particle filter should work with any bearing-only measurement in 2D or 3D.

Please contact the author for links to the bag files.

# Code Layout

Detailed description are available in individual files.

* particle_filter/src/nlos_particle_filter/nodes/particle_filter_main.cc: This is the main ROS node that initialises the particle filter, sets the parameter and subscribes to robot pose and sensor observation topics.
* particle_filter/[src]|[include]/nlos_particle_filter/class_files/*: 
    * landmark_moments: Struct to store mean and covariance (first and second statistical moments) of converged particles.
    * particle_filter: Class for generating, storing and resampling particles.
    * particle_parameters: Struct for storing relevant parameters for filter. Allows a single object containing all necessary parameters to be passed into the particle filter.
    * particles_class: Randomly generates particles in specified region around the robot and stores the particles and their properties.
    * robot_pose: Class for storing robot pose and providing various accessor and mutator methods for getting/setting the orientation in different formats (euler/quaternion).

# Building and Running the code

The sensor msg (currently using a placeholder name: sensor_msg::SensorMsg) needs to be changed to a sensor message which contains an azimuth and elevation bearing angle inside the main node (particle_filter_main.cc). The particle filter parameters should also be set. A launch file can be created to set the parameters on lines 36:43 without recompiling the ROS package or they can be set within the source file.

Package should be placed withing your catkin workspace: e.g. ~/catkin_ws/src/particle_filter. Move to the root of the workspace and build with catkin_ws.
