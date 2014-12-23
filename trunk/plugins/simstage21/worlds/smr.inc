
# Desc: Device definitions for "Small Mobile Robot" from DTU
# Author: Lars Valdemar Mogensen
# Date: 3 Oct 2008
# CVS: $Id: smr.inc,v 1.30 2008-01-15 01:25:42 rtv Exp $

# The AUT Sharp IR distance sensor
define AUT_irSensor ranger
(
  scount 5 # the number of IR sensors

  # define the pose of each transducer [xpos ypos heading]
  spose[0] [ 0.135  0.135  90 ]
  spose[1] [ 0.14   0.09    0 ]
  spose[2] [ 0.14   0.00    0 ]
  spose[3] [ 0.14  -0.09    0 ]
  spose[4] [ 0.135 -0.135 -90 ]
    
  # define the field of view of each IR sensor [range_min range_max view_angle]
  sview [0.1 0.8 15]

  # define the size of each transducer [xsize ysize] in meters
  ssize [0.02 0.02]
)

define smr_base position 
(
  color "grey"			# Default color.
  drive "diff"		 	# Differential steering model.
  gui_nose 1                  	# Draw a nose on the robot so we can see which way it points
  obstacle_return 1           	# Can hit things.
  laser_return 1                 # reflects laser beams
  ranger_return 1             	# reflects sonar beams
  blobfinder_return 1         	# Seen by blobfinders  
  fiducial_return 1           	# Seen as "1" fiducial finders

  localization "gps"
  localization_origin [0 0 0] 	# Start odometry at (0, 0, 0).

  # alternative odometric localization with simple error model
  # localization "odom"         	# Change to "gps" to have impossibly perfect, global odometry
  #  odom_error [ 0.05 0.05 0.1 ]   	# Odometry error or slip in X, Y and Theta
                                   	# (Uniform random distribution)   
)


define smr smr_base
(
  # actual size
  size [0.37 0.28 0.38] # sizes from MobileRobots' web site

  # the pioneer's center of rotation is offset from its center of area
  origin [0.095 0 0 0]

  # draw a nose on the robot so we can see which way it points
  gui_nose 1

  # estimated mass in KG
  mass 8.0 

  # use the sonar array defined above with a small vertical offset to
  # drop the sensors into the robot body
  # p2dx_sonar( pose [0.04 0 -0.03 0] ) 

  # use the URG laser scanner
  URGlaser( pose [0.255 0 -0.37 0] )

  # use the AUT IR distance sensor
  AUT_irSensor( pose [0.08 0 -0.20 0])

  # differential steering model
  drive "diff"
  
  # this polygon approximates the shape of a pioneer
  blocks 10

  # Top part body
  block[0].points 4
  block[0].point[0] [  0.14  0.14 ]
  block[0].point[1] [ -0.14  0.14 ]
  block[0].point[2] [ -0.14 -0.14 ]
  block[0].point[3] [  0.14 -0.14 ]
  block[0].z [0.07 0.29]

  # Bottom part body
  block[1].points 4
  block[1].point[0] [  0.14  0.10 ]
  block[1].point[1] [ -0.14  0.10 ]
  block[1].point[2] [ -0.14 -0.10 ]
  block[1].point[3] [  0.14 -0.10 ]
  block[1].z [0.04 0.07]

  # URG laser scanne housing
  block[2].points 4
  block[2].point[0] [  0.20  0.03 ]
  block[2].point[1] [  0.14  0.03 ]
  block[2].point[2] [  0.14 -0.03 ]
  block[2].point[3] [  0.20 -0.03 ]
  block[2].z [0.095 0.10]

  # Camera stand
  block[3].points 4
  block[3].point[0] [ -0.14 -0.02 ]
  block[3].point[1] [ -0.17 -0.02 ]
  block[3].point[2] [ -0.17 -0.05 ]
  block[3].point[3] [ -0.14 -0.05 ]
  block[3].z [0.06 0.38]

  # Camera house
  block[4].points 4
  block[4].point[0] [ -0.14  0.015 ]
  block[4].point[1] [ -0.17  0.015 ]
  block[4].point[2] [ -0.17 -0.015 ]
  block[4].point[3] [ -0.14 -0.015 ]
  block[4].z [0.41 0.44]
  block[4].color "red"

  # Camera lense
  block[5].points 4
  block[5].point[0] [ -0.11  0.015 ]
  block[5].point[1] [ -0.14  0.015 ]
  block[5].point[2] [ -0.14 -0.015 ]
  block[5].point[3] [ -0.11 -0.015 ]
  block[5].z [0.41 0.44]
  block[5].color "black"

  # Left back wheel
  block[6].points 4
  block[6].point[0] [ -0.065 0.14 ]
  block[6].point[1] [ -0.125 0.14 ]
  block[6].point[2] [ -0.125 0.11 ]
  block[6].point[3] [ -0.065 0.11 ]
  block[6].z [0 0.065]
  block[6].color "black"

  # Right back wheel
  block[7].points 4
  block[7].point[0] [ -0.065 -0.11 ]
  block[7].point[1] [ -0.125 -0.11 ]
  block[7].point[2] [ -0.125 -0.14 ]
  block[7].point[3] [ -0.065 -0.14 ]
  block[7].z [0 0.065]
  block[7].color "black"

  # Left front wheel
  block[8].points 4
  block[8].point[0] [  0.14  0.13 ]
  block[8].point[1] [  0.08  0.13 ]
  block[8].point[2] [  0.08  0.11 ]
  block[8].point[3] [  0.14  0.11 ]
  block[8].z [0 0.065]
  block[8].color "black"

  # Right front wheel
  block[9].points 4
  block[9].point[0] [  0.14  -0.11 ]
  block[9].point[1] [  0.08  -0.11 ]
  block[9].point[2] [  0.08  -0.13 ]
  block[9].point[3] [  0.14  -0.13 ]
  block[9].z [0 0.065]
  block[9].color "black"

)
