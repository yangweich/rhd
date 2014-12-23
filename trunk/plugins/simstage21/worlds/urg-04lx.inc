
define urg04lx laser
(
  # laser-specific properties

  # factory settings for Hokuyo URG-04LX	
  range_min 0.02
  range_max 4.00
  fov 240.0
  samples 667

  # generic model properties
  color "grey"
  size [ 0.050 0.050 0.071 ] # dimensions from Hokuyo URG-04LX data sheet	
)

define URGlaser urg04lx
(
  blocks 2

  # bottom
  block[0].points 4
  block[0].point[0] [  0.025  0.025 ]
  block[0].point[1] [ -0.025  0.025 ]
  block[0].point[2] [ -0.025 -0.025 ]
  block[0].point[3] [  0.025 -0.025 ]
  block[0].z [0 0.041 ]
  block[0].color "grey"

  # laser house
  block[1].points 8
  block[1].point[0] [  0.02    0.00   ]
  block[1].point[1] [  0.0141  0.0141 ]
  block[1].point[2] [  0.0     0.02   ]
  block[1].point[3] [ -0.0141  0.0141 ]
  block[1].point[4] [ -0.02    0.0    ]
  block[1].point[5] [ -0.0141 -0.0141 ]
  block[1].point[6] [  0.0    -0.02   ]
  block[1].point[7] [  0.0141 -0.0141 ]
  block[1].z [0.041 0.071 ]
  block[1].color "black"
)



