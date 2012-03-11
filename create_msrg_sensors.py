import sys

coverage = float(sys.argv[1])
fov = float(sys.argv[2])
Rp = float(sys.argv[3])

print """
define msrg_sensor sensor
(
  size [0.01 0.05 0.01 ]
  # define the range bounds [min max]
"""
print "  range [0 %f]" % Rp
print """
  # define the angular field of view in degrees
  # define the number of samples spread over the fov
  samples 1
  # define the color that ranges are drawn in the gui
  color_rgba [ 0 1 0 0.2 ]
)
"""

print "define msrg_sensor_array ranger ("
for i in range(0, int(coverage/fov)):
  print '  msrg_sensor( pose [ 0 0 0 %f ]' % (i*fov)
  print '    fov %f )' % fov
  for j in range(0, int(fov+1)):
    print '  msrg_sensor( pose [ 0 0 0 %f ]' % (i*fov-fov/2+j)
    print '    fov 0)'

print ")"


print """
define msrg position
(
  color "blue"
  block
  (
    points 9
    point[0] [ -0.075 -0.025 ]
    point[1] [ -0.075 0.025 ]
    point[2] [ -0.06 0.05 ]
    point[3] [ 0 0.065 ]
    point[4] [ 0.040 0.045 ]
    point[5] [ 0.055 0 ]
    point[6] [ 0.040 -0.045 ]
    point[7] [ 0 -0.065 ]
    point[8] [ -0.06 -0.050 ]
    z [ 0 0.24 ]
  )

  msrg_sensor_array( pose [0 0 -0.03 0] )

  obstacle_return 1.0
  ranger_return 1.0
)
"""


