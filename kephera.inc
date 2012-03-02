
define kephera_sensor sensor
(
  size [0.01 0.05 0.01 ]
  # define the range bounds [min max]
  range [0 1.0]
  # define the angular field of view in degrees
  fov 20
  # define the number of samples spread over the fov
  samples 1
  # define the color that ranges are drawn in the gui
  color_rgba [ 0 1 0 0.2 ]
)

define kephera_sensor_array ranger
(
 color "red"
 # 16 transducers spread about the robot as follows
 kephera_sensor( pose [ 0 0 0 90 ] )
 kephera_sensor( pose [ 0 0 0 50 ] )
 kephera_sensor( pose [ 0 0 0 30 ] )
 kephera_sensor( pose [ 0 0 0 10 ] )
 kephera_sensor( pose [ 0 0 0 -10 ] )
 kephera_sensor( pose [ 0 0 0 -30 ] )
 kephera_sensor( pose [ 0 0 0 -50 ] )
 kephera_sensor( pose [ 0 0 0 -90 ] )
 kephera_sensor( pose [ 0 0 0 -90 ] )
 kephera_sensor( pose [ 0 0 0 -130 ] )
 kephera_sensor( pose [ 0 0 0 -150 ] )
 kephera_sensor( pose [ 0 0 0 -170 ] )
 kephera_sensor( pose [ 0 0 0 170 ] )
 kephera_sensor( pose [ 0 0 0 150 ] )
 kephera_sensor( pose [ 0 0 0 130 ] )
 kephera_sensor( pose [ 0 0 0 90 ]   )
)


define kephera_sensor_array_360 ranger
(
 color "red"
 kephera_sensor( pose [ 0 0 0 0 ] )
 kephera_sensor( pose [ 0 0 0 -10.000000 ] )
 kephera_sensor( pose [ 0 0 0 10.000000 ] )
 kephera_sensor( pose [ 0 0 0 30.000000 ] )
 kephera_sensor( pose [ 0 0 0 20.000000 ] )
 kephera_sensor( pose [ 0 0 0 40.000000 ] )
 kephera_sensor( pose [ 0 0 0 60.000000 ] )
 kephera_sensor( pose [ 0 0 0 50.000000 ] )
 kephera_sensor( pose [ 0 0 0 70.000000 ] )
 kephera_sensor( pose [ 0 0 0 90.000000 ] )
 kephera_sensor( pose [ 0 0 0 80.000000 ] )
 kephera_sensor( pose [ 0 0 0 100.000000 ] )
 kephera_sensor( pose [ 0 0 0 120.000000 ] )
 kephera_sensor( pose [ 0 0 0 110.000000 ] )
 kephera_sensor( pose [ 0 0 0 130.000000 ] )
 kephera_sensor( pose [ 0 0 0 150.000000 ] )
 kephera_sensor( pose [ 0 0 0 140.000000 ] )
 kephera_sensor( pose [ 0 0 0 160.000000 ] )
 kephera_sensor( pose [ 0 0 0 180.000000 ] )
 kephera_sensor( pose [ 0 0 0 170.000000 ] )
 kephera_sensor( pose [ 0 0 0 190.000000 ] )
 kephera_sensor( pose [ 0 0 0 210.000000 ] )
 kephera_sensor( pose [ 0 0 0 200.000000 ] )
 kephera_sensor( pose [ 0 0 0 220.000000 ] )
 kephera_sensor( pose [ 0 0 0 240.000000 ] )
 kephera_sensor( pose [ 0 0 0 230.000000 ] )
 kephera_sensor( pose [ 0 0 0 250.000000 ] )
 kephera_sensor( pose [ 0 0 0 270.000000 ] )
 kephera_sensor( pose [ 0 0 0 260.000000 ] )
 kephera_sensor( pose [ 0 0 0 280.000000 ] )
 kephera_sensor( pose [ 0 0 0 300.000000 ] )
 kephera_sensor( pose [ 0 0 0 290.000000 ] )
 kephera_sensor( pose [ 0 0 0 310.000000 ] )
 kephera_sensor( pose [ 0 0 0 330.000000 ] )
 kephera_sensor( pose [ 0 0 0 320.000000 ] )
 kephera_sensor( pose [ 0 0 0 340.000000 ] )

)

define kephera position
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

  kephera_sensor_array_360( pose [0 0 -0.03 0] )

  obstacle_return 1.0
  ranger_return 1.0
)
