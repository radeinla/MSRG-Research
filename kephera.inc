
define kephera_sensor sensor
(
  size [0.01 0.05 0.01 ]
  # define the range bounds [min max]
  range [0 2.0]
  # define the angular field of view in degrees
  fov 30
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
################
 kephera_sensor( pose [ 0 0 0 0.000000 ] )
 kephera_sensor( pose [ 0 0 0 -10.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 -9.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 -8.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 -7.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 -6.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 -5.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 -4.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 -3.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 -2.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 -1.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 1.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 2.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 3.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 4.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 5.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 6.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 7.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 8.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 9.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 10.000000 ]
   fov 0 )
################
 kephera_sensor( pose [ 0 0 0 20.000000 ] )
 kephera_sensor( pose [ 0 0 0 10.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 11.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 12.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 13.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 14.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 15.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 16.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 17.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 18.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 19.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 21.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 22.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 23.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 24.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 25.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 26.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 27.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 28.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 29.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 30.000000 ]
   fov 0 )
################
 kephera_sensor( pose [ 0 0 0 40.000000 ] )
 kephera_sensor( pose [ 0 0 0 30.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 31.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 32.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 33.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 34.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 35.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 36.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 37.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 38.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 39.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 41.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 42.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 43.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 44.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 45.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 46.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 47.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 48.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 49.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 50.000000 ]
   fov 0 )
################
 kephera_sensor( pose [ 0 0 0 60.000000 ] )
 kephera_sensor( pose [ 0 0 0 50.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 51.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 52.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 53.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 54.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 55.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 56.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 57.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 58.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 59.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 61.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 62.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 63.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 64.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 65.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 66.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 67.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 68.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 69.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 70.000000 ]
   fov 0 )
################
 kephera_sensor( pose [ 0 0 0 80.000000 ] )
 kephera_sensor( pose [ 0 0 0 70.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 71.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 72.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 73.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 74.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 75.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 76.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 77.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 78.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 79.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 81.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 82.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 83.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 84.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 85.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 86.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 87.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 88.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 89.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 90.000000 ]
   fov 0 )
################
 kephera_sensor( pose [ 0 0 0 100.000000 ] )
 kephera_sensor( pose [ 0 0 0 90.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 91.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 92.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 93.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 94.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 95.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 96.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 97.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 98.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 99.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 101.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 102.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 103.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 104.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 105.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 106.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 107.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 108.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 109.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 110.000000 ]
   fov 0 )
################
 kephera_sensor( pose [ 0 0 0 120.000000 ] )
 kephera_sensor( pose [ 0 0 0 110.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 111.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 112.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 113.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 114.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 115.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 116.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 117.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 118.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 119.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 121.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 122.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 123.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 124.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 125.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 126.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 127.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 128.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 129.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 130.000000 ]
   fov 0 )
################
 kephera_sensor( pose [ 0 0 0 140.000000 ] )
 kephera_sensor( pose [ 0 0 0 130.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 131.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 132.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 133.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 134.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 135.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 136.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 137.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 138.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 139.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 141.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 142.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 143.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 144.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 145.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 146.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 147.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 148.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 149.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 150.000000 ]
   fov 0 )
################
 kephera_sensor( pose [ 0 0 0 160.000000 ] )
 kephera_sensor( pose [ 0 0 0 150.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 151.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 152.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 153.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 154.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 155.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 156.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 157.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 158.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 159.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 161.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 162.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 163.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 164.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 165.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 166.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 167.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 168.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 169.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 170.000000 ]
   fov 0 )
################
 kephera_sensor( pose [ 0 0 0 180.000000 ] )
 kephera_sensor( pose [ 0 0 0 170.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 171.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 172.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 173.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 174.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 175.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 176.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 177.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 178.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 179.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 181.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 182.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 183.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 184.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 185.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 186.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 187.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 188.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 189.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 190.000000 ]
   fov 0 )
################
 kephera_sensor( pose [ 0 0 0 200.000000 ] )
 kephera_sensor( pose [ 0 0 0 190.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 191.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 192.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 193.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 194.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 195.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 196.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 197.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 198.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 199.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 201.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 202.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 203.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 204.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 205.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 206.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 207.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 208.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 209.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 210.000000 ]
   fov 0 )
################
 kephera_sensor( pose [ 0 0 0 220.000000 ] )
 kephera_sensor( pose [ 0 0 0 210.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 211.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 212.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 213.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 214.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 215.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 216.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 217.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 218.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 219.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 221.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 222.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 223.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 224.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 225.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 226.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 227.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 228.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 229.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 230.000000 ]
   fov 0 )
################
 kephera_sensor( pose [ 0 0 0 240.000000 ] )
 kephera_sensor( pose [ 0 0 0 230.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 231.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 232.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 233.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 234.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 235.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 236.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 237.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 238.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 239.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 241.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 242.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 243.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 244.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 245.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 246.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 247.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 248.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 249.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 250.000000 ]
   fov 0 )
################
 kephera_sensor( pose [ 0 0 0 260.000000 ] )
 kephera_sensor( pose [ 0 0 0 250.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 251.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 252.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 253.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 254.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 255.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 256.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 257.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 258.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 259.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 261.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 262.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 263.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 264.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 265.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 266.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 267.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 268.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 269.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 270.000000 ]
   fov 0 )
################
 kephera_sensor( pose [ 0 0 0 280.000000 ] )
 kephera_sensor( pose [ 0 0 0 270.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 271.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 272.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 273.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 274.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 275.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 276.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 277.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 278.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 279.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 281.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 282.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 283.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 284.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 285.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 286.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 287.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 288.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 289.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 290.000000 ]
   fov 0 )
################
 kephera_sensor( pose [ 0 0 0 300.000000 ] )
 kephera_sensor( pose [ 0 0 0 290.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 291.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 292.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 293.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 294.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 295.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 296.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 297.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 298.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 299.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 301.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 302.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 303.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 304.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 305.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 306.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 307.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 308.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 309.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 310.000000 ]
   fov 0 )
################
 kephera_sensor( pose [ 0 0 0 320.000000 ] )
 kephera_sensor( pose [ 0 0 0 310.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 311.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 312.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 313.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 314.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 315.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 316.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 317.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 318.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 319.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 321.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 322.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 323.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 324.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 325.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 326.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 327.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 328.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 329.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 330.000000 ]
   fov 0 )
################
 kephera_sensor( pose [ 0 0 0 340.000000 ] )
 kephera_sensor( pose [ 0 0 0 330.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 331.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 332.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 333.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 334.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 335.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 336.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 337.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 338.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 339.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 341.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 342.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 343.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 344.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 345.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 346.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 347.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 348.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 349.000000 ]
   fov 0 )
 kephera_sensor( pose [ 0 0 0 350.000000 ]
   fov 0 )
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
