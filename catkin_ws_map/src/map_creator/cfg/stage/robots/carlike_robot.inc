define carlike_robot position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  size [ 0.6 0.25 0.40 ] # this models the footprint (rectangular), but shifted such that the bottom-left corner is in [0,0]. The center of rotation now here at [0.3, 0.125, 0.2]
  # correct center of rotation:
  origin [ 0.2 0.0 0.0 0.0]
  gui_nose 1
  color "red"

  # kinematics
  drive "car"
  wheelbase 0.650 # distance between both axles
  
)
