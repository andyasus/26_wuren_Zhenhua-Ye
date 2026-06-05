geometry_msgs/Point position        # coordinate of cone in [x, y]
std_msgs/String color                 # color of cone, 'b' = blue, 'y' = yellow, 'o' = orange
std_msgs/Float32 pose_confidence         # confidence of cone detect
std_msgs/Float32 color_confidence         # confidence of cone detect

uint8 cluster
uint8 point_type
uint8 pts
uint8 visited
uint8[] corepts
