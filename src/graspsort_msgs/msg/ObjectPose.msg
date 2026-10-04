# One perceived object (architecture section 6).
std_msgs/Header header                 # frame_id = "world"
uint32 id
string class_name                      # e.g. "cup", "bottle"
float32 confidence
geometry_msgs/Pose pose                # centre of the object, yaw only (upright objects)
geometry_msgs/Vector3 size             # x, y footprint and height (m)
uint8 shape                            # 0 = cylinder, 1 = box
