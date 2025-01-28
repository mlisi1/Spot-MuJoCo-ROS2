# ROS 2 message for describing contact information from Mujoco
std_msgs/Header header

# Position of the contact point (world frame)
geometry_msgs/Point position

# Normal vector of the contact frame (z-axis)
geometry_msgs/Vector3 normal

# Tangential vectors of the contact frame (x and y axes)
geometry_msgs/Vector3 tangent1
geometry_msgs/Vector3 tangent2

# Force applied at the contact point in the contact frame
# (first 3 values: forces in x, y, z, last 3 values: torques)
geometry_msgs/Wrench contact_force

# Distance between the closest points of the colliding objects
# Negative values indicate penetration
float64 distance

# Friction coefficients
# friction[0]: tangent1
# friction[1]: tangent2
# friction[2]: spin
# friction[3]: roll1
# friction[4]: roll2
float64[5] friction

# Solver reference and impedance parameters
float64[2] solref    # Constraint solver reference
float64[2] solimp    # Constraint solver impedance

# IDs of the colliding objects
int32 object1_id
int32 object2_id

string object1_name
string object2_name

# Dimensionality of the contact space (1, 3, 4, or 6)
int32 contact_dim

# Exclusion flag (0: include, 1: in gap, 2: fused, etc.)
int32 exclude_flag

# Address in the solver's constraint force array (-1 if not included)
int32 efc_address
