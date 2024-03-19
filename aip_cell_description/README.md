# AIP Cell Description

This part includes the necessary URDF, config and launch files for visualization and further trajectory planning of the robot and it´s surrounding. 

## EKI

Path for EKI xml interface:
```bash
C://KRC/ROBOTER/Config/User/Common/EthernetKRL/<interface>.xml
```
Path for SRC file:
```bash
C://R1/Programme/ros2_driver/<file>.src
```

## Testing

Test if port is open
```bash
sudo hping3 -S -p 54601 10.166.32.145
```
if the reply has flags=RA, you received a reset response, so the port is closed, but if you receive flags=SA, the port is open

```bash
sudo nmap -sU 10.166.32.145 -p 1-
```

Home Pose
```xml
<Action ID="MoveArmToJoints" joint1="0" joint2="-1.57" joint3="1.57" joint4="0" joint5="1.57" joint6="0"/>
<Action ID="MoveArmToPose" cartesian="False" q_w="1" q_x="0" q_y="0" q_z="0" x="0.74" y="1" z="1.88"/>
```

Find out the cartesian pose 
```bash
ros2 run tf2_ros tf2_echo table_link tcp
```
