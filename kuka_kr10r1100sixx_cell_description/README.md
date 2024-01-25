# AIP

## How to start
Rebuild or start the docker container.
```bash
source rebuild_container.sh
source start_docker.sh
```

```bash
ros2 launch kuka_kr10r1100sixx_cell_description aip.launch.py
# To use real hardware robot switch use_fake_hardware:=false
```

To open new bash in same container
```bash
docker exec -it r2e_cell bash
```

## How to start for C++ debugging

Install VSCode extension:
- Remote Development
- ROS
- C/C++

Mount settings folder .vscode to target directory for development
```bash
# Add parameter to docker run command
-v $PWD/.vscode:/home/docker/ros2_ws/src/.vscode
```

```bash
source start_docker.sh
```

1. Attach to running docker container with VSCode remote extension
2. Open remote folder where .vscode is mounted to
3. Install `ROS` and `C/C++` extension in container
4. Use command palette (strg + shift + p) and `Tasks: Run Task` and `Build`
5. Use VSCode debugger and stop points for debugging

## EKI

Path for EKI xml interface:
```bash
C://KRC/ROBOTER/Config/User/Common/EthernetKRL/<interface>.xml
```
Path for SRC file:
```bash
C://R1/Programme/ros2_driver/<file>.src
```

# Testing

Test if port is open
```bash
sudo hping3 -S -p 54601 10.166.32.145
```
if the reply has flags=RA, you received a reset response, so the port is closed, but if you receive flags=SA, the port is open

```bash
sudo nmap -sU 10.166.32.145 -p 1-
```

# TODO:

- [ ] Greifer IOs in KUKA smartPad überprüfen
- [ ] Druckluftschlauch legen zu mmt zelle
- [ ] Greifer ios schalten über smartPad
- [ ] EKI Schnittstelle um Greifer IOs erweitern/überprüfen
- [ ] Demoprogramm Roboter fährt über ROS zwischen zwei Posen hin und her
- [ ] Demoprogramm Roboter fährt und schaltet Greifer Druckluft                           
