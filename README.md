[![KiRoM]((http://img.youtube.com/vi/VpCH3-6NSFA/0.jpg)]](https://www.youtube.com/watch?v=VpCH3-6NSFA)

# KiRoM: Kinova Gen3 & Robotiq 2F-85 + MoveIt 2 + Isaac Sim  
---  
## README (English) 
### Overview 
This project demonstrates how to control a **Kinova Gen3 (6-DoF)** manipulator equipped with a **Robotiq 2F-85** gripper in **Isaac Sim** using **ROS 2 (Humble)** and **MoveIt2**. 

- **Operating System**: Ubuntu 22.04   
- **Isaac Sim**: Version 4.0.0 
- **GPU**: RTX 4090  
- **ROS2**: Humble    

Refer to the [Isaac Sim ROS documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_ros.html) for information on connecting Isaac Sim to ROS2. 
### 1. Environment Setup  

1. **Clone the repository**:   

```bash
git clone https://github.com/KiRoM-Isaac-SIM/KiRoM.git
```    

2. **Build the Docker image:**

```bash
docker build -t kirom -f dockerFile .
```

3. **Run the Docker container**:

```bash
docker run -it --gpus all \
    --network host \
    -e DISPLAY=$DISPLAY \    
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v ~/.ros/fastdds.xml:/root/.ros/fastdds.xml:ro \  
    --name kirom \
    kirom
```
 
- The environment variables and volume mounts above forward GUI windows and unify ROS2 DDS settings.

4. **Grant X server access** from another terminal on the host:

```bash
xhost +local:docker
```

5. **Enter the running container**:
```bash
docker exec -it kirom bash
```

You now have a ROS2 + MoveIt2 environment inside the container.

### 2. Running the Project

1. Go to the workspace inside the container:

```bash
cd /moveit2_ws
```

2. Source the workspace:

```bash
source install/setup.bash
```

3. Launch the main nodes:

```bash
ros2 launch kirom kirom.launch.py
```

This starts the ROS2 nodes that control the Kinova Gen3 + Robotiq gripper in Isaac Sim with MoveIt2.

### 3. (Optional) Pick & Place Demo

1. In **another terminal** (inside the container), source again:

```bash
source /moveit2_ws/install/setup.bash
```

2. Launch the pick & place demo:
 
```bash
ros2 launch kirom_picknplace pick_place_demo.launch.py
```

3. Publish a topic message to trigger pick & place:

```bash
ros2 topic pub --once /command std_msgs/msg/String "data: 'world 0.1 0.1 0.1 0.5 0.0 0.2 0.0 0.0 0.0 1 0.4 0.0 0.2 0.0 0.0 0.0 1'"
```

The string contains **18 values** in the following order:    
    1. `frame_id`
    2. `width`
    3. `depth`
    4. `height`
    5. `x`
    6. `y`
    7. `z`
    8. `qx`
    9. `qy`
    10. `qz`
    11. `qw`
    12. `target_x`
    13. `target_y`
    14. `target_z`
    15. `target_qx`
    16. `target_qy`
    17. `target_qz`
    18. `target_qw`
    
**Example**
    - Object frame: `"world"`, Size: `(0.1, 0.1, 0.1)`
    - Object position: `x=0.5, y=0.0, z=0.2, q=(0.0, 0.0, 0.0, 1.0)`        - Target position: `x=0.4, y=0.0, z=0.2, q=(0.0, 0.0, 0.0, 1.0)`

The collision object is added to the planning scene, and MoveIt 2 will handle the pick & place pipeline.

#### Notes

- Isaac Sim should run with `--network host` so that the ROS 2 nodes can communicate properly.
- Make sure the ROS 2 bridge is enabled in Isaac Sim.
- For issues or additional help, check the official Isaac Sim documentation or file an Issue in this repository.
