# Articulated-6-Dof-Robot

This repository contains the steps to set up and control an articulated 6-DOF robotic arm using Docker and ROS2.

---

## 🛠️ Prerequisites

- Linux-based system (e.g., Ubuntu 22.04)
- Docker installed
- ROS2 Humble installed

---

## 🚀 Tutorial

### 1️⃣ Install Docker Compose
Ensure Docker Compose is installed:
```sh
sudo apt-get update
sudo apt-get install docker-compose
```

### 2️⃣ Clone the Repository
Navigate to your workspace and clone this repository:
```sh
cd ~/workspace
git clone https://github.com/KobchokTime/Articulated-6-Dof-Robot.git -b Software-Team
cd Articulated-6-Dof-Robot
```

### 3️⃣ Build the Docker Image
Build the Docker image using the provided `Dockerfile`:
```sh
sudo docker build -t robot-arm-image .
```

### 4️⃣ Run a Container from the Image
Run the container with the following command:
```sh
sudo docker run -it --rm \
    -p 6080:80 \
    --device=/dev/ttyUSB0 \
    --device=/dev/ttyACM0 \
    --privileged \
    --shm-size=4096m \
    --security-opt seccomp=unconfined \
    -v ~/workspace/Articulated-6-Dof-Robot/ros2_ws:/home/ubuntu/robot_ws \
    robot-arm-image /bin/bash
```

### 5️⃣ Verify the Docker Container
List all containers to ensure your container is running:
```sh
sudo docker ps -a
```
#### Example Output
```sh
CONTAINER ID   IMAGE            COMMAND                  CREATED         STATUS         PORTS                  NAMES
472cf442c4da   robot-arm-image  "/bin/bash -c /entry…"   16 minutes ago  Up 5 minutes   0.0.0.0:6080->80/tcp   clever_robot
```

### 🔧 Common Docker Commands
- **View container logs:**
    ```sh
    sudo docker logs <container_id>
    ```
- **Restart a container:**
    ```sh
    sudo docker start <container_id>
    ```
- **Remove a container:**
    ```sh
    sudo docker rm <container_id>
    ```
- **Start a shell in a running container:**
    ```sh
    sudo docker exec -it <container_id> /bin/bash
    ```

### 6️⃣ Start the Docker Container
To start an existing container:
```sh
sudo docker start <container_id>
sudo docker attach <container_id>
```
Copy the `.bashrc` configuration (first-time setup):
```sh
cp ~/robot_ws/.bashrc ~/
```

---

## 🛡️ Troubleshooting

- Ensure all necessary ROS2 packages are installed in the Docker container.
- Verify that the Docker container is running correctly and check logs for errors.
- Confirm that `/dev/ttyACM0` and `/dev/ttyUSB0` are available and have correct permissions.

---

## 📦 Deploy Code

### Save a Docker Container to a File
1. Commit the container to a new image:
```sh
sudo docker commit <container_id> <new_image_name>
```
Example:
```sh
sudo docker commit ff89ac00951c articulated_robot_image
```

2. Save the image to a tar file:
```sh
sudo docker save -o articulated_robot_image.tar articulated_robot_image
```

3. Load the image from a tar file:
```sh
sudo docker load -i articulated_robot_image.tar
```

4. Verify the loaded image:
```sh
sudo docker images
```

5. Run a container from the loaded image:
```sh
sudo docker run -it --rm \
    -p 6080:80 \
    --device=/dev/ttyUSB0 \
    --device=/dev/ttyACM0 \
    --privileged \
    --shm-size=4096m \
    --security-opt seccomp=unconfined \
    -v ~/workspace/Articulated-6-Dof-Robot/ros2_ws:/home/ubuntu/robot_ws \
    articulated_robot_image /bin/bash
```

### Delete Docker Images
To delete an image by ID:
```sh
sudo docker rmi <image_id>
```

---

## 🌟 Features

- Fully compatible with ROS2 Humble
- Supports Dockerized development for an articulated robotic arm
- Easy to deploy and test in different environments

---

## 📝 License
This project is licensed under the MIT License - see the LICENSE file for details.

---

## 👥 Contributors
- [Songkarn](https://github.com/pannatron)

Feel free to contribute to this repository by submitting issues or pull requests!
