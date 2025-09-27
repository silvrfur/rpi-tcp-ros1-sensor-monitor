# RPi-ROS1-TCP-SensorMonitor

## 🧠 Project Overview

**RPi-ROS1-TCP-SensorMonitor** is a mini-project that demonstrates how to read sensor data from a Raspberry Pi **without installing ROS on it**.  
Instead, the Raspberry Pi sends sensor values over a **TCP connection** to a separate **ROS1 (Noetic)** machine, where multiple subscriber nodes process the data.

🔌 Features:
- LED control based on LDR (light) readings  
- Temperature and humidity monitoring (DHT11)  
- Gas detection (MQ9)  
- Real-time data publishing over ROS topics  

This setup keeps the Raspberry Pi lightweight and uses the full power of ROS on your Ubuntu system.

---

## 🧰 Raspberry Pi Side

### 📡 Pin Connections

| Component              | GPIO (BCM) | Description                               |
|-----------------------|------------|-------------------------------------------|
| **DHT11** (Temp/Humidity) | 4          | Sensor data pin                           |
| **MQ9** (Gas sensor)      | 21         | Digital output pin                        |
| **LDR** (Light sensor)    | 17         | Input pin                                 |
| **LED**                  | 5          | Output pin for LED control                |

---

### ▶️ Running the Raspberry Pi Script

1. Save your code as `rpi_code.py` on the Pi.  
2. Run it:

```bash
python3 rpi_code.py
````

3. If you face issues (e.g., `externally managed environment`), create and use a virtual environment:

```bash
python3 -m venv ~/rpi_env
source ~/rpi_env/bin/activate
pip install adafruit-blinka adafruit-circuitpython-dht RPi.GPIO
python3 rpi_code.py
```

4. Get your Pi’s IPv4 address (needed in the ROS node):

```bash
hostname -I
```

✅ Use the **first IPv4** shown as the `HOST` address in your ROS TCP client.

---

## 🤖 ROS Side (Ubuntu Machine)

> ⚠️ ROS1 **Noetic** officially runs on **Ubuntu 20.04 (Focal)**. It’s **not supported on Ubuntu 22.04 (Jammy)**.

---

### 📦 Install ROS1 Noetic (if not already)

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

---

### 🛠️ Create and Setup a ROS Workspace

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_create_pkg rpi_tcp_reader std_msgs rospy
```

Place all your scripts here:

```
~/catkin_ws/src/rpi_tcp_reader/scripts/
├── rpi_reader.py
├── led_controller.py
├── env_printer.py
└── gas_detector.py
```

Make them executable:

```bash
chmod +x ~/catkin_ws/src/rpi_tcp_reader/scripts/*.py
```

Build the workspace:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## 🚀 Running Everything

1. Start ROS Master:

```bash
roscore
```

2. Run the TCP reader node (receives data from Pi):

```bash
rosrun rpi_tcp_reader rpi_reader.py
```

3. Run subscriber nodes:

```bash
rosrun rpi_tcp_reader led_controller.py
rosrun rpi_tcp_reader env_printer.py
rosrun rpi_tcp_reader gas_detector.py
```

4. (Optional) View the sensor topic:

```bash
rostopic echo /sensor_data
```

---

## 📌 Tips & Notes

* ✅ Make sure **Raspberry Pi and ROS system are on the same network**.
* 🔌 The TCP `PORT` in both the Pi script and `rpi_reader.py` must match (default: `5005`).
* 🤖 The Raspberry Pi runs **no ROS components** — all ROS functionality is handled on the Ubuntu side.

---

## 📜 License

This project is open-source and free to use under the MIT License.
