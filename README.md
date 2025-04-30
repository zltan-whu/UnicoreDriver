# UnicoreDriver

**Authors/Maintainers:** Zhiliang Tan (zltan0906@gmail.com)

## About **UnicoreDriver**

**UnicoreDriver** provides the essential functionality for **Unicore GNSS receivers**.  
The driver was developed against the official protocol specification of the **[Unicore UM982](https://www.unicorecomm.com/products/detail/26)** module and has been fully tested on both **UM982** and **UM980** devices.  
Any other Unicore-series receiver that follows the same interface specification should work seamlessly with this driver.

## 1. Prerequisites

### 1.1 C++11 Compiler
This package requires some features of C++11.

### 1.2 ROS
This package is developed under [[ROS Noetic](https://wiki.ros.org/noetic)] environment.

### 1.3 Eigen
We use [Eigen 3.3.7](https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.zip) for matrix manipulation.

### 1.4 Boost
Our software utilizes [Boost](https://www.boost.org/) library for serial and socket manipulation. Using command `sudo apt-get install libboost-all-dev` to install *Boost*.

## 2. Build UnicoreDriver

```
mkdir -p catkin_ws/src && cd catkin_ws/src
git clone https://github.com/zltan-whu/UnicoreDriver.git
cd ..
catkin_make
source ~/catkin_ws/devel/setup.bash
```
## 3. Run with your unicore receiver
Before launching the driver, connect the receiver and use [`UPRECISE`](https://m.unicorecomm.com/products/detail/57) to configure it so that it outputs the `BESTNAVXYZB` message.

### 3.1 Edit the launch file  
Open **`launch/unicore_driver.launch`** and replace the defaults with the
serial port and baud-rate of *your* receiver:

```xml
<arg name="port"      default="/dev/ttyUSB0"/>
<arg name="baud_rate" default="921600"/>
```

### 3.2 Connecting multiple receivers

Need more than one UM982/UM980 on the same PC?Just duplicate the entire block:
```xml 
<group> … </group> 
```
- give each block its own namespace (ns="RTK", ns="SPP", …)
- set the corresponding port and baud_rate
- configure NTRIP parameters independently if you need RTK

The launch file already contains a two-receiver example for reference.

### 3.3 Start the driver
```
roslaunch unicore_driver unicore_driver.launch
```
Confirm that the navigation message (`nav_msgs/Odometry`) is flowing:
```
rostopic echo /SPP/unicore_driver/best_nav_ecef
```
(Replace `/SPP/` with the namespace you chose.)

### 3.4 Getting an RTK fix (optional)
1. Acquire CORS/NTRIP credentials (host, port, mount-point, user, password).
2. In the relevant `<group>` set:
    ```xml
    <arg name="enable_rtk" default="true"/>
    <arg name="gga_sentence"
        default="$GNGGA,hhmmss.ss,lat,N,lon,E,1,05,4.0,0.0,M,0.0,M,,*CS\r\n"/>
    <arg name="ntrip_host"       default="YOUR_HOST"/>
    <arg name="ntrip_port"       default="YOUR_PORT"/>
    <arg name="ntrip_mountpoint" default="YOUR_MP"/>
    <arg name="ntrip_user"       default="YOUR_USER"/>
    <arg name="ntrip_pass"       default="YOUR_PASS"/>
    ```
    `gga_sentence` only needs an **approximate** position; 

3. Relaunch the driver. When the CORS stream is healthy and the antenna has a clear view of the sky, an integer solution should appear:
    ```
    rostopic echo /RTK/unicore_driver/best_nav_ecef | grep child_frame_id
    # Expected:
    # child_frame_id: "NARROW_INT"
    ```
    If the field stays `"SINGLE"` or `"PSRDIFF"`, verify antenna placement, account balance, and network connectivity.


## 4. Todo List

- [ ] Publish more types of messages  
- [ ] Integrate with [**gnss_comm**](https://github.com/HKUST-Aerial-Robotics/gnss_comm) and output raw observables  
- [ ] Provide a one-click script to configure the receiver


## 5. Acknowledgements
This driver is adapted from the open-source **[ublox_driver](https://github.com/HKUST-Aerial-Robotics/ublox_driver.git)**—many thanks to its authors for the solid foundation.  


## 6. License
The source code is released under [GPLv3](https://www.gnu.org/licenses/gpl-3.0.html) license.

