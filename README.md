<a href="https://colab.research.google.com/github/nivannelson/AamaBot-Docker/blob/main/README.ipynb" target="_parent"><img src="https://colab.research.google.com/assets/colab-badge.svg" alt="Open In Colab"/></a>

# AmmaBot Documentation building with Docker

Installing Ubuntu 22.04

Follow the documentation https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi

Install Ubuntu 22.04 Server using Raspberry Pi imager to an SD card.

Follow through Default image guide https://www.youtube.com/watch?v=Cw_34fuve6E

set username to ihsl

use a short password like 123

later on you are able to set desired network ssid in etc/netplan/  configuration file in this format.

50-cloud-init.yaml


```python
# This file is generated from information provided by the datasource.  Changes
# to it will not persist across an instance reboot.  To disable cloud-init's
# network configuration capabilities, write a file
# /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg with the following:
# network: {config: disabled}
network:
    version: 2
    wifis:
        renderer: networkd
        wlan0:
            access-points:
                "Network SSID":
                    password: "The Network Password"
            dhcp4: true
            optional: true
```

Check if pubkeyauthentication yes by
sudo sshd -T | grep -E "pubkeyauthentication"

If no change it sshd_config.d
sudo nano /etc/ssh/sshd_config.d

Save changes

Restart your Raspberry pi to see the changes.

2. Use .local Hostname via mDNS
Goal: Connect using raspberrypi.local (no need to know IP)

Prerequisites:
Your Pi has avahi-daemon installed (for mDNS)

Your PC supports .local names (Linux/macOS do; Windows needs extra setup)

On Raspberry Pi:


```python
sudo apt install avahi-daemon

```

Make sure you hostname is set


```python
hostname
```

to change hostname to any diffrerent name(rpi2ubuntu,rpi3ubuntu,rpi4ubuntu....etc). Use this:


```python
sudo hostnamectl set-hostname rpi2ubuntu

```

if its raspberry, you can now connect like:


```python
ssh ishl@rpi2ubuntu.local
```

# Docker Installation


Follow this documentation

https://www.docker.com/blog/getting-started-with-docker-for-arm-on-linux/

untill creating a hello-world container



**Working with Docker using Portainer**


Portainer is a GUI for docker for ease of use.
It is run as a container in docker and initialises on system boot.
for Installation:



```python
docker volume create portainer_data
```


```python
docker run -d -p 9000:9000   --name=portainer   --restart=always   -v /var/run/docker.sock:/var/run/docker.sock   -v portainer_data:/data   portainer/portainer-ce:linux-arm
```

Now to check if portainer is running type :

docker ps



to connect to portainer you can use the ip of the host and port number 9443.

here if the host name is rpi2ubuntu then use

https://rpi2ubuntu.local:9000/

set username and password.

user:admin
password:123

In portainer you are able to add,remove,visualize containers,images and volumes easly.

**Working with portainer**

Delete unused container by going to the container tab and selecting the check box & remove.

Pull new Docker Image by going to the Images tab add the image tag in Pull image text box.

Ammabot images are managed by mrxnelson in DockerHub.

type:
mrxnelson/aammabot:v1

to get the first complete version of ammabot image.

Pull the image.

Alternatively type this in the bash terminal instead(If the download takes too long).


docker pull mrxnelson/aammabot:v1




Go to the container tab

Add container
Image name as mrxnelson/aammabot:v1

To run the Image type this command in the terminal


```python
docker run -it --name aammabot  --restart=always --net=host \
  --privileged --device /dev/gpiomem \
  -v /sys/class/gpio:/sys/class/gpio \
  -v /dev:/dev -e ROS_LOCALHOST_ONLY=0 \
  mrxnelson/aammabot:v1
```

You can see the container is running in portainer called aamabot.

Access the container through VS code attach to running container feature after connecting to the ssh of the raspberry pi in the host pc.

**running the ros2 launch file**

Start the ros node by:


```python
ros2 launch aama_bringup bringup.launch.py
```

Additionally the ros launch can be automated by adding this command in /root/entrypoint.sh(Optional)

connecting to ros2 is done by setting the environmental variable:
ROS_DOMAIN_ID which is 134 in entrypoint.sh.

(Optional)Change to a different ID value less than 200.



#Accessing the ROS2 topics




To accest the Robot topics


```python
export ROS_DOMAIN_ID=134       # Change the ID to match with the ID mentioned in entrypoint.sh
```


```python
export /opt/ros/humble/setup.bash
```


```python
ros2 topic list
```

You will get the output as:



```python
/cmd_vel
/odom_rf2o
/parameter_events
/point_cloud
/rosout
/scan
/tf
/tf_static

```

The github repository of Ammabot is

https://github.com/ebeyrajJY/AamaBot

clone the packages into a ros workspace and only build packages for SLAM and navigation.

colcon build --packages-select navigation
colcon build --packages-select robot_cartographer_config

After building source the workspace.

to run Mapping:

ros2 launch robot_cartographer_config mapping.launch.py

Save the map in navigation/map as map.pgm & map.yaml

to run Navigation:

ros2 launch navigation navigation2.launch.py


rviz2 needs initial pose to recognise map frame.

