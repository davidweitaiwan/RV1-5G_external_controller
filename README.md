*`Established: 2024/04/29`* *`Updated: 2024/04/29`*

## About The Project
The external remote controller for robot vehicle ver. 1 project. The project is used to transfer the remote control signal from external ID server signal to ROS2 control signal, then send the control signal to the control server.
**NOTE:** This package requires the `libIDClient.so` library to build the package. The library is not included in the package. Please contact the author for the library.

## Getting Started

### Prerequisites
- ROS2 `Foxy` or later (`Humble` recommended)
    Install ROS2 from official website: [ROS2 official website](https://docs.ros.org/en/humble/Installation.html) or simply run the following command to automatically install ROS2:
    ```bash
    curl -fsSL ftp://61.220.23.239/scripts/install-ros2.sh | bash
    ```
    **NOTE:** The script only supports `Foxy` and `Humble` versions depending on the Ubuntu version.
    **NOTE:** The script will create a new workspace at `~/ros2_ws`.
    **NOTE:** The script will create an alias `humble` or `foxy` for global ROS2 environment setup (e.g. `source /opt/ros/<$ROS_DISTRO>/setup.bash`) depending on the ROS2 version.
- [vehicle_interfaces](https://github.com/cocobird231/RV1-vehicle_interfaces.git)
- `libIDClient.so` library
    The library is not included in the package. Please contact the author for the library.

### Installation
There are two ways to install the package: manually or using the `vcu-installer`. 

#### Install Manually
1. Check if `vehicle_interfaces` package is installed. If not, install the package by following the instructions in the [vehicle_interfaces](https://github.com/cocobird231/RV1-vehicle_interfaces.git).
2. Clone the repository under `~/ros2_ws/src` and rename it to `cpp_external_controller`:
    ```bash
    git clone https://github.com/cocobird231/RV1-external_controller.git cpp_external_controller
    ```
3. Change the directory to the `~/ros2_ws` workspace and build the package:
    ```bash
    # Change directory to workspace.
    cd ~/ros2_ws

    # Source the local environment.
    . install/setup.bash

    # Build the package.
    colcon build --symlink-install --packages-select cpp_external_controller
    ```
    **NOTE:** The package is installed in the local workspace.


#### Install Using `vcu-installer`
1. Run the installer and press `Scan` button under Package Management window. If the installer not installed, install the installer by following the instructions in the [`vcu-installer`](https://github.com/cocobird231/RV1-vcu-install.git).

2. Checked the `External Controller` checkbox under package list, right-click to modify the internet setting, then press the `Install` button to install the package.

3. The installer will create the start-up script for the package under `/etc/xdg/autostart` directory. The package will be started automatically after the system boot-up.


## Usage

### Run the Executable
1. Source the local workspace:
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

2. Run the external controller using `launch`:
    ```bash
    ros2 launch cpp_external_controller launch.py
    ```
    **NOTE:** The `common.yaml` file default the namespace to `V0`.


## Description

### `common.yaml` File
The `common.yaml` file is used to set the parameters for the external controller. The parameters are listed below:
- `controller_prop`:
    - `msg_type`: (int) Set the message type of the output control signal.
        The message type can be whether steering wheel or chassis. See `ControllerInfo.msg` for more information.
    - `controller_mode`: (int) Set the controller mode.
        The controller mode can be whether topic or service. See `ControllerInfo.msg` for more information.
    - `service_name`: (string) The external controller service name.
        If controller mode set to service, the `service_name` would be the service name. If controller mode set to topic, the `service_name` would be the topic name.
    - `timeout_ms`: (double) The timeout of the external controller.
        Tell control server the timeout of the external controller.
    - `period_ms`: (double) The period of the external controller.
        If controller mode set to topic, the `period_ms` would be the publish period; if controller mode set to service, the `period_ms` would be the period of the control server to call the service.
    - `privilege`: (int) The privilege of the external controller.
        The lower, the higher privilege. The privilege is used to determine the priority of the control server output signal.
    - `pub_type`: (int) The publish type of the external controller.
        Decide whether external controller or control server or both to publish the control signal. See `ControllerInfo.msg` for more information.
- `input_prop`:
    - `hostIP`: (string) The host IP address of the external ID server.
        The external ID server is used to communicate between the external controller and the remote controller far from the Internet.
    - `port`: (string) The port of the external ID server.
    - `ID`: (string) The ID of the external controller.
    - `timeout_ms`: (double) The timeout of the external ID server.
        Determine whether the external controller is connected to the external ID server.
- `controlserver_prop`:
    - `controlService`: (string) The control server service name.
    - `controllerRegService`: (string) The controller register service name.
- `topic_prop`:
    - `topicName`: (string) The topic name.
        The topic name revealed on the ROS2 network.
- `generic_prop`:
    - `namespace`: (string) The namespace of the node.
        The namespace is used to separate the services.
    - `nodeName`: (string) The node name.
        The node name revealed on the ROS2 network.
    - `id`: (int) The node id.
        The id of the node.