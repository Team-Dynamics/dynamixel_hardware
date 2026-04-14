# dynamixel_control

The [`ros2_control`](https://github.com/ros-controls/ros2_control) implementation for any kind of [ROBOTIS Dynamixel](https://emanual.robotis.com/docs/en/dxl/) robots.

- `dynamixel_hardware`: the [`SystemInterface`](https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/system_interface.hpp) implementation for the multiple ROBOTIS Dynamixel servos.
- `open_manipulator_x_description`: the reference implementation of the `ros2_control` robot using [ROBOTIS OpenManipulator-X](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/).

The `dynamixel_hardware` package is hopefully compatible any configuration of ROBOTIS Dynamixel servos thanks to the `ros2_control`'s flexible architecture.

## Set up

First [install ROS 2 Humble on Ubuntu 22.04](http://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). Then follow the instruction below.

```shell
$ source /opt/ros/humble/setup.bash
$ mkdir -p ~/ros/humble && cd ~/ros/humble
$ git clone https://github.com/youtalk/dynamixel_control.git src
$ vcs import src < src/dynamixel_control.repos
$ rosdep install --from-paths src --ignore-src -r -y
$ colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
$ . install/setup.bash
```

## Demo with real ROBOTIS OpenManipulator-X

### Configure Dynamixel motor parameters

Update the `usb_port`, `baud_rate`, and `joint_ids` parameters on [`open_manipulator_x_description/urdf/open_manipulator_x.ros2_control.xacro`](https://github.com/youtalk/dynamixel_control/blob/main/open_manipulator_x_description/urdf/open_manipulator_x.ros2_control.xacro#L9-L12) to correctly communicate with Dynamixel motors.
The `use_dummy` parameter is required if you don't have a real OpenManipulator-X.

Note that `joint_ids` parameters must be splited by `,`.

```xml
<hardware>
  <plugin>dynamixel_hardware/DynamixelHardware</plugin>
  <param name="usb_port">/dev/ttyUSB0</param>
  <param name="baud_rate">1000000</param>
  <!-- <param name="use_dummy">true</param> -->
</hardware>
```

- Terminal 1

Launch the `ros2_control` manager for the OpenManipulator-X.

```shell
$ ros2 launch open_manipulator_x_description open_manipulator_x.launch.py
```

- Terminal 2

Start the `joint_trajectory_controller` and send a `/joint_trajectory_controller/follow_joint_trajectory` goal to move the OpenManipulator-X.

```shell
$ ros2 control switch_controllers --activate joint_state_broadcaster --activate joint_trajectory_controller --deactivate velocity_controller
$ ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory -f "{
  trajectory: {
    joint_names: [joint1, joint2, joint3, joint4, gripper],
    points: [
      { positions: [0.1, 0.1, 0.1, 0.1, 0], time_from_start: { sec: 2 } },
      { positions: [-0.1, -0.1, -0.1, -0.1, 0], time_from_start: { sec: 4 } },
      { positions: [0, 0, 0, 0, 0], time_from_start: { sec: 6 } }
    ]
  }
}"
```

If you would like to use the velocity control instead, switch to the `velocity_controller` and publish a `/velocity_controller/commands` message to move the OpenManipulator-X.

```shell
$ ros2 control switch_controllers --activate joint_state_broadcaster --deactivate joint_trajectory_controller --activate velocity_controller
$ ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray "data: [0.1, 0.1, 0.1, 0.1, 0]"
```

[![dynamixel_control: the ros2_control implementation for any kind of ROBOTIS Dynamixel robots](https://img.youtube.com/vi/EZtBaU-otzI/0.jpg)](https://www.youtube.com/watch?v=EZtBaU-otzI)

## Demo with dummy ROBOTIS OpenManipulator-X

The `use_dummy` parameter is required if you use the dummy OpenManipulator-X.

```diff
diff --git a/open_manipulator_x_description/urdf/open_manipulator_x.ros2_control.xacro b/open_manipulator_x_description/urdf/open_manipulator_x.ros2_control.xacro
index c6cdb74..111846d 100644
--- a/open_manipulator_x_description/urdf/open_manipulator_x.ros2_control.xacro
+++ b/open_manipulator_x_description/urdf/open_manipulator_x.ros2_control.xacro
@@ -9,7 +9,7 @@
         <param name="usb_port">/dev/ttyUSB0</param>
         <param name="baud_rate">1000000</param>
-        <!-- <param name="use_dummy">true</param> -->
+        <param name="use_dummy">true</param>
       </hardware>
       <joint name="joint1">
         <param name="id">11</param>
```

Then follow the same instruction of the real robot one.

Note that the dummy implementation has no interpolation so far.
If you sent a joint message, the robot would move directly to the joints without interpolation.

---

## 🌡️ Feature: Hardware-Temperatur auslesen (Deutsch)

Dieses Fork/Paket wurde erweitert, um effizient und standardkonform die **Motortemperatur** der Dynamixels auszulesen, ohne den seriellen RS485-Bus durch redundante Abfragen zu überlasten.

### 1. Zweck & Architektur
* **Hardware-Ebene (`dynamixel_hardware`):** Die Temperatur wird im selben, einzigen `SyncRead`-Block (zusammen mit Position, Velocity und Current/Effort) vom SDK ausgelesen. Es gibt keinen separaten, die Bandbreite störenden USB-Aufruf.
* **ROS 2 Controller-Ebene (`a2_control`):** Da sich die Temperatur im Gegensatz zur Position nur extrem langsam ändert, sollte sie nicht mit 100 Hz auf dem ROS-Bus publiziert werden. Dafür gibt es den dedizierten Controller `dynamixel_temperature_broadcaster`, der aus dem gecachten RAM-Wert der Hardware liest und diesen gedrosselt (z.B. mit 1 Hz) publiziert.

### 2. Konfiguration & Start

#### URDF / Xacro (Robot Description)
In der `.ros2_control.xacro` Datei deines Roboters muss bei jedem entsprechenden Gelenk das State-Interface für die Temperatur hinzugefügt werden:
```xml
<joint name="joint1">
  <param name="id">1</param>
  <!-- Standard Interfaces -->
  <state_interface name="position"/>
  <state_interface name="velocity"/>
  <state_interface name="effort"/>
  <!-- NEU: Temperatur Interface -->
  <state_interface name="temperature"/>
</joint>
```

#### Controller Registrierung (`ros2_controllers.yaml`)
Füge den eigenständigen Controller hinzu und drossele seine Update-Rate auf 1 Hz, um CPU und Netzwerk zu schonen:
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Haupt-Rate für den Roboter (Position/Velocity)
    
    dynamixel_temperature_broadcaster:
      type: a2_control/DynamixelTemperatureBroadcaster

dynamixel_temperature_broadcaster:
  ros__parameters:
    update_rate: 1    # Drosselung: Temperatur wird 1x pro Sekunde publiziert
```

#### Launch File
Vergiss nicht, den Spawner für den neuen Controller in dein Launch-File (z.B. `moveit.launch.py`) einzutragen:
```python
dynamixel_temperature_broadcaster_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["dynamixel_temperature_broadcaster", "--controller-manager", "/controller_manager"],
)
```

### 3. Diagnose & Troubleshooting (Nutzung)
Sobald der Controller Manager läuft, kannst du die geparsten Temperaturen via Terminal abrufen. Der Controller generiert automatisch ein gebündeltes Topic `~/temperatures` vom Typ `sensor_msgs/JointState`. Darin sind die Temperaturwerte im Array `effort` als Array aufgelistet, was für Diagnosetools (PlotJuggler/Foxglove) extrem effizient ist:

```bash
# Aktive Controller prüfen
ros2 control list_controllers

# Gebündeltes Temperatur-Topic anzeigen (enthält alle Gelenke gebündelt)
ros2 topic echo /dynamixel_temperature_broadcaster/temperatures
```
