# REV Robotics 2M Distance Sensor
roboRIO library for the [REV Robotics 2M Distance Sensor](http://www.revrobotics.com/rev-31-1505/)

## Installing For C++/Java 
Download the latest `REV-2m-Distance-Sensor-roboRIO-SDK-[version].zip` from the [release](https://github.com/REVrobotics/2m-Distance-Sensor/releases) tab and unzip it.

Inside `REV-2m-Distance-Sensor-roboRIO-SDK-[version]` there will be vendordeps and maven folders. Copy the file `REV2mDistanceSensor.json` under vendordeps to the frc vendordeps folder on your machine. It is typically located in the following places:
* Windows: C:\Users\Public\wpilib\2023\vendordeps
* Mac/Linux: ~/wpilib/2023/vendordeps

Next, merge the maven folder with the frc maven folder on your machine, typically located at:
* Windows: C:\Users\Public\wpilib\2023\maven
* Mac/Linux: ~/wpilib/2023/maven

In order to use these libraries in your robot code, open a project in VsCode. 
* Press `Ctrl-Shift-P` to open the WPI commands window.
* Select `WPILib: Manage Vendor Libraries`
* Select `Install new libraries (offline)`
* Select the check box next to `REV2mDistanceSensor` and press `OK`

The REV2mDistance sensor library can now be used in your code.

## Installing For LabVIEW
Download the `rev-2m-distance-sensor_1.0.0-0_windows_x64.nipkg` installer under the releases tab. Simply run the installer and you will have access to REV2mDistanceSensor functions in your projects.

## Examples
Examples for this library can be found in the `Examples` folder for both C++ and Java.
