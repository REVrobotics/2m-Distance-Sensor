/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

#include "rev/Rev2mDistanceSensor.h"

void Robot::RobotInit() {
  // initialize distance sensor on OnBoard port, set units to inches
  distSensor = new rev::Rev2mDistanceSensor{rev::Rev2mDistanceSensor::Port::kOnboard, 
                                            rev::Rev2mDistanceSensor::DistanceUnit::kInches};
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  /**
   * Call SetAutomaticMode(true) to begin background autonomous
   * thread. Will periodically ping sensor for new data 
   */
  distSensor->SetAutomaticMode(true);

  /**
   * By default, sensor is enabled. However, SetEnabled(false) is called
   * in DisabledInit() below, so we need to reenable every time we go into
   * teleop mode
   */
  distSensor->SetEnabled(true);
}

void Robot::TeleopPeriodic() {
  /**
   * The current measurement is considered valid if IsRangeValid()
   * returns true.
   */
  bool isValid = distSensor->IsRangeValid();

  frc::SmartDashboard::PutBoolean("Data Valid", isValid);

  if(isValid) {
    /**
     * The current measured range is returned from GetRange(). By default
     * this range is returned in inches.
     */
    frc::SmartDashboard::PutNumber("Distance (in)", distSensor->GetRange());

    /**
     * The timestamp of the last valid measurement (measured in seconds since 
     * the program started), is returned by GetTimestamp().
     */
    frc::SmartDashboard::PutNumber("Timestamp", distSensor->GetTimestamp());
  }
  else {
    frc::SmartDashboard::PutNumber("Distance (in)", -1);
  }
}

void Robot::TestPeriodic() {}

void Robot::DisabledInit() {
  /**
   * The background round robin thread can be stopped by calling 
   * SetAutomaticMode(false)
   */
  distSensor->SetAutomaticMode(false);

  /**
   * In order to completely kill the background thread, all current
   * sensors need to be disabled by calling SetEnabled(false)
   */
  distSensor->SetEnabled(false);
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
