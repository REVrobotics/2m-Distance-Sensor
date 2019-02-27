/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  distSensor = new rev::Rev2mDistanceSensor(rev::Rev2mDistanceSensor::Port::kOnboard,
                rev::Rev2mDistanceSensor::DistanceUnit::kMilliMeters,
                rev::Rev2mDistanceSensor::RangeProfile::kDefault);

  m_chooser.SetDefaultOption(kRangeDefault, kRangeDefault);
  m_chooser.AddOption(kHighAccuracy, kHighAccuracy);
  m_chooser.AddOption(kLongRange, kLongRange);
  m_chooser.AddOption(kHighSpeed, kHighSpeed);
  frc::SmartDashboard::PutData("Range Profile", &m_chooser);
}

void Robot::RobotPeriodic() {
  m_profileSelected = m_chooser.GetSelected();
  static std::string prevProfile = m_profileSelected;

  /**
   * The sensor range profile can be set when the sensor is disabled or in automatic mode.
   * 
   * If in automatic mode, the current measurement must finish before the background thread
   * can change the profile.
   * 
   * Each profile is a tradeoff between accuracy, speed and range. Refer to the VL53L0X
   * datasheet for specific tolerances of each mode.
   */
  if (m_profileSelected != prevProfile) {
    if(m_profileSelected == kHighAccuracy){
      distSensor->SetRangeProfile(rev::Rev2mDistanceSensor::RangeProfile::kHighAccuracy);
    }
    else if(m_profileSelected == kHighSpeed){
      distSensor->SetRangeProfile(rev::Rev2mDistanceSensor::RangeProfile::kHighSpeed);
    }
    else if(m_profileSelected == kLongRange){
      distSensor->SetRangeProfile(rev::Rev2mDistanceSensor::RangeProfile::kLongRange);
    }
    else{
      distSensor->SetRangeProfile(rev::Rev2mDistanceSensor::RangeProfile::kDefault);
    }

    prevProfile = m_profileSelected;
  }
}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  distSensor->SetAutomaticMode(true);
  distSensor->SetEnabled(true);
}

void Robot::TeleopPeriodic() {
  double dist;

  dist = distSensor->GetRange();

  bool isValid = distSensor->IsRangeValid();

  frc::SmartDashboard::PutBoolean("Range Valid", isValid);

  if(isValid) {
    frc::SmartDashboard::PutNumber("Range", distSensor->GetRange());
    frc::SmartDashboard::PutNumber("Timestamp", distSensor->GetTimestamp());
  }
}

void Robot::TestPeriodic() {}

void Robot::DisabledInit() {
  distSensor->SetEnabled(false);
  distSensor->SetAutomaticMode(false);
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
