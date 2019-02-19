/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>
#include "frc/Watchdog.h"

/** Library includes **/
#include "2MDistanceSensor.h"

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  frc::SmartDashboard::PutBoolean("Onboard Enabled", false);
  frc::SmartDashboard::PutBoolean("MXP Enabled", false);

  frc::SmartDashboard::PutNumber("Onboard Dist", -1);
  frc::SmartDashboard::PutNumber("MXP Dist", -1);

  dso = new rev::DistanceSensor(rev::DistanceSensor::Port::kOnboard);
  dsm = new rev::DistanceSensor(rev::DistanceSensor::Port::kMXP);

  uint32_t budget1, budget2;
  dso->GetMeasurementTimingBudget(&budget1);
  dsm->GetMeasurementTimingBudget(&budget2);

  if(dso->GetMeasurementTimingBudget(&budget1))
    frc::SmartDashboard::PutNumber("Measurement Period 1", budget1);
  else
    frc::SmartDashboard::PutNumber("Measurement Period 1", -1);

  if(dsm->GetMeasurementTimingBudget(&budget2))
    frc::SmartDashboard::PutNumber("Measurement Period 2", budget2);
  else
    frc::SmartDashboard::PutNumber("Measurement Period 2", -1);

  frc::SmartDashboard::PutNumber("MP", 0.05);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  double measurementPeriod = frc::SmartDashboard::GetNumber("MP", 0.05);

  if(measurementPeriod != dso->GetMeasurementPeriod()) {
    dso->SetMeasurementPeriod(measurementPeriod);
    frc::SmartDashboard::PutNumber("MP", dso->GetMeasurementPeriod());
  }

  /*if(!dso->IsAutomatic()) {
    uint32_t budget1 = frc::SmartDashboard::GetNumber("Measurement Period 1", 20000);
    uint32_t currBudget;
    if((dso->GetMeasurementTimingBudget(&currBudget)) && (currBudget != budget1)) {
      dso->SetMeasurementTimingBudget(budget1);
      //printf("Timing budget set to: %d\n", currBudget);
      if(dso->GetMeasurementTimingBudget(&currBudget)) {
        frc::SmartDashboard::PutNumber("Measurement Period 1", currBudget);
        printf("Timing budget set to: %d\n", currBudget);
      }
    }
  }

  if(!dsm->IsAutomatic()) {
    uint32_t budget2 = frc::SmartDashboard::GetNumber("Measurement Period 2", 20000);
    uint32_t currBudget;
    if((dsm->GetMeasurementTimingBudget(&currBudget)) && (currBudget != budget2)) {
      dsm->SetMeasurementTimingBudget(budget2);
      //printf("Timing budget set to: %d\n", currBudget);
      if(dsm->GetMeasurementTimingBudget(&currBudget)) {
        frc::SmartDashboard::PutNumber("Measurement Period 2", currBudget);
        printf("Timing budget set to: %d\n", currBudget);
      }
    }
  }*/
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {
  dso->SetAutomaticMode(true);
  
  frc::SmartDashboard::PutBoolean("Onboard Enabled", true);
  frc::SmartDashboard::PutBoolean("MXP Enabled", true);
}

void Robot::TeleopPeriodic() {
  bool dsoEnable = frc::SmartDashboard::GetBoolean("Onboard Enabled", true);
  bool dsmEnable = frc::SmartDashboard::GetBoolean("MXP Enabled", true);

  dso->SetEnabled(dsoEnable);
  dsm->SetEnabled(dsmEnable);
  
  uint8_t stat1, stat2;
  double dist1, dist2;

  dist1 = dso->GetRangeMM(&stat1);
  dist2 = dsm->GetRangeMM(&stat2);

  if(stat1 == 0)
  {
    frc::SmartDashboard::PutNumber("Onboard Dist", dist1);
  }
  else
  {
    frc::SmartDashboard::PutNumber("Onboard Dist", -1);
  }

  if(stat2 == 0)
  {
    frc::SmartDashboard::PutNumber("MXP Dist", dist2);
  }
  else
  {
    frc::SmartDashboard::PutNumber("MXP Dist", -1);
  }
}

void Robot::DisabledInit() {
  dso->SetAutomaticMode(false);
  
  frc::SmartDashboard::PutBoolean("Onboard Enabled", false);
  frc::SmartDashboard::PutBoolean("MXP Enabled", false);
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
