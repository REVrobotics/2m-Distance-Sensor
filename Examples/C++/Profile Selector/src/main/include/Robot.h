/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/IterativeRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "rev/Rev2mDistanceSensor.h"

class Robot : public frc::IterativeRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void DisabledInit() override;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kRangeDefault = "Default";
  const std::string kHighAccuracy = "High Accuracy";
  const std::string kLongRange = "Long Range";
  const std::string kHighSpeed = "High Speed";
  std::string m_profileSelected;

  rev::Rev2mDistanceSensor *distSensor;
};
