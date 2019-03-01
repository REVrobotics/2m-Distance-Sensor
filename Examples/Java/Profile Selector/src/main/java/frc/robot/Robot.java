/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.*;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;

public class Robot extends IterativeRobot {
  private static final String profileDefault = "Default";
  private static final String highSpeed = "High Speed";
  private static final String highAccuracy = "High Accuracy";
  private static final String longRange = "Long Range";
  private String m_profileSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private Rev2mDistanceSensor distSens;

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default", profileDefault);
    m_chooser.addOption("High Speed", highSpeed);
    m_chooser.addOption("High Accuracy", highAccuracy);
    m_chooser.addOption("Long Range", longRange);
    SmartDashboard.putData("Profile", m_chooser);

    distSens = new Rev2mDistanceSensor(Port.kOnboard);
  }

  @Override
  public void disabledInit() {
    distSens.setEnabled(false);
  }

  @Override
  public void teleopInit() {
    distSens.setAutomaticMode(true);
    distSens.setEnabled(true);
  }

  @Override
  public void robotPeriodic() {
    m_profileSelected = m_chooser.getSelected();

    /**
     * The sensor range profile can be set when the sensor is disabled or in automatic mode.
     * 
     * If in automatic mode, the current measurement must finish before the background thread
     * can change the profile.
     * 
     * Each profile is a tradeoff between accuracy, speed and range. Refer to the VL53L0X
     * datasheet for specific tolerances of each mode.
     */
    switch (m_profileSelected) {
      case highSpeed:
        distSens.setRangeProfile(RangeProfile.kHighSpeed);
        break;
      case highAccuracy:
        distSens.setRangeProfile(RangeProfile.kHighAccuracy);
        break;
      case longRange:
        distSens.setRangeProfile(RangeProfile.kLongRange);
        break;
      default:
        distSens.setRangeProfile(RangeProfile.kDefault);
        break;
    }

    boolean isValid = distSens.isRangeValid();
    SmartDashboard.putBoolean("Valid", isValid);
    if(isValid) {
      SmartDashboard.putNumber("Range", distSens.getRange());
      SmartDashboard.putNumber("Timestamp", distSens.getTimestamp());
    }
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testPeriodic() {}
}
