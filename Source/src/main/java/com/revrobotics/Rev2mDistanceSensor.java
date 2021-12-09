/*
 * Copyright (c) 2018-2020 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

package com.revrobotics;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.jni.VL53L0XJNI;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Rev2mDistanceSensor class.
 *
 * The Distance sensor class is intended to be used with the RevRobotics
 * 2M Distance Sensor.
 */
public class Rev2mDistanceSensor {
    public enum Port { kOnboard, kMXP }
    public enum RangeProfile { kDefault, kHighAccuracy, kLongRange, kHighSpeed }
    public enum Unit { kInches, kMillimeters }

    // sensor list
    private static final List<Rev2mDistanceSensor> m_sensors = new ArrayList<>();
    // automatic round robin mode
    private static volatile boolean m_automaticEnabled;
    private boolean m_enabled;
    // task doing the round-robin automatic sensing
    private static Thread m_task;
    private Unit m_units;

    private RangeProfile m_profile = RangeProfile.kDefault;
    private RangeProfile m_newProfile = RangeProfile.kDefault;
    private boolean m_stopped = true;
    private boolean m_rangeValid = false;
    private double m_currentRange = -1;
    private double m_timestamp = -1;
    private boolean m_stopping = false;
    private static double m_measurementPeriod = 0.05;
    private int m_addr = 0x53;
    private int m_port;

    /**
     * Creat an instance of the Rev 2m Distance Sensor.
     *
     * @param port          The I2C port to which the device is connected.
     * @param units         The units returned in either kInches or kMilliMeters
     * @param profile       Range profile for the device. Valid options are:
     *                          kDefault
     *                          kHighAccuracy
     *                          kLongRange
     *                          kHighSpeed
     *                      Refer to data sheet for profile specific performance specs.
     */
    public Rev2mDistanceSensor(Port port, Unit units, RangeProfile profile) {
        m_units = units;

        if(port == Port.kOnboard)
            m_port = 0;
        else
            m_port = 1;

        VL53L0XJNI.Init(m_port, m_addr);

        if(!initialize()) {
            DriverStation.reportError(String.format("Error initializing Rev 2M device on port " +
                                                    "%s. Please check your connections",
                                                    port == Port.kMXP ? "MXP" : "Onboard"), false);
        }

    }

    public Rev2mDistanceSensor(Port port) {
        this(port, Unit.kInches, RangeProfile.kDefault);
    }

    /**
     * Check if the device is enabled in round robin mode
     *
     * @return  True - Device enabled
     *          False - Device disabled
     */
    public boolean isEnabled() {
        return m_enabled;
    }

    /**
     * Turn automatic mode on/off.
     *
     * When automatic mode is on, all enabled sensors will be polled at a set rate.
     *
     * @param enabling  Set to true if round robin scheduling should start. This
     *                  will command all sensors to begin measurements in
     *                  continuous ranging mode.
     *
     *                  Setting to false will command all sensors to stop
     *                  measurements.
     */
    public void setAutomaticMode(boolean enabling) {
        if(enabling == m_automaticEnabled) return;

        m_automaticEnabled = enabling;

        if(enabling) {
            if(m_task == null) {
                m_task = new DoContinuous();
                m_task.start();
            }
            else if(m_task.isAlive()) {
                try {
                    m_task.join();
                    m_task = new DoContinuous();
                    m_task.start();
                } catch (InterruptedException ex) {
                    Thread.currentThread().interrupt();
                    ex.printStackTrace();
                }
            }
            else {
                m_task = new DoContinuous();
                m_task.start();
            }
        }
        else {
            for(Rev2mDistanceSensor sensor: m_sensors) {
                sensor.setEnabled(false);
            }
        }
    }

    /**
     * Determine if the current range is valid.
     *
     * @return  True - current range is valid
     *          False - current range is not valid
     */
    public boolean isRangeValid() {
        return m_rangeValid;
    }

    /**
     * Get the range in current units from the sensor.
     *
     * @return range in current units.
     */
    public double getRange() {
        return getRange(m_units);
    }

    /**
     * Get the range in specified units from the sensor.
     *
     * @param units units in which to return the measurement.
     *              Valid options are:
     *                  kInches
     *                  kMilliMeters
     *
     * @return range in specified units.
     */
    public double getRange(Unit units) {
        if(units == Unit.kInches)
            return getRangeInches();
        else if(units == Unit.kMillimeters)
            return getRangeMM();
        else
            return -1;
    }

    /**
     * Get the timestamp of the current measurement. Measured in seconds
     * since the program began.
     *
     * Uses Timer.GetFPGATimestamp()
     *
     * @return  timestamp of current measurement
     */
    public double getTimestamp() {
        return m_timestamp;
    }

    /**
     * Enable/disable the sensor in round robin scheduling. Automatic mode
     * must be separately enabled/disabled.
     *
     * @param enable    true - enable device
     *                  false - disable device
     */
    public void setEnabled(boolean enable) {
        m_enabled = enable;
    }

    /**
     * Sets the range profile for the sensor. Valid options are:
     *      kDefault
     *      kHighAccuracy
     *      kLongRange
     *      kHighSpeed
     *
     * Range profiles are based on timing budgets defined in the VL53L0X
     * datasheet
     *
     * If called in automatic mode, the sensor will be stopped until the
     * new profile is set. Measurements will not be available during this
     * period
     *
     * @param profile The range profile to set in the sensor
     *
     * @return Profile successfully changed
     */
    public boolean setRangeProfile(RangeProfile profile) {
        if(profile == m_profile) return true; // ignore the case of no change

        m_newProfile = profile;

        if(m_stopped && !m_automaticEnabled) {
            //System.out.println("Sensor stopped. Changing profile.");
            if(profile == RangeProfile.kHighAccuracy)
                return setProfileHighAccuracy();
            else if(profile == RangeProfile.kLongRange)
                return setProfileLongRange();
            else if(profile == RangeProfile.kHighSpeed)
                return setProfileHighSpeed();
            else
                return setProfileDefault();
        }
        else {
            //System.out.println("Sensor not stopped");
            m_enabled = false;
            return false;
        }
    }

    /**
     * Set the measurement period for the round robin scheduling loop.
     *
     * @param period    Delay in seconds used by the round robin scheduling
     *                  loop. Should be set to a value larger than the
     *                  measurement timing budget configured in the device, or
     *                  measurements will likely return not ready.
     */
    public void setMeasurementPeriod(double period) {
        if(period < 0.01) {
            period = 0.01;
        }
        else if(period > 1) {
            period = 1;
        }

        m_measurementPeriod = period;
    }

    /**
     * Get the measurement period for the round robin scheduling loop
     *
     * @return measurement period in seconds.
     */
    public double getMeasurementPeriod() {
        return m_measurementPeriod;
    }

    /**
     * Set the current DistanceUnit that should be used for the PIDSource base object.
     *
     * @param units The DistanceUnit that should be used.
     */
    public void setDistanceUnits(Unit units) {
        m_units = units;
    }

    /**
     * Get the current DistanceUnit that is used for the PIDSource base object.
     *
     * @return The type of DistanceUnit that is being used.
     */
    public Unit getDistanceUnits() {
        return m_units;
    }

    private synchronized boolean initialize() {
        boolean status = false;
        int statusInt;

        if((statusInt = VL53L0XJNI.ValidateI2C(m_port, m_addr)) != 0) {
            DriverStation.reportError(String.format("Error 0x%08X: Could not communicate" +
                                                    " with Rev 2M sensor over I2C.",
                                                    statusInt), false);
        }

        status = VL53L0XJNI.DataInit(m_port, m_addr);

        if(status)
            status = VL53L0XJNI.GetDeviceInfo(m_port, m_addr);

        if(status)
            status = VL53L0XJNI.StaticInit(m_port, m_addr);

        if(status)
            status = VL53L0XJNI.PerformRefCalibration(m_port, m_addr);

        if(status)
            status = VL53L0XJNI.PerformRefSpadManagement(m_port, m_addr);

        if(status)
            status = VL53L0XJNI.SetDeviceMode(1, m_port, m_addr);

        if(status) {
            status = setRangeProfile(RangeProfile.kDefault);
            if(status) {
                m_sensors.add(this);
                m_enabled = true;
            }
        }

        return status;
    }

    private static class DoContinuous extends Thread {
        @Override
        public synchronized void run() {
            boolean allStopped;
            do {
                allStopped = true;
                for (Rev2mDistanceSensor sensor: m_sensors) {
                    if(sensor.isEnabled()) {
                        if(sensor.m_stopped) {
                            if(VL53L0XJNI.StartMeasurement(sensor.m_port, sensor.m_addr)) {
                                sensor.m_stopped = false;
                            }
                        }
                        else {
                            if(VL53L0XJNI.GetMeasurementDataReady(sensor.m_port, sensor.m_addr)) {
                                double currentRange = VL53L0XJNI.GetRangingMeasurementData(sensor.m_port, sensor.m_addr);

                                if(currentRange > 0) {
                                    sensor.m_currentRange = currentRange;
                                    sensor.m_rangeValid = true;
                                    sensor.m_timestamp = Timer.getFPGATimestamp();
                                }
                                else {
                                    sensor.m_rangeValid = false;
                                }
                                VL53L0XJNI.ClearInterruptMask(sensor.m_port, sensor.m_addr);
                            }
                            else {
                                sensor.m_rangeValid = false;
                            }
                        }
                    }
                    else if(!sensor.m_stopped) {
                        if(!sensor.m_stopping) {
                            if(VL53L0XJNI.StopMeasurement(sensor.m_port, sensor.m_addr)) {
                                sensor.m_stopping = true;
                            }
                        }
                        else {
                            if(VL53L0XJNI.GetStopCompletedStatus(sensor.m_port, sensor.m_addr)) {
                                sensor.m_stopped = true;
                                sensor.m_stopping = false;
                                VL53L0XJNI.ClearInterruptMask(sensor.m_port, sensor.m_addr);
                            }
                        }
                    }
                    else if(sensor.m_newProfile != sensor.m_profile) {
                        allStopped = false;

                        if(sensor.m_newProfile == RangeProfile.kHighAccuracy)
                            sensor.m_enabled = sensor.setProfileHighAccuracy();
                        else if(sensor.m_newProfile == RangeProfile.kLongRange)
                            sensor.m_enabled = sensor.setProfileLongRange();
                        else if(sensor.m_newProfile == RangeProfile.kHighSpeed)
                            sensor.m_enabled = sensor.setProfileHighSpeed();
                        else
                            sensor.m_enabled = sensor.setProfileDefault();
                    }

                    if(!sensor.m_stopped) {
                        allStopped = false;
                    }
                }

                Timer.delay(m_measurementPeriod);
            }
            while(!allStopped || m_automaticEnabled);
        }
    }

    private double getRangeInches() {
        if (isRangeValid()) {
            return m_currentRange / 25.4;
        } else {
            return -1;
        }
    }

    private double getRangeMM() {
        if (isRangeValid()) {
            return m_currentRange;
        } else {
            return -1;
        }
    }

    /**
     * Get the range in the current DistanceUnit.
     *
     * @return The range in DistanceUnit
     */
    public double GetRange() {
        switch (m_units) {
        case kInches:
            return getRangeInches();
        case kMillimeters:
            return getRangeMM();
        default:
            return 0.0;
        }
    }

    private boolean setProfileLongRange() {
        //System.out.println("Setting profile to long range");
        boolean status = false;
        status = VL53L0XJNI.SetLimitCheckEnableSigmaFinalRange(1, m_port, m_addr);

        if(status)
            status = VL53L0XJNI.SetLimitCheckEnableSignalRateFinalRange(1, m_port, m_addr);

        if(status)
            status = VL53L0XJNI.SetLimitCheckEnableRangeIgnoreThreshold(0, m_port, m_addr);

        if(status)
            status = VL53L0XJNI.SetLimitCheckValueSigmaFinalRange(60, m_port, m_addr);

        if(status)
            status = VL53L0XJNI.SetLimitCheckValueSignalRateFinalRange(0.1, m_port, m_addr);

        if(status)
            status = VL53L0XJNI.SetMeasurementTimingBudgetMicroSeconds(33000, m_port, m_addr);

        if(status)
            status = VL53L0XJNI.SetVcselPulsePeriodPreRange(18, m_port, m_addr);

        if(status)
            status = VL53L0XJNI.SetVcselPulsePeriodFinalRange(14, m_port, m_addr);

        if(status)
            m_profile = RangeProfile.kLongRange;

        return status;
    }

    private boolean setProfileHighAccuracy() {
        //System.out.println("Setting profile to high accuracy");
        boolean status = false;
        status = VL53L0XJNI.SetLimitCheckEnableSigmaFinalRange(1, m_port, m_addr);

        if(status)
            status = VL53L0XJNI.SetLimitCheckEnableSignalRateFinalRange(1, m_port, m_addr);

        if(status)
            status = VL53L0XJNI.SetLimitCheckEnableRangeIgnoreThreshold(0, m_port, m_addr);

        if(status)
            status = VL53L0XJNI.SetLimitCheckValueSigmaFinalRange(18, m_port, m_addr);

        if(status)
            status = VL53L0XJNI.SetLimitCheckValueSignalRateFinalRange(0.25, m_port, m_addr);

        if(status)
            status = VL53L0XJNI.SetMeasurementTimingBudgetMicroSeconds(200000, m_port, m_addr);

        if(status)
            status = VL53L0XJNI.SetVcselPulsePeriodPreRange(14, m_port, m_addr);

        if(status)
            status = VL53L0XJNI.SetVcselPulsePeriodFinalRange(10, m_port, m_addr);

        if(status)
            m_profile = RangeProfile.kHighAccuracy;

        return status;
    }

    private boolean setProfileHighSpeed() {
        //System.out.println("Setting profile to high speed");
        boolean status = false;
        status = VL53L0XJNI.SetLimitCheckEnableSigmaFinalRange(1, m_port, m_addr);

        if(status)
            status = VL53L0XJNI.SetLimitCheckEnableSignalRateFinalRange(1, m_port, m_addr);

        if(status)
            status = VL53L0XJNI.SetLimitCheckEnableRangeIgnoreThreshold(0, m_port, m_addr);

        if(status)
            status = VL53L0XJNI.SetLimitCheckValueSigmaFinalRange(32, m_port, m_addr);

        if(status)
            status = VL53L0XJNI.SetLimitCheckValueSignalRateFinalRange(0.25, m_port, m_addr);

        if(status)
            status = VL53L0XJNI.SetMeasurementTimingBudgetMicroSeconds(30000, m_port, m_addr);

        if(status)
            status = VL53L0XJNI.SetVcselPulsePeriodPreRange(14, m_port, m_addr);

        if(status)
            status = VL53L0XJNI.SetVcselPulsePeriodFinalRange(10, m_port, m_addr);

        if(status)
            m_profile = RangeProfile.kHighSpeed;

        return status;
    }

    private boolean setProfileDefault() {
        //System.out.println("Setting profile to default");
        boolean status = false;
        status = VL53L0XJNI.SetLimitCheckEnableSigmaFinalRange(1, m_port, m_addr);

        if(status)
            status = VL53L0XJNI.SetLimitCheckEnableSignalRateFinalRange(1, m_port, m_addr);

        if(status)
            status = VL53L0XJNI.SetLimitCheckEnableRangeIgnoreThreshold(0, m_port, m_addr);

        if(status)
            status = VL53L0XJNI.SetLimitCheckValueSigmaFinalRange(18, m_port, m_addr);

        if(status)
            status = VL53L0XJNI.SetLimitCheckValueSignalRateFinalRange(0.25, m_port, m_addr);

        if(status)
            status = VL53L0XJNI.SetMeasurementTimingBudgetMicroSeconds(33823, m_port, m_addr);

        if(status)
            status = VL53L0XJNI.SetVcselPulsePeriodPreRange(14, m_port, m_addr);

        if(status)
            status = VL53L0XJNI.SetVcselPulsePeriodFinalRange(10, m_port, m_addr);

        if(status)
            m_profile = RangeProfile.kDefault;

        return status;
    }
}