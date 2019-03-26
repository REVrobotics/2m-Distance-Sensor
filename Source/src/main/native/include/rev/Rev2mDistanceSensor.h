/*
 * Copyright (c) 2018-2019 REV Robotics
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

#pragma once

#include <stdint.h>

#include <frc/ErrorBase.h>
#include <frc/smartdashboard/SendableBase.h>
#include <frc/smartdashboard/SendableBuilder.h>
#include <frc/PIDSource.h>

#include <HAL/I2C.h>

#include "vl53l0x_platform.h"
#include "vl53l0x_def.h"

#include <thread>
#include <vector>
#include <atomic>

namespace rev {

/**
 * Rev2mDistanceSensor class.
 *
 * The Distance sensor class is intended to be used with the RevRobotics
 * 2M Distance Sensor.
 */
class Rev2mDistanceSensor : public frc::ErrorBase, public frc::SendableBase, public frc::PIDSource {
    public:
        enum Port { kOnboard = 0, kMXP };
        enum DistanceUnit { kInches = 0, kMilliMeters = 1, kCurrent = 2 };
        enum RangeProfile { kDefault = 0, kHighAccuracy, kLongRange, kHighSpeed };

        /**
         * Creat an instance of the Distance Sensor.
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
        Rev2mDistanceSensor(Port port, DistanceUnit units, RangeProfile profile = RangeProfile::kDefault);

        ~Rev2mDistanceSensor() override;

        Rev2mDistanceSensor(Rev2mDistanceSensor&&) = default;
        Rev2mDistanceSensor& operator=(Rev2mDistanceSensor&&) = default;

        /**
         * Check if the device is enabled in round robin mode
         * 
         * @return  True - Device enabled
         *          False - Device disabled
         */
        bool IsEnabled() const;

        /**
         * Turn automatic mode on/off.
         * 
         * When automatic mode is on, all enabled sensors will
         * be polled at a set rate.
         * 
         * @param enabling  Set to true if round robin scheduling should start. This 
         *                  will command all sensors to begin measurements in 
         *                  continuous ranging mode.
         * 
         *                  Setting to false will command all sensors to stop 
         *                  measurements.
         */
        void SetAutomaticMode(bool enabling);

        /**
         * Determine if the current range is valid.
         * 
         * @return True - current range is valid
         *          False - current range is not valid
         */
        bool IsRangeValid() const;

        /**
         * Get the range in current units from the sensor.
         * 
         * @param units Units in which the measurement will be returned
         * 
         * @return current range in default or user specified units.
         */
        double GetRange(DistanceUnit units = kCurrent);

        /**
         * Get the timestamp of the current measurement. 
         * Uses frc::Timer::GetFPGATimestamp()
         * 
         * @return  timestamp of current measurement
         */
        double GetTimestamp(void);

        /**
         * Enable/disable the sensor in round robin scheduling. Automatic mode
         * must be separately enabled/disabled.
         * 
         * @pre SetAutomaticMode(true) called
         * 
         * @param enable    true - enable device
         *                  false - disable device
         */
        void SetEnabled(bool enable);

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
         * @param RangeProfile The range profile to set in the sensor
         */
        bool SetRangeProfile(RangeProfile);

        /**
         * Set the measurement period for the round robin scheduling loop.
         * 
         * @param period    Delay in seconds used by the round robin scheduling
         *                  loop. Should be set to a value larger than the 
         *                  measurement timing budget configured in the device, or
         *                  measurements will likely return not ready.
         */
        void SetMeasurementPeriod(double period);

        /**
         * Get the measurement period for the round robin scheduling loop
         * 
         * @return measurement period in seconds.
         */
        double GetMeasurementPeriod(void);

        /**
         * Set the current DistanceUnit that should be used for the PIDSource base
         * object.
         *
         * @param units The DistanceUnit that should be used.
         */
        void SetDistanceUnits(DistanceUnit units);

        /**
         * Get the current DistanceUnit that is used for the PIDSource base object.
         *
         * @return The type of DistanceUnit that is being used.
         */
        DistanceUnit GetDistanceUnits() const;

        /**
         * Get the range in the current DistanceUnit for the PIDSource base object.
         *
         * @return The range in DistanceUnit
         */
        double PIDGet() override;

        void SetPIDSourceType(frc::PIDSourceType pidSource) override;

        void InitSendable(frc::SendableBuilder& builder) override;

    private:
        bool Initialize(RangeProfile);
        static void DoContinuous(void);
        int32_t ValidateI2C(void);
        double GetRangeMM(void);
        double GetRangeInches(void);
        bool SetProfileLongRange(void);
        bool SetProfileHighAccuracy(void);
        bool SetProfileHighSpeed(void);
        bool SetProfileDefault(void);
        void StartMeasurement(void);
        bool GetMeasurementDataReady(void);
        void GetMeasurementData(void);
        void StopMeasurement(void);
        void GetStopCompletedStatus(void);
        void SetProfile(RangeProfile);

        // measurement parameters
        double m_currentRange = -1;
        double m_timestamp = -1;
        bool m_rangeValid = false;

        // VL53L0X API specific parameters
        VL53L0X_Error Status = VL53L0X_ERROR_NONE;
        VL53L0X_Dev_t *pDevice = new VL53L0X_Dev_t;
        VL53L0X_RangingMeasurementData_t *pRangingMeasurementData = new VL53L0X_RangingMeasurementData_t;

        HAL_I2CPort m_port = HAL_I2C_kInvalid;
        bool m_enabled = false;
        bool m_stopped = true;
        bool m_stopping = false;
        DistanceUnit m_units;
        RangeProfile m_profile = RangeProfile::kDefault;
        RangeProfile m_newProfile = RangeProfile::kDefault;

        static std::atomic<double> m_measurementPeriod;
        static std::thread m_thread;
        static std::vector<Rev2mDistanceSensor*> m_sensors;
        static std::atomic<bool> m_automaticEnabled;
};

}  // namespace rev