/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <stdint.h>

#include "frc/ErrorBase.h"
#include "frc/smartdashboard/SendableBase.h"
#include "frc/smartdashboard/SendableBuilder.h"
#include <frc/PIDSource.h>

#include <HAL/I2C.h>

#include "vl53l0x_platform.h"
#include "vl53l0x_def.h"

#include <thread>
#include <vector>

namespace rev {

/**
 * DistanceSensor class.
 *
 * The Distance sensor class is intended to be used with the RevRobotics
 * 2M Distance Sensor.
 */
class DistanceSensor : public frc::ErrorBase, public frc::SendableBase, public frc::PIDSource {
    public:
        enum Port { kOnboard = 0, kMXP };
        enum Mode { kSingleRanging = 0, kContinuousRanging = 1, kContinuousTimedRanging = 3 };
        enum DistanceUnit { kInches = 0, kMilliMeters = 1 };

        /**
         * Creat an instance of the Distance Sensor.
         *
         * @param port          The I2C port to which the device is connected.
         * @param deviceAddress The address of the device on the I2C bus.
         * @param units         The units returned in either kInches or kMilliMeters
         */
        DistanceSensor(Port port, int deviceAddress = 0x53, DistanceUnit units = kInches);

        ~DistanceSensor() override;

        DistanceSensor(DistanceSensor&&) = default;
        DistanceSensor& operator=(DistanceSensor&&) = default;

        /**
         * 
         */
        void StopMeasurements(void);

        /**
         * Check if the device is enabled in round robin mode
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
         * TODO: Figure out if this is needed or if it is better to use
         * pRangingMeasurementData->RangeStatus;
         */
        bool IsRangeValid() const;

        /**
         * Get the range in millimeters from the sensor.
         * 
         * @param stat  pointer to uint8_t that will contain the current measurement
         *              status. Will indicate if the measurement is 
         *                  255 NONE
         *                  5 HW fail
         *                  4 Phase fail
         *                  3 Min range
         *                  2 Signal Fail
         *                  1 Sigma Fail
         *                  0 Range Valid
         * 
         * @return     current range in millimeters.
         */
        double GetRangeMM(uint8_t* stat);

        /**
         * Get the range in inches from the sensor.
         * 
         * @param stat  pointer to uint8_t that will contain the current measurement
         *              status. Will indicate if the measurement is 
         *                  255 NONE
         *                  5 HW fail
         *                  4 Phase fail
         *                  3 Min range
         *                  2 Signal Fail
         *                  1 Sigma Fail
         *                  0 Range Valid
         * 
         * @return     current range in inches.
         */
        double GetRangeInches(uint8_t* stat);

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
         * Print the status of the current range measurement.
         */
        void PrintRangeStatus(void);

        /**
         * Set the range timing budget for the sensor.
         * 
         * @param budget_us The max timing budget in microseconds
         * 
         * @return  Will return true if budget is successfully set,
         *          false otherwise.
         */
        bool SetMeasurementTimingBudget(uint32_t budget_us);

        /**
         * Get the range timing budget from the sensor.
         * 
         * @param budget_us Pointer to uint32_t that will contain
         *                  the buget read back from the sensor
         * 
         * @return  Will return true if budget was successfully read,
         *          false otherwise.
         */
        bool GetMeasurementTimingBudget(uint32_t*);

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
        HAL_I2CPort m_port = HAL_I2C_kInvalid;
        int m_deviceAddress;
        VL53L0X_Error Status = VL53L0X_ERROR_NONE;
        VL53L0X_Dev_t *pDevice = new VL53L0X_Dev_t;
        VL53L0X_RangingMeasurementData_t *pRangingMeasurementData = new VL53L0X_RangingMeasurementData_t;
        bool m_rangeValid = false;
        static std::atomic<double> m_measurementPeriod;

        /**
         * 
         */
        bool Initialize(void);

        /**
         * 
         */
        static void DoContinuous(void);

        bool ValidateI2C(void);

        bool IsStopped(void);

        /**
         * 
         */
        static std::thread m_thread;

        /**
         * 
         */
        static std::vector<DistanceSensor*> m_sensors;

        /**
         * 
         */
        static std::atomic<bool> m_automaticEnabled;

        bool m_enabled = false;
        bool m_stopped = true;
        bool m_stopping = false;
        DistanceUnit m_units;
};

}  // namespace rev