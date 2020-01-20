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

#include "rev/Rev2mDistanceSensor.h"
#include "vl53l0x_api.h"

#include <hal/HAL.h>
#include <hal/I2C.h>

#include <thread>
#include <atomic>
#include <vector>
#include <memory>

#include <frc/Timer.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SendableBase.h>
#include <frc/smartdashboard/SendableBuilder.h>
#include <frc/Utility.h>
#include <wpi/SmallString.h>
#include <wpi/raw_ostream.h>
#include <wpi/Format.h>

//#define _DEBUG_
#define VALIDATE_I2C_SUCCESS    0
#define VALIDATE_I2C_PARAM_ERR  1
#define VALIDATE_I2C_HAL_ERR    2

using namespace rev;

constexpr int Rev2mDistanceSensorAddress = 0x53;
constexpr double defaultMeasurementPeriod = 0.05;

std::atomic<bool> Rev2mDistanceSensor::m_automaticEnabled{false};
std::atomic<double> Rev2mDistanceSensor::m_measurementPeriod{defaultMeasurementPeriod};
std::vector<Rev2mDistanceSensor*> Rev2mDistanceSensor::m_sensors;
std::thread Rev2mDistanceSensor::m_thread;

void print_pal_error(VL53L0X_Error Status){
    char buf[VL53L0X_MAX_STRING_LENGTH];
    VL53L0X_GetPalErrorString(Status, buf);
    printf("API Status: %i : %s\n", Status, buf);
}

void print_pal_state(VL53L0X_State PalState) {
    char buf[VL53L0X_MAX_STRING_LENGTH];
    VL53L0X_GetPalStateString(PalState, buf);
    printf("Pal Status: %s\n", buf);
}

Rev2mDistanceSensor::Rev2mDistanceSensor(Port port, DistanceUnit units, RangeProfile profile)
    : m_port(static_cast<HAL_I2CPort>(port)) {

    pDevice->I2cDevAddr = Rev2mDistanceSensorAddress;
    pDevice->port = m_port;
    m_units = units;

    int32_t status = 0;
    HAL_InitializeI2C(m_port, &status);

    HAL_Report(HALUsageReporting::kResourceType_I2C, Rev2mDistanceSensorAddress);

    if(!Initialize(profile)) {
        wpi::SmallString<255> buf;
        wpi::raw_svector_ostream errorString{buf};
        errorString << "Error initializing Rev 2M device on port " 
                    << wpi::format("%s", port == Port::kMXP ? "MXP" : "Onboard")
                    << ". Please check your connections.";

        frc::DriverStation::ReportError(errorString.str());
    }
}

Rev2mDistanceSensor::~Rev2mDistanceSensor() { 
    bool wasAutomaticMode = m_automaticEnabled;
    SetAutomaticMode(false);

    // No synchronization needed because the background task is stopped.
    m_sensors.erase(std::remove(m_sensors.begin(), m_sensors.end(), this),
                    m_sensors.end());

    if (!m_sensors.empty() && wasAutomaticMode) {
        SetAutomaticMode(true);
    }    
}

bool Rev2mDistanceSensor::IsRangeValid() const {
    return m_rangeValid;
}

double Rev2mDistanceSensor::GetRange(DistanceUnit units) {
    if(units == kCurrent)
        units = m_units;

    switch (units) {
        case Rev2mDistanceSensor::kInches:
            return GetRangeInches();
        case Rev2mDistanceSensor::kMilliMeters:
            return GetRangeMM();
        default:
            return -1;
    }
}

double Rev2mDistanceSensor::GetTimestamp(void) {
    return m_timestamp;
}

double Rev2mDistanceSensor::GetRangeMM() {
    return m_currentRange;
}

double Rev2mDistanceSensor::GetRangeInches() {
    return m_currentRange / 25.4;
}

void Rev2mDistanceSensor::SetEnabled(bool enable) { m_enabled = enable; }

bool Rev2mDistanceSensor::SetRangeProfile(RangeProfile profile) {
    if(profile == m_profile) return true; // ignore the case of no change

    m_newProfile = profile;

    if(m_stopped && !m_automaticEnabled) {
        #ifdef _DEBUG_
        printf("Sensor stopped. Changing profile\n");
        #endif
        if (profile == RangeProfile::kHighAccuracy) 
            return SetProfileHighAccuracy();
        else if (profile == RangeProfile::kLongRange)
            return SetProfileLongRange();
        else if (profile == RangeProfile::kHighSpeed)
            return SetProfileHighSpeed();
        else
            return SetProfileDefault();
    }
    else {
        #ifdef _DEBUG_
        printf("Sensor not stopped. Disabling\n");
        #endif
        m_enabled = false;
        return false;
    }
}

void Rev2mDistanceSensor::SetMeasurementPeriod(double period) {
    if(period < 0.01) {
        period = 0.01;
    } 
    else if(period > 1) {
        period = 1;
    }
    
    m_measurementPeriod = period;
}

double Rev2mDistanceSensor::GetMeasurementPeriod(void) {
    return m_measurementPeriod;
}

void Rev2mDistanceSensor::SetDistanceUnits(DistanceUnit units) { m_units = units; }

Rev2mDistanceSensor::DistanceUnit Rev2mDistanceSensor::GetDistanceUnits() const {
  return m_units;
}

bool Rev2mDistanceSensor::Initialize(RangeProfile profile) {
    Status = VL53L0X_ERROR_NONE;
    VL53L0X_Version_t *pVersion = new VL53L0X_Version_t;
    VL53L0X_DeviceInfo_t *pDeviceInfo = new VL53L0X_DeviceInfo_t;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
    uint32_t I2C_status;
    
    #ifdef  _DEBUG_
    printf("Initializing device on port: ");
    if(m_port == HAL_I2C_kOnboard)
        printf("Onboard\n");
    else
        printf("MXP\n");
    #endif

    if((I2C_status = ValidateI2C()) != VALIDATE_I2C_SUCCESS) {
        wpi::SmallString<255> buf;
        wpi::raw_svector_ostream errorString{buf};
        errorString << "Error " << wpi::format("0x%08X", I2C_status)
                    << ": Could not communicate with Rev 2M sensor over I2C.";

        frc::DriverStation::ReportError(errorString.str());
    }

    Status = VL53L0X_GetVersion(pVersion);
    #ifdef  _DEBUG_
    if(Status != VL53L0X_ERROR_NONE) {
        frc::DriverStation::ReportError("Version error.");
        print_pal_error(Status);
    }
    #endif

    if(Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_DataInit(pDevice); // Data initialization
    }
    #ifdef _DEBUG_
    else {
        print_pal_error(Status);
    }
    #endif

    if(Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_GetDeviceInfo(pDevice, pDeviceInfo);
    }
    #ifdef _DEBUG_
    else {
        print_pal_error(Status);
    }
    #endif
    
    if(Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_StaticInit(pDevice); // Device Initialization
    }
    #ifdef _DEBUG_
    else {
        print_pal_error(Status);
    }
    #endif
    
    if(Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_PerformRefCalibration(pDevice, &VhvSettings, &PhaseCal); // Device Initialization
    }
    #ifdef _DEBUG_
    else {
        print_pal_error(Status);
    }
    #endif

    if(Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_PerformRefSpadManagement(pDevice, &refSpadCount, &isApertureSpads); // Device Initialization
    }
    #ifdef _DEBUG_
    else {
        print_pal_error(Status);
    }
    #endif

    if(Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetDeviceMode(pDevice, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    }
    #ifdef _DEBUG_
    else {
        print_pal_error(Status);
    }
    #endif

    if(Status == VL53L0X_ERROR_NONE) {
        if(SetRangeProfile(profile)) {
            m_sensors.emplace_back(this);
            m_enabled = true;
        }
        #ifdef _DEBUG_
        else {
            frc::DriverStation::ReportWarning("Error setting range profile\n");
        }
        #endif
    }

    // return true if initialization was successful
    return Status >= 0;
}

int32_t Rev2mDistanceSensor::ValidateI2C(void) {
    uint8_t reg = 0xC0, res[2];

    if(HAL_TransactionI2C(pDevice->port, pDevice->I2cDevAddr >> 1, &reg, 1, res, 1) < 0)
        return VALIDATE_I2C_HAL_ERR | (1 << 24);

    if(*res != 0xEE)
        return VALIDATE_I2C_PARAM_ERR | (((uint32_t) reg) << 8) | (((uint32_t) res[0]) << 16);

    reg = 0xC1;
    if(HAL_TransactionI2C(pDevice->port, pDevice->I2cDevAddr >> 1, &reg, 1, res, 1) < 0)
        return VALIDATE_I2C_HAL_ERR | (1 << 25);

    if(*res != 0xAA)
        return VALIDATE_I2C_PARAM_ERR | (((uint32_t) reg) << 8) | (((uint32_t) res[0]) << 16);

    reg = 0xC2;
    if(HAL_TransactionI2C(pDevice->port, pDevice->I2cDevAddr >> 1, &reg, 1, res, 1) < 0)
        return VALIDATE_I2C_HAL_ERR | (1 << 26);

    if(*res != 0x10)
        return VALIDATE_I2C_PARAM_ERR | (((uint32_t) reg) << 8) | (((uint32_t) res[0]) << 16);

    reg = 0x51;
    if(HAL_TransactionI2C(pDevice->port, pDevice->I2cDevAddr >> 1, &reg, 1, res, 2) < 0)
        return VALIDATE_I2C_HAL_ERR | (1 << 27);

    reg = 0x61;
    if(HAL_TransactionI2C(pDevice->port, pDevice->I2cDevAddr >> 1, &reg, 1, res, 2) < 0)
        return VALIDATE_I2C_HAL_ERR | (1 << 28);

    if((res[0] != 0x00) || (res[1] != 0x00))
        return VALIDATE_I2C_PARAM_ERR | (((uint32_t) reg) << 8) | (((uint32_t) res[0]) << 16) | (((uint32_t) res[1]) << 24);

    return VALIDATE_I2C_SUCCESS;
}

bool Rev2mDistanceSensor::IsEnabled() const { return m_enabled; }

void Rev2mDistanceSensor::SetAutomaticMode(bool enabling)
{
    if (enabling == m_automaticEnabled) return;  // ignore the case of no change

    m_automaticEnabled = enabling;

    // if automatic is being enabled
    if(enabling) {
        // if thread is not currently running, start it
        if(!m_thread.joinable()) {
            #ifdef _DEBUG_
            printf("Starting thread\n");
            #endif
            m_thread = std::thread(&Rev2mDistanceSensor::DoContinuous);
        }
    } 
    // else automatic mode is being disabled
    else {
        m_automaticEnabled = false;

        // disable all sensors
        for(auto& sensor : m_sensors)
            sensor->SetEnabled(false);
    }
}

void Rev2mDistanceSensor::DoContinuous(void) {
    bool allStopped;

    #ifdef _DEBUG_
    printf("Thread started\n");
    #endif

    do {
        // used to stop loop. allStopped will be changed to false if any sensors still running
        allStopped = true;

        // iterate through sensors
        for(auto& sensor : m_sensors)
        {
            // if sensor is enabled
            if(sensor->IsEnabled()) {
                // if sensor has not been started yet
                if(sensor->m_stopped) {
                    sensor->StartMeasurement();
                }
                // else sensor has been started
                else {
                    sensor->GetMeasurementData();
                }
            } 
            // else automatic is not enabled
            else if(!sensor->m_stopped) {
                // if sensor is not currently stopping
                if(!sensor->m_stopping) {
                    // command device to stop measurements
                    sensor->StopMeasurement();
                }
                // else sensor is currently stopping
                else {
                    // check if sensor has finished
                    sensor->GetStopCompletedStatus();
                }
            }
            // else if sensor is stopped and measurement timing budget needs to be changed
            else if(sensor->m_newProfile != sensor->m_profile) {
                allStopped = false;
                sensor->SetProfile(sensor->m_newProfile);
            }

            // check if there are any sensors still enabled
            if(!sensor->m_stopped) {
                allStopped = false;
            } 
            #ifdef _DEBUG_
            else {
                printf("All sensors stopped\n");
            }
            #endif
        }

        frc::Wait(m_measurementPeriod);
    }
    while(!allStopped || m_automaticEnabled);

    #ifdef _DEBUG_
    printf("Stopping thread\n");
    #endif

    // all sensors stopped and automatic is disabled so detach thread
    m_thread.detach();
}

void Rev2mDistanceSensor::StartMeasurement(void) {
    // command device to start measurement
    VL53L0X_Error stat = VL53L0X_StartMeasurement(pDevice);

    if(stat == VL53L0X_ERROR_NONE) {
        m_stopped = false;
        #ifdef _DEBUG_
        printf("Sensor started\n");
        #endif
    }
}

bool Rev2mDistanceSensor::GetMeasurementDataReady(void) {
    // check for new data
    uint8_t NewDatReady = 0;
    VL53L0X_Error stat = VL53L0X_GetMeasurementDataReady(pDevice, 
                                                         &NewDatReady);

    // if new data is ready
    return ((NewDatReady == 0x01) && (stat == VL53L0X_ERROR_NONE));
}

void Rev2mDistanceSensor::GetMeasurementData(void) {
    if(GetMeasurementDataReady()) {
        
        VL53L0X_Error stat = VL53L0X_GetRangingMeasurementData(pDevice, 
                                pRangingMeasurementData);

        VL53L0X_ClearInterruptMask(pDevice, 
                    VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);

        if(stat == VL53L0X_ERROR_NONE) {
            // range is valid when RangeStatus equals 0
            m_rangeValid = pRangingMeasurementData->RangeStatus == 0;
            m_currentRange = pRangingMeasurementData->RangeMilliMeter;
            m_timestamp = frc::Timer::GetFPGATimestamp();
        } else {
            m_rangeValid = false;
        }
    } 
    else {
        m_rangeValid = false;
    }
}

void Rev2mDistanceSensor::StopMeasurement(void) {
    #ifdef _DEBUG_
    printf("Sensor Stopping\n");
    #endif
    VL53L0X_Error stat = VL53L0X_StopMeasurement(pDevice);

    if(stat == VL53L0X_ERROR_NONE) {
        m_stopping = true;
    }
}

void Rev2mDistanceSensor::GetStopCompletedStatus(void) {
    uint32_t StopCompleted = 0;
    VL53L0X_Error stat = VL53L0X_GetStopCompletedStatus(pDevice, &StopCompleted);

    if((StopCompleted == 0x00) || stat != VL53L0X_ERROR_NONE) {
        m_stopped = true;
        m_stopping = false;
        VL53L0X_ClearInterruptMask(pDevice, 
                    VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);

        #ifdef _DEBUG_
        printf("Sensor Stopped\n");
        #endif
    }
    #ifdef _DEBUG_
    else {
        printf("Sensor not stopped yet.\n");
    }
    #endif
}

void Rev2mDistanceSensor::SetProfile(RangeProfile profile) {
    #ifdef _DEBUG_
    printf("Changing profile in autonomous\n");
    #endif

    if (m_newProfile == RangeProfile::kHighAccuracy) 
        m_enabled = SetProfileHighAccuracy();
    else if (m_newProfile == RangeProfile::kLongRange)
        m_enabled = SetProfileLongRange();
    else if (m_newProfile == RangeProfile::kHighSpeed)
        m_enabled = SetProfileHighSpeed();
    else
        m_enabled = SetProfileDefault();

    #ifdef _DEBUG_
    if(m_enabled)
        printf("Sensor reenabled\n");
    #endif
}

void Rev2mDistanceSensor::InitSendable(frc::SendableBuilder& builder) {
    builder.SetSmartDashboardType("Distance");
    builder.AddDoubleProperty("Value", [=]() { return GetRange(); }, nullptr);
}

double Rev2mDistanceSensor::PIDGet() {
    switch (m_units) {
        case Rev2mDistanceSensor::kInches:
            return GetRangeInches();
        case Rev2mDistanceSensor::kMilliMeters:
            return GetRangeMM();
        default:
            return 0.0;
    }
}

void Rev2mDistanceSensor::SetPIDSourceType(frc::PIDSourceType pidSource) {
    if (wpi_assert(pidSource == frc::PIDSourceType::kDisplacement)) {
        m_pidSource = pidSource;
    }
}

bool Rev2mDistanceSensor::SetProfileLongRange(void) {
    #ifdef _DEBUG_
    printf("Setting profile to long range\n");
    #endif
    Status = VL53L0X_SetLimitCheckEnable(pDevice,
            VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(pDevice,
        		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(pDevice,
        		VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 0);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckValue(pDevice,
        		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
        		(FixPoint1616_t)(60*65536));
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckValue(pDevice,
        		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
        		(FixPoint1616_t)(0.1*65536));
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pDevice,
        		33000);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetVcselPulsePeriod(pDevice, 
		        VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetVcselPulsePeriod(pDevice, 
		        VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        #ifdef _DEBUG_
        printf("Profile changed to long range\n");
        #endif
        m_profile = RangeProfile::kLongRange;
    }

    return Status == VL53L0X_ERROR_NONE;
}

bool Rev2mDistanceSensor::SetProfileHighAccuracy(void) {
    #ifdef _DEBUG_
    printf("Setting profile to high accuracy\n");
    #endif
    Status = VL53L0X_SetLimitCheckEnable(pDevice,
            VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(pDevice,
        		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(pDevice,
        		VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 0);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckValue(pDevice,
        		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
        		(FixPoint1616_t)(18*65536));
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckValue(pDevice,
        		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
        		(FixPoint1616_t)(0.25*65536));
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pDevice,
        		200000);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetVcselPulsePeriod(pDevice, 
		        VL53L0X_VCSEL_PERIOD_PRE_RANGE, 14);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetVcselPulsePeriod(pDevice, 
		        VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 10);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        #ifdef _DEBUG_
        printf("Profile changed to high accuracy\n");
        #endif
        m_profile = RangeProfile::kHighAccuracy;
    }

    return Status == VL53L0X_ERROR_NONE;
}

bool Rev2mDistanceSensor::SetProfileHighSpeed(void) {
    #ifdef _DEBUG_
    printf("Setting profile to high speed\n");
    #endif
    Status = VL53L0X_SetLimitCheckEnable(pDevice,
            VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(pDevice,
        		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(pDevice,
        		VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 0);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckValue(pDevice,
        		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
        		(FixPoint1616_t)(32*65536));
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckValue(pDevice,
        		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
        		(FixPoint1616_t)(0.25*65536));
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pDevice,
        		30000);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetVcselPulsePeriod(pDevice, 
		        VL53L0X_VCSEL_PERIOD_PRE_RANGE, 14);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetVcselPulsePeriod(pDevice, 
		        VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 10);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        #ifdef _DEBUG_
        printf("Profile changed to high range\n");
        #endif
        m_profile = RangeProfile::kHighSpeed;
    }

    return Status == VL53L0X_ERROR_NONE;
}

bool Rev2mDistanceSensor::SetProfileDefault(void) {
    #ifdef _DEBUG_
    printf("Setting profile to default\n");
    #endif
    Status = VL53L0X_SetLimitCheckEnable(pDevice,
            VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(pDevice,
        		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(pDevice,
        		VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 0);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckValue(pDevice,
        		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
        		(FixPoint1616_t)(18*65536));
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckValue(pDevice,
        		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
        		(FixPoint1616_t)(0.25*65536));
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pDevice,
        		33823);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetVcselPulsePeriod(pDevice, 
		        VL53L0X_VCSEL_PERIOD_PRE_RANGE, 14);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetVcselPulsePeriod(pDevice, 
		        VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 10);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        #ifdef _DEBUG_
        printf("Profile changed to default\n");
        #endif
        m_profile = RangeProfile::kDefault;
    }

    return Status == VL53L0X_ERROR_NONE;
}