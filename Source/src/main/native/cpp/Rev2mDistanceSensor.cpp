#include "Rev2mDistanceSensor.h"
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

#define USE_I2C_2V8
//#define _DEBUG_

using namespace rev;

std::atomic<bool> Rev2mDistanceSensor::m_automaticEnabled{false};
std::atomic<double> Rev2mDistanceSensor::m_measurementPeriod{0.05};
std::vector<Rev2mDistanceSensor*> Rev2mDistanceSensor::m_sensors;
std::thread Rev2mDistanceSensor::m_thread;

void print_pal_error(VL53L0X_Error Status){
    char buf[VL53L0X_MAX_STRING_LENGTH];
    VL53L0X_GetPalErrorString(Status, buf);
    printf("API Status: %i : %s\n", Status, buf);
}

Rev2mDistanceSensor::Rev2mDistanceSensor(Port port, int deviceAddress, DistanceUnit units)
    : m_port(static_cast<HAL_I2CPort>(port)), m_deviceAddress(deviceAddress) {

    pDevice->I2cDevAddr = deviceAddress;
    pDevice->port = m_port;
    m_units = units;

    int32_t status = 0;
    HAL_InitializeI2C(m_port, &status);

    HAL_Report(HALUsageReporting::kResourceType_I2C, deviceAddress);

    if(!Initialize())
        frc::DriverStation::ReportWarning("Error initializing Rev 2M device.");
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

void Rev2mDistanceSensor::SetRangeProfile(RangeProfile profile) {
    uint32_t budget;

    if (profile == RangeProfile::kHighAccuracy) {
        m_profile = RangeProfile::kHighAccuracy;
        budget = 200000;
    }
    else if (profile == RangeProfile::kLongRange) {
        m_profile = RangeProfile::kLongRange;
        budget = 33000;
    }
    else if (profile == RangeProfile::kHighSpeed) {
        m_profile = RangeProfile::kLongRange;
        budget = 20000;
    }
    else {
        m_profile = RangeProfile::kLongRange;
        budget = 20000;
    }

    if(m_stopped) {
        #ifdef _DEBUG_
        printf("Sensor stopped. Setting timing budget\n");
        #endif
        SetMeasurementTimingBudget(budget);
    }
    else {
        #ifdef _DEBUG_
        printf("Sensor disabled. Setting budget to %d\n", budget);
        #endif
        m_enabled = false;
        m_newMeasurementTimingBudget = budget;
    }
}

void Rev2mDistanceSensor::SetMeasurementTimingBudget(uint32_t budget_us) {
    Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pDevice, budget_us);

    if(Status == VL53L0X_ERROR_NONE) {
        m_currentMeasurementTimingBudget = budget_us;
        m_newMeasurementTimingBudget = budget_us;
        #ifdef _DEBUG_
        printf("Timing budget changed to %dus\n", m_currentMeasurementTimingBudget);
        #endif
    }
    else {
        #ifdef _DEBUG_
        printf("Error setting timing buget\n");
        #endif
    }
}

bool Rev2mDistanceSensor::GetMeasurementTimingBudget(uint32_t *budget_us) {
    Status = VL53L0X_GetMeasurementTimingBudgetMicroSeconds(pDevice, budget_us);

    return Status == 0;
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

bool Rev2mDistanceSensor::Initialize(void) {
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
    
    #ifdef  _DEBUG_
    printf("Initializing device on port: ");
    if(m_port == HAL_I2C_kOnboard)
        printf("Onboard\n");
    else
        printf("MXP\n");
    #endif

    if(ValidateI2C() != true)
        frc::DriverStation::ReportError("Error communicating with Rev 2M sensor over I2C.");

    VL53L0X_Version_t *pVersion = new VL53L0X_Version_t;
    Status = VL53L0X_GetVersion(pVersion);
    #ifdef  _DEBUG_
    if(Status != VL53L0X_ERROR_NONE) {
        frc::DriverStation::ReportError("Version error.");
        print_pal_error(Status);
    }
    #endif

    Status = VL53L0X_DataInit(pDevice); // Data initialization
    #ifdef _DEBUG_
    if(Status != VL53L0X_ERROR_NONE) {
        frc::DriverStation::ReportError("Data init error.");
        print_pal_error(Status);
    }
    #endif

    VL53L0X_DeviceInfo_t *pDeviceInfo = new VL53L0X_DeviceInfo_t;
    Status = VL53L0X_GetDeviceInfo(pDevice, pDeviceInfo);
    #ifdef _DEBUG_
    if(Status != VL53L0X_ERROR_NONE) {
        frc::DriverStation::ReportError("Get device info error.");
        print_pal_error(Status);
    }
    #endif
    
    Status = VL53L0X_StaticInit(pDevice); // Device Initialization
    #ifdef _DEBUG_
    if(Status != VL53L0X_ERROR_NONE) {
        frc::DriverStation::ReportError("Static init error.");
        print_pal_error(Status);
    }
    #endif
    
    Status = VL53L0X_PerformRefCalibration(pDevice, &VhvSettings, &PhaseCal); // Device Initialization
    #ifdef _DEBUG_
    if(Status != VL53L0X_ERROR_NONE) {
        frc::DriverStation::ReportError("Static init error.");
        print_pal_error(Status);
    }
    #endif

    Status = VL53L0X_PerformRefSpadManagement(pDevice, &refSpadCount, &isApertureSpads); // Device Initialization
    #ifdef _DEBUG_
    if(Status != VL53L0X_ERROR_NONE) {
        frc::DriverStation::ReportError("Erroring performing ref spad management.");
        print_pal_error(Status);
    }
    #endif

    Status = VL53L0X_SetDeviceMode(pDevice, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    #ifdef _DEBUG_
    if(Status != VL53L0X_ERROR_NONE) {
        frc::DriverStation::ReportWarning("Error starting Rev 2M sensor\n");
    }
    #endif

    m_sensors.emplace_back(this);
    m_enabled = true;

    // return true if initialization was successful
    return Status >= 0;
}

bool Rev2mDistanceSensor::ValidateI2C(void) {
    uint8_t reg = 0xC0, res[2];

    HAL_TransactionI2C(pDevice->port, pDevice->I2cDevAddr >> 1, &reg, 1, res, 1);

    if(*res != 0xEE)
        return false;

    reg = 0xC1;
    HAL_TransactionI2C(pDevice->port, pDevice->I2cDevAddr >> 1, &reg, 1, res, 1);

    if(*res != 0xAA)
        return false;

    reg = 0xC2;
    HAL_TransactionI2C(pDevice->port, pDevice->I2cDevAddr >> 1, &reg, 1, res, 1);

    if(*res != 0x10)
        return false;

    reg = 0x51;
    HAL_TransactionI2C(pDevice->port, pDevice->I2cDevAddr >> 1, &reg, 1, res, 2);

    reg = 0x61;
    HAL_TransactionI2C(pDevice->port, pDevice->I2cDevAddr >> 1, &reg, 1, res, 2);

    if((res[0] != 0x00) || (res[1] != 0x00))
        return false;

    return true;
}

bool Rev2mDistanceSensor::IsEnabled() const { return m_enabled; }

void Rev2mDistanceSensor::SetAutomaticMode(bool enabling)
{
    if (enabling == m_automaticEnabled) return;  // ignore the case of no change

    m_automaticEnabled = enabling;

    // if automatic is being enabled
    if(enabling) {
        // if thread is not currently running, start it
        if(!m_thread.joinable())
            #ifdef _DEBUG_
            printf("Starting thread\n");
            #endif
            m_thread = std::thread(&Rev2mDistanceSensor::DoContinuous);
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
    VL53L0X_Error stat = VL53L0X_ERROR_NONE;
    VL53L0X_RangingMeasurementData_t *pRangingMeasurementData = new VL53L0X_RangingMeasurementData_t;
    uint8_t NewDatReady = 0;
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
                    // command device to start measurement
                    stat = VL53L0X_StartMeasurement(sensor->pDevice);

                    if(stat == VL53L0X_ERROR_NONE)
                        sensor->m_stopped = false;
                }
                // else sensor has been started
                else {
                    // check for new data
                    stat = VL53L0X_GetMeasurementDataReady(sensor->pDevice, 
                                                            &NewDatReady);

                    // if new data is ready
                    if((NewDatReady == 0x01) && (stat == VL53L0X_ERROR_NONE)) {
                        stat = VL53L0X_GetRangingMeasurementData(sensor->pDevice, 
                                    pRangingMeasurementData);
                        VL53L0X_ClearInterruptMask(sensor->pDevice, 
                                    VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);

                        if(stat == VL53L0X_ERROR_NONE) {
                            // range is valid when RangeStatus equals 0
                            sensor->m_rangeValid = pRangingMeasurementData->RangeStatus == 0;
                            sensor->m_currentRange = pRangingMeasurementData->RangeMilliMeter;
                            sensor->m_timestamp = frc::Timer::GetFPGATimestamp();
                        } else {
                            sensor->m_rangeValid = false;
                        }
                    } else {
                        sensor->m_rangeValid = false;
                    }
                }
            } 
            // else automatic is not enabled
            else if(!sensor->m_stopped) {
                // if sensor is not currently stopping
                if(!sensor->m_stopping) {
                    // command device to stop measurements
                    stat = VL53L0X_StopMeasurement(sensor->pDevice);

                    if(stat == VL53L0X_ERROR_NONE)
                        sensor->m_stopping = true;
                }
                // else sensor is currently stopping
                else {
                    // check if sensor has finished
                    uint32_t StopCompleted = 0;
                    stat = VL53L0X_GetStopCompletedStatus(sensor->pDevice, &StopCompleted);

                    if ((StopCompleted == 0x00) || stat != VL53L0X_ERROR_NONE) {
                        sensor->m_stopped = true;
                        sensor->m_stopping = false;
                        stat = VL53L0X_ClearInterruptMask(sensor->pDevice, 
                                    VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
                    }
                }
            }
            // else if sensor is stopped and measurement timing budget needs to be changed
            else if(sensor->m_newMeasurementTimingBudget != sensor->m_currentMeasurementTimingBudget) {
                sensor->SetMeasurementTimingBudget(sensor->m_newMeasurementTimingBudget);

                if(sensor->m_newMeasurementTimingBudget == sensor->m_currentMeasurementTimingBudget) {
                    // if timing budget is successfully set, reenable sensor
                    sensor->m_enabled = true;
                }
            }

            // check if there are any sensors still enabled
            if(!sensor->m_stopped)
                allStopped = false;
        }

        frc::Wait(m_measurementPeriod);
    }
    while(!allStopped && m_automaticEnabled);

    #ifdef _DEBUG_
    printf("Stopping thread\n");
    #endif

    // all sensors stopped and automatic is disabled so detach thread
    m_thread.detach();
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