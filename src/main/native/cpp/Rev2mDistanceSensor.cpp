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

void print_range_status(VL53L0X_RangingMeasurementData_t* pRangingMeasurementData){
    char buf[VL53L0X_MAX_STRING_LENGTH];
    uint8_t RangeStatus;

    /*
     * New Range Status: data is valid when pRangingMeasurementData->RangeStatus = 0
     */

    RangeStatus = pRangingMeasurementData->RangeStatus;

    VL53L0X_GetRangeStatusString(RangeStatus, buf);
    printf("Range Status: %i : %s\n", RangeStatus, buf);

}

void PrintDeviceInfo(VL53L0X_DeviceInfo_t *DeviceInfo) {
    printf("VL53L0X_GetDeviceInfo:\n");
    printf("Device Name : %s\n", DeviceInfo->Name);
    printf("Device Type : %s\n", DeviceInfo->Type);
    printf("Device ID : %s\n", DeviceInfo->ProductId);
    printf("ProductRevisionMajor : %d\n", DeviceInfo->ProductRevisionMajor);
    printf("ProductRevisionMinor : %d\n", DeviceInfo->ProductRevisionMinor);
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

double Rev2mDistanceSensor::GetRangeMM(uint8_t *stat) {
    *stat = pRangingMeasurementData->RangeStatus;
    return pRangingMeasurementData->RangeMilliMeter;
}

double Rev2mDistanceSensor::GetRangeInches(uint8_t *stat) {
    return GetRangeMM(stat) / 25.4;
}

void Rev2mDistanceSensor::SetEnabled(bool enable) { m_enabled = enable; }

void Rev2mDistanceSensor::PrintRangeStatus(void) {
    print_range_status(pRangingMeasurementData);
}

bool Rev2mDistanceSensor::SetMeasurementTimingBudget(uint32_t budget_us) {
    if(m_automaticEnabled) {
        frc::DriverStation::ReportWarning("Measurement timing budget cannot be updated "
                                            "while autonomous is running");

        return false;
    }

    Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pDevice, budget_us);
    return Status == 0;
}

bool Rev2mDistanceSensor::GetMeasurementTimingBudget(uint32_t *budget_us) {
    if(m_automaticEnabled) {
        frc::DriverStation::ReportWarning("Measurement timing budget cannot be updated "
                                            "while autonomous is running");

        return false;
    }

    Status = VL53L0X_GetMeasurementTimingBudgetMicroSeconds(pDevice, budget_us);

    return Status == 0;
}

void Rev2mDistanceSensor::SetMeasurementPeriod(double period) {
    if(m_automaticEnabled) {
        frc::DriverStation::ReportWarning("Measurement period cannot be updated "
                                            "while autonomous is running");

        return;
    }

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
    
    #ifdef  _DEBUG
    printf("Initializing device on port: ");
    if(m_port == HAL_I2C_kOnboard)
        printf("Onboard\n");
    else
        printf("MXP\n");
    #endif

    if(ValidateI2C() != true)
        frc::DriverStation::ReportError("Error communicating with 2M sensor over I2C.");

    VL53L0X_Version_t *pVersion = new VL53L0X_Version_t;
    Status = VL53L0X_GetVersion(pVersion);
    #ifdef  _DEBUG
    if(Status != VL53L0X_ERROR_NONE)
    {
        frc::DriverStation::ReportError("Version error.");
        print_pal_error(Status);
    }
    #endif

    Status = VL53L0X_DataInit(pDevice); // Data initialization
    #ifdef _DEBUG
    if(Status != VL53L0X_ERROR_NONE)
    {
        frc::DriverStation::ReportError("Data init error.");
        print_pal_error(Status);
    }
    #endif

    VL53L0X_DeviceInfo_t *pDeviceInfo = new VL53L0X_DeviceInfo_t;
    Status = VL53L0X_GetDeviceInfo(pDevice, pDeviceInfo);
    #ifdef _DEBUG
    if(Status != VL53L0X_ERROR_NONE)
    {
        frc::DriverStation::ReportError("Get device info error.");
        print_pal_error(Status);
    }
    #endif
    
    Status = VL53L0X_StaticInit(pDevice); // Device Initialization
    #ifdef _DEBUG
    if(Status != VL53L0X_ERROR_NONE)
    {
        frc::DriverStation::ReportError("Static init error.");
        print_pal_error(Status);
    }
    #endif
    
    Status = VL53L0X_PerformRefCalibration(pDevice, &VhvSettings, &PhaseCal); // Device Initialization
    #ifdef _DEBUG
    if(Status != VL53L0X_ERROR_NONE)
    {
        frc::DriverStation::ReportError("Static init error.");
        print_pal_error(Status);
    }
    #endif

    Status = VL53L0X_PerformRefSpadManagement(pDevice, &refSpadCount, &isApertureSpads); // Device Initialization
    #ifdef _DEBUG
    if(Status != VL53L0X_ERROR_NONE)
    {
        frc::DriverStation::ReportError("Erroring performing ref spad management.");
        print_pal_error(Status);
    }
    #endif

    Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pDevice, 20000);
    #ifdef _DEBUG
    if(Status != VL53L0X_ERROR_NONE)
    {
        frc::DriverStation::ReportError("Error setting timing budget.");
        print_pal_error(Status);
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

    #ifdef _DEBUG
    if((res[0] != 0x00) || (res[1] != 0x99))
    {
        frc::DriverStation::ReportWarning("Device not in default settings.");
    }
    #endif

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

    if(enabling) {
        // command devices to automatic mode
        for(auto& sensor : m_sensors)
        {
            Status = VL53L0X_SetDeviceMode(sensor->pDevice, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
            if(Status == VL53L0X_ERROR_NONE) {
                // command device to start measurement
                Status = VL53L0X_StartMeasurement(sensor->pDevice);
                sensor->m_stopped = false;
            } 
            #ifdef _DEBUG
            else {
                frc::DriverStation::ReportWarning("Error setting automatic mode\n");
            }
            #endif
        }

        if(!m_thread.joinable())
            m_thread = std::thread(&Rev2mDistanceSensor::DoContinuous);
    } else {
        m_automaticEnabled = false;
    }
}

void Rev2mDistanceSensor::DoContinuous(void) {
    VL53L0X_Error stat = VL53L0X_ERROR_NONE;
    uint8_t NewDatReady = 0;

    while(1)
    {
        // iterate through sensors
        for(auto& sensor : m_sensors)
        {
            // if automatic is enabled
            if(m_automaticEnabled) {
                // if current sensor is enabled
                if(sensor->IsEnabled()) {
                    // check for new data
                    stat = VL53L0X_GetMeasurementDataReady(sensor->pDevice, 
                                                            &NewDatReady);

                    // if new data is ready
                    if((NewDatReady == 0x01) && (stat == VL53L0X_ERROR_NONE)) {
                        stat = VL53L0X_GetRangingMeasurementData(sensor->pDevice, 
                                    sensor->pRangingMeasurementData);
                        VL53L0X_ClearInterruptMask(sensor->pDevice, 
                                    VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);

                        if(stat == VL53L0X_ERROR_NONE) {
                            sensor->m_rangeValid = true;
                        } else {
                            sensor->m_rangeValid = false;
                        }
                    } else {
                        sensor->m_rangeValid = false;
                    }
                }
            } 
            // else automatic is not enabled
            else {
                // if sensor is not yet stopped
                if(!sensor->IsStopped()) {
                    // if sensor is not currently stopping
                    if(!sensor->m_stopping) {
                        stat = VL53L0X_StopMeasurement(sensor->pDevice);

                        if(stat == VL53L0X_ERROR_NONE) {
                            sensor->m_stopping = true;
                        }
                    }
                    // else sensor is currently stopping
                    else {
                        // check if sensor has finished
                        uint32_t StopCompleted = 0;
                        stat = VL53L0X_GetStopCompletedStatus(sensor->pDevice, &StopCompleted);

                        if ((StopCompleted == 0x00) || stat != VL53L0X_ERROR_NONE) {
                            sensor->m_stopped = true;
                            stat = VL53L0X_ClearInterruptMask(sensor->pDevice, 
                                        VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
                        }
                    }
                }
            }
        }

        frc::Wait(m_measurementPeriod);
    }
}

bool Rev2mDistanceSensor::IsStopped(void) {
    return m_stopped;
}

void Rev2mDistanceSensor::InitSendable(frc::SendableBuilder& builder) {
    uint8_t *stat;
    builder.SetSmartDashboardType("Distance");
    builder.AddDoubleProperty("Value", [=]() { return GetRangeInches(stat); }, nullptr);
}

double Rev2mDistanceSensor::PIDGet() {
    uint8_t stat;
    switch (m_units) {
        case Rev2mDistanceSensor::kInches:
            return GetRangeInches(&stat);
        case Rev2mDistanceSensor::kMilliMeters:
            return GetRangeMM(&stat);
        default:
            return 0.0;
    }
}

void Rev2mDistanceSensor::SetPIDSourceType(frc::PIDSourceType pidSource) {
    if (wpi_assert(pidSource == frc::PIDSourceType::kDisplacement)) {
        m_pidSource = pidSource;
    }
}