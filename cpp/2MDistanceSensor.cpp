#include "2MDistanceSensor.h"

#include <hal/HAL.h>
#include <hal/I2C.h>
#include <frc/Watchdog.h>

#include "vl53l0x_api.h"

#include <thread>
#include <atomic>
#include <vector>
#include <memory>

#include <hal/HAL.h>

#include "frc/Timer.h"
#include <frc/DriverStation.h>
#include "frc/smartdashboard/SendableBase.h"
#include "frc/smartdashboard/SendableBuilder.h"
#include <frc/Utility.h>

#define USE_I2C_2V8

using namespace rev;

std::atomic<bool> DistanceSensor::m_automaticEnabled{false};
std::atomic<double> DistanceSensor::m_measurementPeriod{0.05};
std::vector<DistanceSensor*> DistanceSensor::m_sensors;
std::thread DistanceSensor::m_thread;

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

void DistanceSensor::PrintRangeStatus(void) {
    print_range_status(pRangingMeasurementData);
}

DistanceSensor::DistanceSensor(Port port, int deviceAddress, DistanceUnit units)
    : m_port(static_cast<HAL_I2CPort>(port)), m_deviceAddress(deviceAddress) {

    pDevice->I2cDevAddr = deviceAddress;
    pDevice->port = m_port;
    m_units = units;

    int32_t status = 0;
    HAL_InitializeI2C(m_port, &status);

    HAL_Report(HALUsageReporting::kResourceType_I2C, deviceAddress);

    if(!Initialize())
        frc::DriverStation::ReportWarning("Error initializing device.");
}

DistanceSensor::~DistanceSensor() { 
    bool wasAutomaticMode = m_automaticEnabled;
    SetAutomaticMode(false);

    // No synchronization needed because the background task is stopped.
    m_sensors.erase(std::remove(m_sensors.begin(), m_sensors.end(), this),
                    m_sensors.end());

    if (!m_sensors.empty() && wasAutomaticMode) {
        SetAutomaticMode(true);
    }    
}

bool DistanceSensor::Initialize(void) {
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    printf("Initializing device on port: ");
    if(m_port == HAL_I2C_kOnboard)
        printf("Onboard\n");
    else
        printf("MXP\n");

    if(ValidateI2C() != true)
        frc::DriverStation::ReportError("Error communicating with 2M sensor over I2C.");

    VL53L0X_Version_t *pVersion = new VL53L0X_Version_t;
    Status = VL53L0X_GetVersion(pVersion);
    if(Status != VL53L0X_ERROR_NONE)
    {
        frc::DriverStation::ReportError("Version error.");
        print_pal_error(Status);
    }

    // FIXME: Add version check?

    Status = VL53L0X_DataInit(pDevice); // Data initialization
    if(Status != VL53L0X_ERROR_NONE)
    {
        frc::DriverStation::ReportError("Data init error.");
        print_pal_error(Status);
    }

    VL53L0X_DeviceInfo_t *pDeviceInfo = new VL53L0X_DeviceInfo_t;
    Status = VL53L0X_GetDeviceInfo(pDevice, pDeviceInfo);
    if(Status != VL53L0X_ERROR_NONE)
    {
        frc::DriverStation::ReportError("Get device info error.");
        print_pal_error(Status);
    }
    
    Status = VL53L0X_StaticInit(pDevice); // Device Initialization
    if(Status != VL53L0X_ERROR_NONE)
    {
        frc::DriverStation::ReportError("Static init error.");
        print_pal_error(Status);
    }
    
    Status = VL53L0X_PerformRefCalibration(pDevice, &VhvSettings, &PhaseCal); // Device Initialization
    if(Status != VL53L0X_ERROR_NONE)
    {
        frc::DriverStation::ReportError("Static init error.");
        print_pal_error(Status);
    }

    Status = VL53L0X_PerformRefSpadManagement(pDevice, &refSpadCount, &isApertureSpads); // Device Initialization
    if(Status != VL53L0X_ERROR_NONE)
    {
        frc::DriverStation::ReportError("Erroring performing ref spad management.");
        print_pal_error(Status);
    }

    Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pDevice, 20000);
    if(Status != VL53L0X_ERROR_NONE)
    {
        frc::DriverStation::ReportError("Error setting timing budget.");
        print_pal_error(Status);
    }

    m_sensors.emplace_back(this);
    m_enabled = true;

    // return true if initialization was successful
    return Status >= 0;
}

bool DistanceSensor::ValidateI2C(void) {
    uint8_t reg = 0xC0, res[2];
    uint16_t res2;

    HAL_TransactionI2C(pDevice->port, pDevice->I2cDevAddr >> 1, &reg, 1, res, 1);
    //printf("Reg: %x\tRes: %x\n", reg, *res);

    if(*res != 0xEE)
        return false;

    reg = 0xC1;
    HAL_TransactionI2C(pDevice->port, pDevice->I2cDevAddr >> 1, &reg, 1, res, 1);
    //printf("Reg: %x\tRes: %x\n", reg, *res);

    if(*res != 0xAA)
        return false;


    reg = 0xC2;
    HAL_TransactionI2C(pDevice->port, pDevice->I2cDevAddr >> 1, &reg, 1, res, 1);
    //printf("Reg: %x\tRes: %x\n", reg, *res);

    if(*res != 0x10)
        return false;

    reg = 0x51;
    HAL_TransactionI2C(pDevice->port, pDevice->I2cDevAddr >> 1, &reg, 1, res, 2);
    //printf("Reg: %x\tRes: %x%x\n", reg, res[0], res[1]);

    if((res[0] != 0x00) || (res[1] != 0x99))
    {
        frc::DriverStation::ReportWarning("Device not in default settings.");
    }

    reg = 0x61;
    HAL_TransactionI2C(pDevice->port, pDevice->I2cDevAddr >> 1, &reg, 1, res, 2);
    //printf("Reg: %x\tRes: %x%x\n", reg, res[0], res[1]);

    if((res[0] != 0x00) || (res[1] != 0x00))
        return false;

    return true;
}

void DistanceSensor::SetAutomaticMode(bool enabling)
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
            } else {
                printf("Error setting automatic mode\n");
            }
        }

        if(!m_thread.joinable())
            m_thread = std::thread(&DistanceSensor::DoContinuous);
    } else {
        m_automaticEnabled = false;

        /*if (m_thread.joinable()) {
            m_thread.join();
            printf("Thread joined\n");
        }

        for(auto& sensor : m_sensors) {
            sensor->StopMeasurements();
        }*/
    }
}

bool DistanceSensor::IsEnabled() const { return m_enabled; }

void DistanceSensor::DoContinuous(void) {
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

bool DistanceSensor::IsRangeValid() const {
    return m_rangeValid;
}

double DistanceSensor::GetRangeMM(uint8_t *stat) {
    *stat = pRangingMeasurementData->RangeStatus;
    return pRangingMeasurementData->RangeMilliMeter;
}

double DistanceSensor::GetRangeInches(uint8_t *stat) {
    return GetRangeMM(stat) / 25.4;
}

void DistanceSensor::SetEnabled(bool enable) { m_enabled = enable; }

void DistanceSensor::InitSendable(frc::SendableBuilder& builder) {
    uint8_t *stat;
    builder.SetSmartDashboardType("Distance");
    builder.AddDoubleProperty("Value", [=]() { return GetRangeInches(stat); }, nullptr);
}

bool DistanceSensor::SetMeasurementTimingBudget(uint32_t budget_us) {
    if(m_automaticEnabled) {
        frc::DriverStation::ReportWarning("Measurement timing budget cannot be updated "
                                            "while autonomous is running");

        return false;
    }

    Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pDevice, budget_us);
    return Status == 0;
}

bool DistanceSensor::GetMeasurementTimingBudget(uint32_t *budget_us) {
    if(m_automaticEnabled) {
        frc::DriverStation::ReportWarning("Measurement timing budget cannot be updated "
                                            "while autonomous is running");

        return false;
    }

    Status = VL53L0X_GetMeasurementTimingBudgetMicroSeconds(pDevice, budget_us);

    return Status == 0;
}

void DistanceSensor::SetMeasurementPeriod(double period) {
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

double DistanceSensor::GetMeasurementPeriod(void) {
    return m_measurementPeriod;
}

bool DistanceSensor::IsStopped(void) {
    return m_stopped;
}

void DistanceSensor::SetDistanceUnits(DistanceUnit units) { m_units = units; }

DistanceSensor::DistanceUnit DistanceSensor::GetDistanceUnits() const {
  return m_units;
}

double DistanceSensor::PIDGet() {
    uint8_t stat;
    switch (m_units) {
        case DistanceSensor::kInches:
            return GetRangeInches(&stat);
        case DistanceSensor::kMilliMeters:
            return GetRangeMM(&stat);
        default:
            return 0.0;
    }
}

void DistanceSensor::SetPIDSourceType(frc::PIDSourceType pidSource) {
    if (wpi_assert(pidSource == frc::PIDSourceType::kDisplacement)) {
        m_pidSource = pidSource;
    }
}