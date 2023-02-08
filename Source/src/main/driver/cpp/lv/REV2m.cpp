#include "vl53l0x_api.h"
#include <hal/I2C.h>

extern "C" {

int32_t REV2m_InitializeI2C(int32_t port) {
    int32_t status = 0;
    HAL_InitializeI2C(static_cast<HAL_I2CPort>(port), &status);

    return status;
}

void REV2m_CloseI2C(int32_t port) {
    HAL_CloseI2C(static_cast<HAL_I2CPort>(port));
}

VL53L0X_Dev_t* REV2m_getDevice(void) { return new VL53L0X_Dev_t; }

void REV2m_destroyDevice(VL53L0X_Dev_t* Dev) { delete Dev; }

VL53L0X_RangingMeasurementData_t* REV2m_getMeasurementDataPointer(void) {
    return new VL53L0X_RangingMeasurementData_t;
}

void REV2m_destroyDat(VL53L0X_RangingMeasurementData_t* Dat) { delete Dat; }

void REV2m_setPort(VL53L0X_Dev_t* Dev, int32_t port) {
    if (Dev != NULL) {
        Dev->port = static_cast<HAL_I2CPort>(port);
    }
}

uint8_t REV2m_getPort(VL53L0X_Dev_t* Dev) {
    if (Dev != NULL) {
        return static_cast<uint8_t>(Dev->port);
    }
}

void REV2m_setAddr(VL53L0X_Dev_t* Dev, int32_t addr) {
    if (Dev != NULL) {
        Dev->I2cDevAddr = addr;
    }
}

uint8_t REV2m_getAddr(VL53L0X_Dev_t* Dev) {
    if (Dev != NULL) {
        return Dev->I2cDevAddr;
    }
}

int8_t REV2m_getMeasurement(VL53L0X_Dev_t* Dev,
                            VL53L0X_RangingMeasurementData_t* Dat,
                            uint8_t* RangeStatus, uint16_t* RangeMilliMeter) {
    VL53L0X_Error status = VL53L0X_GetRangingMeasurementData(Dev, Dat);
    *RangeStatus = Dat->RangeStatus;
    *RangeMilliMeter = Dat->RangeMilliMeter;

    return status;
}
}