/*
 * Copyright (c) 2019 REV Robotics
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

#include "com_revrobotics_jni_VL53L0XJNI.h"

// We have a lot of JNI utilities here
// If you need them. For things like strings and
// arrays
#include <wpi/jni_util.h>

#include "vl53l0x_api.h"
#include <HAL/I2C.h>

#define VALIDATE_I2C_SUCCESS    (jint)0
#define VALIDATE_I2C_PARAM_ERR  (jint)1
#define VALIDATE_I2C_HAL_ERR    (jint)2

extern "C" {
JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM* vm, void* reserved) {
    // Check to ensure the JNI version is valid

    JNIEnv* env;
    if (vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) != JNI_OK)
        return JNI_ERR;

    // In here is also where you store things like class references
    // if they are ever needed

    return JNI_VERSION_1_6;
}

JNIEXPORT void JNICALL JNI_OnUnload(JavaVM* vm, void* reserved) {}

VL53L0X_Dev_t *pDevice = NULL;
VL53L0X_RangingMeasurementData_t *pRangingMeasurementData = new VL53L0X_RangingMeasurementData_t;

/*
 * Class:     com_revrobotics_jni_VL53L0XJNI
 * Method:    Init
 * Signature: (II)V
 */
JNIEXPORT void JNICALL
Java_com_revrobotics_jni_VL53L0XJNI_Init
  (JNIEnv * env, jclass, jint port, jint addr)
{
    if(pDevice == NULL)
        pDevice = new VL53L0X_Dev_t;

    pDevice->I2cDevAddr = static_cast<uint8_t>(addr);
    pDevice->port = static_cast<HAL_I2CPort>(port);

    int32_t status = 0;
    HAL_InitializeI2C(pDevice->port, &status);
}

/*
 * Class:     com_revrobotics_jni_VL53L0XJNI
 * Method:    GetVersion
 * Signature: ()Z
 */
JNIEXPORT jboolean JNICALL
Java_com_revrobotics_jni_VL53L0XJNI_GetVersion
  (JNIEnv*, jclass)
{
    VL53L0X_Version_t *pVersion = new VL53L0X_Version_t;
    VL53L0X_Error Status = VL53L0X_GetVersion(pVersion);

    return Status == 0;
}

/*
 * Class:     com_revrobotics_jni_VL53L0XJNI
 * Method:    DataInit
 * Signature: (II)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_revrobotics_jni_VL53L0XJNI_DataInit
  (JNIEnv*, jclass, jint port, jint addr)
{
    if(pDevice == NULL)
        return false;

    pDevice->port = static_cast<HAL_I2CPort>(port);
    pDevice->I2cDevAddr = static_cast<uint8_t>(addr);
    VL53L0X_Error Status = VL53L0X_DataInit(pDevice);

    return Status == 0;
}

/*
 * Class:     com_revrobotics_jni_VL53L0XJNI
 * Method:    GetDeviceInfo
 * Signature: (II)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_revrobotics_jni_VL53L0XJNI_GetDeviceInfo
  (JNIEnv*, jclass, jint port, jint addr)
{
    if(pDevice == NULL)
        return false;

    pDevice->port = static_cast<HAL_I2CPort>(port);
    pDevice->I2cDevAddr = static_cast<uint8_t>(addr);
    VL53L0X_DeviceInfo_t *pDeviceInfo = new VL53L0X_DeviceInfo_t;
    VL53L0X_Error Status = VL53L0X_GetDeviceInfo(pDevice, pDeviceInfo);

    return Status == 0;
}

/*
 * Class:     com_revrobotics_jni_VL53L0XJNI
 * Method:    StaticInit
 * Signature: (II)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_revrobotics_jni_VL53L0XJNI_StaticInit
  (JNIEnv*, jclass, jint port, jint addr)
{
    if(pDevice == NULL)
        return false;

    pDevice->port = static_cast<HAL_I2CPort>(port);
    pDevice->I2cDevAddr = static_cast<uint8_t>(addr);
    VL53L0X_Error Status = VL53L0X_StaticInit(pDevice);

    return Status == 0;
}

/*
 * Class:     com_revrobotics_jni_VL53L0XJNI
 * Method:    PerformRefCalibration
 * Signature: (II)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_revrobotics_jni_VL53L0XJNI_PerformRefCalibration
  (JNIEnv*, jclass, jint port, jint addr)
{
    if(pDevice == NULL)
        return false;

    pDevice->port = static_cast<HAL_I2CPort>(port);
    pDevice->I2cDevAddr = static_cast<uint8_t>(addr);
    uint8_t VhvSettings, PhaseCal;
    VL53L0X_Error Status = VL53L0X_PerformRefCalibration(pDevice, &VhvSettings, &PhaseCal);

    return Status == 0;
}

/*
 * Class:     com_revrobotics_jni_VL53L0XJNI
 * Method:    PerformRefSpadManagement
 * Signature: (II)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_revrobotics_jni_VL53L0XJNI_PerformRefSpadManagement
  (JNIEnv*, jclass, jint port, jint addr)
{
    if(pDevice == NULL)
        return false;

    pDevice->port = static_cast<HAL_I2CPort>(port);
    pDevice->I2cDevAddr = static_cast<uint8_t>(addr);
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    VL53L0X_Error Status = VL53L0X_PerformRefSpadManagement(pDevice, &refSpadCount, &isApertureSpads);

    return Status == 0;
}

/*
 * Class:     com_revrobotics_jni_VL53L0XJNI
 * Method:    SetDeviceMode
 * Signature: (III)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_revrobotics_jni_VL53L0XJNI_SetDeviceMode
  (JNIEnv*, jclass, jint devMode, jint port, jint addr)
{
    if(pDevice == NULL)
        return false;

    pDevice->port = static_cast<HAL_I2CPort>(port);
    pDevice->I2cDevAddr = static_cast<uint8_t>(addr);
    VL53L0X_Error Status = VL53L0X_SetDeviceMode(pDevice, static_cast<VL53L0X_DeviceModes>(devMode));

    return Status == 0;
}

/*
 * Class:     com_revrobotics_jni_VL53L0XJNI
 * Method:    ValidateI2C
 * Signature: (II)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_VL53L0XJNI_ValidateI2C
  (JNIEnv*, jclass, jint port, jint addr)
{
    if(pDevice == NULL)
        return VALIDATE_I2C_HAL_ERR;

    pDevice->port = static_cast<HAL_I2CPort>(port);
    pDevice->I2cDevAddr = static_cast<uint8_t>(addr);

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

/*
 * Class:     com_revrobotics_jni_VL53L0XJNI
 * Method:    SetLimitCheckEnableSigmaFinalRange
 * Signature: (III)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_revrobotics_jni_VL53L0XJNI_SetLimitCheckEnableSigmaFinalRange
  (JNIEnv*, jclass, jint enable, jint port, jint addr)
{
    if(pDevice == NULL)
        return false;

    pDevice->port = static_cast<HAL_I2CPort>(port);
    pDevice->I2cDevAddr = static_cast<uint8_t>(addr);
    VL53L0X_Error Status = VL53L0X_SetLimitCheckEnable(pDevice,
                            VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, enable);

    return Status == 0;
}

/*
 * Class:     com_revrobotics_jni_VL53L0XJNI
 * Method:    SetLimitCheckEnableSignalRateFinalRange
 * Signature: (III)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_revrobotics_jni_VL53L0XJNI_SetLimitCheckEnableSignalRateFinalRange
  (JNIEnv*, jclass, jint enable, jint port, jint addr)
{
    if(pDevice == NULL)
        return false;

    pDevice->port = static_cast<HAL_I2CPort>(port);
    pDevice->I2cDevAddr = static_cast<uint8_t>(addr);
    VL53L0X_Error Status = VL53L0X_SetLimitCheckEnable(pDevice,
                            VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, enable);

    return Status == 0;
}

/*
 * Class:     com_revrobotics_jni_VL53L0XJNI
 * Method:    SetLimitCheckEnableRangeIgnoreThreshold
 * Signature: (III)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_revrobotics_jni_VL53L0XJNI_SetLimitCheckEnableRangeIgnoreThreshold
  (JNIEnv*, jclass, jint enable, jint port, jint addr)
{
    if(pDevice == NULL)
        return false;

    pDevice->port = static_cast<HAL_I2CPort>(port);
    pDevice->I2cDevAddr = static_cast<uint8_t>(addr);
    VL53L0X_Error Status = VL53L0X_SetLimitCheckEnable(pDevice,
                            VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 
                            enable);

    return Status == 0;
}

/*
 * Class:     com_revrobotics_jni_VL53L0XJNI
 * Method:    SetLimitCheckValueSigmaFinalRange
 * Signature: (III)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_revrobotics_jni_VL53L0XJNI_SetLimitCheckValueSigmaFinalRange
  (JNIEnv*, jclass, jint val, jint port, jint addr)
{
    if(pDevice == NULL)
        return false;

    pDevice->port = static_cast<HAL_I2CPort>(port);
    pDevice->I2cDevAddr = static_cast<uint8_t>(addr);
    VL53L0X_Error Status = VL53L0X_SetLimitCheckValue(pDevice,
                            VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                            (FixPoint1616_t)(val*65536));
    return Status == 0;
}

/*
 * Class:     com_revrobotics_jni_VL53L0XJNI
 * Method:    SetLimitCheckValueSignalRateFinalRange
 * Signature: (DII)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_revrobotics_jni_VL53L0XJNI_SetLimitCheckValueSignalRateFinalRange
  (JNIEnv*, jclass, jdouble val, jint port, jint addr)
{
    if(pDevice == NULL)
        return false;

    pDevice->port = static_cast<HAL_I2CPort>(port);
    pDevice->I2cDevAddr = static_cast<uint8_t>(addr);
    VL53L0X_Error Status = VL53L0X_SetLimitCheckValue(pDevice,
                            VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                            (FixPoint1616_t)(val*65536));
    return Status == 0;
}

/*
 * Class:     com_revrobotics_jni_VL53L0XJNI
 * Method:    SetMeasurementTimingBudgetMicroSeconds
 * Signature: (III)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_revrobotics_jni_VL53L0XJNI_SetMeasurementTimingBudgetMicroSeconds
  (JNIEnv*, jclass, jint val, jint port, jint addr)
{
    if(pDevice == NULL)
        return false;

    pDevice->port = static_cast<HAL_I2CPort>(port);
    pDevice->I2cDevAddr = static_cast<uint8_t>(addr);
    VL53L0X_Error Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pDevice, val);
    return Status == 0;
}

/*
 * Class:     com_revrobotics_jni_VL53L0XJNI
 * Method:    SetVcselPulsePeriodPreRange
 * Signature: (III)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_revrobotics_jni_VL53L0XJNI_SetVcselPulsePeriodPreRange
  (JNIEnv*, jclass, jint val, jint port, jint addr)
{
    if(pDevice == NULL)
        return false;

    pDevice->port = static_cast<HAL_I2CPort>(port);
    pDevice->I2cDevAddr = static_cast<uint8_t>(addr);
    VL53L0X_Error Status = VL53L0X_SetVcselPulsePeriod(pDevice, 
		                    VL53L0X_VCSEL_PERIOD_PRE_RANGE, val);

    return Status == 0;
}

/*
 * Class:     com_revrobotics_jni_VL53L0XJNI
 * Method:    SetVcselPulsePeriodFinalRange
 * Signature: (III)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_revrobotics_jni_VL53L0XJNI_SetVcselPulsePeriodFinalRange
  (JNIEnv*, jclass, jint val, jint port, jint addr)
{
    if(pDevice == NULL)
        return false;

    pDevice->port = static_cast<HAL_I2CPort>(port);
    pDevice->I2cDevAddr = static_cast<uint8_t>(addr);
    VL53L0X_Error Status = VL53L0X_SetVcselPulsePeriod(pDevice, 
		                    VL53L0X_VCSEL_PERIOD_FINAL_RANGE, val);

    return Status == 0;
}

/*
 * Class:     com_revrobotics_jni_VL53L0XJNI
 * Method:    StartMeasurement
 * Signature: (II)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_revrobotics_jni_VL53L0XJNI_StartMeasurement
  (JNIEnv*, jclass, jint port, jint addr)
{
    if(pDevice == NULL)
        return false;

    pDevice->port = static_cast<HAL_I2CPort>(port);
    pDevice->I2cDevAddr = static_cast<uint8_t>(addr);
    VL53L0X_Error Status = VL53L0X_StartMeasurement(pDevice);

    return Status == 0;
}

/*
 * Class:     com_revrobotics_jni_VL53L0XJNI
 * Method:    GetMeasurementDataReady
 * Signature: (II)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_revrobotics_jni_VL53L0XJNI_GetMeasurementDataReady
  (JNIEnv*, jclass, jint port, jint addr)
{
    if(pDevice == NULL)
        return false;

    pDevice->port = static_cast<HAL_I2CPort>(port);
    pDevice->I2cDevAddr = static_cast<uint8_t>(addr);
    VL53L0X_Error Status = VL53L0X_GetRangingMeasurementData(pDevice, 
                                    pRangingMeasurementData);

    return Status == 0;
}

/*
 * Class:     com_revrobotics_jni_VL53L0XJNI
 * Method:    ClearInterruptMask
 * Signature: (II)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_revrobotics_jni_VL53L0XJNI_ClearInterruptMask
  (JNIEnv*, jclass, jint port, jint addr)
{
    if(pDevice == NULL)
        return false;

    pDevice->port = static_cast<HAL_I2CPort>(port);
    pDevice->I2cDevAddr = static_cast<uint8_t>(addr);
    VL53L0X_Error Status = VL53L0X_ClearInterruptMask(pDevice, 
                        VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);

    return Status == 0;
}

/*
 * Class:     com_revrobotics_jni_VL53L0XJNI
 * Method:    StopMeasurement
 * Signature: (II)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_revrobotics_jni_VL53L0XJNI_StopMeasurement
  (JNIEnv*, jclass, jint port, jint addr)
{
    if(pDevice == NULL)
        return false;

    pDevice->port = static_cast<HAL_I2CPort>(port);
    pDevice->I2cDevAddr = static_cast<uint8_t>(addr);
    VL53L0X_Error Status = VL53L0X_StopMeasurement(pDevice);

    return Status == 0;
}

/*
 * Class:     com_revrobotics_jni_VL53L0XJNI
 * Method:    GetStopCompletedStatus
 * Signature: (II)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_revrobotics_jni_VL53L0XJNI_GetStopCompletedStatus
  (JNIEnv*, jclass, jint port, jint addr)
{
    if(pDevice == NULL)
        return false;

    pDevice->port = static_cast<HAL_I2CPort>(port);
    pDevice->I2cDevAddr = static_cast<uint8_t>(addr);
    uint32_t StopCompleted = 0;
    VL53L0X_Error Status = VL53L0X_GetStopCompletedStatus(pDevice, &StopCompleted);

    return ((StopCompleted == 0x00) || Status != VL53L0X_ERROR_NONE);
}

/*
 * Class:     com_revrobotics_jni_VL53L0XJNI
 * Method:    GetRangingMeasurementData
 * Signature: (II)D
 */
JNIEXPORT jdouble JNICALL
Java_com_revrobotics_jni_VL53L0XJNI_GetRangingMeasurementData
  (JNIEnv*, jclass, jint port, jint addr)
{
    if(pDevice == NULL)
        return false;

    pDevice->port = static_cast<HAL_I2CPort>(port);
    pDevice->I2cDevAddr = static_cast<uint8_t>(addr);
    VL53L0X_GetRangingMeasurementData(pDevice, 
                                    pRangingMeasurementData);

    if(pRangingMeasurementData->RangeStatus == 0)
        return pRangingMeasurementData->RangeMilliMeter;
    else
        return -1;
}
}