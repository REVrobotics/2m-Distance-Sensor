/*
 * Copyright (c) 2020 REV Robotics
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

package com.revrobotics.jni;

public class VL53L0XJNI extends DistanceSensorJNIWrapper {
  public static native void Init(int port, int addr);
  public static native boolean GetVersion();
  public static native boolean DataInit(int port, int addr);
  public static native boolean GetDeviceInfo(int port, int addr);
  public static native boolean StaticInit(int port, int addr);
  public static native boolean PerformRefCalibration(int port, int addr);
  public static native boolean PerformRefSpadManagement(int port, int addr);
  public static native boolean SetDeviceMode(int devMode, int port, int addr);
  public static native int ValidateI2C(int port, int addr);
  public static native boolean SetLimitCheckEnableSigmaFinalRange(int enable, int port, int addr);
  public static native boolean SetLimitCheckEnableSignalRateFinalRange(int enable, int port, int addr);
  public static native boolean SetLimitCheckEnableRangeIgnoreThreshold(int enable, int port, int addr);
  public static native boolean SetLimitCheckValueSigmaFinalRange(int val, int port, int addr);
  public static native boolean SetLimitCheckValueSignalRateFinalRange(double val, int port, int addr);
  public static native boolean SetMeasurementTimingBudgetMicroSeconds(int val, int port, int addr);
  public static native boolean SetVcselPulsePeriodPreRange(int val, int port, int addr);
  public static native boolean SetVcselPulsePeriodFinalRange(int val, int port, int addr);
  public static native boolean StartMeasurement(int port, int addr);
  public static native boolean GetMeasurementDataReady(int port, int addr);
  public static native boolean ClearInterruptMask(int port, int addr);
  public static native boolean StopMeasurement(int port, int addr);
  public static native boolean GetStopCompletedStatus(int port, int addr);
  public static native double GetRangingMeasurementData(int port, int addr);
}