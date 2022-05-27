// Copyright (c) 2022 Michal Drwiega
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MDMC_USB_API_H
#define MDMC_USB_API_H

#include <iostream>
#include <assert.h>
#include <string>

#include <hidapi.h>

#ifndef NDEBUG
	#define NDEBUG
#endif

#define READ_TIMEOUT        4
#define DATASET_IN_LEN      45
#define CTRL_OUT_REPORT_LEN 19
#define DIGITAL_INPUTS_NR    4
#define DIGITAL_OUTPUTS_NR    4
#define MAX_STR 255

#define DEVICE_VID 0x0012   ///< Vendor ID of USB device
#define DEVICE_PID 0x0666   ///< Product ID of USB device

/// Determines motor of robot
enum Motor {BL = 0U, BR = 1U, FL = 2U, FR = 3U};

/// Determines direction of rotations
enum RotationDir {CW = 0U, CCW = 1U};

/// Defines digital input pulling mode
enum InputPull {NonePull = 0U, Up = 1U, Down = 2U };

/// Defines specific function of digital input
enum InputFunction {NoneFcn = 0U, Emergency = 1U, SlowMode = 2U, DeadManSwitch = 3U };
/// Defines digital state
enum DigitalState { Low = 0U, High = 1U };
/// Defines how many and how motors are connected to controller
enum MotorsStructure { Four = 0U, TwoFront = 1U, TwoBack = 2U, TwoDoubled = 3U };
/// Type of control loop
enum ControlLoop {Open = 0U, Close = 1U};
/// Control loop frequency in Hz
enum ControlLoopFreq {Hz100 = 0U, Hz200 = 1U, Hz500 = 2U, Hz1000 = 3U };

/**
 * @brief The MotorControllerDataSet class stores motor controller processing data
 */
class MotorControllerDataSet
{
public:
  /**
   * @brief report1ToDataSet converts USB report to object of MotorControllerDataset class
   * @param buff Buffer which contains USB report
   * @param len Length of report in bytes
   * @return Returns true if conversion is correct. Otherwise returns false.
   */
  bool report1ToDataSet(uint8_t * buff, uint8_t len);
  /**
   * @brief getDigitalOutputState
   * @param nr
   * @return
   */
  uint8_t getDigitalOutputState(uint8_t nr) const;
  /**
   * @brief getDigitalInputState
   * @param nr
   * @return
   */
  uint8_t getDigitalInputState(uint8_t nr) const;
  /**
   * @brief getWheelSpeed_ms
   * @param mot
   * @return
   */
  float   getWheelSpeed(Motor mot) const;
  /**
   * @brief getWheelDistance_m
   * @param mot
   * @return
   */
  float   getWheelPosition(Motor mot) const;
  /**
   * @brief getMotorCurrent_mA
   * @param mot
   * @return
   */
  int16_t getMotorCurrent_mA(Motor mot) const;
  /**
   * @brief getMotorsVoltage_mV
   * @return
   */
  int16_t getMotorsVoltage_mV() const;

  uint8_t state_reg0;       ///< State register 0
  uint8_t state_reg1;       ///< State register 1
  uint8_t state_DIO;        ///< Digital inputs and outputs register
  float   speed_ms[4];      ///< Measured wheels speeds in m/s
  float   distance_m[4];    ///< Wheels distances in meters
  int16_t current_ma[4];    ///< Motors currents in milliampers (mA)
  int16_t motor_voltage_mv; ///< Motor power voltage in millivolts (mV)

  float x_, y_, theta_, lin_vel_, ang_vel_;
};

/**
 * @brief The MotorControllerConfig class stores motor controller configuration data
 */
class MotorControllerConfig
{
public:
  /**
   * @brief setMainConfig
   * @param motors
   * @param mode
   * @param freq
   */
  void setMainConfig(MotorsStructure motors, ControlLoop mode, ControlLoopFreq freq);
  /**
   * @brief setWheelDiameter
   * @param motor
   * @param wheel_d
   */
  void setWheelDiameter(Motor motor, float diameter);
  /**
   * @brief setEncPulsesPerRevolution
   * @param enc
   * @param ppr
   */
  void setEncPulsesPerRevolution(Motor enc, uint16_t ppr);
  /**
   * @brief setDIConfig
   * @param nr
   * @param pull
   * @param fcn
   * @param normal_state
   */
  void setDIConfig(uint8_t nr, InputPull pull,  InputFunction fcn, DigitalState normal_state);
  /**
   * @brief setSlowModeSpeed
   * @param speed
   */
  void setSlowModeSpeed(float speed);
  /**
   * @brief setPIDTunings
   * @param reg
   * @param kp
   * @param ki
   * @param kd
   */
  void setPIDTunings(Motor reg, float kp, float ki, float kd);
  /**
   * @brief getPIDTunings
   * @param reg
   * @param kp
   * @param ki
   * @param kd
   */
  void getPIDTunings(Motor reg, float &kp, float &ki, float &kd);
  /**
   * @brief report1ToDataSet converts a received USB HID report to configuration data set
   *
   * @param buff Buffer which contains USB report
   * @param len Length of report in bytes
   * @return True if conversion is correct. Otherwise false.
   */
  bool report1ToDataSet(uint8_t * buff, uint8_t len);

  uint8_t  config_reg0;         ///< Configuration register 0
  float    wheel_diameter_m[4]; ///< Wheels diameters in meters
  uint16_t pulses_per_rev[4];   ///< Pulses per wheel revolution
  uint8_t  config_di[4];        ///< Digital inputs configuration
  float    slow_mode_speed_ms;  ///< Maximum speed value in slow mode (m/s)
  float    reg_kp[4];           ///< Proportional factor
  float    reg_ki[4];           ///< Integral factor
  float    reg_kd[4];           ///< Derivative factor
};

/**
 * @brief The ControlDataSet class stores motor controller control values
 */
class ControlDataSet
{
public:
  /**
   * @brief dataSetToReport
   * @param buff
   * @param len
   * @return
   */
  bool dataSetToReport(uint8_t * buff, uint8_t * len) const;
  /**
   * @brief setDigitalOutput
   * @param nr
   * @return
   */
  bool setDigitalOutput(uint8_t nr);
  /**
   * @brief clrDigitalOutput
   * @param nr
   * @return
   */
  bool clrDigitalOutput(uint8_t nr);
  /**
   * @brief getDigitalOutput
   * @param nr
   * @return
   */
  bool getDigitalOutput(uint8_t nr);
  /**
   * @brief setReferenceSpeed
   * @param label
   * @param ref_speed
   * @return
   */
  void setReferenceSpeed(Motor label, float ref_speed);
  /**
   * @brief setDistancesReset
   */
  void setDistancesReset();
  /**
   * @brief clrDistancesReset
   */
  void clrDistancesReset();

  uint8_t ctrl_main;        ///< Main control register
  uint8_t ctrl_DO;          ///< Digital outputs control register
  float   ref_speed_ms[4];  ///< References speeds

  float lin_x_vel, ang_z_vel;
};

/**
 * @brief The MotorControllerAPI class
 */
class MotorControllerAPI
{
public:
  MotorControllerAPI();
  /**
   * @brief Frees memory of HID device object
   */
  ~MotorControllerAPI();
  /**
   * @brief isConnected determines if interface is connected to device
   * @return
   */
  bool isConnected() const { return (handle_ == NULL ? false : true); }
  /**
   * @brief connectToDevice connects to motor controller device by USB
   *
   * @param vid Vendor id
   * @param pid Product id
   * @return Returns true if connection established correctly. Otherwise returns false.
   */
  bool connectToDevice(unsigned short vid, unsigned short pid);
  /**
   * @brief disconnectDevice disconnects from motor controller device
   */
  void disconnectDevice();
  /**
   * @brief getDataFromDevice
   * @param ds
    */
  bool getProcessDataFromDevice(MotorControllerDataSet & ds) const;
  /**
   * @brief sendCtrlDataToDevice
   * @param ds
   */
  bool sendCtrlDataToDevice(const ControlDataSet & ds);
  /**
   * @brief sendConfigurationToDevice
   * @param ds
   */
  void sendConfigurationToDevice(const MotorControllerConfig & ds);
  /**
   * @brief getConfigurationFromDevice
   * @param ds
   */
  void getConfigurationFromDevice(MotorControllerConfig & ds) const;

  // Private fields
private:
  hid_device              *handle_;       ///< USB HID device handler
};

#endif // MDMC_USB_API_H
