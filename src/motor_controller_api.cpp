#include "drive_controller/motor_controller_api.h"

#include <string>

#define MAX_STR 255

#define READ_TIMEOUT          4
#define DATASET_IN_LEN        45
#define CTRL_OUT_REPORT_LEN   19
#define DIGITAL_INPUTS_NR     4
#define DIGITAL_OUTPUTS_NR    4

//=============================================================================
// Converts four bytes to float number (single precision, 32 bits)
float bytesToFloat(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3)
{
  float out;

  *((uint8_t*)(&out) + 0) = b0;
  *((uint8_t*)(&out) + 1) = b1;
  *((uint8_t*)(&out) + 2) = b2;
  *((uint8_t*)(&out) + 3) = b3;

  return out;
}

//=============================================================================
// Converts two bytes to int16 number (16 bits)
int16_t bytesToInt16(uint8_t b0, uint8_t b1)
{
  int16_t out;

  *((uint8_t*)(&out) + 0) = b0;
  *((uint8_t*)(&out) + 1) = b1;

  return out;
}

//=============================================================================
// MOTOR CONTROLLER API CLASS METHODS
//=============================================================================
MotorControllerAPI::MotorControllerAPI(): handle_(NULL)
{
}

//=============================================================================
MotorControllerAPI::~MotorControllerAPI()
{
  if (handle_ != NULL)
  {
    hid_close(handle_);
  }
  hid_exit(); // Free HID memory
}

//=============================================================================
bool MotorControllerAPI::connectToDevice(unsigned short vid, unsigned short pid)
{
  wchar_t wstr[MAX_STR];

  // Open the device using the VID, PID, and optionally the Serial number.
  handle_ = hid_open(vid, pid, NULL);

  if (handle_ == NULL)
  {
    std::cerr << "\nUSB device with VID: " << std::hex << vid << " and PID: " << pid;
    std::cerr << std::dec << " was not found.\n";

    return false;
  }
  else
  {
    int res;
    std::cerr << "\nConnected to device with VID: " << std::hex << vid;
    std::cerr << " and PID: " << pid << std::dec << " is active.";

    // Read the Manufacturer String
    res = hid_get_manufacturer_string(handle_, wstr, MAX_STR);
    std::cerr << "\nManufacturer String: " << wstr;

    // Read the Product String
    res = hid_get_product_string(handle_, wstr, MAX_STR);
    std::cerr << "\nProduct String: " << wstr;

    // Read the Serial Number String
    res = hid_get_serial_number_string(handle_, wstr, MAX_STR);
    std::cerr << "\nSerial Number String: " << wstr << '\n';

    // Set the hid_read() function to be non-blocking.
    hid_set_nonblocking(handle_, 1);

    return true;
  }
}

//=============================================================================
void MotorControllerAPI::disconnectDevice()
{
  hid_close(handle_);
}

//=============================================================================
bool MotorControllerAPI::getProcessDataFromDevice(MotorControllerDataSet & ds) const
{
  if (!isConnected())
  {
    std::cerr << std::dec << "\nDevice not connected. Unable to read data from it.\n";
    return false;
  }

  uint8_t buf[DATASET_IN_LEN];
  buf[0] = 0x00;

  int result = hid_read_timeout(handle_, buf, DATASET_IN_LEN, READ_TIMEOUT);

#ifndef NDEBUG
  for (int i = 0; i < DATASET_IN_LEN; i++)
    std::cerr << std::hex << (int)buf[i] << " ";
  std::cerr << std::dec << "\nhid_read() status = " << result << "\n";
#endif

  if (result != DATASET_IN_LEN)
  {
    std::cerr << "\nUnable to read data from device.\n";
    return false;
  }

  ds.report1ToDataSet(buf, DATASET_IN_LEN);
  return true;
}

//=============================================================================
bool MotorControllerAPI::sendCtrlDataToDevice(const ControlDataSet & ds)
{
  if (!isConnected())
  {
    std::cerr << std::dec << "\nDevice not connected. Unable to write data to it.\n";
    return false;
  }

  uint8_t buf[CTRL_OUT_REPORT_LEN], len;

  ds.dataSetToReport(buf, &len); // Data set to report conversion

  // Send data to the device
  int result = hid_write(handle_, buf, CTRL_OUT_REPORT_LEN);

#ifndef NDEBUG
  for (int i = 0; i < 19; i++)
    std::cerr << std::hex << (int)buf[i] << " ";
  std::cerr << std::dec << "\nhid_write() status = " << result << "\n";
#endif

  if (result != CTRL_OUT_REPORT_LEN)
  {
    std::cerr << "\nUnable to send data to device.\n";
    return false;
  }

  return true;
}

//=============================================================================
void MotorControllerAPI::sendConfigurationToDevice(const MotorControllerConfig & ds)
{

}

//=============================================================================
void MotorControllerAPI::getConfigurationFromDevice(MotorControllerConfig &ds) const
{

}


//=============================================================================
//=============================================================================
// MOTOR CONTROLLER DATASET
//===========================================================================
bool MotorControllerDataSet::report1ToDataSet(uint8_t * buf, uint8_t len)
{
  if (len != DATASET_IN_LEN)
    return false;

  state_reg0 = buf[0];
  state_reg1 = buf[1];
  state_DIO  = buf[2];

  int k = 3;
  if (0)
  {
    for (int j = 0; j < 4; j++)
    {
      speed_ms[j] = bytesToFloat(buf[k], buf[k+1], buf[k+2], buf[k+3]);
      k += 4;
    }
    for (int j = 0; j < 4; j++)
    {
      distance_m[j] = bytesToFloat(buf[k], buf[k+1], buf[k+2], buf[k+3]);
      k += 4;
    }
  }
  else if (1) // Second mode
  {
    x_ = bytesToFloat(buf[k], buf[k+1], buf[k+2], buf[k+3]);
    k += 4;
    y_ = bytesToFloat(buf[k], buf[k+1], buf[k+2], buf[k+3]);
    k += 4;
    theta_ = bytesToFloat(buf[k], buf[k+1], buf[k+2], buf[k+3]);
    k += 4;
    lin_vel_ = bytesToFloat(buf[k], buf[k+1], buf[k+2], buf[k+3]);
    k += 4;
    ang_vel_ = bytesToFloat(buf[k], buf[k+1], buf[k+2], buf[k+3]);
    k += 4;
  }

  for (int j = 0; j < 4; j++)
  {
    current_ma[j] = bytesToInt16(buf[k], buf[k+1]);
    k += 2;
  }
  motor_voltage_mv = bytesToInt16(buf[k], buf[k+1]);
  return true;
}

//===========================================================================
uint8_t MotorControllerDataSet::getDigitalOutputState(uint8_t nr) const
{
  return state_DIO & (1 << (4 + nr -1));
}

//===========================================================================
uint8_t MotorControllerDataSet::getDigitalInputState(uint8_t nr) const
{
  return state_DIO & (1 << (nr -1));
}

//===========================================================================
float MotorControllerDataSet::getWheelSpeed(Motor mot) const
{
  return speed_ms[mot];
}

//===========================================================================
float MotorControllerDataSet::getWheelPosition(Motor mot) const
{
  return distance_m[mot];
}

//===========================================================================
int16_t MotorControllerDataSet::getMotorCurrent_mA(Motor mot) const
{
  return current_ma[mot];
}

//===========================================================================
int16_t MotorControllerDataSet::getMotorsVoltage_mV() const
{
  return motor_voltage_mv;
}

//===========================================================================

#define MOTORS_STRUCTURE_BIT_1 0
#define MOTORS_STRUCTURE_BIT_2 1
#define CONTROL_LOOP_BIT 2
#define CONTROL_LOOP_FREQ_BIT_1 3
#define CONTROL_LOOP_FREQ_BIT_2 4

//===========================================================================
// MOTOR CONTROLLER CONFIG
//===========================================================================
//===========================================================================
void MotorControllerConfig::setMainConfig(MotorsStructure motors, ControlLoop mode,
                                          ControlLoopFreq freq)
{
  // Set motors structure
  if (motors & MOTORS_STRUCTURE_BIT_1)
    config_reg0 |= (1 << MOTORS_STRUCTURE_BIT_1);
  else
    config_reg0 &= ~(1 << MOTORS_STRUCTURE_BIT_1);
  if (motors & MOTORS_STRUCTURE_BIT_2)
    config_reg0 |= (1 << MOTORS_STRUCTURE_BIT_2);
  else
    config_reg0 &= ~(1 << MOTORS_STRUCTURE_BIT_2);

  // Set control loop mode
  if (mode & CONTROL_LOOP_BIT)
    config_reg0 |= (1 << CONTROL_LOOP_BIT);
  else
    config_reg0 &= ~(1 << CONTROL_LOOP_BIT);

  // Set control loop frequency
  if (freq & CONTROL_LOOP_FREQ_BIT_1)
    config_reg0 |= (1 << CONTROL_LOOP_FREQ_BIT_1);
  else
    config_reg0 &= ~(1 << CONTROL_LOOP_FREQ_BIT_1);
  if (freq & CONTROL_LOOP_FREQ_BIT_2)
    config_reg0 |= (1 << CONTROL_LOOP_FREQ_BIT_2);
  else
    config_reg0 &= ~(1 << CONTROL_LOOP_FREQ_BIT_2);
}

//===========================================================================
void MotorControllerConfig::setWheelDiameter(Motor motor, float diameter)
{
  if (diameter > 0)
  {
    wheel_diameter_m[motor] = diameter;
  }
}

//===========================================================================
void MotorControllerConfig::setEncPulsesPerRevolution(Motor enc, uint16_t ppr)
{
  pulses_per_rev[enc] = ppr;
}

#define DI_CFG_PULL_BIT_1 0
#define DI_CFG_PULL_BIT_2 1
#define DI_CFG_FCN_BIT_1 2
#define DI_CFG_FCN_BIT_2 3
#define DI_CFG_NORMAL_BIT 4

//===========================================================================
void MotorControllerConfig::setDIConfig(uint8_t nr, InputPull pull,
                                        InputFunction fcn, DigitalState normal_state)
{
  // Set digital input pulling configuration
  if (pull & DI_CFG_PULL_BIT_1)
    config_di[nr] |= (1 << DI_CFG_PULL_BIT_1);
  else
    config_di[nr] &= ~(1 << DI_CFG_PULL_BIT_1);
  if (pull & DI_CFG_PULL_BIT_2)
    config_di[nr] |= (1 << DI_CFG_PULL_BIT_2);
  else
    config_di[nr] &= ~(1 << DI_CFG_PULL_BIT_2);

  // Set digial input function configuration
  if (fcn & DI_CFG_FCN_BIT_1)
    config_di[nr] |= (1 << DI_CFG_FCN_BIT_1);
  else
    config_di[nr] &= ~(1 << DI_CFG_FCN_BIT_1);
  if (fcn & DI_CFG_FCN_BIT_2)
    config_di[nr] |= (1 << DI_CFG_FCN_BIT_2);
  else
    config_di[nr] &= ~(1 << DI_CFG_FCN_BIT_2);

  // Set digial input normal state
  if (normal_state & DI_CFG_NORMAL_BIT)
    config_di[nr] |= (1 << DI_CFG_NORMAL_BIT);
  else
    config_di[nr] &= ~(1 << DI_CFG_NORMAL_BIT);
}

//===========================================================================
void MotorControllerConfig::setSlowModeSpeed(float speed)
{
  slow_mode_speed_ms = speed;
}

//===========================================================================
void MotorControllerConfig::setPIDTunings(Motor reg, float kp, float ki, float kd)
{
  reg_kp[reg] = kp;
  reg_ki[reg] = ki;
  reg_kd[reg] = kd;
}

//===========================================================================
void MotorControllerConfig::getPIDTunings(Motor reg, float &kp, float &ki, float &kd)
{
  kp = reg_kp[reg];
  ki = reg_ki[reg];
  kd = reg_kd[reg];
}

//===========================================================================
// CONTROL DATASET
//===========================================================================
bool ControlDataSet::dataSetToReport(uint8_t *buf, uint8_t *len) const
{
  uint8_t *chptr;

  buf[0] = 0x00;
  buf[1] = ctrl_main;
  buf[2] = ctrl_DO;

  uint8_t k = 3;
  if (0)
  {
    for (int i = 0; i < 4; i++)
    {
      chptr = (uint8_t *) &ref_speed_ms[i];
      buf[k] = *chptr++;  buf[k+1] = *chptr++;
      buf[k+2] = *chptr++;  buf[k+3] = *chptr;
      k += 4;
    }
  }
  else if (1) // Second mode
  {
    chptr = (uint8_t *) &lin_x_vel;
    buf[k] = *chptr++;  buf[k+1] = *chptr++;
    buf[k+2] = *chptr++;  buf[k+3] = *chptr;
    k += 4;

    chptr = (uint8_t *) &ang_z_vel;
    buf[k] = *chptr++;  buf[k+1] = *chptr++;
    buf[k+2] = *chptr++;  buf[k+3] = *chptr;
  }
  *len = 19;
  return true;
}

//=============================================================================
bool ControlDataSet::setDigitalOutput(uint8_t nr)
{
  assert(nr >= 1 && nr <= 4);

  if (nr < 1 || nr > 4)
    return false;

  ctrl_DO |= (1 << (nr - 1));
  return true;
}

//=============================================================================
bool ControlDataSet::clrDigitalOutput(uint8_t nr)
{
  assert(nr >= 1 && nr <= 4);

  if (nr < 1 || nr > 4)
    return false;

  ctrl_DO &= ~(1 << (nr -1));
  return true;
}

//=============================================================================
bool ControlDataSet::getDigitalOutput(uint8_t nr)
{
  assert(nr >= 1 && nr <= 4);

  return (ctrl_DO & (1 << (nr -1)));
}

//=============================================================================
void ControlDataSet::setReferenceSpeed(Motor label, float ref_speed)
{
  ref_speed_ms[label] = ref_speed;
}

#define DISTANCE_RESET_BIT 1  // Distance reset bit in main control register

//=============================================================================
void ControlDataSet::setDistancesReset()
{
  ctrl_main |= (1 << DISTANCE_RESET_BIT);
}

//=============================================================================
void ControlDataSet::clrDistancesReset()
{
  ctrl_main &= ~(1 << DISTANCE_RESET_BIT);
}

//=============================================================================
//=============================================================================
