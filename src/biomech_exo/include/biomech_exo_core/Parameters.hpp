#ifndef PARAMETERS_HEADER
#define PARAMETERS_HEADER

// Selects the board to compile for
#define QUAD_BOARD

const int TORQUE_AVERAGE_COUNT = 5;

const bool FLAG_PRINT_TORQUES = false;
const bool FLAG_PID_VALS = false;
const bool FLAG_TWO_TOE_SENSORS = false;

const double PID_DEFAULTS[] = {0.2, 0.1, 0};
const int TORQUE_CALIBRATION_TIME_IN_MS = 1000;

const double MOTOR_ZERO_OFFSET_DEFAULT = 0.0;

// ===== FSR Parameters =====
// To set FSR bias and to identify the states
const double FSR_Sensors_type = 10;
const int FSR_CALIBRATION_TIME_MS = 5000;

// ===== Shaping Parameters =====
//Paramenters used to shape the reference
const double Ts = 0.001;
const double RECHARGE_TIME = 1;

// ===== State Machine Parameters =====
const double state_counter_th = 3;
const double step_time_length = 150;
const double STEADY_STATE_TIMEOUT = 2500;

// ===== PID and CTRL Parameters =====
//Includes the PID library so we can utilize PID control
const int PID_sample_time = 1;                                             //PID operates at 1000Hz, calling at a freq of 1 ms.

// ===== Memory Addrss Parameters =====
//To store in memory. These are the address of the EEPROM of the Teensy
const int address_params = 54;

// ===== Calibrate and read Parameters =====
const double p[4] = {0.0787, -0.8471, 20.599, -22.670};
const double p_prop[3] = {128.1, -50.82, 22.06};

// ===== Proportional Control Parameters =====
const double Max_Prop = 18;
const double Min_Prop = 0;

// ===== Auto KF Parameters =====
const double max_ERR = 0.20;
const double min_ERR = -0.20;
const double MAX_KF = 1.8;
const double MIN_KF = 0.8;

// ===== IMU Parameters =====
const int BNO055_SAMPLERATE_DELAY_MS = 100;
const double stability_trq_gain = 1;

// ===== Torque Speed Adjust Parameters =====
const int n_step_baseline = 6;
const int ACTIVATION_STEP_COUNT = 6;

// ===== Shaping Parameters =====
// the smoothing value, i.e. the sigmoind number of steps as a function of the EXO state
const double DEFAULT_ITER_SWING = 500;
const double DEFAULT_ITER_LATE_STANCE = 4;

const int FSR_CALIBRATION_PEAK_COUNT = 4;
const double FSR_UPPER_THRESHOLD = 0.7;
const double FSR_LOWER_THRESHOLD = 0.3;
const double VOLTAGE_TO_TORQUE_CONVERSION = 3.3 * 56.5 / 2.1;

#endif
