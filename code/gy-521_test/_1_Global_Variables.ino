
// User Variable Definitions

#define   encoder_Tick_Resolution   10                    // Defines how many ticks to 'wait' until calculating encoder positions and time measurments. Higher values lead to decreased ISR times, but decreased encoder accuracy.
#define   pmw_Length                256                   // Defines wavelength of PMW signals by modifying divisor, [ 1, 8, 64(default), 256, 1024].  Base Freq.: 31250 Hz. Lower values lead to smoother motors, but worse current readings (or better, not sure).
#define   encoder_Speed_Time_Limit  100                   // Defines how long to wait for a new tick before assuming wheel is stopped [ms]
#define   motor_Current_Sample_Size 400 //50              // Defines the number of current samples to take, should be atleast 255, [20us*400 = 8000us = PMW Length@Divisor=256]
#define   motor_Current_Sample_Delay 0                    // Delay between samples [us]
#define   time_Delay                1                     // Defines amount of time program pauses
#define   voltage_Offset            0                     // Defines amount of volateg added to compensate for motor sticktion, [0 - 255].
#define   twbr_Value                24                    // Defines clocck speed of I2C, higher value leads to slower speeds and more robust comms 'aledgledy', [1-255, 24(default)].
#define   angle_Rounding_Value      1000.                   // Defines what value to multiple by before round theta and omega. Cuts down on noise, [10 = x.1,  100 = x.x1 , 1(Default)]
//Line 305 of MPU6050_6Axis_MotionApps20.h  = 1[10ms]    // Defines frequency of FIFO, higher value leads to less frequent updates, (200Hz /(1+value)), [0x02 (default)].


#define   theta_Filter              0.7
#define   theta_Speed_Filter        0.7
#define   theta_Integral_Max        3.0 
#define   left_Speed_Filter         0.8

#define   omega_Filter              0.7
#define   omega_Speed_Filter        0.7
#define   omega_Integral_Max        3.0
#define   right_Speed_Filter        0.8


   
float     angle_Average_Filter  = 0.970;
float     theta_Zero_Filter     = 0.995;
float     omega_Zero_Filter     = 0.986;
float     angle_Smoothed_Filter = 0.997;
float     friction_Value        = 10;

// Definitions

#define   encoder_PPR               334.                   // Encoder pulse per revolution
#define   serial_Frequency          250000 //230400               // Frequency of serial interface


// Motor Global Variables

volatile unsigned int   left_Rel_Tick_Count = 0;          // Encoder ticks untill resolution
volatile long           left_Abs_Tick_Count = 0;          // Absolute total of encoder ticks
volatile long           left_Abs_Tick_Count_Prev = 0;     // Previous Absolute total of encoder ticks
volatile int            left_Encoder_Direction = 0;       // Encoder measured direction, [-1 or 1]
volatile int            left_Encoder_Direction_Prev = 0;  // Encoder measured direction, [-1 or 1]
volatile int            left_Encoder_Direction_Now = 0;   // Encoder measured direction, [-1 or 1]
volatile int            left_Encoder_Direction_Debug = 0;   // Encoder measured direction, [-1 or 1]
volatile unsigned long  left_Time_Prev = 0;               // Time at previous absolute tick change, [us]
volatile unsigned long  left_Time_Now = 0;                // Time at current absolute tick change, [us]
volatile unsigned long  left_Time_Dif = 0;            
volatile unsigned long  left_Time_Acc_Prev = 0;           // Time at previous absolute tick change, [us]
volatile unsigned long  left_Time_Acc_Now = 0;            // Time at current absolute tick change, [us]
         unsigned long  left_Time_Age = 0;                // Time at current absolute tick change, [us]
         unsigned long  left_Time_Saved = 0;              // Time at current absolute tick change, [us]

volatile unsigned int   right_Rel_Tick_Count = 0;          // Encoder ticks untill resolution
volatile long           right_Abs_Tick_Count = 0;          // Absolute total of encoder ticks
volatile int            right_Encoder_Direction = 0;       // Encoder measured direction, [-1 or 1]
volatile int            right_Encoder_Direction_Prev = 0;  // Encoder measured direction, [-1 or 1]
volatile int            right_Encoder_Direction_Now = 0;   // Encoder measured direction, [-1 or 1]
volatile unsigned long  right_Time_Prev = 0;               // Time at previous absolute tick change, [us]
volatile unsigned long  right_Time_Now = 0;                // Time at current absolute tick change, [us]
volatile unsigned long  right_Time_Dif = 0;            
volatile unsigned long  right_Time_Acc_Prev = 0;           // Time at previous absolute tick change, [us]
volatile unsigned long  right_Time_Acc_Now = 0;            // Time at current absolute tick change, [us]
         unsigned long  right_Time_Age = 0;                // Time at current absolute tick change, [us]
         unsigned long  right_Time_Saved = 0;              // Time at current absolute tick change, [us]
         
float   left_Speed_RPM = 0;
float   left_Speed_RPM_Unfiltered = 0;
float   left_Speed_RPM_Prev = 0;
float   left_Speed_RPM_Alt = 0;
float   left_Speed_RPM_Alt_Prev = 0;
float   left_Acceleration = 0;                     
int     left_Current_Max = 0;
int     left_Current_Avg = 0;
long    left_Current_Total = 0;
int     left_Current;
int     left_Voltage = 0;

float   right_Speed_RPM = 0;
float   right_Speed_RPM_Prev = 0;
float   right_Acceleration = 0;                     
int     right_Current_Max = 0;
int     right_Current_Avg = 0;
long    right_Current_Total = 0;
int     right_Current;
int     right_Voltage = 0;


float   encoder_Resolution_Size;                          // Encoder Revolution per each encrment in resolution * 100000, calculated in setup 




//  Controller Variables

unsigned long   imu_Time_Now = 0;
unsigned long   imu_Time_Prev = 0;


float           left_Speed_Const = 0;                     // Relates Voltage to Speed.  Determines how much extra voltage is added to PID Voltage.
int             left_PID_Voltage = 0;
float           left_PID_Accel = 0;
float           left_P_Accel = 0;
float           left_I_Accel = 0;
float           left_D_Accel = 0;
float           left_S_Accel = 0;

float           right_Speed_Const = 0;                     // Relates Voltage to Speed.  Determines how much extra voltage is added to PID Voltage.
int             right_PID_Voltage = 0;
float           right_PID_Accel = 0;
float           right_P_Accel = 0;
float           right_I_Accel = 0;
float           right_D_Accel = 0;
float           right_S_Accel = 0;

float           theta_Kp = 1200;
float           theta_Ki = 0;
float           theta_Kd = 130;
float           theta_Ks = -25;
float           theta_Kt = 0.6;
float           theta_Ktd = 0;
float           theta_Zero = 0;
float           theta_Zero_Initial = 0;
float           theta_Zero_Prev = 0;
float           theta_Zero_Unfiltered = 0;
float           theta_Error = 0;
float           theta_Error_Prev = 0;
float           theta_Error_Unfiltered = 0;
float           theta_Now = 0;
float           theta_Now_Unfiltered = 0;
float           theta_Prev = 0;
float           theta_Average = 0;
float           theta_Average_Prev = 0;
float           theta_Speed_Now = 0;
float           theta_Speed_Prev = 0;
float           theta_Speed_Unfiltered = 0;
float           theta_Integral = 0;
float           theta_Smoothed = 0;
float           theta_Smoothed_Prev = 0;
float           theta_Smoothed_Speed = 0;


float           omega_Kp = 1600;
float           omega_Ki = 0;
float           omega_Kd = 130;
float           omega_Ks = -25;
float           omega_Kt = 0.6;
float           omega_Ktd = 1.0;
float           omega_Zero = 0;
float           omega_Zero_Initial = 0;
float           omega_Zero_Prev = 0;
float           omega_Zero_Unfiltered = 0;
float           omega_Error = 0;
float           omega_Error_Prev = 0;
float           omega_Now = 0;
float           omega_Now_Unfiltered = 0;
float           omega_Average = 0;
float           omega_Average_Prev = 0;
float           omega_Prev = 0;
float           omega_Speed_Now = 0;
float           omega_Speed_Prev = 0;
float           omega_Speed_Error = 0;
float           omega_Integral = 0;
float           omega_Smoothed = 0;
float           omega_Smoothed_Prev = 0;
float           omega_Smoothed_Speed = 0;


// OTHERS
int     pot_Value = 0;
long    time_Now = 0;
long    time_Prev = 0;
float   hertz = 0;
int     stat = 0;                                       // Brings you to Change theta_zero mode

int     voltage_Max = 0;
String  package;


// DEBUG
volatile long l = 0;
int     m = 0;
long    o = 0;
int     p = 0;
int     p_Prev = 0;

long    time_IMU_Prev = 0;
long    time_IMU_Now = 0;
long    time_IMU_Dif = 0;

long    time_Probe_Prev = 0;
long    time_Probe_Now = 0;
long    time_Probe_Dif = 0;

// TestBed_Accleration
int left_Target_Voltage = 0;
int right_Target_Voltage = 0;



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




// IMU Global Variables

bool          blinkState = false;
bool          dmpReady = false;     // set true if DMP init was successful
uint8_t       mpuIntStatus;         // holds actual interrupt status byte from MPU
uint8_t       devStatus;            // return status after each device operation (0 = success, !0 = error)
uint16_t      packetSize;           // expected DMP packet size (default is 42 bytes)
uint16_t      fifoCount;            // count of all bytes currently in FIFO
uint8_t       fifoBuffer[64];       // FIFO storage buffer
float         euler[3];             // [psi, theta, phi]    Euler angle container
float         ypr[3];               // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
MPU6050       mpu;                  // Creating object 'mpu', I think
Quaternion    q;                    // [w, x, y, z]         quaternion container
VectorInt16   aa;                   // [x, y, z]            accel sensor measurements
VectorInt16   aaReal;               // [x, y, z]            gravity-free accel sensor measurements
VectorInt16   aaWorld;              // [x, y, z]            world-frame accel sensor measurements
VectorFloat   gravity;              // [x, y, z]            gravity vector

uint8_t       teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };     // packet structure for InvenSense teapot demo
volatile bool mpuInterrupt = false;                                                                 // indicates whether MPU interrupt pin has gone high


