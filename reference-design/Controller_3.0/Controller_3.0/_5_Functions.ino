///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////// IMU ////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void get_IMU_Values() {

///////////////////////////////////////////////////// IMU Comms ////////////////////////////////////////////////////////////////////////
  if (!dmpReady) {
    Serial.println("Shit failed yo");
    return;                                               // if programming failed, don't try to do anything
  }

  time_IMU_Prev = micros();

  while (!mpuInterrupt) { // && fifoCount < packetSize) {       // wait for MPU interrupt or extra packet(s) available
    // other program behavior stuff here
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    //Serial.println("Waiting in IMU function");

  }

  time_IMU_Dif = micros() - time_IMU_Prev;


  mpuInterrupt = false;                                             // reset interrupt flag
  mpuIntStatus = mpu.getIntStatus();                                // get INT_STATUS byte
  fifoCount = mpu.getFIFOCount();                                   // get current FIFO count

  if ((mpuIntStatus & 0x10) || fifoCount >= 1024) {                 // check for overflow (this should never happen unless our code is too inefficient)
    mpu.resetFIFO();                                                // reset so we can continue cleanly
    Serial.println(F("FIFO overflow!"));
  }


  else if (mpuIntStatus & 0x02) {                                   // otherwise, check for DMP data ready interrupt (this should happen frequently)
    while (fifoCount < packetSize) {
      delay(1);  // wait for correct available data length, should be a VERY short wait
      fifoCount = mpu.getFIFOCount();
      mpuIntStatus = mpu.getIntStatus();
    }
    mpu.getFIFOBytes(fifoBuffer, packetSize);                       // read a packet from FIFO
    fifoCount -= packetSize;                                        // track FIFO count here in case there is > 1 packet available (this lets us immediately read more without waiting for an interrupt)
    mpu.dmpGetQuaternion(&q, fifoBuffer);                           // display YPR angles in degrees
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }

  mpu.resetFIFO();      // ADDED BY ME!!!!!!!!!!!





  if (p == 1 && p_Prev == 0) {        // Resets integral when balancing mode is started to avoid wide up while holding
    theta_Integral = 0;
    omega_Integral = 0;
    theta_Error = 0;
    omega_Error = 0;
    theta_Smoothed = theta_Now;
    omega_Smoothed = omega_Now;
    theta_Smoothed_Speed = 0;
    omega_Smoothed_Speed - 0;
  }

  p_Prev = p;
  imu_Time_Prev = imu_Time_Now;
  imu_Time_Now = micros();



  ////////////////////////////////////////////////////////// Theta Calcs//////////////////////////////////////////////////////////////////////////////////////////
  
  theta_Prev = theta_Now;
  theta_Now_Unfiltered = round((ypr[2] * 180 / M_PI) * angle_Rounding_Value) / angle_Rounding_Value;    //undo
  theta_Now = (1 - theta_Filter) * (theta_Now_Unfiltered) + (theta_Filter * theta_Prev); //undo


  theta_Error = theta_Now - theta_Zero;

  theta_Integral += theta_Error * (imu_Time_Now - imu_Time_Prev) / 1000000.0;
  if (theta_Integral > theta_Integral_Max) {
    theta_Integral = theta_Integral_Max;
  } else if (theta_Integral < -theta_Integral_Max) {
    theta_Integral = - theta_Integral_Max;
  }


  theta_Speed_Prev = theta_Speed_Now;
  theta_Speed_Now = ((1 - theta_Speed_Filter) * (theta_Now - theta_Prev) / (imu_Time_Now - imu_Time_Prev) * 1000000.) + theta_Speed_Filter * theta_Speed_Prev; // Relative to absolute zero
  theta_Average_Prev = theta_Average;
  theta_Average = (1 - angle_Average_Filter) * (theta_Now) + (angle_Average_Filter * theta_Average_Prev);

  theta_Smoothed_Prev = theta_Smoothed;
  theta_Smoothed = (1 - angle_Smoothed_Filter) * theta_Now + angle_Smoothed_Filter * theta_Smoothed_Prev;
  theta_Smoothed_Speed = (theta_Smoothed - theta_Smoothed_Prev) / (imu_Time_Now - imu_Time_Prev) * 1000000.;
  
  theta_Zero_Prev = theta_Zero; 
  if ( p == 1) {                                                                                                        // Automatic setpoint adjustment
    theta_Zero_Unfiltered = theta_Zero - theta_Kt * theta_Error - theta_Ktd * theta_Smoothed_Speed;
    theta_Zero = (1 - theta_Zero_Filter) * theta_Zero_Unfiltered + theta_Zero_Filter * theta_Zero_Prev;
  } else {
    theta_Zero = theta_Average;
  }

  


  //////////////////////////////////////////////////////////////// Omega Calcs//////////////////////////////////////////////////////////////////////////////////////
  omega_Prev = omega_Now;
  omega_Now_Unfiltered = round((-ypr[1] * 180 / M_PI) * angle_Rounding_Value) / angle_Rounding_Value;    //undo
  omega_Now = (1 - omega_Filter) * (omega_Now_Unfiltered) + omega_Filter * (omega_Prev); //undo
  omega_Error = omega_Now - omega_Zero;


  omega_Error = omega_Now - omega_Zero;

  omega_Integral += omega_Error * (imu_Time_Now - imu_Time_Prev) / 1000000.0;
  if (omega_Integral > omega_Integral_Max) {
    omega_Integral = omega_Integral_Max;
  } else if (omega_Integral < -omega_Integral_Max) {
    omega_Integral = - omega_Integral_Max;
  }


  omega_Speed_Prev = omega_Speed_Now;
  omega_Speed_Now = ((1 - omega_Speed_Filter) * (omega_Now - omega_Prev) / (imu_Time_Now - imu_Time_Prev) * 1000000.) + omega_Speed_Filter * omega_Speed_Prev; // Relative to absolute zero
  omega_Average_Prev = omega_Average;
  omega_Average = (1 - angle_Average_Filter) * (omega_Now) + (angle_Average_Filter * omega_Average_Prev);

  omega_Smoothed_Prev = omega_Smoothed;
  omega_Smoothed = (1 - angle_Smoothed_Filter) * omega_Now + angle_Smoothed_Filter * omega_Smoothed_Prev;
  omega_Smoothed_Speed = (omega_Smoothed - omega_Smoothed_Prev) / (imu_Time_Now - imu_Time_Prev) * 1000000.;
  
  omega_Zero_Prev = omega_Zero;
  if ( p == 1) {                                                                                                     // Automatic setpoint adjustment
    omega_Zero_Unfiltered = omega_Zero - omega_Kt * omega_Error - omega_Ktd * omega_Smoothed_Speed;
    omega_Zero = (1 - omega_Zero_Filter) * omega_Zero_Unfiltered + omega_Zero_Filter * omega_Zero_Prev;
  } else {
    omega_Zero = omega_Average;
  }

}




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////  SPEED  ////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void get_left_Encoder_Speeds() {
  left_Speed_RPM_Prev = left_Speed_RPM;
  //left_Speed_RPM_Alt_Prev = left_Speed_RPM_Alt;

  left_Time_Saved = left_Time_Now;
  left_Time_Age = micros();

  left_Encoder_Direction_Debug = left_Encoder_Direction;

  left_Time_Acc_Prev = left_Time_Acc_Now;
  left_Time_Acc_Now = micros();

  if ( (left_Time_Age - left_Time_Saved) > (encoder_Speed_Time_Limit * 1000L)) {                                                 // If last time measurment was taken too long ago, it means wheel has stopped
    left_Speed_RPM = 0;
    left_Speed_RPM_Unfiltered = 0;
   } else {
    left_Speed_RPM_Unfiltered = encoder_Resolution_Size / left_Time_Dif * 60 * left_Encoder_Direction_Debug;    // Speed measurement
      }

  left_Abs_Tick_Count_Prev = left_Abs_Tick_Count;
  left_Speed_RPM = ((1 - left_Speed_Filter) * left_Speed_RPM_Unfiltered) + (left_Speed_Filter * left_Speed_RPM_Prev);
  left_Acceleration = (left_Speed_RPM - left_Speed_RPM_Prev) / (left_Time_Acc_Now - left_Time_Acc_Prev) * 1000000;
}


void get_right_Encoder_Speeds() {
  right_Speed_RPM_Prev = right_Speed_RPM;
  right_Time_Acc_Prev = right_Time_Acc_Now;
  right_Time_Acc_Now = micros();

  right_Time_Saved = right_Time_Now;
  right_Time_Age = micros();


  if ( (right_Time_Age - right_Time_Saved) > (encoder_Speed_Time_Limit * 1000L)) {                                                 // If last time measurment was taken too long ago, it means wheel has stopped
    right_Speed_RPM = 0;
      } else {
    right_Speed_RPM = encoder_Resolution_Size / right_Time_Dif * 60 * right_Encoder_Direction;    // Speed measurement
    
  }

  right_Speed_RPM = ((1 - right_Speed_Filter) * right_Speed_RPM) + (right_Speed_Filter * right_Speed_RPM_Prev);
  right_Acceleration = (right_Speed_RPM - right_Speed_RPM_Prev) / (right_Time_Acc_Now - right_Time_Acc_Prev) * 1000000;
}




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////SET VOLTAGE /////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void set_left_Motor_Voltage() {         // Switched directions (0's and 1's) from originial
  if (left_Voltage == 0) {
    analogWrite(pin_left_PMW, 0);
  }
  else {
    if (left_Voltage < 0) {
      digitalWrite(pin_IN1, 1);
      digitalWrite(pin_IN2, 0);
    } else {
      digitalWrite(pin_IN1, 0);
      digitalWrite(pin_IN2, 1);
    }
    analogWrite(pin_left_PMW, abs(left_Voltage) + voltage_Offset);
  }
}


void set_right_Motor_Voltage() {         // Switched directions (0's and 1's) from originial
  if (right_Voltage == 0) {
    analogWrite(pin_right_PMW, 0);
  }
  else {
    if (right_Voltage < 0) {
      digitalWrite(pin_IN3, 1);
      digitalWrite(pin_IN4, 0);
    } else {
      digitalWrite(pin_IN3, 0);
      digitalWrite(pin_IN4, 1);
    }
    analogWrite(pin_right_PMW, abs(right_Voltage) + voltage_Offset);
  }
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////// UPDATE VALUE USING SERIAL CONSOLE //////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float update_Value(float x) {
  analogWrite(pin_left_PMW, 0);
  analogWrite(pin_right_PMW, 0);
  Serial.print(F("\n\n"));
  Serial.print(F("Current Value:\t"));
  Serial.println(x, 4);
  Serial.println(F("Input New Value:"));
  x = get_Serial();
  Serial.print(F("Value changed to:\t"));
  Serial.println(x, 4);
  delay(100);
  mpu.resetFIFO();
  delay(5);
  stat = 0;
  return x;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////GET SERIAL INPUT FROM CONSOLE//////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


float get_Serial() {

  String inString = "";
  //  Serial.println("Input New Value:");
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data


  delay(1);

  while (Serial.available() > 0 ) {
    inString += (char)Serial.read();
  }

  return inString.toFloat();
}
