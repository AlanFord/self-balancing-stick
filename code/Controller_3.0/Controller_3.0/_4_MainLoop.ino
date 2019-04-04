void loop() {

  if (Serial.available() > 0) {
    stat = Serial.read();
  }


  // Main State Machine, dictated by user's input through serial monitor
  switch (stat) {

    // MODE - Main Loop, enters if either charging or balancing
    case  127:
    case 121:
    case 104:
    case 110:
    case 117:
    case 106:
    case 109:

      p = 0;                                        // If switching to balanceing, reset integrals
      if (stat == 117 || stat == 106 || stat == 109) {
        p = 1;
      }

      get_IMU_Values();
      get_left_Encoder_Speeds();
      get_right_Encoder_Speeds();
      
      get_left_PID_Voltage_Value ();
      get_right_PID_Voltage_Value ();

      switch (stat) {
        case 121:                                     // y - Charge left motor
          left_Voltage = left_Target_Voltage;
          right_Voltage = 0;
          break;

        case 104:                                     // h - Charge right motor
          left_Voltage = 0;
          right_Voltage = right_Target_Voltage;
          break;

        case 110:                                     // n - Charge both motors
          left_Voltage = left_Target_Voltage;
          right_Voltage = right_Target_Voltage;
          break;

        case 117:                                     // u - Balnce with left motor
          left_Voltage = left_PID_Voltage;
          right_Voltage = 0;
          break;

        case 106:                                     // j - Balance with right motor
          left_Voltage = 0;
          right_Voltage = right_PID_Voltage;
          break;

        case 109:                                     // m - Balance with both motors
          left_Voltage = left_PID_Voltage;
          right_Voltage = right_PID_Voltage;
          break;

        default:                                      // space - Motors off, but print updates
          left_Voltage = 0;
          right_Voltage = 0;
          break;
      }


      set_left_Motor_Voltage();
      set_right_Motor_Voltage();



      //////////////////////////////////////////// START SERIAL PRINT///////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      // Serial Console
      package = "";
      time_Probe_Prev = micros();
      //    Serial.print(l);
      //    Serial.print("\t");


      //       Serial.print((int)hertz);
      //       Serial.print("\t");
      package += millis();
      package += "\t";
      ////      Serial.print("PID:\t");
      ////      Serial.print(int(theta_Kp));
      ////      Serial.print("\t");
      ////      Serial.print(int(theta_Ki));
      ////      Serial.print("\t");
      ////      Serial.print(int(theta_Kd));
      //      Serial.print("\t");


      //      Serial.print(F("Speed|Accel:")); undo
      //      Serial.print("\t");
      //
      //      //    Serial.print(left_Time_Age);
      //      //    Serial.print("\t");
      //      //    Serial.print(left_Time_Saved);
      //      //    Serial.print("\t");
      package += "\t";
//      package += (int)left_Abs_Tick_Count;
//      package += "\t\t";
      package += (int)left_Speed_RPM;
      package += "\t";
//      package += (int)left_Speed_RPM_Unfiltered;
//      package += "\t";
      package += (int)right_Speed_RPM;
      package += "\t";
//      package += left_Encoder_Direction_Debug * 100;
//      package += "\t";
      //      Serial.print((int)left_Acceleration);
      //      Serial.print("\t");
      //      Serial.print((int)right_Acceleration);
      //      Serial.print("\t");
      //    Serial.print(left_Voltage);
      //    Serial.print("\t");
      //    Serial.print("\t");

      //    Serial.print(left_Voltage);
      //    Serial.print("\t");
      //    Serial.print(right_Voltage);
      //    Serial.print("\t");
      //    Serial.print("\t");
      //
      //      Serial.print("Pos:\t"); undu
      //      Serial.print(theta_Error);
      //      Serial.print("\t");
      //      Serial.print(omega_Error);
      //      Serial.print("\t");
      //      Serial.print("\t");
      //
      //    Serial.print("Wheel Speed:\t");
      //    Serial.print((int)left_Speed_RPM);
      //    Serial.print("\t");
      //    Serial.print(int(right_Speed_RPM));
      //    Serial.print("\t");
      //    Serial.print("\t");
      //        Serial.print(left_Time_Prev);
      //        Serial.print("\t");
      //        Serial.print(left_Time_Now);
      //        Serial.print("\t");
      //    Serial.print(left_Encoder_Direction);
      //    Serial.print("\t");
      //    Serial.print(encoder_Resolution_Size);
      //    Serial.print("\t");

      //    Serial.print("Wheel Acc.:\t");
      //    Serial.print((int)left_Acceleration);
      //    Serial.print("\t");

      //    Serial.print("YPR:\t");
      //    Serial.print(ypr[0] * 180 / M_PI);
      //    Serial.print("\t");
      //    Serial.print(ypr[1] * 180 / M_PI);
      //    Serial.print("\t");
      //    Serial.print(ypr[2] * 180 / M_PI);
      //    Serial.print("\t");
      //    Serial.print("\t");

      //    Serial.print("Theta Error:\t");
      //    Serial.print(theta_Error);
      //    Serial.print("\t");
      //    Serial.print("\t");

      //    Serial.print("Theta Speed:\t");
      //    Serial.print(theta_Speed_Now);
      //    Serial.print("\t");
      //    Serial.print("\t");
      //
      //    Serial.print("Theta Integra;:\t");
      //    Serial.print(theta_Integral);
      //    Serial.print("\t");
      //    Serial.print("\t");

      //    Serial.print("Wheel Pos:\t");
      //    Serial.print(left_Abs_Tick_Count);
      //    Serial.print("\t");
      //    Serial.print(right_Abs_Tick_Count);
      //    Serial.print("\t");
      //    Serial.print("\t");

      //      Serial.print(F("CurrentMax|Avg:\t"));
      //      Serial.print(left_Current_Max);
      //      Serial.print("\t");
      //      //    //    Serial.print(right_Current_Max);
      //      //    //    Serial.print("\t");
      //      //    Serial.print("\t");
      //      //
      //      //    Serial.print("CurrentAvg:\t");
      //      Serial.print(left_Current_Avg);
      //      Serial.print("\t");
      //      //    Serial.print(right_Current_Avg);
      //      //    Serial.print("\t");
      //      Serial.print("\t");

      //    Serial.print("PID:\t");
      //    Serial.print(int(theta_Kp));
      //    Serial.print("\t");
      //    Serial.print(int(theta_Ki));
      //    Serial.print("\t");
      //    Serial.print(int(theta_Kd));
      //    Serial.print("\t");
      //    Serial.print("\t");

      package += F("Left:\t");
      //      Serial.print((int)(theta_Now_Unfiltered * 1000));
      //      Serial.print("\t");
      package += (int)(theta_Now * 1000);
      package += F("\t");
      package += (int)(theta_Zero * 1000);
      package += F("\t");
//      package += (int)(theta_Smoothed * 1000);
//      package += F("\t");
//      package += (int)(theta_Smoothed_Speed * 1000);
//      package += F("\t");
//      package += F("\t");
//      //      Serial.print((int)(theta_Zero*1000));
//      //      Serial.print("\t");
      package += int(left_P_Accel);
      package += "\t";
//      package += int(left_I_Accel);
//      package += "\t";
//      //      Serial.print(int(left_I_Accel));
//      //      Serial.print("\t");
//      //      Serial.print("\t");
//      //      Serial.print(int(theta_Speed_Unfiltered * theta_Kd));
//      //      Serial.print("\t");
      package += (int)(left_D_Accel);
      package += "\t";
      package += (int)(left_S_Accel);
      //      Serial.print("\t");
//      //      Serial.print(int(left_PID_Accel));
//      //      Serial.print("\t");
      package += F("\t\t");

      package += F("Right:\t");
      //      Serial.print((int)(omega_Error*1000));
      //      Serial.print("\t");
      package += (int)(omega_Now * 1000);
      package += "\t";
      package += (int)(omega_Zero * 1000);
      package += F("\t");
      package += (int)(omega_Smoothed * 1000);
      package += F("\t");
//      package += (int)(omega_Smoothed_Speed * 1000);
//      package += F("\t");
//      package += F("\t");
      package += (int)(right_P_Accel);
      package += "\t";
//      package += (int)(right_I_Accel);
//      package += "\t";
//      //      Serial.print(int(right_I_Accel));
//      //      Serial.print("\t");
      package += (int)(right_D_Accel);
      package += "\t";
      package += (int)(right_S_Accel);
//      //      Serial.print("\t");
//      ////      Serial.print(int(left_PID_Accel));
//      ////      Serial.print("\t");
      package += "\t\t";

      package += F("V:\t");
      package += left_Voltage;
      package += F("\t");
      package += right_Voltage;
      package += F("\t");

      //    Serial.print("mpuInterrupt:\t");
      //    Serial.print(mpuInterrupt, HEX);
      //    Serial.print("\t");
      //
      //    Serial.print("mpuIntStatus:\t");
      //    Serial.print(mpuIntStatus, BIN);
      //    Serial.print("\t");
      //
      //    Serial.print("fifoCount:\t");
      //    Serial.print(fifoCount, HEX);
      //    Serial.print("\t");
      //
      //    Serial.print("packetSize:\t");
      //    Serial.print(packetSize);
      //    Serial.print("\t");

      //    Serial.print("m:\t");
      //    Serial.print(m);
      //    Serial.print("\t");

      //    Serial.print("l:\t");
      //    Serial.print(l);
      //    Serial.print("\t");
      //
      //    Serial.print("o:\t");
      //    Serial.print(o);
      //    Serial.print("\t");

      package += F("\t");
      package += p;
      package += F("\t");

      package += "T:\t";
      package += time_IMU_Dif;
      package += F("\t");


      //time_Probe_Dif = micros() - time_Probe_Prev;
      package += time_Probe_Dif;
      //      Serial.print(F("\t");
      //


      //delay(time_Delay);
      time_Prev = time_Now;
      time_Now = millis();
      hertz = 1000. / (time_Now - time_Prev);

      Serial.println(package);
      //////////////////////////////////////////// END SERIAL PRINT ///////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      time_Probe_Dif = micros() - time_Probe_Prev;

      break;


    // MODE - Change Zeros, Input: "z"
    case 122:

      analogWrite(pin_left_PMW, 0);
      analogWrite(pin_right_PMW, 0);
      Serial.print("\n\n");
      mpu.resetFIFO();
      delay(10);
      while (Serial.available() == 0) {
        get_IMU_Values();
        Serial.print(millis());
        Serial.print("\t");
        Serial.print(F("Zeros:\t"));
        Serial.print(theta_Zero_Initial);
        Serial.print(F("\t"));
        Serial.print(omega_Zero_Initial);
        Serial.print(F("\t"));
        Serial.print(F("Current Zeros:\t"));
        Serial.print(theta_Now);
        Serial.print(F("\t"));
        Serial.print(omega_Now);
        Serial.print(F("\t\t\t Enter 'z' to set new zeros."));
        Serial.print("\n");
        delay(1);
      }

      stat = Serial.read();
      switch (stat) {

        case 122:                            // z - Set both Zeros to current Value
          theta_Zero_Initial = theta_Now;
          omega_Zero_Initial = omega_Now;
          Serial.print(F("Zeros Changed to:\t"));
          Serial.print(theta_Zero_Initial);
          Serial.print(F("\t"));
          Serial.print(omega_Zero_Initial);
          Serial.print(F("\n\n"));
          break;

        case 120:                            // x - Set theta_Zeros to current Value
          theta_Zero_Initial = theta_Now;
          Serial.print(F("theta_Zero Changed to:\t"));
          Serial.print(theta_Zero_Initial);
          Serial.print(F("\n\n"));
          break;

        case 99:                            // c - Set omega_Zeros to current Value
          omega_Zero_Initial = omega_Now;
          Serial.print(F("omega_Zero Changed to:\t"));
          Serial.print(omega_Zero_Initial);
          Serial.print(F("\n\n"));
          break;

        case 111:                            // o - Manually set theta_Zero
          Serial.print("\n\n");
          Serial.println(F("Input New Value for Theta Zero:"));
          theta_Zero_Initial = get_Serial();
          Serial.print(F("Theta Zero changed to:\t"));
          Serial.println(theta_Zero_Initial);
          break;

        case 108:                            // l - Manually set omega_Zero
          Serial.print("\n\n");
          Serial.println(F("Input New Value for Omega Zero:"));
          omega_Zero_Initial = get_Serial();
          Serial.print(F("Omega Zero changed to:\t"));
          Serial.println(omega_Zero_Initial);
          break;

        default:
          Serial.print(F("Zeros Not Changed\t"));
          Serial.print("\n\n");
          break;
      }

      delay(1000);
      mpu.resetFIFO();
      delay(1);
      stat = 0;
      break;



    // MODE - Set New Values
    case 113:                                   // q - theta_Kp
      theta_Kp = update_Value(theta_Kp);
      break;

//   case 119:                                   // w - theta_Ki
//      theta_Ktd = update_Value(theta_Ktd);
//      break;
      
    case 119:                                   // w - theta_Ki
      theta_Ktd = update_Value(theta_Ktd);
      break;

    case 101:                                   // e - theta_Kd
      theta_Kd = update_Value(theta_Kd);
      break;

    case 114:                                   // r - theta_Ks
      theta_Ks = update_Value(theta_Ks);
      break;

    case 111:                                   // o - theta_Kt
      theta_Kt = update_Value(theta_Kt);
      break;

    case 97:                                    // a - omega_Kp
      omega_Kp = update_Value(omega_Kp);
      break;

//    case 115:                                   // s - omega_Ki
//      omega_Ki = update_Value(omega_Ki);
//      break;

    case 115:                                   // s - omega_Ki
      omega_Ktd = update_Value(omega_Ktd);
      break;

    case 100:                                   // d - omega_Kd
      omega_Kd = update_Value(omega_Kd);
      break;

    case 102:                                   // f - omega_Ks
      omega_Ks = update_Value(omega_Ks);
      break;

    case 108:                                   // l - theta_Kt
      omega_Kt = update_Value(omega_Kt);
      break;

    case 116:                                   // t - left_Target_Voltage
      left_Target_Voltage = (int)update_Value(left_Target_Voltage);
      break;

    case 103:                                   // g - right_Target_Voltage
      right_Target_Voltage = (int)update_Value(right_Target_Voltage);
      break;

    case 49:                                    // 1 - angle_Average_Filter
      angle_Average_Filter = update_Value(angle_Average_Filter);
      break;

    case 50:                                    // 2 - theta_Zero_Filter
      theta_Zero_Filter = update_Value(theta_Zero_Filter);
      break;

    case 51:                                    // 3 - omega_Zero_Filter
      omega_Zero_Filter = update_Value(omega_Zero_Filter);
      break;

    case 52:                                    // 4 - angle_Smoothed_Filter
      angle_Smoothed_Filter = update_Value(angle_Smoothed_Filter);
      break;
    
    case 57:                                    // 9 - friction_Value
      friction_Value = update_Value(friction_Value);
      break;



    // MODE - Show Extended Data
    case 112:                            // p - Show Extended Data
      analogWrite(pin_left_PMW, 0);
      analogWrite(pin_right_PMW, 0);
      Serial.print(F("\n\n"));
      Serial.print(F("theta PID:\t"));
      Serial.print(int(theta_Kp));
      Serial.print("\t");
//      Serial.print(int(theta_Ki));
      Serial.print(theta_Ktd);
      Serial.print("\t");
      Serial.print(int(theta_Kd));
      Serial.print("\t");
      Serial.print(int(theta_Ks));
      Serial.print("\t");
      Serial.print(theta_Kt, 3);
      Serial.print(F("\n"));

      Serial.print(F("omega PID:\t"));
      Serial.print(int(omega_Kp));
      Serial.print("\t");
//      Serial.print(int(omega_Ki));
      Serial.print(omega_Ktd);
      Serial.print("\t");
      Serial.print(int(omega_Kd));
      Serial.print("\t");
      Serial.print(int(omega_Ks));
      Serial.print("\t");
      Serial.print(omega_Kt, 3);
      Serial.print(F("\n\n"));

      Serial.print(F("Angle Average Filter:"));
      Serial.print("\t");
      Serial.print(angle_Average_Filter, 3);
      Serial.print("\n");
      Serial.print(F("Theta Zero Filter:"));
      Serial.print("\t");
      Serial.print(theta_Zero_Filter, 3);
      Serial.print("\n");
      Serial.print(F("Omega Zero Filter:"));
      Serial.print("\t");
      Serial.print(omega_Zero_Filter, 3);
      Serial.print(F("\n\n"));

      Serial.print(F("Friction:"));
      Serial.print("\t");
      Serial.print(friction_Value);
      Serial.print(F("\n\n"));
      
//      Serial.print(F("left Target Voltage:\t"));
//      Serial.print(int(left_Target_Voltage));
//      Serial.print(F("\n"));
//
//      Serial.print(F("right Target Voltage:\t"));
//      Serial.print(int(right_Target_Voltage));
//      Serial.print(F("\n"));

      Serial.print(F("\n"));
      Serial.println(F("Enter any key to continue ..."));
      while (Serial.available() && Serial.read()); // empty buffer
      while (!Serial.available()) {                // wait for data
        get_IMU_Values();
        delay(1);
      }
      break;



    // MODE - Pause, Input: "all other keys"
    default:
      analogWrite(pin_left_PMW, 0);
      analogWrite(pin_right_PMW, 0);
      Serial.println(F("Program is Paused"));
      while (Serial.available() && Serial.read()); // empty buffer
      mpu.resetFIFO();
      delay(5);
      while (!Serial.available()) {                // wait for data
        //get_IMU_Values();
        delay(5);
      }
      stat = Serial.read();
      if (stat ==  32) {                 // test if space was entered
        stat = 127;                                // if so, enter main loop
      }

      Serial.println(F("Program is Unpaused"));
      Serial.println(stat);
      mpu.resetFIFO();
      delay(1);
      break;

  }
}
