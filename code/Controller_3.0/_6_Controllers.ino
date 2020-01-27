

void get_left_PID_Voltage_Value() {

  left_P_Accel = theta_Kp * (theta_Now - theta_Zero);
  left_I_Accel = theta_Ki * theta_Integral;
  left_D_Accel = theta_Kd * theta_Speed_Now;
  left_S_Accel = theta_Ks * left_Speed_RPM / 1000.;
  left_PID_Accel = left_P_Accel + left_I_Accel + left_D_Accel + left_S_Accel;

  float friction = 0;
  if (left_Speed_RPM > 0.5) {
    friction = friction_Value;
  } else if (left_Speed_RPM < -0.5) {
    friction = -friction_Value;
  }

  float voltage = (left_PID_Accel + 0.303 * left_Speed_RPM + friction) / 9.4;     // Equation measured from Acceleration Motor Tests

  left_PID_Voltage = round(constrain(voltage, -voltage_Max, voltage_Max));
}




void get_right_PID_Voltage_Value() {

  right_P_Accel = omega_Kp * (omega_Now - omega_Zero);
  right_I_Accel = omega_Ki * omega_Integral;
  right_D_Accel = omega_Kd * omega_Speed_Now;
  right_S_Accel = omega_Ks * right_Speed_RPM / 1000.;
  right_PID_Accel = right_P_Accel + right_I_Accel + right_D_Accel + right_S_Accel;

  float friction = 0;
  if (right_Speed_RPM > 0.5) {
    friction = friction_Value;
  } else if (right_Speed_RPM < -0.5) {
    friction = -friction_Value;
  }

  float voltage = (right_PID_Accel + 0.295 * right_Speed_RPM + friction) / 9.4;      // Equation measured from Acceleration Motor Tests


  right_PID_Voltage = round(constrain(voltage, -voltage_Max, voltage_Max));
}


