//////////////////////////////////
//  This loop() is designed to step through
//  possible voltages, min to max, in 50 intervals.
//  at each voltage a delay permits the speed to stabilize
//  and then the encoder results are printed.
//
//  How to Use:
//  Determine the voltage being supplied to the motor driver.
//  From the motor data sheet, determine the max RPM (no load)
//  Scaling the no load max RPM based on the applied voltage,
//  determine if the encoder RPM values are reasonable.
//////////////////////////////////

int multiplier = -1;

void loop() {
  multiplier = -multiplier;
  for (int i=0; i<=250; i=i+10) {
    left_Voltage = i*multiplier;
    right_Voltage = i*multiplier;
    set_left_Motor_Voltage();
    set_right_Motor_Voltage();
    delay(1000);
    get_left_Encoder_Speeds();
    get_right_Encoder_Speeds();
    Serial.print("Voltage:\t");
    Serial.print(left_Voltage);
    Serial.print("\t Left Encoder Speed:\t");
    Serial.print(left_Speed_RPM,4);
    Serial.print("\t Right Encoder Speed:\t");
    Serial.println(right_Speed_RPM,4);
    delay(250);
  }
    for (int i=250; i>=0; i=i-10) {
    left_Voltage = i*multiplier;
    right_Voltage = i*multiplier;
    set_left_Motor_Voltage();
    set_right_Motor_Voltage();
    delay(1000);
    get_left_Encoder_Speeds();
    get_right_Encoder_Speeds();
    Serial.print("Voltage:\t");
    Serial.print(left_Voltage);
    Serial.print("\t Left Encoder Speed:\t");
    Serial.print(left_Speed_RPM,4);
    Serial.print("\t Right Encoder Speed:\t");
    Serial.println(right_Speed_RPM,4);
    delay(250);
  }
}



//void loop() {
//  for (int i=-255; i <= 255; i=i+10) {
//    left_Voltage = i;
//    right_Voltage = i;
//    set_left_Motor_Voltage();
//    set_right_Motor_Voltage();
//    delay(1000);
//    get_left_Encoder_Speeds();
//    get_right_Encoder_Speeds();
//    Serial.print("Voltage:\t");
//    Serial.print(left_Voltage);
//    Serial.print("\t Left Encoder Speed:\t");
//    Serial.print(left_Speed_RPM,4);
//    Serial.print("\t Right Encoder Speed:\t");
//    Serial.println(right_Speed_RPM,4);
//    delay(250);
//  }
//}
