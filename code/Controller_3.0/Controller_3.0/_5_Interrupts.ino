// Interrupts Service Routines for Encoders
// Only trigger when signal A rises.  

void ISR_left_Encoder() {

  left_Rel_Tick_Count++;
  if (left_Rel_Tick_Count >= encoder_Tick_Resolution) {

    left_Encoder_Direction_Now = digitalReadFast (pin_left_EncoderB) ? -1 : +1;

    if (left_Encoder_Direction_Now == left_Encoder_Direction_Prev) {                     // Needs to register to ticks in the same directions. Sometimes the encoder will incorretly read wrong direction for one tick.     
      left_Encoder_Direction = left_Encoder_Direction_Now;
    }

    left_Encoder_Direction_Prev = left_Encoder_Direction_Now;
    left_Abs_Tick_Count += left_Encoder_Direction;

    left_Time_Prev = left_Time_Now;
    left_Time_Now = micros();
    left_Time_Dif = left_Time_Now - left_Time_Prev;
    
    left_Rel_Tick_Count = 0;
  }
}


void ISR_right_Encoder() {

  right_Rel_Tick_Count++;
  if (right_Rel_Tick_Count >= encoder_Tick_Resolution) {

    right_Encoder_Direction_Now = digitalReadFast (pin_right_EncoderB) ? -1 : +1;

    if (right_Encoder_Direction_Now == right_Encoder_Direction_Prev) {                // Needs to register to ticks in the same directions. Sometimes the encoder will incorretly read wrong direction for one tick.  
      right_Encoder_Direction = right_Encoder_Direction_Now;
    }

    right_Encoder_Direction_Prev = right_Encoder_Direction_Now;
    right_Abs_Tick_Count += right_Encoder_Direction;

    right_Time_Prev = right_Time_Now;
    right_Time_Now = micros();
    right_Time_Dif = right_Time_Now - right_Time_Prev;
    
    right_Rel_Tick_Count = 0;
  }
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void dmpDataReady() {
  //l++;
  if (PCintPort::arduinoPin == pin_IMU_Interrupt) {
    mpuInterrupt = true;

  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

