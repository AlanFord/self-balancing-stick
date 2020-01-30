////////////////////////////////////////
// File: _3_Setup.ino
//
//
////////////////////////////////////////
void setup() {

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Serial
  Serial.begin(serial_Frequency);
  
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Pin Modes
  pinMode(pin_IMU_Interrupt, INPUT);
  pinMode(pin_left_PMW, OUTPUT);
  pinMode(pin_right_PMW, OUTPUT);
  pinMode(pin_left_dir, OUTPUT);
  pinMode(pin_right_dir, OUTPUT);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Pull Ups
  digitalWrite(pin_IMU_Interrupt, HIGH);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Interrupts
  PCintPort::attachInterrupt(pin_IMU_Interrupt, dmpDataReady, RISING);
  attachInterrupt(interruptPin_left_Encoder, ISR_left_Encoder, RISING);
  attachInterrupt(interruptPin_right_Encoder, ISR_right_Encoder, RISING);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Setting PMW Wavelength
  byte mode;
  switch (pmw_Length) {
    case 1: mode = 0x01; break;
    case 8: mode = 0x02; break;
    case 64: mode = 0x03; break;
    case 256: mode = 0x04; break;
    case 1024: mode = 0x05; break;
    default:  ;                        // Code that is executed when pmw_Length does not match a case
  }
  TCCR1B = TCCR1B & 0b11111000 | mode;

 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Initializing Other Calculations
  encoder_Resolution_Size = (encoder_Tick_Resolution * 1000000. / encoder_PPR);
  voltage_Max = 255 - voltage_Offset;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // IMU Setup
  Wire.begin();       // Join I2C bus
  TWBR = twbr_Value;  // 12 = 400kHz I2C clock (200kHz if CPU is 8MHz)
  mpu.initialize();   // Initialize IMU

  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed")); // verify connection
  devStatus = mpu.dmpInitialize();                                                // load and configure the DMP

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(233);
  mpu.setYGyroOffset(96);
  mpu.setZGyroOffset(18);
  mpu.setZAccelOffset(1434);                  // 1688 factory default for my test chip

  if (devStatus == 0) {                       // make sure it worked (devStatus returns 0 if so)
    mpu.setDMPEnabled(true);                  // turn on the DMP, now that it's ready
    mpuIntStatus = mpu.getIntStatus();        // enable Arduino interrupt detection (Remove attachInterrupt function here)
    dmpReady = true;                          // set our DMP Ready flag so the main loop() function knows it's okay to use it
    packetSize = mpu.dmpGetFIFOPacketSize();  // get expected DMP packet size for later comparison
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  Serial.println(F("Setup Complete"));
  Serial.println(F(" Press Any Key (but no line return!) to reset FIFO and Start ..."));
  Serial.println(F("\n\n"));
  while (Serial.available() && Serial.read());                                    // empty buffer
  while (!Serial.available());                                                    // wait for data
  while (Serial.available() && Serial.read());                                    // empty buffer again
mpu.resetFIFO();   

}
