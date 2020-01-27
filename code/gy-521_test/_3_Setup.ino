void setup() {

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Serial
  Serial.begin(serial_Frequency);
  
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Pin Modes
  pinMode(pin_IMU_Interrupt, INPUT);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Pull Ups
  digitalWrite(pin_IMU_Interrupt, HIGH);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Interrupts
  PCintPort::attachInterrupt(pin_IMU_Interrupt, dmpDataReady, RISING);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // IMU Setup
  Wire.begin();       // Join I2C bus
  TWBR = twbr_Value;  // 12 = 400kHz I2C clock (200kHz if CPU is 8MHz)
  mpu.initialize();   // Initialize IMU

  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed")); // verify connection
  devStatus = mpu.dmpInitialize();                                    // load and configure the DMP

  // supply your own gyro offsets here, scaled for min sensitivity
  //mpu.setXGyroOffset(220);
  //mpu.setYGyroOffset(76);
  //mpu.setZGyroOffset(-85);
  //mpu.setZAccelOffset(1788);                  // 1688 factory default for my test chip

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
  Serial.println(" Press Any Key (but no line return!) to reset FIFO and Start ...");
  Serial.println(F("\n\n"));
  while (Serial.available() && Serial.read());                                    // empty buffer
  while (!Serial.available());                                                    // wait for data
  while (Serial.available() && Serial.read());                                    // empty buffer again
  mpu.resetFIFO();   
}
