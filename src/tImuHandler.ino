float imuRoll = 0;
float imuPitch = 0;
float imuYaw = 0;
float gyroX = 0;
float gyroY = 0;
float gyroZ = 0;

// PANDA
char headingPanda[6];
char rollPanda[6];
char pitchPanda[6];
char yawRatePanda[6];

//INS
char insHeading[14];
char insRoll[14];
char insPitch[14];

// HPR
float rollDual = 0;
char umHeading[8];
char umRoll[8];

void imuSetup(){
  uint8_t error;
  ImuWire.begin();

  for (int16_t i = 0; i < nrBNO08xAdresses; i++)
  {
    bno08xAddress = bno08xAddresses[i];

    // Serial.print("\r\nChecking for BNO08X on ");
    // Serial.println(bno08xAddress, HEX);
    ImuWire.beginTransmission(bno08xAddress);
    error = ImuWire.endTransmission();

    if (error == 0)
    {
      // Serial.println("Error = 0");
      Serial.print("0x");
      Serial.print(bno08xAddress, HEX);
      Serial.println(" BNO08X Ok.");

      // Initialize BNO080 lib
      if (bno08x.begin(bno08xAddress, ImuWire)) //??? Passing NULL to non pointer argument, remove maybe ???
      {
        // Increase I2C data rate to 400kHz
        ImuWire.setClock(400000);

        //bno08x.calibrateAll();

        delay(300);

        // Use gameRotationVector and set REPORT_INTERVAL
        //bno08x.enableGameRotationVector(REPORT_INTERVAL);
        bno08x.enableGyroIntegratedRotationVector(REPORT_INTERVAL);
        bno08x.enableAccelerometer(REPORT_INTERVAL);
        useBNO08x = true;
        //bno08x.calibrateAccelerometer();
        //bno08x.saveCalibration(); //Saves the current dynamic calibration data (DCD) to memory
        //bno08x.requestCalibrationStatus(); //Sends command to get the latest calibration status
      }
      else
      {
        Serial.println("BNO080 not detected at given I2C address.");
      }
    }
    else
    {
      // Serial.println("Error = 4");
      Serial.print("0x");
      Serial.print(bno08xAddress, HEX);
      Serial.println(" BNO08X not Connected or Found");
    }
    if (useBNO08x)
      break;
  }
}

void readBNO()
{
  if (bno08x.dataAvailable() == true)
  {
    float dqx, dqy, dqz, dqw, dacr;
    uint8_t dac;

    // get quaternion
    bno08x.getQuat(dqx, dqy, dqz, dqw, dacr, dac);

    float norm = sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz);
    dqw = dqw / norm;
    dqx = dqx / norm;
    dqy = dqy / norm;
    dqz = dqz / norm;

    float ysqr = dqy * dqy;

    // yaw (z-axis rotation)
    float t3 = +2.0 * (dqw * dqz + dqx * dqy);
    float t4 = +1.0 - 2.0 * (ysqr + dqz * dqz);
    imuYaw = atan2(t3, t4);

    // Convert yaw to degrees x10
    imuYaw = (int16_t)((imuYaw * -RAD_TO_DEG));
    if (imuYaw < 0)
      imuYaw += 360;

    // pitch (y-axis rotation)
    float t2 = +2.0 * (dqw * dqy - dqz * dqx);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;

    // roll (x-axis rotation)
    float t0 = +2.0 * (dqw * dqx + dqy * dqz);
    float t1 = +1.0 - 2.0 * (dqx * dqx + ysqr);

    if (steerConfig.IsUseY_Axis)
    {
      imuRoll = asin(t2) * RAD_TO_DEG;
      imuPitch = atan2(t0, t1) * RAD_TO_DEG;
    }
    else
    {
      imuPitch = asin(t2) * RAD_TO_DEG;
      imuRoll = atan2(t0, t1) * RAD_TO_DEG;
    }

    gyroX = bno08x.getFastGyroX()* RAD_TO_DEG;
    gyroY = bno08x.getFastGyroY()* RAD_TO_DEG;
    gyroZ = bno08x.getFastGyroZ()* RAD_TO_DEG;

    if (invertRoll)
    {
      imuRoll *= -1;
      gyroX *= -1;
    }




    //float x = bno08x.getAccelX();
    //float y = bno08x.getAccelY();
    //float z = bno08x.getAccelZ();

    // Serial.print(x, 4);
    // Serial.print(",");
    // Serial.print(y, 4);
    // Serial.print(",");
    // Serial.println(z, 4);

    // if(bno08x.calibrationComplete() == true)
    //       {
    //         Serial.println("Calibration data successfully stored");
    //         delay(1000);
    //       }
    // else 
    //   Serial.println("not");

    //Serial.print("imuRoll:");
    //Serial.println(imuRoll);
  }
}

void imuHandler()
{
  int16_t temp = 0;

  if (useBNO08x)
  {
    // BNO is reading in its own timer
    //  Fill rest of Panda Sentence - Heading
    temp = (int16_t)(imuYaw*10);
    itoa(temp, headingPanda, 10);

    // the pitch x10
    temp = (int16_t)(imuPitch*10);
    itoa(temp, pitchPanda, 10);

    // the roll x10
    temp = (int16_t)(imuRoll*10);
    itoa(temp, rollPanda, 10);

    // YawRate - 0 for now
    itoa(0, yawRatePanda, 10);
  }
  else // Not using IMU so put dual Heading & Roll in direct.
  {
    if (makeOGI)
    {
      if (usingUM982)
      {
        snprintf (rollPanda, sizeof(rollPanda), "%f", rollWT);
        strcpy(headingPanda, umHeading);
      }
      else
      {
        strcpy(rollPanda, insRoll);
        strcpy(headingPanda, insHeading);
      }
    }
    else
    {
      float rol = atof(insRoll);
      float head = atof(insHeading);
      itoa(rol * 10, rollPanda, 10);
      itoa(head * 10, headingPanda, 10);
      // itoa(rollWT * 10, rollPanda, 10);
      // itoa(headingWT * 10, headingPanda, 10);
    }

    // the pitch
    dtostrf(0, 4, 4, pitchPanda);

    // YawRate - 0 for now
    itoa(0, yawRatePanda, 10);
  }
}