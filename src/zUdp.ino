
#ifdef ARDUINO_TEENSY41
// UDP Receive
void ReceiveUdp()
{
  // When ethernet is not running, return directly. parsePacket() will block when we don't
  if (!Ethernet_running)
  {
    if (debugState == UDP)
      debugPrintln("Ethernet not running");
    return;
  }

  uint16_t len = Eth_udpAutoSteer.parsePacket();

  // if (len > 0)
  // {
  //  debugPrint("ReceiveUdp: ");
  //  debugPrintln(len);
  // }

  // Check for len > 4, because we check byte 0, 1, 3 and 3
  if (len > 4)
  {
    Eth_udpAutoSteer.read(autoSteerUdpData, 100);

    if (debugState == UDP)
    {
      debugPrint("ReceivedPacket: ");
      for (int l = 0; l < len; l++)
        debugPrint(autoSteerUdpData[l]);
      debugPrintln();
    }

    if (autoSteerUdpData[0] == 0x80 && autoSteerUdpData[1] == 0x81 && autoSteerUdpData[2] == 0x7F) // Data
    {
      if (autoSteerUdpData[3] == 0xFE && Autosteer_running) // 254
      {
        gpsSpeed = ((float)(autoSteerUdpData[5] | autoSteerUdpData[6] << 8)) * 0.1; // is negative when in reverse? No!
        gpsSpeedUpdateTimer = 0;

        prevGuidanceStatus = guidanceStatus;

        guidanceStatus = autoSteerUdpData[7];
        guidanceStatusChanged = (guidanceStatus != prevGuidanceStatus);

        // Bit 8,9    set point steer angle * 100 is sent
        steerAngleSetPoint = ((float)(autoSteerUdpData[8] | ((int8_t)autoSteerUdpData[9]) << 8)) * 0.01; // high low bytes

        if ((bitRead(guidanceStatus, 0) == 0) || (gpsSpeed < 0.1) || (steerSwitch == 1))
        {
          watchdogTimer = WATCHDOG_FORCE_VALUE; // turn off steering motor
        }
        else // valid conditions to turn on autosteer
        {
          watchdogTimer = 0; // reset watchdog
        }

        // Bit 10 Tram
        tram = autoSteerUdpData[10];
        if (tram != 255)
          XTE = XTE * 0.98 + abs(tram - 127) * 2 * 0.02; // cm

        // Bit 11
        relay = autoSteerUdpData[11];

        // Bit 12
        relayHi = autoSteerUdpData[12];

        //----------------------------------------------------------------------------
        // Serial Send to agopenGPS

        int16_t sa = (int16_t)(steerAngleActual * 100);

        PGN_253[5] = (uint8_t)sa;
        PGN_253[6] = sa >> 8;

        // heading
        PGN_253[7] = (uint8_t)9999;
        PGN_253[8] = 9999 >> 8;

        // roll
        PGN_253[9] = (uint8_t)8888;
        PGN_253[10] = 8888 >> 8;

        PGN_253[11] = switchByte;
        PGN_253[12] = (uint8_t)pwmDisplay;

        // checksum
        int16_t CK_A = 0;
        for (uint8_t i = 2; i < PGN_253_Size; i++)
          CK_A = (CK_A + PGN_253[i]);

        PGN_253[PGN_253_Size] = CK_A;

        // off to AOG
        SendUdp(PGN_253, sizeof(PGN_253), Eth_ipDestination, portDestination);

        // Steer Data 2 -------------------------------------------------
        if (steerConfig.PressureSensor || steerConfig.CurrentSensor)
        {
          if (aog2Count++ > 2)
          {
            // Send fromAutosteer2
            PGN_250[5] = (byte)sensorReading;

            // add the checksum for AOG2
            CK_A = 0;

            for (uint8_t i = 2; i < PGN_250_Size; i++)
            {
              CK_A = (CK_A + PGN_250[i]);
            }

            PGN_250[PGN_250_Size] = CK_A;

            // off to AOG
            SendUdp(PGN_250, sizeof(PGN_250), Eth_ipDestination, portDestination);
            aog2Count = 0;
          }
        }

        //--------------------------------------------------------------------------
      }

      // steer settings
      else if (autoSteerUdpData[3] == 0xFC && Autosteer_running) // 252
      {
        // PID values
        steerSettings.Kp = ((float)autoSteerUdpData[5]); // read Kp from AgOpenGPS

        steerSettings.highPWM = autoSteerUdpData[6]; // read high pwm

        steerSettings.lowPWM = (float)autoSteerUdpData[7]; // read lowPWM from AgOpenGPS

        steerSettings.minPWM = autoSteerUdpData[8]; // read the minimum amount of PWM for instant on

        float temp = (float)steerSettings.minPWM * 1.2;
        steerSettings.lowPWM = (byte)temp;

        if (steerConfig.IsDanfoss)
        {
          steerSettings.keyaSteerSensorCounts = autoSteerUdpData[9]; // sent as setting displayed in AOG

          int16_t temp = 0;
          temp = (autoSteerUdpData[10]); // read was zero offset Lo

          temp |= (autoSteerUdpData[11] << 8); // read was zero offset Hi

          steerSettings.keyaDirOffset = abs(temp) * 10;

          steerSettings.keyaAckermanFix = autoSteerUdpData[12];

          debugPrint("Setting for Keya!\ndirOffset: ");
          debugPrintln(steerSettings.keyaDirOffset);
        }
        else
        {
          steerSettings.steerSensorCounts = autoSteerUdpData[9]; // sent as setting displayed in AOG

          steerSettings.wasOffset = (autoSteerUdpData[10]); // read was zero offset Lo

          steerSettings.wasOffset |= (autoSteerUdpData[11] << 8); // read was zero offset Hi

          steerSettings.AckermanFix = (float)autoSteerUdpData[12] * 0.01;
        }

        // crc
        // autoSteerUdpData[13];

        // store in EEPROM
        EEPROM.put(10, steerSettings);

        // Re-Init steer settings
        steerSettingsInit();
      }

      else if (autoSteerUdpData[3] == 0xFB) // 251 FB - SteerConfig
      {
        uint8_t sett = autoSteerUdpData[5]; // setting0

        if (bitRead(sett, 0))
          steerConfig.InvertWAS = 1;
        else
          steerConfig.InvertWAS = 0;
        if (bitRead(sett, 1))
          steerConfig.IsRelayActiveHigh = 1;
        else
          steerConfig.IsRelayActiveHigh = 0;
        if (bitRead(sett, 2))
          steerConfig.MotorDriveDirection = 1;
        else
          steerConfig.MotorDriveDirection = 0;
        if (bitRead(sett, 3))
          steerConfig.SingleInputWAS = 1;
        else
          steerConfig.SingleInputWAS = 0;
        if (bitRead(sett, 4))
          steerConfig.CytronDriver = 1;
        else
          steerConfig.CytronDriver = 0;
        if (bitRead(sett, 5))
          steerConfig.SteerSwitch = 1;
        else
          steerConfig.SteerSwitch = 0;
        if (bitRead(sett, 6))
          steerConfig.SteerButton = 1;
        else
          steerConfig.SteerButton = 0;
        if (bitRead(sett, 7))
          steerConfig.ShaftEncoder = 1;
        else
          steerConfig.ShaftEncoder = 0;

        steerConfig.PulseCountMax = autoSteerUdpData[6];

        // was speed
        // autoSteerUdpData[7];

        sett = autoSteerUdpData[8]; // setting1 - Danfoss valve etc

        if (bitRead(sett, 0))
          steerConfig.IsDanfoss = 1;
        else
          steerConfig.IsDanfoss = 0;
        if (bitRead(sett, 1))
          steerConfig.PressureSensor = 1;
        else
          steerConfig.PressureSensor = 0;
        if (bitRead(sett, 2))
          steerConfig.CurrentSensor = 1;
        else
          steerConfig.CurrentSensor = 0;
        if (bitRead(sett, 3))
          steerConfig.IsUseY_Axis = 1;
        else
          steerConfig.IsUseY_Axis = 0;

        // crc
        // autoSteerUdpData[13];

        EEPROM.put(40, steerConfig);

        // Re-Init
        steerConfigInit();

      } // end FB
      else if (autoSteerUdpData[3] == 200) // Hello from AgIO
      {
        Autosteer_running = true;
        int16_t sa = (int16_t)(steerAngleActual * 100);

        helloFromAutoSteer[5] = (uint8_t)sa;
        helloFromAutoSteer[6] = sa >> 8;

        helloFromAutoSteer[7] = (uint8_t)helloSteerPosition;
        helloFromAutoSteer[8] = helloSteerPosition >> 8;
        helloFromAutoSteer[9] = switchByte;

        SendUdp(helloFromAutoSteer, sizeof(helloFromAutoSteer), Eth_ipDestination, portDestination);
        if (useBNO08x)
        {
          SendUdp(helloFromIMU, sizeof(helloFromIMU), Eth_ipDestination, portDestination);
        }
      }

      else if (autoSteerUdpData[3] == 201)
      {
        // make really sure this is the subnet pgn
        if (autoSteerUdpData[4] == 5 && autoSteerUdpData[5] == 201 && autoSteerUdpData[6] == 201)
        {
          networkAddress.ipOne = autoSteerUdpData[7];
          networkAddress.ipTwo = autoSteerUdpData[8];
          networkAddress.ipThree = autoSteerUdpData[9];

          // save in EEPROM and restart
          EEPROM.put(60, networkAddress);
          SCB_AIRCR = 0x05FA0004; // Teensy Reset
        }
      } // end 201

      // whoami
      else if (autoSteerUdpData[3] == 202)
      {
        // make really sure this is the reply pgn
        if (autoSteerUdpData[4] == 3 && autoSteerUdpData[5] == 202 && autoSteerUdpData[6] == 202)
        {
          IPAddress rem_ip = Eth_udpAutoSteer.remoteIP();

          // hello from AgIO
          uint8_t scanReply[] = {128, 129, Eth_myip[3], 203, 7,
                                 Eth_myip[0], Eth_myip[1], Eth_myip[2], Eth_myip[3],
                                 rem_ip[0], rem_ip[1], rem_ip[2], 23};

          // checksum
          int16_t CK_A = 0;
          for (uint8_t i = 2; i < sizeof(scanReply) - 1; i++)
          {
            CK_A = (CK_A + scanReply[i]);
          }
          scanReply[sizeof(scanReply) - 1] = CK_A;

          static uint8_t ipDest[] = {255, 255, 255, 255};
          uint16_t portDest = 9999; // AOG port that listens

          // off to AOG
          SendUdp(scanReply, sizeof(scanReply), ipDest, portDest);
        }
      }
      else if (autoSteerUdpData[3] == 238) // machine config 0xEE
      {
        debugPrintln("0xEE - Machine Config");

        // analogWork.thresh = autoSteerUdpData[9];
        // analogWork.hyst = autoSteerUdpData[10];
        // pressSensor.hiTriggerLevel = autoSteerUdpData[11];
        // pressSensor.loTriggerLevel = autoSteerUdpData[12];
        // EEPROM.put(70, analogWork);

        // Serial << "user1: " << analogWork.thresh << " user2: " << analogWork.hyst << " user3: " << autoSteerUdpData[11] << " user4: " << autoSteerUdpData[12] << "\r\n";
      }
      else if (autoSteerUdpData[3] == 105) // calibration config 0x69
      {
        debugPrintln("0x69 - calibration config");

        sscanf(reinterpret_cast<const char *>(&autoSteerUdpData[4]), "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
               &calibrationData.wheelBase,
               &calibrationData.IMUtoANTx,
               &calibrationData.IMUtoANTy,
               &calibrationData.IMUtoANTz,
               &calibrationData.INSx,
               &calibrationData.INSy,
               &calibrationData.INSz,
               &calibrationData.INSanglex,
               &calibrationData.INSangley,
               &calibrationData.INSanglez);
        calibrationData.configFlag += 0.01;
        if (calibrationData.configFlag > 2)
          calibrationData.configFlag -= 2;
        printCalibrationData();
        configureUM981();

        if (steerConfig.CytronDriver)
        { // default tractor
          EEPROM.put(100, calibrationData);
        }
        else
        {
          EEPROM.put(150, calibrationData); // second tractor
        }
      }
    } // end if 80 81 7F
  }
}
#endif

#ifdef ARDUINO_TEENSY41
void SendUdp(uint8_t *data, uint8_t datalen, IPAddress dip, uint16_t dport)
{
  Eth_udpAutoSteer.beginPacket(dip, dport);
  Eth_udpAutoSteer.write(data, datalen);
  Eth_udpAutoSteer.endPacket();
}
#endif

void udpNtrip()
{
#ifdef ARDUINO_TEENSY41
  // When ethernet is not running, return directly. parsePacket() will block when we don't
  if (!Ethernet_running)
  {
    return;
  }

  unsigned int packetLength = Eth_udpNtrip.parsePacket();

  if (packetLength > 0)
  {
    Eth_udpNtrip.read(Eth_NTRIP_packetBuffer, packetLength);
    SerialGPS.write(Eth_NTRIP_packetBuffer, packetLength);
  }
#endif
}
