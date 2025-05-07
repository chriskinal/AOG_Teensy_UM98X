
void passthroughSerial()
{
  char incoming = SerialGPS.read();
  // debugPrintln(incoming);
  switch (incoming)
  {
  case '$':
    msgBuf[msgBufLen] = incoming;
    msgBufLen++;
    gotDollar = true;
    break;
  case '\r':
    msgBuf[msgBufLen] = incoming;
    msgBufLen++;
    gotCR = true;
    gotDollar = false;
    break;
  case '\n':
    msgBuf[msgBufLen] = incoming;
    msgBufLen++;
    gotLF = true;
    gotDollar = false;
    break;
  default:
    if (gotDollar)
    {
      msgBuf[msgBufLen] = incoming;
      msgBufLen++;
    }
    break;
  }
  if (gotCR && gotLF)
  {
    // debugPrint(msgBuf);
    // debugPrintln(msgBufLen);
    if (sendUSB)
    {
      Serial.write(msgBuf);
    } // Send USB GPS data if enabled in user settings
    if (Ethernet_running)
    {
      Eth_udpPAOGI.beginPacket(Eth_ipDestination, portDestination);
      Eth_udpPAOGI.write(msgBuf, msgBufLen);
      Eth_udpPAOGI.endPacket();
    }
    gotCR = false;
    gotLF = false;
    gotDollar = false;
    memset(msgBuf, 0, 254);
    msgBufLen = 0;
    if (blink)
    {
      digitalWrite(GGAReceivedLED, HIGH);
    }
    else
    {
      digitalWrite(GGAReceivedLED, LOW);
    }

    blink = !blink;
  }
}

void LedSetup()
{
  pinMode(GGAReceivedLED, OUTPUT);
  pinMode(DEBUG_LED, OUTPUT);
#ifdef PCB_VERSION_0_1
  pinMode(AUTOSTEER_ACTIVE_LED, OUTPUT);
#endif
  pinMode(CAN_ACTIVE_LED, OUTPUT);

  digitalWrite(GGAReceivedLED, 1);
  delay(300);
  digitalWrite(GGAReceivedLED, 0);
  delay(300);
#ifdef PCB_VERSION_0_1
  digitalWrite(AUTOSTEER_ACTIVE_LED, 1);
  delay(300);
  digitalWrite(AUTOSTEER_ACTIVE_LED, 0);
  delay(300);
#endif
  digitalWrite(CAN_ACTIVE_LED, 1);
  delay(300);
  digitalWrite(CAN_ACTIVE_LED, 0);
  delay(300);
  digitalWrite(GGAReceivedLED, 1);
#ifdef PCB_VERSION_0_1
  digitalWrite(AUTOSTEER_ACTIVE_LED, 1);
#endif
  digitalWrite(CAN_ACTIVE_LED, 1);
  delay(800);
  digitalWrite(GGAReceivedLED, 0);
#ifdef PCB_VERSION_0_1
  digitalWrite(AUTOSTEER_ACTIVE_LED, 0);
#endif
  digitalWrite(CAN_ACTIVE_LED, 0);
}

void debugLoop()
{
  int8_t read = digitalRead(debug_pin);
  // debug button handler
  if (read == LOW && debugButton == 1)
  { // just pressed
    if (debugState == STATE_INFO)
      debugState = SETUP;
    else
      debugState = (debugState + 1);

    debugPrintln("\n\n");
    switch (debugState)
    {
    case SETUP:
      debugPrintln("DEBUG: SETUP");
      break;
    case EXPERIMENT:
      debugPrintln("DEBUG: EXPERIMENT");
      break;
    case ROLL:
      debugPrintln("DEBUG: ROLL");
      break;
    case WAS:
      debugPrintln("DEBUG: WAS");
      break;
    case GPS:
      debugPrintln("DEBUG: GPS");
      break;
    case KEYA:
      debugPrintln("DEBUG: KEYA");
      break;
    case SWITCH:
      debugPrintln("DEBUG: SWITCH");
      break;
    case UDP:
      debugPrintln("DEBUG: UDP");
      break;
    case STATE_INFO:
      debugPrintln("DEBUG: STATE_INFO");
      debugTime = systick_millis_count - 5000;
      break;
    }
    debugPrintln("\n");

    if (debugState == SETUP)
      digitalWrite(DEBUG_LED, LOW);
    else
      digitalWrite(DEBUG_LED, HIGH);

    delay(200);
  }
  debugButton = read;

  if ((debugState == STATE_INFO || send_INFO) && systick_millis_count - debugTime > 5000)
  {
    getKeyaInfo();
    debugPrintln();
    if (settings.usingWT61)
    {
      debugPrint("WT61 temperature: ");
      debugPrint(tempWT);
      debugPrintln(" °C");
    }
    debugPrint("Internal Temperature: ");
    debugPrint(InternalTemperature.readTemperatureC(), 1);
    debugPrintln(" °C");
    debugTime = systick_millis_count;

    printCalibrationData();
    printSettings();
  }
}

void printCalibrationData()
{
  debugPrintln("Current Calibration Data:");
  debugPrint("WheelBase: ");
  debugPrintln(calibrationData.wheelBase);
  debugPrint("IMUtoANTx: ");
  debugPrintln(calibrationData.IMUtoANTx);
  debugPrint("IMUtoANTy: ");
  debugPrintln(calibrationData.IMUtoANTy);
  debugPrint("IMUtoANTz: ");
  debugPrintln(calibrationData.IMUtoANTz);
  debugPrint("INSx: ");
  debugPrintln(calibrationData.INSx);
  debugPrint("INSy: ");
  debugPrintln(calibrationData.INSy);
  debugPrint("INSz: ");
  debugPrintln(calibrationData.INSz);
  debugPrint("INSanglex: ");
  debugPrintln(calibrationData.INSanglex);
  debugPrint("INSangley: ");
  debugPrintln(calibrationData.INSangley);
  debugPrint("INSanglez: ");
  debugPrintln(calibrationData.INSanglez);
  debugPrint("configFlag: ");
  debugPrintln(calibrationData.configFlag);
  debugPrintln("-----------------------------");
}

void printSettings()
{
  debugPrintln("Current Settings:");
  debugPrint("usingWT61: ");
  debugPrintln(settings.usingWT61);
  debugPrint("using2serialGPS: ");
  debugPrintln(settings.using2serialGPS);
  debugPrint("useKalmanForSensor: ");
  debugPrintln(settings.useKalmanForSensor);
  debugPrint("intervalINS: ");
  debugPrintln(settings.intervalINS);
  debugPrint("secondsVarianceBuffer: ");
  debugPrintln(settings.secondsVarianceBuffer);
  debugPrint("minSpeedKalman: ");
  debugPrintln(settings.minSpeedKalman);
  debugPrint("kalmanR: ");
  debugPrintln(settings.kalmanR);
  debugPrint("kalmanQ: ");
  debugPrintln(settings.kalmanQ, 6);
  debugPrintln("-----------------------------");
}

void sendPlotData()
{
  // Create CSV string with timestamp
  String plotString = String(millis()) + ",";
  plotString += String(steerAngleSens, 3) + ",";
  plotString += String(insWheelAngle, 3) + ",";
  plotString += String(keyaEncoder, 3) + ",";
  plotString += String(KalmanWheelAngle, 3) + ",";
  plotString += String(angleVariance, 5);

  // Send via UDP
  Eth_udpDebug.beginPacket(Eth_ipDestination, portPlot);
  Eth_udpDebug.write(plotString.c_str());
  Eth_udpDebug.endPacket();
}