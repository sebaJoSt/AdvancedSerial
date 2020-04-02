/*
        File: AdvancedSerial.cpp
        Author: Sebastian Strobl <sebastian.strobl@gmx.de>
        Description: An Advanced Serial Interface for Interfacing with Arduino
*/

#include "AdvancedSerial.h"
#include "Arduino.h"

// static initializer for the static member.
AdvancedSerial* AdvancedSerial::pSingletonInstance = 0;

AdvancedSerial::AdvancedSerial() {

}

AdvancedSerial::~AdvancedSerial() {
  delete Signals;
}

void AdvancedSerial::begin(HardwareSerial *Ref, unsigned int Size)
{
  maxSignalCount = Size;
  SerialRef = Ref;
  Signals = new LoggedSignal[Size];
}

void AdvancedSerial::begin(HardwareSerial *Ref, unsigned int Size, uint32_t WireClockFrequency, bool isMaster, byte SlaveID)
{

  if (isMaster)
  {
    LOGGING_MODE = 1;
    Wire.setClock(WireClockFrequency);
    Wire.begin();
  } else
  {
    LOGGING_MODE = 2;
    SLAVE_ID = SlaveID;
    if (SLAVE_ID > 127) SLAVE_ID = 127;
    String s = "S";
    SlaveSymbolPrefix = s + SLAVE_ID + "_";

    AdvancedSerial::pSingletonInstance = this; // Assign the static singleton used in the static handlers.
    Wire.onReceive(AdvancedSerial::OnSendHandler);
    Wire.onRequest(AdvancedSerial::OnReceiveHandler);
    Wire.begin(SlaveID);
  }

  begin(Ref, Size);
}


void AdvancedSerial::setCommandCallback(void (*readCallback) (char * command, int * parameter, char * string01)) {
  _readCallback = readCallback;
}

void AdvancedSerial::deleteSignals() {
  signalCount = 0;
}

void AdvancedSerial::addSignal(String Name, bool * value) {
  Signals[signalCount].Name = Name;
  if (LOGGING_MODE == 2) {
    Signals[signalCount].Name = SlaveSymbolPrefix + Name;
  }
  Signals[signalCount].Type = asi_bool;
  Signals[signalCount].addr = value;
  signalCount++;
}

void AdvancedSerial::addSignal(String Name, double * value) {
  Signals[signalCount].Name = Name;
  if (LOGGING_MODE == 2) {
    Signals[signalCount].Name = SlaveSymbolPrefix + Name;
  }
  Signals[signalCount].Type = asi_double;
  Signals[signalCount].addr = value;
  signalCount++;
}

void AdvancedSerial::addSignal(String Name, float * value) {
  Signals[signalCount].Name = Name;
  if (LOGGING_MODE == 2) {
    Signals[signalCount].Name = SlaveSymbolPrefix + Name;
  }
  Signals[signalCount].Type = asi_float;
  Signals[signalCount].addr = value;
  signalCount++;
}

void AdvancedSerial::addSignal(String Name, unsigned long * value) {
  Signals[signalCount].Name = Name;
  if (LOGGING_MODE == 2) {
    Signals[signalCount].Name = SlaveSymbolPrefix + Name;
  }
  Signals[signalCount].Type = asi_ulong;
  Signals[signalCount].addr = value;
  signalCount++;
}

void AdvancedSerial::addSignal(String Name, int * value) {
  Signals[signalCount].Name = Name;
  if (LOGGING_MODE == 2) {
    Signals[signalCount].Name = SlaveSymbolPrefix + Name;
  }
  Signals[signalCount].Type = asi_int;
  Signals[signalCount].addr = value;
  signalCount++;
}

void AdvancedSerial::addSignal(String Name, byte * value) {
  Signals[signalCount].Name = Name;
  if (LOGGING_MODE == 2) {
    Signals[signalCount].Name = SlaveSymbolPrefix + Name;
  }
  Signals[signalCount].Type = asi_byte;
  Signals[signalCount].addr = value;
  signalCount++;
}


void AdvancedSerial::Read() {

  if (recvWithStartEndMarkers() == true) {
    parseData();
    Serial.print(F("<"));
    Serial.print(receivedChars);
    Serial.println(F(">"));

    if (strcmp(COMMAND, "LOGGING_GETSIGNALLIST") == 0)
    {
      unsigned long msg_id = ((unsigned long)PARAMETER[3] << 24) | ((unsigned long)PARAMETER[2] << 16)
                             | ((unsigned long)PARAMETER[1] << 8) | ((unsigned long)PARAMETER[0]);

      if (LOGGING_MODE == 0 || LOGGING_MODE == 2) {
        this->TransmitSymbols(msg_id, true);
      } else if (LOGGING_MODE == 1) {
        this->WireTransmitSymbols(msg_id, true);
      }


    } else if (strcmp(COMMAND, "LOGGING_GETDATA") == 0)
    {
      unsigned long msg_id = ((unsigned long)PARAMETER[3] << 24) | ((unsigned long)PARAMETER[2] << 16)
                             | ((unsigned long)PARAMETER[1] << 8) | ((unsigned long)PARAMETER[0]);

      if (LOGGING_MODE == 0 || LOGGING_MODE == 2) {
        this->TransmitData(msg_id, true);
      } else if (LOGGING_MODE == 1) {
        this->WireTransmitData(msg_id, true);
      }

    }  else if (strcmp(COMMAND, "LOGGING_ACTIVATE") == 0)
    {
      unsigned long parameter = PARAMETER[0];
      if (parameter > 32767) parameter = 32767;
      unsigned long unit_multiplicator = 1000;
      unsigned long loggingInterval_ms = parameter * unit_multiplicator;

      setInitialIntervalSettings(true, loggingInterval_ms);

    }  else if (strcmp(COMMAND, "LOGGING_DEACTIVATE") == 0)
    {
      setInitialIntervalSettings(false, LoggingInterval_ms);
    }

    _readCallback(COMMAND, PARAMETER, STRING_01);
  }
}

bool AdvancedSerial::recvWithStartEndMarkers() {
  bool newData = false;
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (SerialRef->available() > 0 && newData == false) {
    rc = SerialRef->read();
    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }
    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }

  return newData;
}

void AdvancedSerial::parseData() {      // split the data into its parts
  //strcpy(tempChars, receivedChars);
  // this temporary copy is necessary to protect the original data
  //   because strtok() used in parseData() replaces the commas with \0
  char tempChars[sizeof(receivedChars)];
  strcpy(tempChars, receivedChars);
  char * strtokIndx;
  strtokIndx = strtok(tempChars, ", ");
  strcpy(COMMAND, strtokIndx);

  strtokIndx = strtok(NULL, ", ");
  //PARAMETER 1 is stored in PARAMETER_01 & STRING_01 (if PARAMETER 1 is a string)
  strncpy(STRING_01, strtokIndx, 15); //Only copy first 15 chars
  STRING_01[15] = '\0';              //16th Char = Null Terminator
  PARAMETER[0] = atoi(strtokIndx);
  strtokIndx = strtok(NULL, ", ");
  PARAMETER[1] = atoi(strtokIndx);
  strtokIndx = strtok(NULL, ", ");
  PARAMETER[2] = atoi(strtokIndx);
  strtokIndx = strtok(NULL, ", ");
  PARAMETER[3] = atoi(strtokIndx);
  strtokIndx = strtok(NULL, ", ");
  PARAMETER[4] = atoi(strtokIndx);
  strtokIndx = strtok(NULL, ", ");
  PARAMETER[5] = atoi(strtokIndx);
  strtokIndx = strtok(NULL, ", ");
  PARAMETER[6] = atoi(strtokIndx);
  strtokIndx = strtok(NULL, ", ");
  PARAMETER[7] = atoi(strtokIndx);
  strtokIndx = strtok(NULL, ", ");
  PARAMETER[8] = atoi(strtokIndx);
  strtokIndx = strtok(NULL, ", ");
  PARAMETER[9] = atoi(strtokIndx);
}

void AdvancedSerial::setInitialIntervalSettings(bool loggingActivated, unsigned long loggingInterval_ms) {
  LoggingActivated = loggingActivated;

  if (LoggingActivated)
  {
	if (loggingInterval_ms == 0)
	{
    LoggingTimeSet_ms = 100;
    LoggingInterval_ms = 100;
	}
	else 
	{
    LoggingTimeSet_ms = loggingInterval_ms;
    LoggingInterval_ms = loggingInterval_ms;
    }
	LoggingFirstTime = true;
  }
}

void AdvancedSerial::TransmitSymbols(unsigned long msg_id, bool send_eol) {
  SerialRef->write("#ASI:");
  byte msg_key = 0xB0;
  SerialRef->write(msg_key);
  SerialRef->write(":");
  ulngCvt.val = msg_id;
  SerialRef->write(ulngCvt.bval, 4);
  SerialRef->write(":");

  for (int i = 0; i < signalCount; i++) {
    intCvt.val = i;
    SerialRef->write(intCvt.bval, 2);
    LoggedSignal sym = Signals[i];
    SerialRef->print(sym.Name);
    SerialRef->write('\0');

    switch (sym.Type) {
      case (asi_bool): {
          SerialRef->write(0x0);
          break;
        }
      case (asi_byte): {
          SerialRef->write(0x1);
          break;
        }
      case (asi_short): {
          SerialRef->write(0x2);
          break;
        }
      case (asi_int): {
          SerialRef->write(0x3);
          break;
        }
      case (asi_uint): {
          SerialRef->write(0x4);
          break;
        }
      case (asi_long): {
          SerialRef->write(0x5);
          break;
        }
      case (asi_ulong): {
          SerialRef->write(0x6);
          break;
        }
      case (asi_float): {
          SerialRef->write(0x7);
          break;
        }
      case (asi_double): {
          SerialRef->write(0x8);
          break;
        }
    }
  }
  if (send_eol) {
    SerialRef->write("ENDOFASI");
    SerialRef->write("\r\n");
  }

  SerialRef->flush();
}

void AdvancedSerial::TransmitData(unsigned long msg_id, bool send_eol) {

  SerialRef->write("#ASI:");
  byte msg_key = 0xB1;
  SerialRef->write(msg_key);
  SerialRef->write(":");
  ulngCvt.val = msg_id;
  SerialRef->write(ulngCvt.bval, 4);
  SerialRef->write(":");

  int bytecount = 22 + signalCount * 2;
  for (int i = 0; i < signalCount; i++) {
    intCvt.val = i;
    SerialRef->write(intCvt.bval, 2);
    LoggedSignal sym = Signals[i];
    switch (sym.Type) {
      case (asi_bool): {
          boolCvt.val = *((bool*)sym.addr);
          SerialRef->write(boolCvt.bval, 1);
          bytecount += 1;
        } break;
      case (asi_byte): {
          SerialRef->write(*((byte*)sym.addr));
          bytecount += 1;
        } break;
      case (asi_short): {
          shortCvt.val = *((short*)sym.addr);
          SerialRef->write(shortCvt.bval, 2);
          bytecount += 2;
        } break;
      case (asi_int): {
          intCvt.val = *((int*)sym.addr);
          SerialRef->write(intCvt.bval, 2);
          bytecount += 2;
        } break;
      case (asi_uint): {
          uintCvt.val = *((unsigned int*)sym.addr);
          SerialRef->write(uintCvt.bval, 2);
          bytecount += 2;
        } break;
      case (asi_long): {
          lngCvt.val = *((long*)sym.addr);
          SerialRef->write(lngCvt.bval, 4);
          bytecount += 4;
        } break;
      case (asi_ulong): {
          ulngCvt.val = *((unsigned long*)sym.addr);
          SerialRef->write(ulngCvt.bval, 4);
          bytecount += 4;
        } break;
      case (asi_float): {
          fltCvt.val = *((float*)sym.addr);
          SerialRef->write(fltCvt.bval, 4);
          bytecount += 4;
        } break;
      case (asi_double): {
          dblCvt.val = *((double*)sym.addr);
          //Serial.print("Double!");
          SerialRef->write(dblCvt.bval, 8);
          bytecount += 8;
        } break;
    }
  }

  if (send_eol) {
    SerialRef->write("ENDOFASI");
    SerialRef->write("\r\n");
  }

  SerialRef->flush();
}


void AdvancedSerial::WireTransmitSymbols(unsigned long msg_id, bool send_eol) {

  this->TransmitSymbols(msg_id, false);

  int signalcount = 0;

  for (int slaveindex = 0; slaveindex <= 127; slaveindex++) { //Cycle through slaves
    byte transmissionIsSuccess = false;

    for (byte retries = 0; retries < 4; retries++) {
      Wire.beginTransmission(slaveindex);
      Wire.write(0);
      transmissionIsSuccess = Wire.endTransmission();
      if (transmissionIsSuccess == 0) break; //0: success
    }

    if (transmissionIsSuccess == 0) {

      SLAVE_FOUND[slaveindex] = false;
      bool eolist_found = false;

      for (int i = 0; i < 1000; i++) {
        byte receivedBytes = Wire.requestFrom(slaveindex, 32);    // request 32 bytes from slave device
        if (receivedBytes < 2) continue; //try again

        bool eosignal_found = false;
        if (i == 0) {
          //Expecting response 0xAA from slave -> Slave found
          char c = Wire.read(); // receive a byte as character

          if (c == char(0xAA)) {
            SLAVE_FOUND[slaveindex] = true;

          } else {
            SLAVE_FOUND[slaveindex] = false;
            break; //exit loop
          }
        }

        if (SLAVE_FOUND[slaveindex]) {
          //Signal Key
          Serial.write(lowByte(signalCount + signalcount));
          Serial.write(highByte(signalCount + signalcount));

          int charsToRead = 32;
          if (i == 0) charsToRead = 31;
          for (int symbolchar = 0; symbolchar <= charsToRead - 1; symbolchar++) { // slave may send less than requested
            //Signal Name + \0 + Signal Type
            char c = Wire.read(); // receive a byte as character
            if (c == char(0x0D)) {
              eosignal_found = true; //"\r"
              signalcount += 1;
            }
            if (c == char(0x0A)) eolist_found = true; //"\n"
            if (eosignal_found != true && eolist_found != true) Serial.print(c);
          }
          if (eolist_found) break;
        }
      }
    }
  }
  if (send_eol) {
    SerialRef->write("ENDOFASI");
    SerialRef->write("\r\n");
  }

  SerialRef->flush();
}

void AdvancedSerial::WireTransmitData(unsigned long msg_id, bool send_eol) {

  this->TransmitData(msg_id, false);

  int signalcount = 0;

  for (int slaveindex = 0; slaveindex <= 127; slaveindex++) { //Cycle through slaves
    if (SLAVE_FOUND[slaveindex]) {
      byte transmissionIsSuccess = false;

      for (byte retries = 0; retries < 4; retries++) {
        Wire.beginTransmission(slaveindex);
        Wire.write(1);
        transmissionIsSuccess = Wire.endTransmission();
        if (transmissionIsSuccess == 0) break; //0: success
      }

      if (transmissionIsSuccess == 0) {
        bool eolist_found = false;
        for (int slaveASIsignal = 0; slaveASIsignal < 1000; slaveASIsignal++) {
          byte receivedBytes = Wire.requestFrom(slaveindex, 32);    // request 32 bytes from slave device
          if (receivedBytes < 2) continue; //try again

          bool eosignal_found = false;

          //Signal Key
          Serial.write(lowByte(signalCount + signalcount));
          Serial.write(highByte(signalCount + signalcount));

          signalcount += 1;
          for (int symbolchar = 0; symbolchar <= 31; symbolchar++) { // slave may send less than requested
            char bytecount = Wire.read(); //first receive number of bytes to expect
            char c;
            for (int i = 0; i < bytecount; i++) {
              c = Wire.read(); //then read the data bytes
              Serial.print(c);
            }
            char c_before = char(0x7F);
            byte endoflist_count = 0;
            //for (int i = 0; i < 32; i++) {//empty buffer
            while (Wire.available()) { //empty buffer
              c = Wire.read();
              if (c == char(0x7F) && c_before == char(0x7F)) endoflist_count += 1;
              c_before = c;
            }
            if (endoflist_count == 8) eolist_found = true; //8 times 0x7F -> All Wire data received from slave
          }
          if (eolist_found) break;
        }
      }

    }
  }

  if (send_eol)
  {
    SerialRef->write("ENDOFASI");
    SerialRef->write("\r\n");
  }

  SerialRef->flush();
}

void AdvancedSerial::TransmitDataInterval(unsigned long msg_id, bool send_eol) {

  if (LoggingFirstTime == true) LoggingFirstTimeDone_ms = millis();
  unsigned long loggingElapsedTime_ms = (millis() - LoggingFirstTimeDone_ms);

  if (((loggingElapsedTime_ms >= LoggingTimeSet_ms) || LoggingFirstTime == true) && LoggingActivated == true) {

    if (LoggingFirstTime == false) LoggingTimeSet_ms += LoggingInterval_ms;
    LoggingFirstTime = false;

    if (LOGGING_MODE == 0 || LOGGING_MODE == 2)
    {
      this->TransmitData(msg_id, true);
    }
    else if (LOGGING_MODE == 1)
    {
      this->WireTransmitData(msg_id, true);
    }
  }
}

void AdvancedSerial::WireSlaveTransmitSingleSymbol() {

  if (wireSignalCount == 0) Wire.write(0xAA);

  LoggedSignal sym = Signals[wireSignalCount];

  char little_s_string[32] = "";
  sym.Name.toCharArray(little_s_string, 32);
  Wire.write(little_s_string);

  Wire.write('\0');

  switch (sym.Type) {
    case (asi_bool): {
        Wire.write(0x0);
        break;
      }
    case (asi_byte): {
        Wire.write(0x1);
        break;
      }
    case (asi_short): {
        Wire.write(0x2);
        break;
      }
    case (asi_int): {
        Wire.write(0x3);
        break;
      }
    case (asi_uint): {
        Wire.write(0x4);
        break;
      }
    case (asi_long): {
        Wire.write(0x5);
        break;
      }
    case (asi_ulong): {
        Wire.write(0x6);
        break;
      }
    case (asi_float): {
        Wire.write(0x7);
        break;
      }
    case (asi_double): {
        Wire.write(0x8);
        break;
      }
  }

  Wire.write(0x0D);

  wireSignalCount += 1;
  if (wireSignalCount >= signalCount) {
    wireSignalCount = 0;
    Wire.write(0x0A);
  }
}

void AdvancedSerial::WireSlaveTransmitSingleDataPoint() {

  LoggedSignal sym = Signals[wireSignalCount];

  switch (sym.Type) {
    case (asi_bool): {
        boolCvt.val = *((bool*)sym.addr);
        Wire.write(1);  //first byte count
        Wire.write(boolCvt.bval, 1); //then data
      } break;
    case (asi_byte): {
        Wire.write(1);
        Wire.write(*((byte*)sym.addr));
      } break;
    case (asi_short): {
        shortCvt.val = *((short*)sym.addr);
        Wire.write(2);
        Wire.write(shortCvt.bval, 2);
      } break;
    case (asi_int): {
        intCvt.val = *((int*)sym.addr);
        Wire.write(2);
        Wire.write(intCvt.bval, 2);
      } break;
    case (asi_uint): {
        uintCvt.val = *((unsigned int*)sym.addr);
        Wire.write(2);
        Wire.write(uintCvt.bval, 2);
      } break;
    case (asi_long): {
        lngCvt.val = *((long*)sym.addr);
        Wire.write(4);
        Wire.write(lngCvt.bval, 4);
      } break;
    case (asi_ulong): {
        ulngCvt.val = *((unsigned long*)sym.addr);
        Wire.write(4);
        Wire.write(ulngCvt.bval, 4);
      } break;
    case (asi_float): {
        fltCvt.val = *((float*)sym.addr);
        Wire.write(4);
        Wire.write(fltCvt.bval, 4);
      } break;
    case (asi_double): {
        dblCvt.val = *((double*)sym.addr);
        //Serial.print("Double!");
        Wire.write(8);
        Wire.write(dblCvt.bval, 8);
      } break;
  }


  wireSignalCount += 1;
  if (wireSignalCount >= signalCount) {
    wireSignalCount = 0;
    Wire.write(0x7F);
    Wire.write(0x7F);
    Wire.write(0x7F);
    Wire.write(0x7F);
    Wire.write(0x7F);
    Wire.write(0x7F);
    Wire.write(0x7F);
    Wire.write(0x7F);
  }
}

void AdvancedSerial::WireSlaveReceive() {

  WireMode = Wire.read();
  //WireMode = 0 -> WireSlaveTransmitSingleSymbol()
  //WireMode = 1 -> WireSlaveTransmitSingleDataPoint()

  wireSignalCount = 0;
}

void AdvancedSerial::WireSlaveTransmitToMaster() {

  if (WireMode == 0) this->WireSlaveTransmitSingleSymbol();
  if (WireMode == 1) this->WireSlaveTransmitSingleDataPoint();
}