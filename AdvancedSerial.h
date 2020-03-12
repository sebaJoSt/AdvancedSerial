/*
        File: AdvancedSerial.h
        Author: Nick Dodds <Nick1787@gmail.com>
        Description: An Advanced Serial Interface for Interfacing with Arduino
        Modified by: Sebastian Strobl <sebastian.strobl@gmx.de>
        Changelog:
        2V0 - 2020-02-14 Big rewrite with new Read functions
        1V1 - 2019-01-22 Changed EOL from <CRNL> to ENDOFASI<CRNL> to make protocol more robust
*/


#ifndef ADVANCEDSERIAL_H
#define ADVANCEDSERIAL_H

#include <Wire.h>
#include <Arduino.h>

//MESAGE DECODER
//
//  All Messages are composed of the following structure
//    |--Header------------|-DATA--------------------|-EOT--|
//    #ASI:<MSGKEY>:<MSGID>:..........................ENDOFASI<CRNL>
//
//    DATA:           TYPE:            DESCRIPTION:
//    <MSGKEY>        byte             Message KEY, A unique key for the type of message being sent
//    <MSGID>         uint32/ulong     Message ID,  A unique message ID is which is echo's back to transmitter to indicate a response to a message (0 to 4294967295)
//    <DATA>          (varying)        Message Data, varying data types and length depending on message
//    ENDOFASI <CRNL> char             'ENDOFASI' + Carriage Return + New Line Character signifying the end of a transmission.
//    <SymbolID>      uint             Symbol ID number
//    <SymbolName>    String0          Symbol Name - Null Terminated String
//    <DTYPE>         byte             DataType  0=Boolean, 1=Byte, 2=short, 3=int, 4=unsigned int, 5=long, 6=unsigned long, 7=float, 8=double
//
//
//  -INCOMING-MESSAGES-
//    MSGKEY:   DATA#:    DATA:                           DESCRIPTION:
//     UPDATEEE A0                            Request for available Symbols and Types
//     UPDATEEE A1                            Request for Data as a single message (all symbols).
//
//  -OUTGOING-MESSAGES-
//    MSGKEY:   DATA#:    DATA:                           DESCRIPTION:
//     B0       N         <SymbolID><SymbolName><DTYPE>   Up to N Items. Response to request for available symbols.
//     B1       N         <SymbolID><DATA>                Up to N Items. Response to request for Data.
//



typedef enum dataType { asi_bool, asi_byte, asi_short, asi_long, asi_ushort, asi_ulong, asi_int, asi_uint, asi_float, asi_double};
struct LoggedSignal {
  String Name;
  dataType Type;
  void * addr;
};

class AdvancedSerial {

  //variables
  public:
  private:
    unsigned int maxSignalCount;
    unsigned int signalCount = 0;
    unsigned int wireSignalCount = 0;

    LoggedSignal * Signals;
    String SlaveSymbolPrefix;
    HardwareSerial *SerialRef;
    int PARAMETER[10];
    char COMMAND[64] = {0};
    char STRING_01[16];
    //STRING_01: Max. 15 chars allowed  + Null Terminator '\0' = 16
    //In case more than 15 chars are sent, the rest is cut off in function void parseData()
    const int numChars = 64;
    char receivedChars[64];

    bool loggingActivated = true;
    bool loggingFirstTime = true;
    unsigned long loggingLastTimeDone = 0;
    unsigned long loggingInterval_ms = 1000;
    byte LOGGING_MODE = 0;
    byte SLAVE_ID;
    bool SLAVE_FOUND[128];
    byte WireMode = 0;
    //LOGGING_MODE = 0: SINGLE DEVICE
    //LOGGING_MODE = 1: MASTER
    //LOGGING_MODE = 2: SLAVE
    //MASTER/SLAVE logging mode uses I2C (Wire) interface
    //e.g. Pins 20 (SDA) and 21 (SCL) on Arduino Mega


  //functions
  public:
    AdvancedSerial();
    ~AdvancedSerial();

    void begin(HardwareSerial *Ref, unsigned int Size);
    void begin(HardwareSerial *Ref, unsigned int Size, uint32_t WireClockFrequency, bool isMaster, byte SlaveID);

    void setCommandCallback(void (*readCallback)(char * command, int * parameter, char * string_01));
    void setInitialIntervalSettings(bool loggingactivated, unsigned long logginginterval_ms);
    void addSignal(String Name, bool * value);
	void addSignal(String Name, byte * value);
    void addSignal(String Name, float * value);
    void addSignal(String Name, double * value);
    void addSignal(String Name, unsigned long * value);
    void addSignal(String Name, int * value);
    void deleteSignals();
    void Read();
    
    void TransmitSymbols(unsigned long MessageID, bool send_eol);
    void TransmitData(unsigned long MessageID, bool send_eol);
    void TransmitDataInterval(unsigned long MessageID, bool send_eol);

    void WireTransmitSymbols(unsigned long MessageID, bool send_eol);
    void WireTransmitData(unsigned long MessageID, bool send_eol);

    void WireSlaveTransmitToMaster();
    void WireSlaveReceive();

    void WireSlaveTransmitSingleSymbol();
    void WireSlaveTransmitSingleDataPoint();

  private:
    static AdvancedSerial* pSingletonInstance;

    static void OnReceiveHandler() {
      if (pSingletonInstance)
        pSingletonInstance->WireSlaveTransmitToMaster();
    }

    static void OnSendHandler() {
      if (pSingletonInstance)
        pSingletonInstance->WireSlaveReceive();
    }
	

    void (*_readCallback)(char * command, int * parameter, char * string01);
    bool recvWithStartEndMarkers();
    void parseData();

    union {
      bool val;
      byte bval[1];
    } boolCvt;
	
    union {
      short val;
      byte bval[2];
    } shortCvt;

    union {
      int val;
      byte bval[2];
    } intCvt;

    union {
      unsigned int val;
      byte bval[2];
    } uintCvt;

    union {
      long val;
      byte bval[4];
    } lngCvt;

    union {
      unsigned long val;
      byte bval[4];
    } ulngCvt;

    union {
      float val;
      byte bval[4];
    } fltCvt;

    union {
      double val;
      byte bval[8];
    } dblCvt;

}; //AdvancedSerial



#endif //  ADVANCEDSERIAL_H