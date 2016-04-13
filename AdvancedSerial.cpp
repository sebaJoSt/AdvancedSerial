/*
 *        File: AdvSerialInterface.c
 *      Author: Nick Dodds <Nick1787@gmail.com>
 * Description: An Adnvancer Serial Interface for Interfacing with Arduino
 * ----------------------------------------------------------------
 *    Revision:
 *    02262015 - NRD - Initial Version
 * ----------------------------------------------------------------
 */
 
#include "AdvancedSerial.h"
#include "Arduino.h"s

AdvancedSerial::AdvancedSerial( HardwareSerial *Ref){
  SerialRef = Ref;
}

AdvancedSerial::~AdvancedSerial(){
  for(int i=0; i<symbols.size(); i++){
    LoggedSymbol sym = symbols.get(i);
    symbols.remove(i);
    delete &sym;
  }
}

void AdvancedSerial::addSymbol(String Name, bool * value){
  LoggedSymbol * sym = new LoggedSymbol;
  sym->Name = Name;
  //Name.toCharArray(sym->Name,25);
  sym->Type = asi_bool;
  sym->addr = value;

  symbols.add(*sym);
}

void AdvancedSerial::addSymbol(String Name, double * value){
  LoggedSymbol * sym = new LoggedSymbol;
  sym->Name = Name;
  //Name.toCharArray(sym->Name,25);
  sym->Type = asi_double;
  sym->addr = value;

  symbols.add(*sym);
}

void AdvancedSerial::addSymbol(String Name, float * value){
  LoggedSymbol * sym = new LoggedSymbol;
  sym->Name = Name;
  //Name.toCharArray(sym->Name,25);
  sym->Type = asi_float;
  sym->addr = value;

  symbols.add(*sym);
}

void AdvancedSerial::addSymbol(String Name, unsigned long * value){
  LoggedSymbol * sym = new LoggedSymbol;
  sym->Name = Name;
  //Name.toCharArray(sym->Name,25);
  sym->Type = asi_ulong;
  sym->addr = value;

  symbols.add(*sym);
}

void AdvancedSerial::addSymbol(String Name, int * value){
  LoggedSymbol * sym = new LoggedSymbol;
  sym->Name = Name;
  //Name.toCharArray(sym->Name,24);
  sym->Type = asi_int;
  sym->addr = value;

  symbols.add(*sym);
}


void AdvancedSerial::exec(){
  this->Read();
}

void AdvancedSerial::Read(){
  char buffer[_advancedserial_inbuffer];
  
  while ((SerialRef->available() > 0) && (ReadBuffPos<_advancedserial_inbuffer)){
    buffer[ReadBuffPos] = SerialRef->read();
    //Serial.print("Data!!!");
    if ((buffer[ReadBuffPos] == '\n')&&(buffer[ReadBuffPos-1] == '\r')){
      ProcessInMsg(buffer);
      ReadBuffPos=0;
      break;
    }
    ReadBuffPos = ReadBuffPos+1;
  }

  //Wrap around buffer
  if(ReadBuffPos == _advancedserial_inbuffer){
    ReadBuffPos = 0;
  }
}

void AdvancedSerial::ProcessInMsg(char * buffer){
   byte msg_key = buffer[5];
   unsigned int msg_id = (buffer[10] << 24) | (buffer[9] << 16) | (buffer[8] << 8) | (buffer[7]);
   switch(msg_key){
      case(0xA0):{
        Serial.println("Transmit Symbols!!");
        this->TransmitSymbolList(msg_id);
      }
      case(0xA1):{
        unsigned int symboldid = (buffer[8] << 24) | (buffer[7] << 16) | (buffer[6] << 8) | (buffer[5]);
        this->TransmitSymbolData(msg_id);
      }
   }
}

void AdvancedSerial::TransmitSymbolList(unsigned int msg_id){
  SerialRef->write("#ASI:");
  byte msg_key = 0xB0;
  SerialRef->write(msg_key);
  SerialRef->write(":");
  SerialRef->write(msg_id);
  SerialRef->write(":");
  
  for(int i=0; i<symbols.size(); i++){
    intCvt.val = i;
    SerialRef->write(intCvt.bval,2);
    LoggedSymbol sym = symbols.get(i);
    SerialRef->print(sym.Name);
    
    switch(sym.Type){
      case(asi_bool):{
        SerialRef->write(0x0);
        break;
      }
      case(asi_byte):{
        SerialRef->write(0x1);
        break;
      }
      case(asi_short):{
        SerialRef->write(0x2);
        break;
      }
      case(asi_int):{
        SerialRef->write(0x3);
        break;
      }
      case(asi_uint):{
        SerialRef->write(0x4);
        break;
      }
      case(asi_long):{
        SerialRef->write(0x5);
        break;
      }
      case(asi_ulong):{
        SerialRef->write(0x6);
        break;
      }
      case(asi_float):{
        SerialRef->write(0x7);
        break;
      }
      case(asi_double):{
        SerialRef->write(0x8);
        break;
      }
    }
  }
  SerialRef->write("\r\n");
}

void AdvancedSerial::TransmitSymbolData(unsigned int msg_id){
  SerialRef->write("#ASI:");
  byte msg_key = 0xB1;
  SerialRef->write(msg_key);
  SerialRef->write(":");
  SerialRef->write(msg_id);
  SerialRef->write(":");
  
  for(int i=0; i<symbols.size(); i++){
    intCvt.val = i;
    SerialRef->write(intCvt.bval,2);
    LoggedSymbol sym = symbols.get(i);
    switch(sym.Type){
      case(asi_bool):{
        boolCvt.val = *((bool*)sym.addr);
        SerialRef->write(boolCvt.bval,1);
      } break;
      case(asi_byte):{
        SerialRef->write(*((byte*)sym.addr));
      } break;
      case(asi_short):{
        shortCvt.val = *((short*)sym.addr);
        SerialRef->write(shortCvt.bval,2);
      } break;
      case(asi_int):{
        intCvt.val = *((int*)sym.addr);
        SerialRef->write(intCvt.bval,2);
      } break;
      case(asi_uint):{
        uintCvt.val = *((unsigned int*)sym.addr);
        SerialRef->write(uintCvt.bval,2);
      } break;
      case(asi_long):{
        lngCvt.val = *((long*)sym.addr);
        SerialRef->write(lngCvt.bval,4);
      } break;
      case(asi_ulong):{
        ulngCvt.val = *((unsigned long*)sym.addr);
        SerialRef->write(ulngCvt.bval,4);
      } break;
      case(asi_float):{
        fltCvt.val = *((float*)sym.addr);
        SerialRef->write(fltCvt.bval,4);
      } break;
      case(asi_double):{
        dblCvt.val = *((double*)sym.addr);
        //Serial.print("Double!");
        SerialRef->write(dblCvt.bval,8);
      } break;
    }
  }
  SerialRef->write("\r\n");
}

