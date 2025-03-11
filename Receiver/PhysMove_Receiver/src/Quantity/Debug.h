#pragma once

#if DebugPhysInstrumentHost 
extern uint8 DebugPhysInstrumentHost_Enable;
extern uint8 PhysInstrumentHostTokenSeed;
extern uint8 TracePhysInstrumentHost_Enable;
#define HostDebug(msg) if (DebugPhysInstrumentHost_Enable){Serial.print(msg); delay(1);}
#define HostDebugln(msg) if (DebugPhysInstrumentHost_Enable){Serial.println(msg); delay(1);}
#define HostDebugln2(msg, args) if (DebugPhysInstrumentHost_Enable){Serial.println(msg, args); delay(1);}
#else
#define HostDebug(msg) ;
#define HostDebugln(msg) ;
#define HostDebugln2(msg, args) ;
#endif

#if TracePhysInstrumentHost
#define HostTrace(msg) if (TracePhysInstrumentHost_Enable){Serial.print(msg); delay(1);}
#define HostTraceln(msg) if (TracePhysInstrumentHost_Enable){Serial.println(msg); delay(1);}
#define HostTraceln2(msg, args) if (TracePhysInstrumentHost_Enable){Serial.println(msg, args); delay(1);}
#else
#define HostTrace(msg) ;
#define HostTraceln(msg) ;
#define HostTraceln2(msg, args);
#endif


#if DebugPhysInstrument 
extern uint8 DebugPhysInstrument_Enable;
extern uint8 TracePhysInstrument_Enable;
#if TracePhysInstrument
#define InstrumentTrace(msg) if (TracePhysInstrument_Enable && Instrument.DebugSerial != 0){Instrument.DebugSerial->print(msg);}
#define InstrumentTraceln(msg) if (TracePhysInstrument_Enable && Instrument.DebugSerial != 0){Instrument.DebugSerial->println(msg); }
#define InstrumentTraceln2(msg, args) if (TracePhysInstrument_Enable && Instrument.DebugSerial != 0){Instrument->DebugSerial.println(msg, args); }
#else
#define InstrumentTrace(msg) ;
#define InstrumentTraceln(msg) ;
#define InstrumentTraceln2(msg, args);
#endif
#define InstrumentDebugf1(msg, p0) if (DebugPhysInstrument_Enable && Instrument.DebugSerial != 0){Instrument.DebugSerial->printf(msg, p0); }
#define InstrumentDebugf2(msg, p1) if (DebugPhysInstrument_Enable && Instrument.DebugSerial != 0){Instrument.DebugSerial->printf(msg, p0, p1); }
#define InstrumentDebugf3(msg, p2) if (DebugPhysInstrument_Enable && Instrument.DebugSerial != 0){Instrument.DebugSerial->printf(msg, p0, p1, p2); }
#define InstrumentDebugf4(msg, p3) if (DebugPhysInstrument_Enable && Instrument.DebugSerial != 0){Instrument.DebugSerial->printf(msg, p0, p1, p2, p3); }
#define InstrumentDebug(msg) if (DebugPhysInstrument_Enable && Instrument.DebugSerial != 0){Instrument.DebugSerial->print(msg); }
#define InstrumentDebugln(msg) if (DebugPhysInstrument_Enable && Instrument.DebugSerial != 0){Instrument.DebugSerial->println(msg); }
#define InstrumentDebugln2(msg, args) if (DebugPhysInstrument_Enable && Instrument.DebugSerial != 0){Instrument->DebugSerial.println(msg, args); }
#else
#define InstrumentTrace(msg) {}
#define InstrumentTraceln(msg) {}
#define InstrumentTraceln2(msg, args){}
#define InstrumentDebug(msg) {}
#define InstrumentDebugf1(msg, p0) {}
#define InstrumentDebugf2(msg, p0, p1) {}
#define InstrumentDebugf3(msg, p0, p1, p2) {}
#define InstrumentDebugf4(msg, p0, p1, p2, p3) {}
#define InstrumentDebugln(msg) {}
#define InstrumentDebugln2(msg, args) {}
#endif
