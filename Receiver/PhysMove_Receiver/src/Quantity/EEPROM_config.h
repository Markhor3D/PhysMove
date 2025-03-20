// Flag States:
// 10 = Some Data
// 20 = Self Instrument
// EEPin 16bit. Use 2 bytes for a float
#define EEP_DeviceIDOffset 0
#define EEP_ADCCalibDataBaseOffset (EEP_DeviceIDOffset + 1) // 1 word for the device ID field
#define EEP_ADCCalibDataIndividualSize ((2 /*calibM is float*/ + 1 /*calibC is int16*/) * 3 /*no of gains*/)
#define EEP_DACCalibDataBaseOffset (EEP_ADCCalibDataBaseOffset + 1 /* dg version */ +  EEP_ADCCalibDataIndividualSize * 4)
#define EEP_DACCalibDataIndividualSize (2 + 2 + 51) // cpm, cpc, 51x words
#define EEP_PhysInstrumentBaseOffset (EEP_DACCalibDataBaseOffset + EEP_DACCalibDataIndividualSize * 2)
// @ EEP_PhysInstrumentBaseOffset	> Session Type_8 > Data.
// @ EEP_PhysInstrumentBaseOffset  in case of Host
//									> Session Type_8 > Session ID_32 > Data.
// @ EEP_PhysInstrumentBaseOffset  in case of instrument 
//									> Session Type_8 > Instrument Data.