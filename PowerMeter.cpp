#include <Arduino.h>
#include <esp_int_wdt.h>
#include <esp_task_wdt.h>
#include "PowerMeter.h"

String Measure::toString()const {
  return "Voltage: " + String(rmsVoltage) + " Power: " + String(realPower) + " Current: " + String(rmsCurrent) +
         " Power sec.: " + String(realPowerSec) + " Current sec.: " + String(rmsCurrentSec);
}

PowerMeter::PowerMeter(double FS_V, double FS_I, double FS_I_SEC,
                       byte pinVoltageInput, byte pinCurrentInput, byte pinCurrentInputSec, int ADCBits): FS_V(FS_V), FS_I(FS_I), FS_I_SEC(FS_I_SEC) {
  DebugPort = &Serial;
  debug = true;
  this->pinVoltageInput = pinVoltageInput;
  this->pinCurrentInput = pinCurrentInput;
  this->pinCurrentInputSec = pinCurrentInputSec;

  MAX_COUNT = pow(2, ADCBits) - 1;
  MID_COUNT = MAX_COUNT / 2;

  ZERO_MAX = MAX_COUNT * 0.55;
  ZERO_MIN = MAX_COUNT * 0.45;

  voltageOffset = 1832.0;
  currentOffset = 1798.0;
  currentOffsetSec = 1790.0;
}

void PowerMeter::loop(int timeoutForReadAll, int semicyclesForReadAll) {

  // ---------------------------------------- Semi-cycles and samples counters
  unsigned int semicycleCounter;
  unsigned int sampleCounter;

  //----------------------------------------- Raw analog input values
  int voltageInputSample;
  int currentInputSample;
  int currentInputSampleSec;

  // ---------------------------------------- Raw analog input values minus the DC offset
  double filteredV;
  double filteredI;
  double filteredISec;

  // ---------------------------------------- Phase calibration coefficient
  double filteredV_old1;
  double filteredV_old2;
  double filteredI_old1;
  double filteredI_old2;
  double filteredISec_old1;
  double filteredISec_old2;
  double phaseCalibration = 1.0;

  // ---------------------------------------- Phase shifted voltage and current
  double phaseShiftedV;
  double phaseShiftedI;
  double phaseShiftedISec;
  // ---------------------------------------- Instantaneous voltage at the start of the sample window
  int startV;
  // ---------------------------------------- Used to measure number of times threshold is crossed
  boolean oldFlagUpDown, flagUpDown;
  // ---------------------------------------- Calculated final values
  double rmsVoltage, rmsCurrent, rmsCurrentSec, realPower, realPowerSec;
  // ---------------------------------------- Voltage, Current and Power accumulators
  double voltageSum, currentSum, currentSumSec, powerSum, powerSumSec;

  if (debug) DebugPort->println("Start reading");
  uint32_t t1 = millis();
  // ------------------------------------------------------------------------------ reset counters and variables
  semicycleCounter = 0;
  sampleCounter = 0;
  voltageSum = 0;
  currentSum = 0;
  currentSumSec = 0;
  powerSum = 0;
  powerSumSec = 0;
  //------------------------------------------------------------------------------- wait voltage near to zero (512 +/-10)
  unsigned long start = millis();
  while (true) {
    esp_task_wdt_reset();
    startV = analogRead(pinVoltageInput);
    if (startV > ZERO_MIN && startV < ZERO_MAX) break;
    if (millis() - start > timeoutForReadAll) break;
  }
  //------------------------------------------------------------------------------- LOOP for crossings times
  while (semicycleCounter < semicyclesForReadAll) {
    esp_task_wdt_reset();
    // ---------------------------------------------------------------------------- test timeout
    if (millis() - start > timeoutForReadAll) {
      rmsVoltage = 0;
      rmsCurrent = 0;
      rmsCurrentSec = 0;
      realPower = 0;
      realPowerSec = 0;
      if (debug){
        DebugPort->print("Stop reading for timeout, read in: ");
        DebugPort->println(millis()-t1);
      } 
      return;
    }

    // ---------------------------------------------------------------------------- increment the sammple counter
    sampleCounter++;
    // ---------------------------------------------------------------------------- save the Last Filtered Voltage and Current values
    filteredV_old2 = filteredV_old1;
    filteredV_old1 = filteredV;
    filteredI_old2 = filteredI_old1;
    filteredI_old1 = filteredI;
    filteredISec_old2 = filteredISec_old1;
    filteredISec_old1 = filteredISec;
    //----------------------------------------------------------------------------- read Voltage and Current samples
    voltageInputSample = analogRead(pinVoltageInput);
    currentInputSample = analogRead(pinCurrentInput);
    currentInputSampleSec = analogRead(pinCurrentInputSec);
    // ---------------------------------------------------------------------------- calc the mean voltage with a low pass filter
    voltageOffset += (voltageInputSample - voltageOffset) * 0.001;
    // ---------------------------------------------------------------------------- subtract the mean - filtered voltage is around zero
    filteredV = voltageInputSample - voltageOffset;
    //----------------------------------------------------------------------------- calc the mean current with a low pass filter
    currentOffset += (currentInputSample - currentOffset) * 0.001;
    currentOffsetSec += (currentInputSampleSec - currentOffsetSec) * 0.001;
    // ---------------------------------------------------------------------------- subtract the mean - filtered current is around zero
    filteredI = currentInputSample - currentOffset;
    filteredISec = currentInputSampleSec - currentOffsetSec;
    // ---------------------------------------------------------------------------- phase calibration
    //  The voltage delay respect to the current is +/-2 samples
    //  -2.0 / -1.9 ... -0.2 / -0.1 / 0 / 0.1 / 0.2 ... 1.9 / 2.0
    // ----------------------------------------------------------------------------
    if (phaseCalibration > 1) {       // 2 = voltage old2  /  1 = voltage old1
      phaseShiftedV = filteredV_old1 + (phaseCalibration - 1) * (filteredV_old2 - filteredV_old1);
      phaseShiftedI = filteredI;
      phaseShiftedISec = filteredISec;
    }
    else if (phaseCalibration > 0) {  // 1 = voltage old1  /  0 = voltage now (unshifted)
      phaseShiftedV = filteredV + phaseCalibration * (filteredV_old1 - filteredV);
      phaseShiftedI = filteredI;
      phaseShiftedISec = filteredISec;
    }
    else if (phaseCalibration < -1) { // -2 = current old2  /  -1 = current old1
      phaseShiftedI = filteredI_old1 + (abs(phaseCalibration) - 1) * (filteredI_old2 - filteredI_old1);
      phaseShiftedISec = filteredISec_old1 + (abs(phaseCalibration) - 1) * (filteredISec_old2 - filteredISec_old1);
      phaseShiftedV = filteredV;
    }
    else if (phaseCalibration < 0) {  // -1 = current old1  /  0 = current now (unshifted)
      phaseShiftedI = filteredI + (abs(phaseCalibration)) * (filteredI_old1 - filteredI);
      phaseShiftedISec = filteredISec + (abs(phaseCalibration)) * (filteredISec_old1 - filteredISec);
      phaseShiftedV = filteredV;
    }
    else { // phaseCalibration == 0   // 0 = voltage and current unshifted
      phaseShiftedV = filteredV;
      phaseShiftedI = filteredI;
      phaseShiftedISec = filteredISec;
    }
    //----------------------------------------------------------------------------- sum of Squared-Voltage samples
    voltageSum += (phaseShiftedV * FS_V)  * (phaseShiftedV * FS_V ); //(phaseShiftedV)  * (phaseShiftedV);
    //----------------------------------------------------------------------------- sum of Squared-Current samples
    currentSum += (phaseShiftedI * FS_I) * (phaseShiftedI * FS_I);  //(phaseShiftedI ) * (phaseShiftedI);
    currentSumSec += (phaseShiftedISec * FS_I_SEC) * (phaseShiftedISec * FS_I_SEC);  //(phaseShiftedISec ) * (phaseShiftedISec);
    //----------------------------------------------------------------------------- sum of the Instantaneous-Power samples
    powerSum += (phaseShiftedV * FS_V) * (phaseShiftedI * FS_I);
    powerSumSec += (phaseShiftedV * FS_V) * (phaseShiftedISec * FS_I_SEC);
    // ---------------------------------------------------------------------------- increase the semicyclesCounter
    oldFlagUpDown = flagUpDown;
    flagUpDown = voltageInputSample > startV;

    if (sampleCounter == 1) {
      oldFlagUpDown = flagUpDown;
    }
    if (oldFlagUpDown != flagUpDown) {
      semicycleCounter++;
    }
  }
  //------------------------------------------------------------------------------- RMS Voltage
  rmsVoltage = sqrt(voltageSum / sampleCounter);
  //------------------------------------------------------------------------------- RMS Current
  rmsCurrent = sqrt(currentSum / sampleCounter);
  rmsCurrentSec = sqrt(currentSumSec / sampleCounter);
  //------------------------------------------------------------------------------- Real Power
  realPower = powerSum / sampleCounter;
  realPowerSec = powerSumSec / sampleCounter;

  measure.rmsVoltage = rmsVoltage;
  measure.rmsCurrent = rmsCurrent;
  measure.rmsCurrentSec = rmsCurrentSec;
  measure.realPower = realPower;
  measure.realPowerSec = realPowerSec;
  if (debug){
    DebugPort->print("Stop reading, read in: ");
    DebugPort->println(millis()-t1);
  } 
}

double PowerMeter::getVoltage()const {
  return measure.rmsVoltage;
}
double PowerMeter::getCurrent()const {
  return measure.rmsCurrent;
}
double PowerMeter::getCurrentSec()const {
  return measure.rmsCurrentSec;
}
double PowerMeter::getPower()const {
  return measure.realPower;
}
double PowerMeter::getPowerSec()const {
  return measure.realPowerSec;
}
Measure PowerMeter::getMeasure()const {
  return measure;
}

void PowerMeter::getADCValues(int values[])const{
  values[0] = analogRead(pinVoltageInput);
  values[1] = analogRead(pinCurrentInput);
  values[2] = analogRead(pinCurrentInputSec);
  if(debug && DebugPort != NULL){
    DebugPort->println(String(values[0]) + ", " + String(values[1]) + ", " + String(values[2]));
  }
}

void PowerMeter::printADCValues()const{
  if(DebugPort != NULL){
    DebugPort->print(analogRead(pinVoltageInput));
    DebugPort->print(", ");
    DebugPort->print(analogRead(pinCurrentInput));
    DebugPort->print(", ");
    DebugPort->print(analogRead(pinCurrentInputSec));
  }
}

void PowerMeter::setPins(byte pinVoltageInput, byte pinCurrentInput, byte pinCurrentInputSec) {
  this->pinVoltageInput = pinVoltageInput;
  this->pinCurrentInput = pinCurrentInput;
  this->pinCurrentInputSec = pinCurrentInputSec;
}

void PowerMeter::setADCBits(uint8_t bits) {
  MAX_COUNT = pow(2, bits) - 1;
  MID_COUNT = MAX_COUNT / 2;

  ZERO_MAX = MAX_COUNT * 0.55;
  ZERO_MIN = MAX_COUNT * 0.45;
}

void PowerMeter::setVoltageOffset(uint32_t ofset) {
  voltageOffset = ofset;
}
void PowerMeter::setCurrentOffset(uint32_t ofset) {
  currentOffset = ofset;
}
void PowerMeter::setCurrentSecOffset(uint32_t ofset) {
  currentOffsetSec = ofset;
}

void PowerMeter::setOffsets(uint32_t vOffset, uint32_t iOffset, uint32_t i2Offset){
  voltageOffset = vOffset;
  currentOffset = iOffset;
  currentOffsetSec = i2Offset;
}

void PowerMeter::setDebug(bool debug) {
  this->debug = debug;
}
