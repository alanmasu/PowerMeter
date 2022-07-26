#ifndef __POWERMETER_H__
#define __POWERMETER_H__

#include <Arduino.h>
#include <esp_int_wdt.h>
#include <esp_task_wdt.h>

typedef struct Measure {
  double realPower;
  double realPowerSec;
  double rmsVoltage;
  double rmsCurrent;
  double rmsCurrentSec;
  String toString()const;
} Measure;


class PowerMeter {
  public:
    PowerMeter(byte pinVoltageInput = 34, byte pinCurrentInput = 32, byte pinCurrentInputSec = 35, int ADCBits = 12,
               double FS_V = 189.33E-3, double FS_I = 67.27E-3, double FS_I_SEC = 63.63E-3);
    ~PowerMeter() {};

    void loop(int timeoutForReadAll = 500, int semicyclesForReadAll = 20);

    double getVoltage()const;
    double getCurrent()const;
    double getCurrentSec()const;
    double getPower()const;
    double getPowerSec()const;
    Measure getMeasure()const;

    void setPins(byte pinVoltageInput, byte pinCurrentInput, byte pinCurrentInputSec);
    void setADCBits(uint8_t bits);


    //    void setVoltageFS(double FS);
    //    void setCurrentFS(double FS);
    //    void setCurrentSecFS(double FS);

    void setVoltageOfset(uint32_t ofset);
    void setCurrentOfset(uint32_t ofset);
    void setCurrentSecOfset(uint32_t ofset);
    void setDebug(bool debug);

  protected:
    Stream* DebugPort;
    bool debug;
    byte pinVoltageInput;
    byte pinCurrentInput;
    byte pinCurrentInputSec;
    const double FS_V;
    const double FS_I;
    const double FS_I_SEC;

    //----------------------------------------- Valori massimi e medi
    int MAX_COUNT;
    int MID_COUNT;

    // ---------------------------------------- valori di tolleranza sullo 0 ZERO_MAX e ZERO_MIN
    int ZERO_MAX;
    int ZERO_MIN;

    // ---------------------------------------- Low-pass filter output (starting at mid voltage for fast settling)
    double voltageOffset;
    double currentOffset;
    double currentOffsetSec;

    //----------------------------------------- Struct measure for the loop readings
    Measure measure;
};
#endif
