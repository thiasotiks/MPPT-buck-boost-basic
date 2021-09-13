/*
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~MPPT Buck-Boost Charge Controller [Rev. 1]~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

   This code uses the popular "Hill Climbing" algorithm to track the maximum power point

   This code is under MIT License
   Copyright (c) 2021 Sayantan Sinha
*/

#include <LiquidCrystal.h>

#define VL_MAX 610                                                        // Max load voltage (CV mode) (approx. 14V)
#define IL_MAX 200                                                        // Max load current (CC mode)
#define DUTY_MAX 200                                                      // Max allowable duty cycle count
#define DUTY_MIN 5                                                        // Min allowable duty cycle count (D_min = 10%)
#define H(X) (X > 0 ? (X > 20 ? X / 20 : 1) : (X < -20 ? X / 20 : -1))    // A gain factor in the proportional control action
#define P_HYST 50                                                         // PV power hysteresis

const int pinVload = 0, pinIload = 1, pinVpv = 2, pinIpv = 3;             // Sense pins: Load voltage = A0, Load current = A1, so on...
bool softstrt = 1;                                                        // Soft start enable flag

const int rs = A5, en = A4, d4 = 10, d5 = 11, d6 = 12, d7 = 13;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup()
{
  DDRB |= (1 << PB1);                       // Set D9 as output
  TCCR1A = 0b10000010;                      // Pin D9 (OCR1A): Clear on compare match
  TCCR1B = 0b00011001;                      // Fast PWM (Mode 14); Clock Select = System clock (No Prescaling) [Ref. Data Sheet, p. 172]
  ICR1 = 399;                               // f_sw = 16 MHz / (ICR1 + 1) = 40 kHz
  OCR1A = 0;                                // d = 0
  ADMUX = 0b01000000;                       // Select A_ref = +5 V
  ADCSRA = 0b10010111;                      // ADC enable, prescaler = f_CPU / 128 [Ref: Atmega328P datasheet, pp. 317-320]
  softstrt = 1;                             // Initially increase the voltage slowly
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Initializing...");
}

void loop()
{
  static unsigned long tUpdate = millis();                    // Records time of updating vref during softstart
  static unsigned long tUpdateDisp = millis();                // Records time of updating display
  static unsigned long tUpdateBase = millis();                // Records time of updating pBase
  static unsigned long tUpdateIref = millis();                // Records time of updating pBase
  static unsigned int pBase = 15000;                          // Initialize PV base power with a small non zero value
  static unsigned int vpvPrev = adcRead(pinVpv);              // Previous PV voltage
  static unsigned int pPeak = 0;                              // Previous Peak Power from PV module
  static unsigned int dMax = DUTY_MIN;                        // Maximum duty cycle counter
  static int iref = IL_MAX;                                   // Reference current for load
  static int vref = 40;                                       // Reference voltage for load
  static int dDuty = 1;                                       // Duty cycle change direction: +ve = increase, -ve = decrease
  static int iloadOffset = adcRead(pinIload);                 // Read load current
  static int ipvOffset = adcRead(pinIpv);                     // Read PV current
  static bool ccMode = 0;                                     // Flag to indiacte CC mode of operation
  static bool cvMode = 0;                                     // Flag to indiacte CC mode of operation
  static bool toggle = false;                                 // Flag to indiacte PV power crossed the base-power and toggling of duty cycle change direction is enabled
  unsigned int vload = adcRead(pinVload);                     // Read output voltage
  unsigned int iload = adcRead(pinIload) - iloadOffset;       // Read load current
  unsigned int vpv = adcRead(pinVpv);                         // Read PV voltage
  unsigned int ipv = adcRead(pinIpv) - ipvOffset;             // Read PV current
  if (ipv == 65535)                                           // Current reading went below offset value
    ipv = 0;
  if (iload == 65535)
    iload = 0;
  unsigned int p = (ipv / 4) * (vpv / 4);                     // Power from PV module

  if (p >= (pBase + 20)) {
    toggle = false;
    vpvPrev = vpv;
    if (pPeak < p)
      pPeak = p;
  }

  if (millis() - tUpdateIref > 100) {
    static int i = 0;
    if (p < pBase) {
      if (!toggle) {
        pPeak = 0;
        if (vpv < vpvPrev)
          dDuty = -1;
        else
          dDuty = 1;
        toggle = true;
      }
    }
    else {
      toggle = false;
    }

    if (vpv < 380)
      dDuty = -1;
    else if (ipv < 200)
      dDuty = 1;

    if (dDuty < 0)
      dMax = dMax > DUTY_MIN ? dMax - 1 : DUTY_MIN;
    else if (dMax < DUTY_MAX)
      dMax++;
    if (i == 20)
      dispData(vpv, ipv, vload, iload, ccMode, cvMode, dMax);
    i = i < 20 ? i + 1 : 0;
    tUpdateIref = millis();
  }

  int ie = iref - iload;                                      // Calculate current error
  int ve = vref - vload;                                      // Calculate voltage error
  int e = ve < ie ? ve / 2 : ie;                              // Calculate error

  if (ie < 10) {                                              // CC mode
    ccMode = 1;                                               // CC mode flag set
    cvMode = 0;                                               // CV mode flag clear
  }
  else if (ie > 20 && ve < 10) {                              // CV mode
    ccMode = 0;
    cvMode = 1;
  }
  else if (ie > 20 && ve > 20) {
    ccMode = 0;                                               // CC mode flag set
    cvMode = 0;
  }

  if (e > 0 && vload < VL_MAX && vpv > 340) {                              // Load current and load voltage are less than reference current
    OCR1A = (OCR1A + H(e)) < dMax ? OCR1A + H(e) : dMax;
  }
  else if (e < 0 || vload > VL_MAX) {                         // Output current/voltage or both are higher than the reference current/voltage
    OCR1A = OCR1A > (-H(e) + DUTY_MIN) ? OCR1A + H(e) : DUTY_MIN;
  }

  if ((millis() - tUpdateBase) > 1) {
    if ((pBase + P_HYST + 2) <= pPeak)
      pBase += 2;
    else if (pBase > 2400)                                    // pBase_min = vpv_min * ipv_min / 16
      pBase -= 2;
    tUpdateBase = millis();
  }

  if (softstrt) {
    if ((millis() - tUpdate) > 4) {
      vref += vref < VL_MAX ? 1 : 0;
      tUpdate = millis();
    }
    if (vref >= VL_MAX)
      softstrt = 0;
  }
}

void dispData(const unsigned int __vpv, const unsigned int __ipv, const unsigned int __vb, const unsigned int __ib, const bool __ccMode, const bool __cvMode, const unsigned int __dMax)
{
  static bool ivData = true;
  lcd.clear();
  float pvVoltage = (float)__vpv / 35.80;
  float pvCurrent = (float)__ipv / 765.70;
  float baVoltage = (float)__vb / 44.75;
  float baCurrent = (float)__ib / 169.60;
  if (ivData) {
    lcd.print(pvVoltage);
    lcd.print("V ");
    lcd.print(baVoltage);
    lcd.print(__cvMode ? "V*" : "V");
    lcd.setCursor(0, 1);
    lcd.print(pvCurrent);
    lcd.print("A ");
    lcd.print(baCurrent);
    lcd.print(__ccMode ? "A*" : "A");
  }
  else {
    lcd.print("PV ");
    lcd.print(pvVoltage * pvCurrent);
    lcd.print("W");
    lcd.setCursor(0, 1);
    lcd.print("LD ");
    lcd.print(baVoltage * baCurrent);
    lcd.print("W ");
    lcd.print(((baVoltage * baCurrent) / (pvVoltage * pvCurrent)) * 100);                 // Efficiency
    lcd.print("%");
  }
}

unsigned int adcRead(byte ch)
{
  if (ch > 15)
    return 0;
  ADMUX &= 0xF0;
  ADMUX |= ch;
  ADCSRA |= 0x40;            // Start conversion
  while (ADCSRA & 0x40);     // Wait until ADC start conversion bit goes low
  unsigned int a = ADCL;
  a |= (ADCH << 8);
  return (a);
}
