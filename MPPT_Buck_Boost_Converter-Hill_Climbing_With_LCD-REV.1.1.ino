/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ MPPT Buck-Boost Charge Controller [Rev. 1.1]~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

   This code uses the popular "Hill Climbing" algorithm to track the maximum power point.
   The battery charging has two modes: constant current (CC) and constant voltage (CV).

   This code is under MIT License
   Copyright (c) 2021 Sayantan Sinha
*/

#include <LiquidCrystal.h>

#define VL_MAX 626                                                        // Max load voltage (CV mode) (approx. 14V)
#define IL_MAX 300                                                        // Max load current (CC mode)
#define DUTY_MAX 300                                                      // Max allowable duty cycle count (D_max = 75%)
#define DUTY_MIN 1                                                        // Min allowable duty cycle count (D_min = 0.25%)
#define H(X) (X > 0 ? (X > 20 ? X / 20 : 1) : (X < -20 ? X / 20 : -1))    // A gain factor in the proportional control action
#define P_HYST 50                                                         // PV power hysteresis

const int pinVload = 0, pinIload = 1, pinVpv = 2, pinIpv = 3;             // Sense pins: Load voltage = A0, Load current = A1, PV voltage = A2, PV current = A3

const int rs = A5, en = A4, d4 = 10, d5 = 11, d6 = 12, d7 = 13;           // Pin assignment for 16x2 LCD interfacing
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup()
{
  DDRB |= (1 << PB1);                                                     // Set D9 as output
  TCCR1A = 0b10000010;                                                    // Pin D9 (OCR1A): Clear on compare match
  TCCR1B = 0b00011001;                                                    // Fast PWM (Mode 14); Clock Select = System clock (No Prescaling) [Ref. Data Sheet, p. 172]
  ICR1 = 399;                                                             // f_sw = 16 MHz / (ICR1 + 1) = 40 kHz
  OCR1A = 0;                                                              // d = 0
  ADMUX = 0b01000000;                                                     // Select analog ref = +5 V
  ADCSRA = 0b10010111;                                                    // ADC enable, prescaler = f_CPU / 128 [Ref: Atmega328P datasheet, pp. 317-320]
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Initializing...");
  delay(800);
}
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ MPPT & CC-CV CHARGING ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void loop()
{
  static unsigned long tUpdateDisp = millis();                // Records time of updating display
  static unsigned long tUpdateBase = millis();                // Records time of updating pBase
  static unsigned long tUpdateDmax = millis();                // Records time of updating pBase
  static unsigned int pBase = 2400;                           // Initialize PV base power with a small non zero value
  static unsigned int vpvPrev = adcRead(pinVpv);              // Previous PV voltage
  static unsigned int pPeak = 0;                              // Peak Power from PV module
  static unsigned int pPeakCycle = 0;                         // Peak Power from PV module in a hovering cycle
  static unsigned int dMax = DUTY_MIN;                        // Maximum duty cycle counter
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
  if (ipv > 1023)                                             // Current reading went below offset value
    ipv = 0;
  if (iload > 1023)
    iload = 0;
  unsigned int p = (ipv / 4) * (vpv / 4);                     // Power from PV module

  if (p >= (pBase + 5)) {
    toggle = false;
  }
  if (pPeakCycle < p)
    pPeak = pPeakCycle = p;

  if (millis() - tUpdateDmax > 100) {
    static int i = 0;
    if (p < pBase) {
      if (!toggle) {
        pPeak = pPeakCycle;
        pPeakCycle = 0;
        if (vpv < vpvPrev)
          dDuty = -1;
        else
          dDuty = 1;
        vpvPrev = vpv;
        toggle = true;
      }
    }
    else {
      toggle = false;
    }

    if (vpv <= 480 || dMax >= DUTY_MAX) {
      dDuty = -1;
      pPeak = pPeakCycle;
      pPeakCycle = 0;
      vpvPrev = vpv;
      toggle = true;
    }
    else if (dMax <= DUTY_MIN) {
      dDuty = 1;
      pPeak = pPeakCycle;
      pPeakCycle = 0;
      vpvPrev = vpv;
      toggle = true;
    }

    if (vpv < 400) {
      OCR1A = 0;
      dMax = 0;
      lcd.clear();
      lcd.print("PV Voltage LO!");
      lcd.setCursor(0, 1);
      lcd.print((float)vpv / 35.80);
      lcd.print("V");
      delay (5000);
      lcd.clear();
      dDuty = 1;
    }

    if (dDuty < 0)
      dMax = dMax > DUTY_MIN ? dMax - 1 : DUTY_MIN;
    else if (dMax < DUTY_MAX)
      dMax++;
    if (i == 30)                                              // Update display every 3 s
      dispData(vpv, ipv, vload, iload, ccMode, cvMode, dMax);
    i = i < 30 ? i + 1 : 0;                                    // Counter for 2 s interval between display updates
    tUpdateDmax = millis();
  }

  int ie = IL_MAX - iload;                                     // Calculate current error
  int ve = VL_MAX - vload;                                     // Calculate voltage error
  int e = ve < ie ? ve / 2 : ie;                               // Calculate error

  if (ie < 10) {                                               // CC mode
    ccMode = 1;                                                // CC mode flag set
    cvMode = 0;                                                // CV mode flag clear
  }
  else if (ie > 20 && ve < 10) {                               // CV mode
    ccMode = 0;
    cvMode = 1;
  }
  else if (ie > 20 && ve > 20) {
    ccMode = 0;                                                // CC mode flag set
    cvMode = 0;
  }

  if (e > 0 && vload < VL_MAX && vpv > 400) {                              // Load current and load voltage are less than reference current and reference voltage
    OCR1A = (OCR1A + H(e)) < dMax ? OCR1A + H(e) : dMax;
  }
  else if (e < 0 || vload > VL_MAX) {                                      // Output current/voltage or both are higher than the reference current/voltage
    OCR1A = OCR1A > (-H(e) + DUTY_MIN) ? OCR1A + H(e) : DUTY_MIN;
  }

  if ((millis() - tUpdateBase) > 5) {
    static int j = 0;
    if ((pBase + P_HYST + 2) <= pPeak)
      pBase += 2;
    else if (pBase > 2400)                                    // pBase_min = vpv_min * ipv_min / 16
      pBase -= 2;
    j = j < 5 ? j + 1 : 0;
    tUpdateBase = millis();
  }
}
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ FUNCTION DEFINATIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void dispData(const unsigned int __vpv, const unsigned int __ipv, const unsigned int __vb, const unsigned int __ib, const bool __ccMode, const bool __cvMode, const unsigned int __dMax)
{
  static bool ivData = true;
  static float pvVoltage, pvCurrent, baVoltage, baCurrent;
  lcd.clear();
  if (ivData) {
    pvVoltage = (float)__vpv / 35.80;
    pvCurrent = (float)__ipv / 765.70;
    baVoltage = (float)__vb / 44.75;
    baCurrent = (float)__ib / 169.60;

    lcd.print(pvVoltage);                                                                 // Print PV voltage
    lcd.print("V ");
    lcd.print(baVoltage);                                                                 // Print battery voltage
    lcd.print(__cvMode ? "V*" : "V");                                                     // Print V* -> CV mode, V -> Batt voltage is less than max voltage
    lcd.setCursor(0, 1);
    lcd.print(pvCurrent);                                                                 // Print PV current
    lcd.print("A ");
    lcd.print(baCurrent);                                                                 // Print battery current
    lcd.print(__ccMode ? "A*" : "A");                                                     // Print A* -> CC mode, A -> Current is less than max current
  }
  else {
    lcd.print(pvVoltage * pvCurrent);                                                     // Print PV power
    lcd.print("W ");
    lcd.print(baVoltage * baCurrent);                                                     // Print Load power
    lcd.print("W");
    if (__vpv && __ipv && __vb && __ib) {
      lcd.setCursor(0, 1);
      lcd.print("e ");
      lcd.print(((baVoltage * baCurrent) / (pvVoltage * pvCurrent)) * 100);                 // Print Efficiency
      lcd.print("%");
    }
  }
  ivData = !ivData;
}

unsigned int adcRead(byte ch)
{
  if (ch > 15)
    return 0;
  ADMUX &= 0xF0;
  ADMUX |= ch;
  ADCSRA |= 0x40;                                                                         // Start conversion
  while (ADCSRA & 0x40);                                                                  // Wait until ADC start conversion bit goes low
  unsigned int a = ADCL;
  a |= (ADCH << 8);
  return (a);
}
