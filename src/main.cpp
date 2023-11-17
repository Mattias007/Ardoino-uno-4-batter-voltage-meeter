#include <Arduino.h>
#include <SoftwareSerial.h>
float R4 = 3.45;  //Ratios between voltage dividers 
float R3 = 5.02;
float R2 = 7.77;
float R1 = 10.87;



const int SSERIAL_RX_PIN = 10;  //Soft Serial Receive pin
const int SSERIAL_TX_PIN = 11;  //Soft Serial Transmit pin
const int SSERIAL_CTRL_PIN = 3;   //RS485 Direction control
const int RS485_TRANSMIT = HIGH;
const int RS485_RECEIVE = LOW;

SoftwareSerial RS485Serial(10, 11);


int byteReceived;
int byteSent;


long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
     ADMUX = _BV(MUX5) | _BV(MUX0) ;
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

void batterVoltage() {
  
float B1Voltage=0;  //Battery Voltager
float B2Voltage=0;
float B3Voltage=0;
float B4Voltage=0;
float RealBat1V = 0; //converted signal in 5v domain
float RealBat2V = 0;
float RealBat3V = 0;
float RealBat4V = 0;
float VolBat1 = 0;  //Analog channel raw value
float VolBat2 = 0; 
float VolBat3 = 0; 
float VolBat4 = 0; 
  
float vccValue=readVcc()/1000.0;  //Arduino refence voltage

VolBat4 = analogRead(A0);    // Reading battery voltages
VolBat3 = analogRead(A1);    
VolBat2 = analogRead(A2);    
VolBat1 = analogRead(A3);    
    
  RealBat1V = VolBat1 * vccValue/1024.0;  //Converting raw value in 5v domian
  RealBat2V = VolBat2 * vccValue/1024.0;
  RealBat3V = VolBat3 * vccValue/1024;
  RealBat4V = VolBat4 * vccValue/1024;
  
  B4Voltage = RealBat4V * R4;     //Calculating actual voltages
  B3Voltage = RealBat3V * R3 - B4Voltage;    
  B2Voltage = RealBat2V * R2 - B4Voltage - B3Voltage;
  B1Voltage = RealBat1V * R1 - B4Voltage - B3Voltage - B2Voltage;

// Serial.print("Battery-1 Voltage="); //Print voltages on serial monitor
// Serial.println(B1Voltage);
// Serial.print("Battery-2 Voltage=");
// Serial.println(B2Voltage);
// Serial.print("Battery-3 Voltage=");
// Serial.println(B3Voltage);
// Serial.print("Battery-4 Voltage=");
// Serial.println(B4Voltage);
// Serial.println(">>>>>>>>>>>>>.");

  digitalWrite(SSERIAL_CTRL_PIN, RS485_TRANSMIT);

RS485Serial.print("BatteryVoltage1");
RS485Serial.print(",");
RS485Serial.print(B1Voltage);
RS485Serial.print(",");
RS485Serial.print(B2Voltage);
RS485Serial.print(",");
RS485Serial.print(B3Voltage);
RS485Serial.print(",");
RS485Serial.print(B4Voltage);

  digitalWrite(SSERIAL_CTRL_PIN, RS485_RECEIVE);
}



void setup() {
  // initialize serial port
  Serial.begin(9600); 
  // Configure any output pins
  pinMode(SSERIAL_CTRL_PIN, OUTPUT);  
  digitalWrite(SSERIAL_CTRL_PIN, RS485_RECEIVE);
  RS485Serial.begin(9600);
}


void loop() {

  if (RS485Serial.available())
   {

    unsigned long channel = RS485Serial.read();
    Serial.println(channel);

    if(channel == 5){
      batterVoltage();
    }
   }  


     
}
