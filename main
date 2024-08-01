// programa original que apaga e acende led do arduino e no protoboard (5) de acordo com o tempo setado, enquanto também mexe o motor

#include "HX711.h"                    // Biblioteca HX711 
#include <avr/io.h>
#include <avr/interrupt.h>
#define DOUT  A2                      // HX711 DATA OUT = pino A0 do Arduino 
#define CLK   A3                      // HX711 SCK IN = pino A1 do Arduino 
HX711 balanca;          // define instancia balança HX711
float calibration_factor = 70000;     // fator de calibração para teste inicial

bool LED_state = LOW;
int segundos;
const int LED_pin = 13; 
volatile int count;
int count_segundos;
byte reload = 0x9C;
int incomingByte = 0;

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  100 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  290 // this is the 'maximum' pulse length count (out of 4096)

int Ti = 2000;       //Inspiratory Time in ms
int novo_Ti;
int Te = 4000;      //Expiratory Time in ms
int novo_Te;
int On = 0;
int Off = 0;
int On_Off_Flag = 1;
int InspP_Tg_novo;

// our servo # counter
uint8_t servonum = 0;

char Phase = 0; // 0 - Insp; 1 - Exp
char InspBegin_Flag = 1; // 1 - before beginning; 0 - after beginning
char ExpBegin_Flag = 1; // 1 - before beginning; 0 - after beginning

unsigned long phase_start_instant = 0;
int openINSP = 0;     //0 to 100% Insp Valve Openning

#define ErroMax 25
#define K 4

float Insp_P = 0;
int ErroP = 0;
int InspValveOpen = 0;    //variar de 0 a 100 (%) - valor corrente da abertura da Válvula Inspiratória
int InspP_Tg = 25;
int Alarm = 0;
int New_Alarm = 0;

float Insp_P_exp = 0;
int ErroP_exp = 0;

ISR(TIMER1_COMPA_vect)
{
count++;
if (count == 10){
 if (Phase == 1) Serial.print(String(Insp_P_exp)+"$"+"1"+"!");
 else  Serial.print(String(Insp_P)+"$"+"0"+"!");
  count = 0;
  count_segundos += 1;
  }
if (count_segundos == 100){
  segundos += 1;
  count_segundos = 0;
}
}

void setup() {
  pinMode(7,OUTPUT);
  Serial.begin(115200);
  //Serial.println("8 channel Servo test!");

  balanca.begin(DOUT, CLK);      // inicializa a balança
  balanca.set_scale();                                             // configura a escala da Balança
  zeraBalanca ();                                                  // zera a Balança

  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  pinMode(LED_pin, OUTPUT);
  digitalWrite(LED_pin, LOW);
  cli();
  TCCR1A = 0;
  TCCR1B = 1<<WGM12 | 0<<CS12 | 1<<CS11 | 1<<CS10;
  TCNT1 = 0;          // reset Timer 1 counter
  // OCR1A = ((F_clock / prescaler) / Fs) - 1 = 2499
  OCR1A = 2499;       // Set sampling frequency Fs = 100 Hz
  TIMSK1 = 1<<OCIE1A; // Enable Timer 1 interrupt 
  sei();
  On_Off_Flag = 0;
  On = 0;
  Off = 1;
}

void zeraBalanca ()
{
  //  Serial.println();                                               // salta uma linha
  balanca.tare();                                                 // zera a Balança
  //  Serial.println("Balança Zerada ");
}


void loop() {
  if (Serial.available()) {
      String informacoes_recebidas = Serial.readString();

      // Parse the received_data to extract variables and special characters
      //int variable1 = received_data.substring(1, received_data.indexOf('!')).toInt();
      novo_Ti = informacoes_recebidas.substring(informacoes_recebidas.indexOf('!') + 1, informacoes_recebidas.indexOf('*')).toInt();
      novo_Te = informacoes_recebidas.substring(informacoes_recebidas.indexOf('@') + 1, informacoes_recebidas.indexOf('#')).toInt();
      On = informacoes_recebidas.substring(informacoes_recebidas.indexOf('*') + 1, informacoes_recebidas.indexOf('@')).toInt();
      New_Alarm = informacoes_recebidas.substring(informacoes_recebidas.indexOf('^') + 1, informacoes_recebidas.indexOf('&')).toInt();
      Off = informacoes_recebidas.substring(informacoes_recebidas.indexOf('#') + 1, informacoes_recebidas.indexOf('^')).toInt();
      InspP_Tg_novo = informacoes_recebidas.substring(informacoes_recebidas.indexOf('&') + 1, informacoes_recebidas.indexOf('¨')).toInt();
    if (On == 1){
      digitalWrite (4,HIGH);
      On_Off_Flag = 1;
    }
    if (Off == 1){
      digitalWrite (4,LOW);
      On_Off_Flag = 0;
    }
    if (InspP_Tg_novo != InspP_Tg){
      InspP_Tg = InspP_Tg_novo;
    }
    if (novo_Ti != Ti /*&& novo_Ti != 0*/){ // É UMA MANEIRA DE CONTORNAR O PROBLEMA, MAS COM OUTROS PARAMETROS PODE NÃO SER IDEAL
      Ti = novo_Ti;
    }
    if (novo_Te != Te /*&& novo_Te != 0*/){
      Te = novo_Te;
    }
    if (New_Alarm != Alarm){
      Alarm = New_Alarm;
      digitalWrite(7,HIGH);
    }

      // Print the received variables
      /*Serial.print("Variable 1: ");
      Serial.println(variable1);
      Serial.print("Variable 2: ");
      Serial.println(variable2);
      Serial.print("Variable 3: ");
      Serial.println(variable3);8*/
      //if (variable1==3 && variable2 == 2 && variable3 == 0){
      //  digitalWrite (3, HIGH);
      //}
      //else{
      //  digitalWrite (3,LOW);
      //}     
  }

  if (Phase == 0 && On_Off_Flag) { //Inspiration
    if (InspBegin_Flag) {         // store time of beginning of inspiration
      phase_start_instant = millis();
      InspBegin_Flag = 0;
      ControlExpValve(0);        // close ExpValve
//      Serial.println("InspBegin_Flag");
    }


//****controls InspValve openning during Inspiratory phase
   balanca.set_scale(calibration_factor);                     // ajusta fator de calibração
   Insp_P = balanca.get_units();
//      Serial.println("Leitura Balança");
   if(Insp_P<0)
    Insp_P = 0;
   if(Insp_P>100)
    Insp_P=100;
  //cmH20 = 1.08225*Insp_P-24.06924;
//  if (cmH20 < 0)
//    cmH20 = 0;
   //Serial.println(Insp_P);
   ErroP = (InspP_Tg - Insp_P); 
//   Serial.print(" ");
//   Serial.println(ErroP);
   if(ErroP > ErroMax)
    ErroP = ErroMax; 
   InspValveOpen = ErroP * K;
   if(InspValveOpen > 100)        //limit the openning value to 100% - depends on Servo Motor PWM Min and Max values
     InspValveOpen = 100;
   if(ErroP > 0)                  //if current pressure is under target, open inspiratory valve - proportional to error value
     ControlInspValve(InspValveOpen); 
   if(ErroP <= 0)                 //if current pressure is above target, close inspiratory valve
    ControlInspValve(0);

//****

    if (millis() > phase_start_instant + Ti) {      // end of inspiration
      ControlInspValve(0);      // close InspValve
      ControlExpValve(0);        // close ExpValve
      Phase = 1;                 // toogle to Expiratory Phase
      InspBegin_Flag = 1;        // ready to start new inspiration
    }
  }
  else{
    ControlInspValve(0);
    ControlExpValve(1);
    balanca.set_scale(calibration_factor);
    Insp_P = balanca.get_units();
  }

  if (Phase == 1) { //Expiration
    if (ExpBegin_Flag) {         // store time of beginning of expiration
      phase_start_instant = millis();
      ExpBegin_Flag = 0;
      ControlInspValve(0);      // close InspValve
      ControlExpValve(1);        // open ExpValve
//      Serial.println("Início de Expiração");
    }
    if (millis() > phase_start_instant + Te) {      //end of expiration
      ControlInspValve(0);      // close InspValve
      ControlExpValve(0);        // close ExpValve
      Phase = 0;                 // toogle to Expiratory Phase
      ExpBegin_Flag = 1;        // ready to start new inspiration
//      Serial.println("Fim de Expiração");
    }

   balanca.set_scale(calibration_factor);                     // ajusta fator de calibração
   Insp_P_exp = balanca.get_units();
   if(Insp_P_exp<0)
    Insp_P_exp = 0;
   if(Insp_P_exp>100)
    Insp_P_exp=100;
   //Serial.println(Insp_P_exp);
   ErroP_exp = (InspP_Tg - Insp_P_exp); 
//   Serial.print(" ");
//   Serial.println(ErroP_exp);

  }

}

void ControlInspValve(int opencloseInsp) {  //opencloseInsp - 0 to 100 % openning
  pwm.setPWM(1, 0, (SERVOMAX-(SERVOMAX-SERVOMIN)*opencloseInsp/100));
}


void ControlExpValve(char opencloseExp) {  //close ExpValve if "openclose" = 0, open ExpValve if "openclose" = 1
  if (!opencloseExp)
    pwm.setPWM(0, 0, SERVOMAX);
  if (opencloseExp)
    pwm.setPWM(0, 0, SERVOMIN);
}

