//********************************************************************************
// Projeto - bar graph com Arduino e CD4017
// Desenvolvido por: Eng. Márcio José Soares
// Versão: 1.0 - 01/10/2021
//
// Plataforma: Arduino Pro Mini (Mega328 16NHz 5V) ou 
//             Arduino Nano (Mega328) ou
//         
// IDE Arduino: 1.8.10 ou superior
//
// Objetivo: Controlar 10 leds bargraph utilizando um 4017 como CI auxiliar
// Obs.: Serão duas barras bargraph!
//
// Pinos:
// PD4 - RES4017_1  - Reset 4017 
// PD3 - CLK4017_1  - clock 4017
// PD2 - EN4017_1   - habilita 4017
//
// PD5 - RES4017_2  - Reset 4017
// PD6 - CLK4017_2  - clock 4017
// PD7 - EN4017_2   - habilita 4017
//
// PC0 - ANPIN1    - Entrada AD - entrada do conversor AD para bargraph 1
// PC1 - ANPIN2    - Entrada AD - entrada do conversor AD para bargraph 2
// PB5 - LED_LIVE - LED Built in da placa 
//
//********************************************************************************
//
// Últimas atualizações
//  em 01/10/2021:
//    - criado esse projeto
//********************************************************************************

//********************************************************************************
// Definições de pré-compilação
//********************************************************************************
#define __BAR_4017__

//********************************************************************************
// Arquivos incluídos no módulo
//********************************************************************************
#include <Arduino.h>
#include "pindeclarations.h"

//********************************************************************************
// Definições para pinos de I/O
//********************************************************************************
#define RES4017_1     PD4
#define CLK4017_1     PD3
#define EN4017_1      PD2

#define RES4017_2     PD5
#define CLK4017_2     PD6
#define EN4017_2      PD7

#define LED_LIVE      PB5   
#define AN_PIN1       PC0
#define AN_PIN2       PC1

//********************************************************************************
// Definições importantes do módulo
//********************************************************************************
#define TRUE          1
#define FALSE         0

//********************************************************************************
// Variáveis globais do módulo
//********************************************************************************
int LEDliveState = LOW;
long interval = 100;                 //ms  - tempo para intervalo
long previous = 0;
int k = 1;
int l = 1;

//********************************************************************************
// Constantes globais do módulo
//********************************************************************************
const int LEDTIME = 5;              //ms  - 500ms = 5 x 100ms = 500ms

//********************************************************************************
// Declara funções do módulo
//********************************************************************************

//********************************************************************************
// Setup - Arduino
//********************************************************************************
void setup() {

  Serial.begin(9600);

  pinMode(RES4017_1, OUTPUT);
  pinMode(CLK4017_1, OUTPUT);
  pinMode(EN4017_1, OUTPUT);

  pinMode(RES4017_2, OUTPUT);
  pinMode(CLK4017_2, OUTPUT);
  pinMode(EN4017_2, OUTPUT);
  pinMode(LED_LIVE, OUTPUT);

  digitalWrite(RES4017_1, HIGH);
  digitalWrite(CLK4017_1, LOW);
  digitalWrite(EN4017_1, HIGH);

  digitalWrite(RES4017_2, HIGH);
  digitalWrite(CLK4017_2, LOW);
  digitalWrite(EN4017_2, HIGH);

}

//********************************************************************************
// Loop - Arduino
//********************************************************************************
void loop() {

#ifdef __BAR_4017__

    uint16_t timeLED = 0;                         //variáveis
    uint16_t val = 0;
    long current = 0;

    digitalWrite(EN4017_1,LOW);                     //habilita o CI4017
    digitalWrite(EN4017_2,LOW);                     //habilita o CI4017
    while(TRUE){

        digitalWrite(RES4017_1,LOW);                //
        digitalWrite(RES4017_2,LOW);                //
        
        for(int i=0;i<k;i++){                     //tempo consumido aqui em média de 3 à 21 ms (delay)
            digitalWrite(CLK4017_1,HIGH);
            delayMicroseconds(100);
            digitalWrite(CLK4017_1,LOW);
            delayMicroseconds(100);
        }

        for(int i=0;i<l;i++){                     //tempo consumido aqui em média de 3 à 21 ms (delay)
            digitalWrite(CLK4017_2,HIGH);
            delayMicroseconds(100);
            digitalWrite(CLK4017_2,LOW);
            delayMicroseconds(100);
        }
        
        digitalWrite(RES4017_1,HIGH);               //reseta para limpar
        digitalWrite(RES4017_2,HIGH);               //reseta para limpar
        delayMicroseconds(100);
    
        current = millis();                       //tempo atual em ms
        if((current - previous) > interval)
        {
            previous = current;                   //salva tempo atual
            timeLED++;
            if(timeLED == LEDTIME)
            {
                timeLED = 0;  
            
                if(LEDliveState == LOW)           //troca estado atual do LED
                {
                    LEDliveState = HIGH;
                }
                else
                {
                    LEDliveState = LOW;
                }
            
                digitalWrite(LED_LIVE, LEDliveState);  
            }
             
             val = analogRead(AN_PIN1); 
             k = map(val, 0, 1023, 0, 10);  
             val = analogRead(AN_PIN2); 
             l = map(val, 0, 1023, 0, 10);  
        }
    }

#endif

}
