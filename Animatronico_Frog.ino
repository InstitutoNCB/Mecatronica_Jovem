//**************************************************************
// Animatronico Frog
// Revista Mecatronica Jovem nr3
//
// Desenvolvido por: Eng. Marcio Jose Soares - 12/2021
//
// Plataforma: Arduino UNO
// Sensor    : APDS-9960
// Servos    : 2 microservos 5grs (padrao HXT500)
//
// Referencias: GestureTest by SparkFun APDS-9960
//
// Conexoes para os pinos:
//
// Pinos Arduino UNO     Modulo Sensor APDS-9960   Funcao
// 
// 3.3V                 VCC                       Power
// GND                  GND                       Ground
// A4                   SDA                       I2C Data
// A5                   SCL                       I2C Clock
// 2                    INT                       Interrupcao
//
//                      Servos                    Funcao
// 5                    1                         Mov. Olhos
// 6                    2                         Mov. Boca
//
// Atencao: O sensor APDS-9960 deve ser alimentado atraves do
//          pino 3V3 do Arduino UNO
//
//          Os servos deve ser alimentados atraves do pino 5V
//          do Arduino UNO
//
//          Esse programa foi testado com Arduino UNO. 
//**************************************************************

//**************************************************************
// Arquivos incluidos no modulo
//**************************************************************
#include <Wire.h>
#include <SparkFun_APDS9960.h>
#include <Servo.h>

//**************************************************************
// Definicao dos Pinos
//**************************************************************
#define APDS9960_INT    2   // pino de int
#define servoOlhos      5 
#define servoBoca       6

//**************************************************************
// Definicoes para controle dos servos
//**************************************************************
#define UP    70
#define DOWN  125
#define LEFT  125
#define RIGHT 60

#define CENTER1 90
#define CENTER2 70

//**************************************************************
// Outras Definicoes importantes para o modulo
//**************************************************************
#define TRUE  1
#define FALSE 0

//**************************************************************
// Variaveis globais
//**************************************************************
SparkFun_APDS9960 apds = SparkFun_APDS9960();
int isr_flag = 0;

Servo Anima[2];
int servoPins[2] = {5,6};

//**************************************************************
// Setup
//**************************************************************
void setup() {

  //Configura pinos
  pinMode(APDS9960_INT, INPUT);
  pinMode(servoOlhos, OUTPUT);
  pinMode(servoBoca, OUTPUT);
  
  //Inicializa porta serial
  Serial.begin(9600);
  Serial.println();
  Serial.println(F("--------------------------------"));
  Serial.println(F("SparkFun APDS-9960 - GestureTest"));
  Serial.println(F("--------------------------------"));
  
  // Inicializa serviço da int
  attachInterrupt(0, interruptRoutine, FALLING);

  // Inicializa APDS-9960 (configura I2C e inicia valores)
  if ( apds.init() ) {
    Serial.println(F("Inicializacao do APDS-9960 completa!"));
  } else {
    Serial.println(F("Algo saiu errado durante a inicalizacao do APDS-9960!"));
  }
  
  // Inicia 
  if ( apds.enableGestureSensor(true) ) {
    Serial.println(F("Sensor de gestos funcionando!"));
  } else {
    Serial.println(F("Algo saiu errado durante a inicializacao do sensor de gestos!"));
  }

  initservos();
  
}

//**************************************************************
// Loop
//**************************************************************
void loop() {

    if( isr_flag == 1 ) {
      detachInterrupt(0);
      handleGesture();
      isr_flag = 0;
      attachInterrupt(0, interruptRoutine, FALLING);
    }
    
}

//**************************************************************
// Interrupcao
//**************************************************************
void interruptRoutine() {
  isr_flag = 1;
}

//**************************************************************
// Funçao para controle do sensor de gestos
//
// Adatada para controlar os servos no Animatronico
//
// Entradas - nenhuma
// Saídas   - nenhuma
//**************************************************************
void handleGesture() {
    if ( apds.isGestureAvailable() ) {
    switch ( apds.readGesture() ) {
      case DIR_UP:
        Serial.println("ABRE BOCA");
        move_DOWN();
        break;
      case DIR_DOWN:
        Serial.println("FECHA BOCA");
        move_UP();
        break;
      case DIR_LEFT:
        Serial.println("OLHE p/ ESQUERDA");
        move_LEFT();
        break;
      case DIR_RIGHT:
        Serial.println("OLHE p/ DIREITA");
        move_RIGHT();
        break;
      /*case DIR_NEAR:
        Serial.println("PERTO");
        move_CENTER();
        break;
      case DIR_FAR:
        Serial.println("LONGE");
        move_CENTER();
        break;*/
      default:
        Serial.println("CENTRALIZANDO...");
        move_CENTER();
    }
  }
}

//****************************************************************************
// Função para iniciar os servos
//
// Entradas - nenhuma
// Saídas   - nenhuma
//****************************************************************************
void initservos(void){
    int i;
    for(i=0; i<2; i++){
        Anima[i].attach(servoPins[i]);
    }
    move_CENTER();
}

//****************************************************************************
// Função para centralizar partes do animatronico
//
// Entradas - nenhuma
// Saídas   - nenhuma
//****************************************************************************
void move_CENTER(void){
    Anima[0].write(CENTER1);
    Anima[1].write(CENTER2);
}

//****************************************************************************
// Função para mover olhos para esquerda
//
// Entradas - nenhuma
// Saídas   - nenhuma
//****************************************************************************
void move_LEFT(void){

    Anima[0].write(LEFT);
    
}

//****************************************************************************
// Função para mover olhos para direita
//
// Entradas - nenhuma
// Saídas   - nenhuma
//****************************************************************************
void move_RIGHT(void){

    Anima[0].write(RIGHT);
    
}

//****************************************************************************
// Função para abrir a boca
//
// Entradas - nenhuma
// Saídas   - nenhuma
//****************************************************************************
void move_DOWN(void){

    Anima[1].write(DOWN);
    
}

//****************************************************************************
// Função para fechar a boca
//
// Entradas - nenhuma
// Saídas   - nenhuma
//****************************************************************************
void move_UP(void){

    Anima[1].write(UP);
    
}
