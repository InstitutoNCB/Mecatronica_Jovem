/****************************************
* PROJETO: ENQUETE YOUTUBER             *
* PARCERIA: INSTITUTO NEWTON C. BRAGA   *
* DESENVOLVEDOR: VANDERLEI ALVES        *
* MCU: ATMEGA328P CLOCK: 16MHZ          *
* DATA DE INÍCIO: 30/08/2025            *
*****************************************/

//------ Mapeamento dos pinos -----------------------------------
#define PIN_ENAB  (1<<DDB0)
#define PIN_STEP  (1<<DDB4)
#define PIN_DIR   (1<<DDB3)
#define LED_R     (1<<DDB2)
#define LED_G     (1<<DDB1)
#define LED_B     (1<<DDB5)
#define BTN_INIT  (1<<DDD5)
#define LD_A      (1<<DDD4)
#define LD_B      (1<<DDD3)
#define PPV       200


//------ Protótipo de funções -----------------------------------
void BarraLeds(int pos);
void DirectionMotor(bool dir);
void EfeitoLeds();
void LigaDesligaMotor(bool en);
void LimparSerial();
int MotorGame(int ang, int ppv, unsigned long t, bool dir);
void MotorPosition(int ang, int ppv, unsigned long t, bool dir);
void MotorRun(unsigned long t, bool dir);
void processarComando(const String& cmd);

//------ Variáveis globais---------------------------------------
String bufferLinha;
bool btnIniciar = false;
bool trava = false;
int pos = 0;

void setup() {
  //Configuração dos pinos de saídas digitais
  DDRB |= (PIN_STEP | PIN_DIR | PIN_ENAB | LED_R | LED_G | LED_B);
  //Mantém o motor desligado com o pino EN do driver em nível alto
  PORTB |= PIN_ENAB;
  DDRD |= (LD_A | LD_B);
  //Configuração dos pinos de entrada digital
  DDRD &= ~BTN_INIT;
  PORTB &= ~PIN_STEP;
  Serial.begin(115200);

  //Posiciona o motor na posição inicial usando o fim de curso
  while(!btnIniciar)
  {
    //Realiza a leitura do botão iniciar
    btnIniciar = PIND & BTN_INIT;

    //Desliga o motor para que possibilite o movimento manual para o ponto de início.
    LigaDesligaMotor(0);
    BarraLeds(3);
    delay(300);
    BarraLeds(4);
    delay(300);
  }
  EfeitoLeds();
  delay(1000);
  BarraLeds(0);
  
  Serial.println("OK READY");
}

void loop() {
  while (Serial.available()) 
  {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') 
    {
      if (bufferLinha.length() > 0) 
      {
        processarComando(bufferLinha);
        bufferLinha = "";
      }
    } 
    else 
    {
      bufferLinha += ch;
      if (bufferLinha.length() > 120) 
      { 
        //Proteção básica
        bufferLinha = "";
        Serial.println("ERRO: LINHA LONGA");
      }
    }
  }

}
