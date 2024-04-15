/****************************************************************
 * ----------------TEMPORIZADOR PARA CARGAS---------------------*
 * EMPRESA: Vandertronic                                        *
 * CLUBE: Mecatrônica Jovem - Instituto Newton C. Braga         *
 * DESENVOLVEDOR: Vanderlei Alves Santos da Silva               *
 * MCU: ATMEGA328P     CLOCK: 16MHz                             *
 * DATA: 31/01/2024                                             *
 ****************************************************************/

//Protótipo de funções-------------------------------------------
void Alertar();
bool Restart();
//---------------------------------------------------------------

//Variáveis de interrupção---------------------------------------
volatile  int T2aux = 0, 
              tempo = 300,    //Inicialização do tempo em segundos
              auxT,
              counter;
//---------------------------------------------------------------

//Variáveis globais----------------------------------------------             
bool alt = true,
     alert = false;
//---------------------------------------------------------------     
             
//------------------FUNÇÃO DE INTERRUPÇÃO------------------------
ISR(TIMER2_OVF_vect) //Vetor de interrupção
{        
  TCNT2 = 56;
  T2aux++;
    if(T2aux == 2500) //T2aux chegou em 1 segundo?
    {
      T2aux = 0;
      //Contadores Down decrementado a cada 1 segundo respectivamente.
      counter--; 
      if(counter <= 0) counter = 0;
    }
}
//---------------FIM DA FUNÇÃO DE INTERRUPÇÃO--------------------

void setup() {

  cli();
  //Registradores de interrupção por Timer2.
  TCCR2A = 0x00;
  TCCR2B = 0x03;
  TCNT2  = 56;
  TIMSK2 = 0x01;
  sei();

  //Registradores de entrada e saída.
  DDRD &= ~(1<<PIND7);                                      //Botão restart.
  DDRB |= ((1<<PB1)|(1<<PB2)|(1<<PB3)|(1<<PB4)|(1<<PB5));   //Configuração da saída para os LEDs
  DDRC |= ((1<<PC3)|(1<<PC5));                              //Configuração de saída para o relé e para o beep
  PORTB &= ~((1<<PB1)|(1<<PB2)|(1<<PB3)|(1<<PB4)|(1<<PB5)); //Inicializa os leds apagados

  while(Restart())
  {
    PORTC &= ~(1<<PC3);
  }
  counter = tempo; //Inicia contador em 5 minutos (5 x 60 = 300 segundos).
}

void loop() {

  if(!Restart())
  {
    PORTC |= (1<<PC3); //Liga o relé.
    counter = tempo;
    while(counter > 0) //Enquanto o tempo decrementa.
    {
      if(!Restart()) counter = tempo, PORTC &= ~(1<<PC5); //Reinicia o tempo e desativa o beep.

      if(counter <= 60) Alertar();

      //----------CONTAGEM POR LEDs-----------------//
      switch(counter)
      {
          case 299:
          PORTB = 0b00111110;
          break;
          case 240:
          PORTB = 0b00011110;
          break;
          case 180:
          PORTB = 0b00001110;
          break;
          case 120:
          PORTB = 0b00000110;
          break;
          case 60:
          PORTB = 0b00000010;
          break;
          case 1:
          PORTB = 0b00000000;
          PORTC &= ~(1<<PC3); //Relé desligado.
          break;    
      }
      //--------------------------------------------//
    }
    PORTC &= ~(1<<PC5); //Desativa o beep.
  }

}
//------Implementações das funções auxiliares--------------
void Alertar()
{
  if(counter%2 == 0) //Verifica se o valor de counter é par
  {
    PORTC |= (1<<PC5); //Aciona o beep
  }
  else 
  {
    PORTC &= ~(1<<PC5); //Desativa o beep
  }
}

bool Restart()
{
  static bool rs = true;
  rs = PIND & (1<<PIND7);
  return rs;
}
//------Fim das funções auxiliares---------------------------
