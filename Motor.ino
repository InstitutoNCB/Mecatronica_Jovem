void MotorPosition(int ang, int ppv, unsigned long t, bool dir)
{
  int passos = ang * ppv / 360;
  LigaDesligaMotor(1);
  if(dir)
  {
    //Sentido horário
    PORTB &= ~PIN_DIR;
  }
  else 
  {
    //Sentido anti horário
    PORTB |= PIN_DIR;
  }

  for(int i = 0; i <= passos; i++)
  {
    PORTB |= PIN_STEP;
    delayMicroseconds(t);
    PORTB &= ~PIN_STEP;
    delayMicroseconds(t);
  }
}

int MotorGame(int ang, int ppv, unsigned long t, bool dir)
{
  static int lado_A = 0;
  int passos = ang * ppv / 360;

  if(dir)
  {
    //Sentido horário
    PORTB &= ~PIN_DIR;
    lado_A++;
  }
  else 
  {
    //Sentido anti horário
    PORTB |= PIN_DIR;
    lado_A--;
  }

  //Gira o motor para a posição
  LigaDesligaMotor(1);
  for(int i = 0; i <= passos; i++)
  {
    PORTB |= PIN_STEP;
    delayMicroseconds(t);
    PORTB &= ~PIN_STEP;
    delayMicroseconds(t);
  }
  
  if(lado_A > 3)
  {
    PORTD |= LD_A;

    btnIniciar = PIND & BTN_INIT;

    delay(1000);
    EfeitoLeds();
    while(!btnIniciar)
    {
      //Realiza a leitura do botão iniciar
      btnIniciar = PIND & BTN_INIT;

      //Desliga o motor para que possibilite o movimento manual para o ponto de início.
      LigaDesligaMotor(0);
    }
    PORTD &= ~LD_A;
    //Azul
    PORTB |= LED_B;  //Azul
    PORTB &= ~LED_G; //Verde
    PORTB &= ~LED_R; //Vermelho
    pos = 0;
    lado_A = 0;
    LimparSerial();
  }
  else if(lado_A < -3)
  {
    PORTD |= LD_B;

    btnIniciar = PIND & BTN_INIT;

    delay(1000);
    EfeitoLeds();
    //Posiciona o motor a 180° no sentido anti horário
    MotorPosition(180, PPV, 1600, 1);
    delay(500);
    while(!btnIniciar)
    {
      //Realiza a leitura do botao iniciar
      btnIniciar = PIND & BTN_INIT;

      //Desliga o motor para que possibilite o movimento manual para o ponto de início.
      LigaDesligaMotor(0);
    }
    PORTD &= ~LD_B;
    //Azul
    PORTB |= LED_B;  //Azul
    PORTB &= ~LED_G; //Verde
    PORTB &= ~LED_R; //Vermelho
    pos = 0;
    lado_A = 0;
    LimparSerial();
  }
  
  Serial.println(lado_A);
  return lado_A;
}

void MotorRun(unsigned long t, bool dir)
{
  LigaDesligaMotor(1);
  if(dir)
  {
    //Sentido horário
    PORTB &= ~PIN_DIR;
  }
  else 
  {
    //Sentido anti horário
    PORTB |= PIN_DIR;
  }
  //Move o motor com a velocidade determinada pelo tempo
  PORTB |= PIN_STEP;
  delayMicroseconds(t);
  PORTB &= ~PIN_STEP;
  delayMicroseconds(t);
}

void DirectionMotor(bool dir)
{
  if(dir)
  {
    //Sentido horário
    PORTB &= ~PIN_DIR;
  }
  else 
  {
    //Sentido anti horário
    PORTB |= PIN_DIR;
  }
}
 void LigaDesligaMotor(bool en)
 {
  if(en)
  {
    //1 Liga motor
    PORTB &= ~PIN_ENAB;
  }
  else 
  {
    //0 Desliga motor
    PORTB |= PIN_ENAB;
  }
    
 }
