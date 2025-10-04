void BarraLeds(int pos)
{
  if(pos == 0)
  {
    //Vermelho
    PORTB |= LED_R;  //Vermelho
    PORTB &= ~LED_G; //Verde
    PORTB &= ~LED_B; //Azul
  }
  else if(pos == 1 || pos == -1)
  {
    //Verde
    PORTB &= ~LED_R;
    PORTB |= LED_G; 
    PORTB &= ~LED_B;
  }
  else if(pos == 2 || pos == -2)
  {
    //Amarelo
    PORTB |= LED_R;
    PORTB |= LED_G;
    PORTB &= ~LED_B;
  }
  else if(pos == 3 || pos == -3)
  {
    //Azul
    PORTB &= ~LED_R;
    PORTB &= ~LED_G;
    PORTB |= LED_B;
  }
  else if(pos == 4)
  {
    //Apaga tudo
    PORTB &= ~LED_R;
    PORTB &= ~LED_G;
    PORTB &= ~LED_B;
  } 
}

void EfeitoLeds()
{
  for(int i = 0; i < 5; i++)
  {
    //Azul
    PORTB &= ~LED_R;
    PORTB &= ~LED_G;
    PORTB |= LED_B; 
    delay(100);
    //Roxo
    PORTB |= LED_R;
    PORTB &= ~LED_G;
    PORTB |= LED_B;  
    delay(100);
    //Verde
    PORTB &= ~LED_R;
    PORTB |= LED_G;
    PORTB &= ~LED_B;
    delay(100);
    //Amarelo
    PORTB |= LED_R;
    PORTB |= LED_G;
    PORTB &= ~LED_B;
    delay(100);
    //Vermelho
    PORTB |= LED_R;
    PORTB &= ~LED_G;
    PORTB &= ~LED_B;
  }
}