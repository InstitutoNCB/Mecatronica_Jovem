void processarComando(const String& cmd) 
{
  String c = cmd;
  c.trim();
  c.toUpperCase();

  if (c == "A") 
  {
    Serial.println("A");
    pos = MotorGame(30, PPV, 1600, 0);
    BarraLeds(pos);
  }
  else if (c == "B") 
  {
    Serial.println("B");
    pos = MotorGame(30, PPV, 1600, 1);
    BarraLeds(pos);
  }
  else 
  {
    Serial.println("COMANDO ERRADO");
  }
}

void LimparSerial()
{
  while (Serial.available()) 
  {
    Serial.read();
  }
}