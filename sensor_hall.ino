// código do projeto caça fantasmas do Tio Rafa, publicado na revista Mecatrônica Jovem - Projetos do Prof. Newton C. Braga

#define ky024A A0 // Definindo o pino A0 como ky024
#define ky024D 9 // Definindo o pino 9 como ky024
#define ledA 13 // Definindo o pino 13 como ledA
#define buzzer 7 // Definindo o pino 7 como buzzer

int valorAnalogico = 0; // Variável responsável por armazenar o valor analógico do KY-024
int valorDigital = 0; // Variável responsável por armazenar o valor digital do KY-024
int novoValorAnalogico = 0; // Variável responsável por armazenar o valor do sinal analógico após mapeá-lo

void setup() {
  Serial.begin(9600); // Inicializando o Monitor Serial
  pinMode(ky024D, INPUT); // Definindo o pino denominado "ky024D" como entrada
  pinMode(ledD, OUTPUT); // Definindo o pino denominado "ledD" como saída
  pinMode(buzzer, OUTPUT); // Definindo o pino do buzzer como saída
}

void loop() {
  valorDigital = digitalRead(ky024D); // Realiza a leitura do pino denominado "ky024D" e armazena na variável "valorDigital"
  valorAnalogico = analogRead(ky024A); // Realiza a leitura do pino denominado "ky024A" e armazena na variável "valorAnalogico"

  if (valorDigital == HIGH) { // Compara se o valor da variável "valorDigital" está em nível alto
    digitalWrite(ledD, HIGH); // Se sim, envia um sinal de nível alto para o pino denominado "ledD"
    digitalWrite(buzzer, HIGH); // Se sim, aciona o buzzer
  } else {
    digitalWrite(ledD, LOW); // Se não, envia um sinal de nível baixo para o pino denominado "ledD"
    digitalWrite(buzzer, LOW); // Se não, desliga o buzzer
  } 

  novoValorAnalogico = map(valorAnalogico, 390, 820, 0, 1023); // Função MAP para converter um intervalo numérico em outro

  Serial.print(valorAnalogico); // Imprime no monitor serial o valor armazenado na variável "valorAnalogico"
  Serial.print(","); // Imprime
  Serial.println(novoValorAnalogico); // Imprime no monitor serial o valor armazenado na variável "novoValorAnalogico"

  analogWrite(ledA, novoValorAnalogico); // Se sim, envia um sinal PWM para o pino denominado "ledA"

  delay(100); // Para o código por 100 milissegundos
}
