/*
              **********************************************
                      PROJETO MINI OPALA LOWRIDER RC
                      ********COM ESP32***********
                           VERSÃO BLUETOOTH
                      CANAL ARDUINO PARA MODELISMO
              **********************************************


  Para conferir as Vídeo Aulas Gratuítas desse projeto, acesse:
  https://www.youtube.com/ArduinoParaModelismo

  Aprenda Arduino para modelismo: https://www.arduinoparamodelismo.com

   Atenção Siga atentamente as instruções das vídeos aulas e e-book para a montagem do hardware
   
   Playlist com todos os vídeos deste projeto:https://www.youtube.com/playlist?list=PLcWVEPpBZCFpa6DIqPusqHmlRZusrKLPk
   
  Desenvolvido por Aldeir de Souza Moreira - aldeirsm@gmail.com - https://www.youtube.com/ArduinoParaModelismo
  C 2024 - Todos os direitos reservados
*/

/* DÊ UM PASSO A MAIS...

🚀 Ele suas Miniaturas a um nível Extremo de automação e personalização com o Curso de Arduino para Modelismo 
Se Inscreva Aqui 👉 https://arduinoparamodelismo.com

✅ Aprenda a usar Arduino e ESP32 para dar Vida às suas Miniaturas,
Controlando tudo via controle remoto de uma forma simples e com baixo custo!

Você ainda vai contar com suporte via WhatsApp diretamente comigo, Aldeir Moreira!

💡 Realize seu sonho e ainda tenha a possibilidade de fazer uma renda extra, automatizando miniaturas por encomendas!  
Se Inscreva Aqui 👉 https://arduinoparamodelismo.com

*/


// 👉APOIE A PRODUÇÃO DE MAIS CONTEÚDOS COM ESTE: https://hotmart.com/pt-br/marketplace/produtos/clube-do-arduino-para-modelismo


// Importando Bibliotecas
#include "BluetoothSerial.h"  // Biblioteca para controle de conexões Bluetooth no ESP32.
#include <ESP32Servo.h>       // Biblioteca para controle de servomotores no ESP32.

// Criando um objeto para controlar a comunicação Bluetooth
BluetoothSerial SerialBT;  // Instância da classe BluetoothSerial para gerenciar a conexão Bluetooth.

// Variável para armazenar os dados recebidos via Bluetooth
char DadosRecebidos;  // Variável do tipo char que guardará os dados recebidos do celular.

// Criando objetos para controle dos servomotores
Servo servoDirecao;  // Objeto para controlar o servo responsável pela direção.
Servo servoAux1;     // Objeto para um servo auxiliar (pode ser usado em qualquer função do modelo).
Servo servoAux2;     // Objeto para outro servo auxiliar.
Servo servoAux3;     // Objeto para um terceiro servo auxiliar.

/////////////// Ajustes ///////////////

boolean AtivaFreio = 1;  // Variável para ativar ou desativar o freio ao usar ponte H (1 para ativar, 0 para desativar).

///////////////////////////////////////

// Definindo nomes para os pinos GPIO usados no código
#define FAROL 15
#define LUZ_FREIO 04
#define SETA_DIREITA 16
#define SETA_ESQUERDA 17
#define LUZ_RE 5

#define SERVO_DIRECAO 13
#define SERVO_AUX01 12
#define SERVO_AUX02 14
#define SERVO_AUX03 27

#define PONTE_H_IN01 18
#define PONTE_H_IN02 19

// Declarando variáveis globais para controle de várias funções no código
boolean AuxPiscaAlerta = 0;  // Variável auxiliar para o pisca alerta.

int AuxjoyEsquerdaY;  // Variável auxiliar para armazenar o valor do eixo Y do joystick esquerdo.
int AuxjoyEsquerdaX;  // Variável auxiliar para armazenar o valor do eixo X do joystick esquerdo.
int AuxjoyDireitaY;   // Variável auxiliar para armazenar o valor do eixo Y do joystick direito.
int AuxjoyDireitaX;   // Variável auxiliar para armazenar o valor do eixo X do joystick direito.

int joyDireitaX = 128;   // Valor inicial do eixo X do joystick direito (128 é o valor central).
int joyDireitaY = 128;   // Valor inicial do eixo Y do joystick direito.
int joyEsquerdaX = 128;  // Valor inicial do eixo X do joystick esquerdo.
int joyEsquerdaY = 128;  // Valor inicial do eixo Y do joystick esquerdo.

boolean Re = 0;         // Variável para indicar se o carro está em marcha ré.
boolean Frente = true;  // Variável para indicar se o carro está movendo para frente.
boolean freio = 0;      // Variável para indicar se o freio está ativado.

int PWM;  // Variável para controle de largura de pulso PWM.

int ServoTD = 90;         // Valor inicial para o servo do lado direito (90 graus é a posição central).
int ServoTE = 90;         // Valor inicial para o servo do lado esquerdo.
int ServoDianteiro = 90;  // Valor inicial para o servo da direção dianteira.
boolean BotaoSeta_D = 0;  // Variável para controlar o estado do botão da seta direita.
boolean BotaoSeta_E = 0;  // Variável para controlar o estado do botão da seta esquerda.
int Farol = 0;            // Variável para controlar o estado do farol.

uint32_t Millis_PiscaAlerta, Millis_SetaDireita, Millis_SetaEsquerda, Millis_Meio;  // Variáveis para contagem de tempo usando millis().

bool Painel = true;  // Variável para identificar se deve ou não ser enviado comando para montar o painel de controle no aplicativo

// Função de configuração inicial
void setup() {

  Serial.begin(115200);  // Inicializa a comunicação serial a 115200 bps (bits por segundo). Pode ser comentado para liberar a porta TX1.

  ////////////////////////////////////////////////
  SerialBT.begin("Opala LowRider");                      // Inicializa o Bluetooth com o nome "Opala LowRider".
  Serial.println("O dispositivo já pode ser pareado!");  // Exibe mensagem de que o dispositivo está pronto para pareamento.
  ////////////////////////////////////////////////

  // Configurando pinos GPIO para controlar a Ponte H (sentido de giro e velocidade do motor)
  ledcAttachPin(PONTE_H_IN01, 10);  // Associa o pino 18 (PONTE_H_IN01) ao canal 10 de PWM.
  ledcSetup(10, 500, 8);            // Configura o canal 10 de PWM com uma frequência de 500 Hz e resolução de 8 bits.

  ledcAttachPin(PONTE_H_IN02, 11);  // Associa o pino 19 (PONTE_H_IN02) ao canal 11 de PWM.
  ledcSetup(11, 500, 8);            // Configura o canal 11 de PWM com a mesma frequência e resolução.

  // Configurando pinos GPIO como saída para controle de LEDs
  pinMode(FAROL, OUTPUT);          // Define o pino 15 como saída para controle do farol.
  pinMode(LUZ_FREIO, OUTPUT);      // Define o pino 4 como saída para controle da luz de freio.
  pinMode(SETA_DIREITA, OUTPUT);   // Define o pino 16 como saída para controle da seta direita.
  pinMode(SETA_ESQUERDA, OUTPUT);  // Define o pino 17 como saída para controle da seta esquerda.
  pinMode(LUZ_RE, OUTPUT);         // Define o pino 5 como saída para controle da luz de ré.

  // Configurando servomotores
  servoDirecao.attach(SERVO_DIRECAO);  // Associa o servo da direção ao pino 13.
  servoAux1.attach(SERVO_AUX01);       // Associa o servo auxiliar 1 ao pino 12.
  servoAux2.attach(SERVO_AUX02);       // Associa o servo auxiliar 2 ao pino 14.
  servoAux3.attach(SERVO_AUX03);       // Associa o servo auxiliar 3 ao pino 27.

  // Definindo um valor inicial para os servomotores
  servoDirecao.write(90);  // Define a posição inicial do servo da direção em 90 graus.
  servoAux1.write(90);     // Define a posição inicial do servo auxiliar 1.
  servoAux2.write(90);     // Define a posição inicial do servo auxiliar 2.
  servoAux3.write(110);    // Define a posição inicial do servo auxiliar 3.

  // Configurando PWM para o LED da luz de freio
  ledcAttachPin(LUZ_FREIO, 13);  // Associa o pino 4 (LUZ_FREIO) ao canal 13 de PWM.
  ledcSetup(13, 500, 8);         // Configura o canal 13 de PWM com frequência de 500 Hz e resolução de 8 bits.
}




/* DÊ UM PASSO A MAIS...

🚀 Ele suas Miniaturas a um nível Extremo de automação e personalização com o Curso de Arduino para Modelismo 
Se Inscreva Aqui 👉 https://arduinoparamodelismo.com

✅ Aprenda a usar Arduino e ESP32 para dar Vida às suas Miniaturas,
Controlando tudo via controle remoto de uma forma simples e com baixo custo!

Você ainda vai contar com suporte via WhatsApp diretamente comigo, Aldeir Moreira!

💡 Realize seu sonho e ainda tenha a possibilidade de fazer uma renda extra, automatizando miniaturas por encomendas!  
Se Inscreva Aqui 👉 https://arduinoparamodelismo.com

*/



// FUNÇÃO PRINCIPAL DO CÓDIGO
void loop() {

  // Verifica se o módulo Bluetooth está conectado
  if (SerialBT.connected()) {

    // Se a variável Painel for verdadeira, chama a função GeraPainel()
    // e desliga os sinais das setas
    if (Painel == true) {
      GeraPainel();
      digitalWrite(SETA_DIREITA, LOW);
      digitalWrite(SETA_ESQUERDA, LOW);
    }
    Painel = false;  // Desativa o painel

    // Lê os dados recebidos da comunicação Bluetooth e armazena em DadosRecebidos
    DadosRecebidos = SerialBT.read();

    // CONTROLANDO O FAROL
    // Se o comando recebido for 'R', liga o farol e ajusta a intensidade da luz
    if (DadosRecebidos == 'R') {
      digitalWrite(FAROL, HIGH);  // Liga o farol
      ledcWrite(13, 100);         // Define a intensidade da lanterna (luz baixa)
      Serial.print("Liga FAROL");
      Farol = 1;  // Atualiza o status do farol
    }

    // Se o comando recebido for 'r', desliga o farol
    if (DadosRecebidos == 'r') {
      digitalWrite(FAROL, LOW);  // Desliga o farol
      ledcWrite(13, 0);          // Apaga a lanterna
      Serial.print("Desliga FAROL ");
      Farol = 0;  // Atualiza o status do farol
    }

    // CONTROLANDO O PISCA-ALERTA
    // Se o comando recebido for 'F', aciona o pisca-alerta
    if (DadosRecebidos == 'F') {
      AuxPiscaAlerta = HIGH;             // Define o estado do pisca-alerta como ativo
      digitalWrite(SETA_DIREITA, LOW);   // Desliga a seta direita
      digitalWrite(SETA_ESQUERDA, LOW);  // Desliga a seta esquerda
      Serial.print("ACIONA PISCA-ALERTA ");
      Serial.println(DadosRecebidos);
    }

    // Se o comando recebido for 'f', desaciona o pisca-alerta
    if (DadosRecebidos == 'f') {
      AuxPiscaAlerta = LOW;  // Define o estado do pisca-alerta como inativo
      Serial.print("DESACIONA PISCA-ALERTA ");
      Serial.println(DadosRecebidos);
      digitalWrite(SETA_DIREITA, LOW);  // Garante que ambas as setas estejam desligadas
      digitalWrite(SETA_ESQUERDA, LOW);
    }

    // Se o pisca-alerta estiver ativo, alterna o estado das setas a cada 375 milissegundos
    if (AuxPiscaAlerta == HIGH) {
      if (millis() - Millis_PiscaAlerta > 375) {
        digitalWrite(SETA_DIREITA, !digitalRead(SETA_DIREITA));
        digitalWrite(SETA_ESQUERDA, !digitalRead(SETA_ESQUERDA));
        Millis_PiscaAlerta = millis();  // Atualiza o tempo do último pisca-alerta
        BotaoSeta_E = 0;                // Garante que o botão da seta esquerda está desativado
        BotaoSeta_D = 0;                // Garante que o botão da seta direita está desativado
      }
    } else {

      // CONTROLANDO AS SETAS DIRECIONAIS
      // Se o comando recebido for 'V', alterna o estado da seta direita
      if (DadosRecebidos == 'V') {
        BotaoSeta_D = !BotaoSeta_D;      // Alterna o estado do botão da seta direita
        BotaoSeta_E = 0;                 // Garante que o botão da seta esquerda está desativado
        digitalWrite(SETA_DIREITA, 0);   // Desliga a seta direita
        digitalWrite(SETA_ESQUERDA, 0);  // Desliga a seta esquerda
      }

      // Se o botão da seta direita estiver ativado, alterna o estado da seta a cada 375 milissegundos
      if (BotaoSeta_D == 1) {
        if (millis() - Millis_SetaDireita > 375) {
          digitalWrite(SETA_DIREITA, !digitalRead(SETA_DIREITA));
          Millis_SetaDireita = millis();  // Atualiza o tempo do último acionamento da seta direita
        }
      }

      // Se o comando recebido for 'X', alterna o estado da seta esquerda
      if (DadosRecebidos == 'X') {
        BotaoSeta_E = !BotaoSeta_E;      // Alterna o estado do botão da seta esquerda
        BotaoSeta_D = 0;                 // Garante que o botão da seta direita está desativado
        digitalWrite(SETA_DIREITA, 0);   // Desliga a seta direita
        digitalWrite(SETA_ESQUERDA, 0);  // Desliga a seta esquerda
      }

      // Se o botão da seta esquerda estiver ativado, alterna o estado da seta a cada 375 milissegundos
      if (BotaoSeta_E == 1) {
        if (millis() - Millis_SetaEsquerda > 375) {
          digitalWrite(SETA_ESQUERDA, !digitalRead(SETA_ESQUERDA));
          Millis_SetaEsquerda = millis();  // Atualiza o tempo do último acionamento da seta esquerda
        }
      }
    }

    // CONTROLANDO OS SERVOS DA SUSPENSÃO
    // Se o comando recebido for 'T', ajusta o servo da suspensão traseira direita
    if (DadosRecebidos == 'T') {
      ServoTE = SerialBT.parseInt();                  // Lê o valor do servo da suspensão traseira direita
      servoAux1.write(map(ServoTE, 0, 180, 0, 180));  // Mapeia e ajusta a posição do servo
    }

    // Se o comando recebido for 'U', ajusta o servo da suspensão traseira esquerda
    if (DadosRecebidos == 'U') {
      ServoTD = SerialBT.parseInt();                  // Lê o valor do servo da suspensão traseira esquerda
      servoAux2.write(map(ServoTD, 0, 180, 180, 0));  // Mapeia e ajusta a posição do servo
    }

    // Se o comando recebido for 'W', ajusta o servo da suspensão dianteira
    if (DadosRecebidos == 'W') {
      ServoDianteiro = SerialBT.parseInt();                      // Lê o valor do servo da suspensão dianteira
      servoAux3.write(map(ServoDianteiro, 100, 180, 100, 180));  // Mapeia e ajusta a posição do servo
    }

    Serial.print("ServoDianteiro: ");  // Exibe o valor do servo dianteiro no monitor serial
    Serial.println(ServoDianteiro);

    // TRATANDO DADOS DOS JOYSTICKS ANALÓGICOS
    // Se o comando recebido for 'A', lê os valores dos joysticks para aceleração
    if (DadosRecebidos == 'A') {
      joyEsquerdaX = SerialBT.parseInt();  // Lê o valor do eixo X do joystick esquerdo
      // Espera até que o próximo comando seja 'B' para ler o valor do eixo Y
      while (DadosRecebidos != 'B') {
        if (SerialBT.available()) {
          DadosRecebidos = SerialBT.read();
          if (DadosRecebidos == 'Y') {
            joyEsquerdaY = SerialBT.parseInt();  // Lê o valor do eixo Y do joystick esquerdo
          }
        }
      }
      // Serial.print("joyEsquerdaX: "); // Comentado: Exibe valor do eixo X
      // Serial.println(joyEsquerdaX);
      // Serial.print("joyEsquerdaY: "); // Comentado: Exibe valor do eixo Y
      // Serial.println(joyEsquerdaY);
    }

    // Se o comando recebido for 'C', lê os valores dos joysticks para direção
    if (DadosRecebidos == 'C') {
      joyDireitaX = SerialBT.parseInt();  // Lê o valor do eixo X do joystick direito
      // Espera até que o próximo comando seja 'D' para ler o valor do eixo Y
      while (DadosRecebidos != 'D') {
        if (SerialBT.available()) {
          DadosRecebidos = SerialBT.read();
          if (DadosRecebidos == 'Y') {
            joyDireitaY = SerialBT.parseInt();  // Lê o valor do eixo Y do joystick direito
          }
        }
      }
      // Serial.print("joyDireitaX: "); // Comentado: Exibe valor do eixo X
      // Serial.println(joyDireitaX);
      // Serial.print("joyDireitaY: "); // Comentado: Exibe valor do eixo Y
      // Serial.println(joyDireitaY);
    }

    // A SEQUÊNCIA DE IFs ABAIXO É NECESSÁRIA PARA DEIXAR OS VALORES RECEBIDOS DOS POTENCIÔMETROS VIRTUAIS LINEARES
    // Ajusta os valores dos joysticks para que se movam suavemente

    // Ajusta o valor do eixo X do joystick esquerdo suavemente
    if (AuxjoyEsquerdaX < joyEsquerdaX) {
      AuxjoyEsquerdaX++;
      if (AuxjoyEsquerdaX > joyEsquerdaX) {
        AuxjoyEsquerdaX = joyEsquerdaX;
      }
    }

    if (AuxjoyEsquerdaX > joyEsquerdaX) {
      AuxjoyEsquerdaX--;
      if (AuxjoyEsquerdaX < joyEsquerdaX) {
        AuxjoyEsquerdaX = joyEsquerdaX;
      }
    }

    // Serial.print("JOYSTICK EX: ");
    // Serial.println(AuxjoyEsquerdaX);

    // Ajusta o valor do eixo Y do joystick esquerdo suavemente
    if (AuxjoyEsquerdaY < joyEsquerdaY) {
      AuxjoyEsquerdaY++;
      if (AuxjoyEsquerdaY > joyEsquerdaY) {
        AuxjoyEsquerdaY = joyEsquerdaY;
      }
    }

    if (AuxjoyEsquerdaY > joyEsquerdaY) {
      AuxjoyEsquerdaY--;
      if (AuxjoyEsquerdaY < joyEsquerdaY) {
        AuxjoyEsquerdaY = joyEsquerdaY;
      }
    }

    // Serial.print("*****JOYSTICK EY: ");
    // Serial.println(AuxjoyEsquerdaY);

    // Ajusta o valor do eixo X do joystick direito suavemente
    if (AuxjoyDireitaX < joyDireitaX) {
      AuxjoyDireitaX++;
      if (AuxjoyDireitaX > joyDireitaX) {
        AuxjoyDireitaX = joyDireitaX;
      }
    }

    if (AuxjoyDireitaX > joyDireitaX) {
      AuxjoyDireitaX--;
      if (AuxjoyDireitaX < joyDireitaX) {
        AuxjoyDireitaX = joyDireitaX;
      }
    }

    // Serial.print("JOYSTICK DX: ");
    // Serial.println(AuxjoyDireitaX);

    // Ajusta o valor do eixo Y do joystick direito suavemente
    if (AuxjoyDireitaY < joyDireitaY) {
      AuxjoyDireitaY++;
      if (AuxjoyDireitaY > joyDireitaY) {
        AuxjoyDireitaY = joyDireitaY;
      }
    }

    if (AuxjoyDireitaY > joyDireitaY) {
      AuxjoyDireitaY--;
      if (AuxjoyDireitaY < joyDireitaY) {
        AuxjoyDireitaY = joyDireitaY;
      }
    }

    // Serial.print("JOYSTICK DY: ");
    // Serial.println(AuxjoyDireitaY);






    ////////////////////////////////////////////////////////




    // ACELERAÇÃO FRENTE RÉ E FREIO

    // *** FRENTE
    if (AuxjoyEsquerdaY <= 120) {

      // TRATANDO PONTE H
      // Mapeia o valor do joystick para ajustar a intensidade do PWM (Controle de Velocidade do Motor)
      PWM = map(AuxjoyEsquerdaY, 120, 0, 0, 255);
      Serial.print("PWM: ");
      Serial.println(PWM);

      // Verifica se o freio está ativado
      if (freio == false) {
        // Desliga o motor de freio e aciona o motor para frente
        ledcWrite(10, 0);    // Desliga motor de freio
        ledcWrite(11, PWM);  // Liga motor para frente com intensidade PWM ajustada
        // ledcWrite(11, map(AuxjoyEsquerdaY, 120, 25, 0, 255)); // Alternativa para ajustar PWM
        // Serial.println(" FRENTE: ");

        // Controla a luz de ré e a luz de freio
        digitalWrite(5, LOW);  // Apaga a luz de ré
        if (Farol == 0) {
          ledcWrite(13, 0);  // Apaga a luz de freio
        } else {
          ledcWrite(13, 100);  // Define a intensidade da luz para Lanterna
        }
        Millis_Meio = millis();  // Atualiza o tempo de referência para o controle de tempo
        Re = false;              // Define o joystick para cima (marcha frente)
        Frente = true;           // Marca que o veículo está em movimento para frente
      }

      // Se o freio está ativado e o joystick está na posição para frente, aciona o freio
      if ((freio == true) && (Re == true) && (AtivaFreio == true)) {
        Serial.println("FREIO ACIONADO");
        ledcWrite(10, 255);  // Aciona o motor de freio com máxima intensidade
        ledcWrite(11, 255);  // Aciona o motor para frente com máxima intensidade
        ledcWrite(13, 255);  // Acende a luz de freio com máxima intensidade
      } else {
        freio = false;  // Desativa o freio
      }
    }

    // *** PARADO - BASTÃO DO JOYSTICK NO MEIO
    if ((AuxjoyEsquerdaY < 135) && (AuxjoyEsquerdaY > 120)) {

      // TRATANDO PONTE H
      ledcWrite(10, LOW);  // Desliga motor de freio
      ledcWrite(11, LOW);  // Desliga motor para frente

      // Controla a luz de ré e a luz de freio
      digitalWrite(5, LOW);  // Apaga a luz de ré
      if (Farol == 0) {
        ledcWrite(13, 0);  // Apaga a luz de freio
      } else {
        ledcWrite(13, 100);  // Define a intensidade da luz para Lanterna
      }

      // Se o joystick está na posição de repouso por menos de 500 ms, ativa o freio
      if (millis() - Millis_Meio < 500) {
        Serial.println("Sem Aceleracao");
        freio = true;  // Ativa o freio
      } else {
        if (Farol == 0) {
          ledcWrite(13, 0);  // Apaga a luz de freio
        } else {
          ledcWrite(13, 100);  // Define a intensidade da luz para Lanterna
        }
        freio = false;  // Desativa o freio
      }
    }

    // *** RÉ
    if (AuxjoyEsquerdaY >= 135) {

      // TRATANDO PONTE H
      // Mapeia o valor do joystick para ajustar a intensidade do PWM para ré
      PWM = map(AuxjoyEsquerdaY, 135, 255, 0, 255);
      Serial.print("PWM: ");
      Serial.println(PWM);

      // Verifica se o freio está ativado
      if (freio == false) {
        // Aciona o motor para ré e desliga o motor de frente
        ledcWrite(10, PWM);  // Liga motor para ré com intensidade PWM ajustada
        ledcWrite(11, LOW);  // Desliga motor de frente
        Serial.println(" RE ");

        // Controla a luz de ré e a luz de freio
        if (Farol == 0) {
          ledcWrite(13, 0);  // Apaga a luz de freio
        } else {
          ledcWrite(13, 100);  // Define a intensidade da luz para Lanterna
        }
        digitalWrite(5, HIGH);   // Acende a luz de ré
        Millis_Meio = millis();  // Atualiza o tempo de referência para o controle de tempo
        Re = true;               // Define o joystick para baixo (marcha ré)
        Frente = false;          // Marca que o veículo está em movimento para ré
      }

      // Se o freio está ativado e o joystick está na posição para ré, aciona o freio
      if ((freio == true) && (Frente == true) && (AtivaFreio == true)) {
        Serial.println("FREIO ACIONADO");
        ledcWrite(10, 255);  // Aciona o motor de freio com máxima intensidade
        ledcWrite(11, 255);  // Aciona o motor de frente com máxima intensidade
        ledcWrite(13, 255);  // Acende a luz de freio com máxima intensidade
      } else {
        freio = false;  // Desativa o freio
      }
    }

    // *** DIREÇÃO
    int GrauServo = 0;  // Variável para armazenar a posição do servo de direção

    // Converte o valor do joystick direito para um valor compatível com o servo de direção
    GrauServo = map(AuxjoyDireitaX, 0, 255, 0, 180);  // Movimentação total do servo (180 graus)
    // GrauServo = map(AuxjoyDireitaX, 0, 255, 30, 150); // Movimentação limitada em 120 graus
    // GrauServo = map(AuxjoyDireitaX, 0, 255, 50, 130); // Movimentação limitada em 80 graus
    // GrauServo = map(AuxjoyDireitaX, 0, 255, 70, 110); // Movimentação limitada em 40 graus

    // Para inverter o movimento do servo, altere os valores para 180, 0
    // GrauServo = map(AuxjoyDireitaX, 0, 255, 180, 0);  // Inverte o movimento do servo

    servoDirecao.write(GrauServo);  // Define a posição do servo de direção

    // Mostra os valores de comando do servo de direção no monitor serial apenas quando o joystick é movimentado
    if ((AuxjoyDireitaX > 130) || (AuxjoyDireitaX < 125)) {
      Serial.print(" Servo: ");
      Serial.println(GrauServo);
    }
  } else {

    Serial.println("CONTROLE DESCONECTADO! ");

    // Se o controle Bluetooth estiver desconectado, alterna as setas a cada 375 milissegundos
    if (millis() - Millis_PiscaAlerta > 375) {
      digitalWrite(SETA_DIREITA, !digitalRead(SETA_DIREITA));
      digitalWrite(SETA_ESQUERDA, !digitalRead(SETA_ESQUERDA));
      Millis_PiscaAlerta = millis();  // Atualiza o tempo de referência para o pisca-alerta
      BotaoSeta_E = 0;                // Garante que o botão da seta esquerda está desativado
      BotaoSeta_D = 0;                // Garante que o botão da seta direita está desativado
    }

    // Apaga todas as luzes
    digitalWrite(FAROL, LOW);  // Desliga o farol
    ledcWrite(13, LOW);        // Apaga a luz de freio
    ledcWrite(10, LOW);        // Desliga o motor de freio
    ledcWrite(11, LOW);        // Desliga o motor para frente

    // Define os servos em uma posição neutra
    servoDirecao.write(90);  // Posição central do servo de direção
    servoAux1.write(90);     // Posição central do servo da suspensão traseira direita
    servoAux2.write(90);     // Posição central do servo da suspensão traseira esquerda
    servoAux3.write(110);    // Posição central do servo da suspensão dianteira

    // Reseta as variáveis
    DadosRecebidos = 9;
    AuxjoyEsquerdaY = 127;
    joyEsquerdaY = 127;

    Painel = true;  // Ativa o painel
  }
}
/* DÊ UM PASSO A MAIS...

🚀 Ele suas Miniaturas a um nível Extremo de automação e personalização com o Curso de Arduino para Modelismo 
Se Inscreva Aqui 👉 https://arduinoparamodelismo.com

✅ Aprenda a usar Arduino e ESP32 para dar Vida às suas Miniaturas,
Controlando tudo via controle remoto de uma forma simples e com baixo custo!

Você ainda vai contar com suporte via WhatsApp diretamente comigo, Aldeir Moreira!

💡 Realize seu sonho e ainda tenha a possibilidade de fazer uma renda extra, automatizando miniaturas por encomendas!  
Se Inscreva Aqui 👉 https://arduinoparamodelismo.com
*/

// Função responsável por gerar o painel no APP BluetoothElectronics
void GeraPainel() {
  delay(200);
  SerialBT.println("*.kwl");
  SerialBT.println("select_panel(4)");
  SerialBT.println("clear_panel(4)");
  SerialBT.println("set_grid_size(12,6)");
  SerialBT.println("add_text(6,5,large,C,www.ArduinoParaModelismo.com,0,183,255,)");
  SerialBT.println("add_text(6,2,xlarge,C,Arduino Para Modelismo ,0,199,255,)");
  SerialBT.println("add_text(6,3,xlarge,C,Opala LowRider,0,179,255,)");
  SerialBT.println("add_text(0,1,small,L,Servo 01,255,216,0,)");
  SerialBT.println("add_text(1,1,small,L,Servo02,255,201,0,)");
  SerialBT.println("add_text(11,1,small,L,Servo 03,255,215,0,)");
  SerialBT.println("add_text(5,4,medium,C,Acelerador      ,255,255,255,)");
  SerialBT.println("add_text(7,4,medium,L,     Volante,255,255,255,)");
  SerialBT.println("add_text(4,1,medium,L,Pisca-Alerta,0,255,0,)");
  SerialBT.println("add_text(7,1,medium,C,                Farol,0,255,0,)");
  SerialBT.println("add_text(10,0,large,R,SetaD,0,255,0,)");
  SerialBT.println("add_text(1,0,large,L,SetaE,0,255,0,)");
  SerialBT.println("add_button(11,0,5,X,x)");
  SerialBT.println("add_button(0,0,4,V,v)");
  SerialBT.println("add_switch(7,0,1,R,r,0,0)");
  SerialBT.println("add_switch(4,0,1,F,f,0,0)");
  SerialBT.println("add_slider(11,2,4,100,180,100,W,Z,0)");
  SerialBT.println("add_slider(1,2,4,0,180,90,U,Z,0)");
  SerialBT.println("add_slider(0,2,4,0,180,90,T,Z,0)");
  SerialBT.println("add_free_pad(8,3,0,255,0,100,C,D)");
  SerialBT.println("add_free_pad(2,3,0,255,0,50,A,B)");
  SerialBT.println("set_panel_notes(-,,,)");
  SerialBT.println("run()");
  SerialBT.println("*");
}

/* DÊ UM PASSO A MAIS...

🚀 Ele suas Miniaturas a um nível Extremo de automação e personalização com o Curso de Arduino para Modelismo 
Se Inscreva Aqui 👉 https://arduinoparamodelismo.com

✅ Aprenda a usar Arduino e ESP32 para dar Vida às suas Miniaturas,
Controlando tudo via controle remoto de uma forma simples e com baixo custo!

Você ainda vai contar com suporte via WhatsApp diretamente comigo, Aldeir Moreira!

💡 Realize seu sonho e ainda tenha a possibilidade de fazer uma renda extra, automatizando miniaturas por encomendas!  
Se Inscreva Aqui 👉 https://arduinoparamodelismo.com

*/
