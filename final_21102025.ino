#include <FastLED.h>

#define LEDS_P1_PIN 6     // Jogador 1
#define LEDS_P2_PIN 7     // Jogador 2
#define BTN_P1_PIN 2      
#define BTN_P2_PIN 3      
#define BTN_RESET_PIN 4   

#define LED_COUNT 64
#define BOARD_SIZE 8

CRGB ledsP1[LED_COUNT];
CRGB ledsP2[LED_COUNT];

// Cores 
#define COLOR_HIT         CRGB::Red      
#define COLOR_MISS        CRGB::Blue
#define COLOR_CLEAR       CRGB::Black

// CORES PARA O TABULEIRO
#define COLOR_SHIP        CRGB(0, 50, 0)  
#define COLOR_DAMAGE_OWN  CRGB::Yellow    
#define COLOR_CURSOR      CRGB::White     

// Vitória e Derrota
#define COLOR_WIN_BG   CRGB::Green   
#define COLOR_LOSE_BG  CRGB::Red       


// VARIÁVEIS 

byte boardP1[BOARD_SIZE][BOARD_SIZE];
byte boardP2[BOARD_SIZE][BOARD_SIZE];

bool player1Turn = true;
bool gameOver = false;
int shipsP1 = 15; // 2x1 + 5x2 + 1x3 = 15
int shipsP2 = 15; // 2x1 + 5x2 + 1x3 = 15

long lastButtonTime = 0;
const long debounceDelay = 200; 

const int SHIP_SIZES[] = {1, 1, 2, 2, 2, 2, 2, 3}; // 8 Navios (2x1, 5x2, 1x3)


// VARIÁVEIS DE CONTROLE DO CURSOR (Máquina de estados removida)

int currentX = 0;
int currentY = 0;
const int CURSOR_DELAY = 100; 
long lastCursorMove = 0; 



// FUNÇÕES AUXILIARES

int xyToIndex(int x, int y) {
  if (y % 2 == 0) {
    return y * BOARD_SIZE + x;
  } else {
    return y * BOARD_SIZE + (BOARD_SIZE - 1 - x);
  }
}


// FUNÇÃO DE POSICIONAMENTO DE NAVIOS

bool isPlacementValid(byte board[BOARD_SIZE][BOARD_SIZE], int x, int y, int size, bool horizontal) {
  for (int i = 0; i < size; i++) {
    int checkX = x + (horizontal ? i : 0);
    int checkY = y + (horizontal ? 0 : i);
    if (checkX >= BOARD_SIZE || checkY >= BOARD_SIZE) return false;
    if (board[checkY][checkX] == 3) return false; 
  }

  for (int i = 0; i < size; i++) {
    int shipX = x + (horizontal ? i : 0);
    int shipY = y + (horizontal ? 0 : i);
    
    for (int dy = -1; dy <= 1; dy++) {
      for (int dx = -1; dx <= 1; dx++) {
        int neighborX = shipX + dx;
        int neighborY = shipY + dy;

        if (neighborX >= 0 && neighborX < BOARD_SIZE && 
            neighborY >= 0 && neighborY < BOARD_SIZE) {
            
            if (board[neighborY][neighborX] == 3) {
                bool isCurrentSegment = (neighborX == shipX && neighborY == shipY);
                if (!isCurrentSegment) return false;
            }
        }
      }
    }
  }
  return true; 
}

void placeShips(byte board[BOARD_SIZE][BOARD_SIZE], const int *shipSizes, size_t numShips) {
  for (int i = 0; i < BOARD_SIZE; i++)
    for (int j = 0; j < BOARD_SIZE; j++)
      board[i][j] = 0; 
  
  for (size_t i = 0; i < numShips; i++) {
    int size = shipSizes[i];
    bool placed = false;
    int attempts = 0;
    
    while (!placed && attempts < 10000) { 
      int x = random(0, BOARD_SIZE);
      int y = random(0, BOARD_SIZE);
      bool horizontal = random(0, 2) == 0; 

      if (isPlacementValid(board, x, y, size, horizontal)) {
        for (int j = 0; j < size; j++) {
          int placeX = x + (horizontal ? j : 0);
          int placeY = y + (horizontal ? 0 : j);
          board[placeY][placeX] = 3; 
        }
        placed = true;
      }
      attempts++;
    }

    if (attempts >= 10000) {
      Serial.println("ERRO: Nao foi possivel colocar todos os navios separados. Reiniciando...");
      i = -1; 
      fill_solid(ledsP1, LED_COUNT, CRGB::Red); FastLED.show(); delay(500); 
      fill_solid(ledsP1, LED_COUNT, COLOR_CLEAR); FastLED.show();
      for (int r = 0; r < BOARD_SIZE; r++)
          for (int c = 0; c < BOARD_SIZE; c++)
              board[r][c] = 0;
    }
  }
}


// FUNÇÕES DE DESENHO 

void drawRadar(CRGB *leds, byte targetBoard[BOARD_SIZE][BOARD_SIZE]) {
  for (int y = 0; y < BOARD_SIZE; y++) {
    for (int x = 0; x < BOARD_SIZE; x++) {
      int index = xyToIndex(x, y);
      if (targetBoard[y][x] == 1) leds[index] = COLOR_HIT;      
      else if (targetBoard[y][x] == 2) leds[index] = COLOR_MISS; 
      else leds[index] = COLOR_CLEAR; 
    }
  }
}


// FUNÇÕES DE JOGO

void fireShot(byte target[BOARD_SIZE][BOARD_SIZE], int &ships, int player, int x, int y) {
  
  Serial.print("P"); Serial.print(player);
  Serial.print(" atirou em ("); Serial.print(x); Serial.print(", "); Serial.print(y); Serial.print(") -> ");
  
  CRGB* activeLeds = player ? ledsP1 : ledsP2;
  
  // Verifica se já atirou nessa posição
  if (target[y][x] == 1 || target[y][x] == 2) {
      Serial.println("ERRO: Tiro ja foi disparado nesta coordenada.");
      return; 
  }

  // Tenta acertar o navio
  if (target[y][x] == 3) {  
    target[y][x] = 1; // Navio atingido
    ships--; 
    Serial.println("ACERTOU UM NAVIO! Pontos restantes: " + String(ships));
    activeLeds[xyToIndex(x,y)] = COLOR_HIT; // Mostra o acerto temporariamente
  } else { 
    target[y][x] = 2; // Água atingida
    Serial.println("Água!");
    activeLeds[xyToIndex(x,y)] = COLOR_MISS; // Mostra o erro temporariamente
  }
  
  // Exibe o resultado do tiro por um momento
  FastLED.show();
  delay(500); // 0.5 segundo para ver o resultado

  // Passa a vez e reseta o cursor
  player1Turn = !player1Turn;
  currentX = 0; 
  currentY = 0;
}

void resetGame() {
  fill_solid(ledsP1, LED_COUNT, COLOR_CLEAR);
  fill_solid(ledsP2, LED_COUNT, COLOR_CLEAR);
  FastLED.show();

  shipsP1 = 15; 
  shipsP2 = 15; 
  gameOver = false;
  player1Turn = true;
  currentX = 0; 
  currentY = 0;
  
  size_t numShips = sizeof(SHIP_SIZES) / sizeof(SHIP_SIZES[0]);
  placeShips(boardP1, SHIP_SIZES, numShips);
  placeShips(boardP2, SHIP_SIZES, numShips);
  Serial.println("=== Novo jogo iniciado (Pontos: P1=" + String(shipsP1) + ", P2=" + String(shipsP2) + ") ===");
}



// SETUP E LOOP

void setup() {
  Serial.begin(9600);
  randomSeed(analogRead(A0));

  FastLED.addLeds<WS2812B, LEDS_P1_PIN, GRB>(ledsP1, LED_COUNT);
  FastLED.addLeds<WS2812B, LEDS_P2_PIN, GRB>(ledsP2, LED_COUNT);
  
  FastLED.setBrightness(8); 

  pinMode(BTN_P1_PIN, INPUT_PULLUP);
  pinMode(BTN_P2_PIN, INPUT_PULLUP);
  pinMode(BTN_RESET_PIN, INPUT_PULLUP);

  resetGame();
}

void loop() {
  if (digitalRead(BTN_RESET_PIN) == LOW) {
    resetGame();
    delay(300); 
  }

  if (gameOver) return;

  CRGB* activeLeds = player1Turn ? ledsP1 : ledsP2;
  byte (*targetBoard)[BOARD_SIZE] = player1Turn ? boardP2 : boardP1;


  if (millis() - lastCursorMove >= CURSOR_DELAY) {
    
    currentX++;
    if (currentX >= BOARD_SIZE) {
      currentX = 0;
      currentY++;
      if (currentY >= BOARD_SIZE) {
        currentY = 0; 
      }
    }
    lastCursorMove = millis();
  }

   if (millis() - lastButtonTime > debounceDelay) {
    
    if (player1Turn && digitalRead(BTN_P1_PIN) == LOW) {
      fireShot(boardP2, shipsP2, 1, currentX, currentY);
      lastButtonTime = millis();
      
    } else if (!player1Turn && digitalRead(BTN_P2_PIN) == LOW) {
      fireShot(boardP1, shipsP1, 2, currentX, currentY);
      lastButtonTime = millis();
    }
  }
  
  drawRadar(activeLeds, targetBoard); 
  
  
  activeLeds[xyToIndex(currentX, currentY)] = COLOR_CURSOR;

  FastLED.show();
  

  if (shipsP1 <= 0 || shipsP2 <= 0) {
    gameOver = true;
    
    if (shipsP2 <= 0) {
      fill_solid(ledsP1, LED_COUNT, COLOR_WIN_BG);  // Verde
      fill_solid(ledsP2, LED_COUNT, COLOR_LOSE_BG); // Vermelho
    } else {
      fill_solid(ledsP1, LED_COUNT, COLOR_LOSE_BG); // Vermelho
      fill_solid(ledsP2, LED_COUNT, COLOR_WIN_BG);  // Verde
    }
    FastLED.show();
  }
}