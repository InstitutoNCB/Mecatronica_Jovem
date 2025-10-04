
import time
import re
import threading
from collections import defaultdict, deque
import numpy

import pytchat
import serial
import serial.tools.list_ports
import simpleaudio as sa

# ============== CONFIGURAÇÕES =============
VIDEO_ID     = "2c8N-Q1RjLo"
SERIAL_PORT  = "COM11"                      
BAUDRATE     = 115200
TOCAR_SOM_LOCAL = True                     
SOM_WAV_PATH = None                        
COOLDOWN_POR_USUARIO = 6                   # segundos: evita spam do mesmo usuário
JANELA_ANTISPAM_GLOBAL = 2                 # segundos: limite global entre execuções
MAX_FILA = 100                             # histórico para estatísticas/diagnóstico

# ============== SCORES ====================
Ponto_A = 0
Ponto_B = 0

# ============== COMANDOS ==================
MAPA_COMANDOS = {
    r"^!vermelho$":       "LED_RED_ON",
    r"^!azul$":           "LED_BLUE_ON",
    r"^!off$":            "LEDS_OFF",
    r"^!porta$":          "SERVO_OPEN",
    r"^!fechar$":         "SERVO_CLOSE",
    r"^!buzzer$":         "BUZZER",
    r"^!P$":              "A",
    r"^!L$":              "B"
}

RESPOSTAS_ESPERADAS = {"OK", "BUSY", "DONE", "ERR"}

# ============== SERIAL ==============
ser = None

def tentar_abrir_serial():
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.2)
        print(f"[SERIAL] Conectado em {SERIAL_PORT} @ {BAUDRATE} baud")
    except Exception as e:
        print(f"[SERIAL] Falha ao abrir {SERIAL_PORT}: {e}\n> Dica: confira a porta. Portas disponíveis:")
        for p in serial.tools.list_ports.comports():
            print(" -", p.device)

def enviar_serial(msg: str):
    if ser and ser.is_open:
        linha = (msg.strip() + "\n").encode("utf-8")
        ser.write(linha)

        try:
            resp = ser.readline().decode("utf-8", errors="ignore").strip()
            if resp:
                print(f"[ESP32_1] {resp}")

                global Ponto_A, Ponto_B

                if resp == "A":
                    Ponto_A += 1
                    print(f"Lado A = {Ponto_A} | Lado B = {Ponto_B}")
                elif resp == "B":
                    Ponto_B += 1
                    print(f"Lado A = {Ponto_A} | Lado B = {Ponto_B}")

            time.sleep(0.1)
        except Exception:
            pass
    else:
        print(f"[SERIAL] (simulado) {msg}")

# ============== SOM LOCAL (opcional) ==============
def tocar_bip():
    if not TOCAR_SOM_LOCAL:
        return
    if SOM_WAV_PATH:
        try:
            wave_obj = sa.WaveObject.from_wave_file(SOM_WAV_PATH)
            wave_obj.play()
        except Exception as e:
            print("[SOM] Erro ao tocar WAV:", e)
    else:
        import numpy as np
        fs = 44100
        t = 0.15
        f = 880.0
        samples = (0.2 * np.sin(2*np.pi*np.arange(fs*t)*f/fs)).astype(np.float32)
        sa.play_buffer((samples * 32767).astype('<i2').tobytes(), 1, 2, fs)

# ============== ANTISPAM ==============
ultimo_comando_por_usuario = defaultdict(lambda: 0.0)
ultimo_comando_global = 0.0
fila_execucoes = deque(maxlen=MAX_FILA)

def pode_executar(usuario: str) -> bool:
    global ultimo_comando_global
    agora = time.time()
    if agora - ultimo_comando_global < JANELA_ANTISPAM_GLOBAL:
        return False
    if agora - ultimo_comando_por_usuario[usuario] < COOLDOWN_POR_USUARIO:
        return False
    ultimo_comando_por_usuario[usuario] = agora
    ultimo_comando_global = agora
    return True

# ============== PROCESSAMENTO DE MENSAGENS ==============
regex_comandos = [(re.compile(pat, re.IGNORECASE), cmd) for pat, cmd in MAPA_COMANDOS.items()]

def identificar_comando(texto: str):
    msg = texto.strip()
    for rgx, cmd in regex_comandos:
        if rgx.match(msg):
            return cmd
    return None

def thread_leitura_serial():
    if not ser:
        return
    while True:
        try:
            if ser.in_waiting:
                msg = ser.readline().decode("utf-8", errors="ignore").strip()
                if msg:
                    print(f"[ESP32] {msg}")
                    
        except Exception:
            break

# ============== LOOP PRINCIPAL DO CHAT ==============
def main():
    print("==== YOUTUBE LIVE CONTROLLER ====")
    print("Comandos disponíveis no chat:")
    print(" ", ", ".join(MAPA_COMANDOS.keys()))
    print("---------------------------------")

    tentar_abrir_serial()
    if ser:
        t = threading.Thread(target=thread_leitura_serial, daemon=True)
        t.start()

    chat = pytchat.create(video_id=VIDEO_ID, interruptable=True)
    print("[CHAT] Conectado. Aguardando mensagens…")

    try:
        while chat.is_alive():
            for c in chat.get().sync_items():
                usuario = c.author.name
                texto   = c.message.strip()
                cmd = identificar_comando(texto)
                if cmd:
                    if pode_executar(usuario):
                        print(f"[CHAT] {usuario}: {texto}  ->  {cmd}")
                        enviar_serial(cmd)
                        tocar_bip()
                        fila_execucoes.append((time.time(), usuario, cmd))
                    else:
                        print(f"[ANTISPAM] Ignorado de {usuario}: {texto}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[CHAT] Encerrado pelo usuário.")
    finally:
        if ser and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()
