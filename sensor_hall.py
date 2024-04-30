# Código do Projeto Caça Fantasmas do Tio Rafa publicado na edição Projetos do Prof. Newton C. Braga da revista Mecatrônica Jovem
import tkinter as tk
from PIL import Image, ImageTk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import serial
import time

# Configuração inicial das variáveis globais
dados_x = [0]  # Dados do eixo X
dados_y1 = [0]  # Dados do primeiro sensor no eixo Y
dados_y2 = [0]  # Dados do segundo sensor no eixo Y

# Inicializa a comunicação serial
arduino = serial.Serial('COM3', 9600)
time.sleep(2)  # Espera para a conexão ser estabelecida

def le_dados():
    if arduino.inWaiting() > 0:
        try:
            linha = arduino.readline().decode('utf-8').rstrip()
            partes = linha.split(',')
            if len(partes) == 2:
                valor_analogico, novo_valor_analogico = map(int, partes)
                return valor_analogico, novo_valor_analogico
            else:
                print(f"Formato de dados inesperado: {linha}")
        except ValueError as e:
            print(f"Erro ao converter dados para inteiros: {e}")
    return None, None

def atualiza_dados():
    valor_analogico, novo_valor_analogico = le_dados()
    if valor_analogico is not None and novo_valor_analogico is not None:
        dados_x.append(dados_x[-1] + 1)
        dados_y1.append(valor_analogico)
        dados_y2.append(novo_valor_analogico)

        if len(dados_x) > 50:
            dados_x.pop(0)
            dados_y1.pop(0)
            dados_y2.pop(0)

        curva1.set_data(dados_x, dados_y1)
        curva2.set_data(dados_x, dados_y2)

        ax.set_xlim(dados_x[0], dados_x[-1])
        ax.set_ylim(min(dados_y1 + dados_y2) - 10, max(dados_y1 + dados_y2) + 10)

        canvas.draw()
    root.after(100, atualiza_dados)

root = tk.Tk()
root.title("Caça Fatasma Mecatrônica Jovem")

# Crie um Canvas Tkinter para a imagem de fundo
background_canvas = tk.Canvas(root, width=400, height=300)
background_canvas.pack(fill="both", expand=True)

# Carrega e exibe a imagem de fundo no Canvas Tkinter
background_image = Image.open(r"C:\Users\Prof Rafael Oliveira\Documents\fundo.jpg")
background_image = background_image.resize((600, 400))  
background_image_tk = ImageTk.PhotoImage(background_image)
# Ajuste aqui: centralizar a imagem no canvas
background_canvas.create_image(1000, 250, image=background_image_tk, anchor="center")


# Configuração do Matplotlib
fig = Figure(figsize=(10, 4), dpi=100)
ax = fig.add_subplot(111)
curva1, = ax.plot(dados_x, dados_y1, label='Valor Analógico', color='lime')
curva2, = ax.plot(dados_x, dados_y2, label='Novo Valor ', color='fuchsia')
ax.legend()

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.draw()
canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

# Inicia a atualização dos dados
root.after(10, atualiza_dados)

# Inicia o loop principal
root.mainloop()
