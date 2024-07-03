# -*- coding: utf-8 -*-
"""
Robot SCARA Arduino UNO - FIUNA - Proyecto 4
@author: Sebas Monje
    El programa presenta una interfaz grafica donde se pueden asignar los 
puntos inicial y final del movimiento además del grado cierre del gripper.
Al presionar "Enviar" se determina si los puntos se encuentran en el area de 
trabajo, se calcula la trayectoria y a esta se le aplica la cinemática inversa
del robot SCARA. Además presenta en el navegador una previsualización de la 
trayectoria a realizar.
    "Parada" detiene el movimiento, impidiendo que se vuelva a desarrollar la 
trayectoria cargada anteriormente, por lo que se debe presionar "Reestablecer",
que llevará el brazo a la posición de origen y preparará el Arduino para recibir 
una nueva trayectoria.
"""

import serial
import time
import numpy as np


import sys
from PyQt5.QtWidgets import  QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton
from PyQt5.QtWidgets import QFrame, QSlider, QSizePolicy, QApplication, QTextEdit

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap, QIcon

import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar

xp = 0
yp = 0
theta1 = 0
theta2 = 0
phi = 0
L1 = 228
L2 = 136.5
data = ""

#################################################################### Funciones

def forwardKinematics(theta1, theta2):
    global xp, yp
    theta1F = theta1 * np.pi / 180  # grados a radianes
    theta2F = theta2 * np.pi / 180
    xp = np.round(L1 * np.cos(theta1F) + L2 * np.cos(theta1F + theta2F))
    yp = np.round(L1 * np.sin(theta1F) + L2 * np.sin(theta1F + theta2F))

def inverseKinematics(x, y):
    global theta1, theta2, phi

    theta2 = np.arccos((x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2))
    if x < 0 and y < 0: # tercer cuadrante
        theta2 = (-1) * theta2
    if x < 0 and y < 0:
        theta1 = np.arctan2(y, x) - np.arctan2((L2 * np.sin(theta2)), (L1 + L2 * np.cos(theta2))) 
    else:
        theta1 = np.arctan2(x, y) - np.arctan2((L2 * np.sin(theta2)), (L1 + L2 * np.cos(theta2)))
    theta2 = (-1) * theta2 * 180 / np.pi
    theta1 = theta1 * 180 / np.pi

    # Ajuste de ángulos dependiendo del cuadrante en el que se encuentra la coordenada final x, y
    if x >= 0 and y >= 0:  # 1er cuadrante
        theta1 = 90 - theta1 
    elif x <= 0 and y > 0:  # 2do cuadrante 
        theta1 = 90 - theta1 
    elif x < 0 and y < 0:  # 3er cuadrante
        theta1 = 360 + theta1 
        theta2 = -theta2 
    elif x > 0 and y < 0:  # 4to cuadrante
        theta1 = 90 - theta1 
    elif x < 0 and y == 0: # eje -x
        theta1 = 90 - theta1 
    
    # Calcular el ángulo "phi" para que el gripper esté paralelo al eje X
    phi = 90 + theta1 + theta2
    phi = (-1) * phi 
    
    # Ajuste de ángulo dependiendo del cuadrante en el que se encuentra la coordenada final x, y
    if x < 0 and y < 0:  # 3er cuadrante
        phi = 90 + theta1 + theta2 
        phi = (-1) * phi
    if np.abs(phi) > 165: # Límites de giro
        phi = 180 + phi
        
    # delimitar para que no se pase de los limites de movimiento, PENDIENTE...
    theta1 = np.round(theta1) # respecto al eje x (positivo en sentido antihorario)
    theta2 = np.round(theta2) # respecto al eje y1 (positivo en sentido antihorario)
    phi = np.round(phi) # respecto al eje y2 (positivo en sentido antihorario)

def TrayRecta(A, B): # Utilizando ec. parametrica de la recta
    if (A[0] > 0 and A[1] >= 0 and B[0] < 0 and B[1] >= 0) or (A[0] < 0 and A[1] >= 0 and B[0] > 0 and B[1] >= 0): # primer y segundo cuadrante
        pasos = 20
        C = np.array([0, 160, (A[2] + B[2])/2])
        direccion = C - A
        trayectoria = np.zeros((pasos, 3))
        for i, t in enumerate(np.linspace(0, 1, int(pasos/2))): # t varia de 0 a 1
            punto_intermedio = A + t * direccion # ec. parametrica de una recta
            trayectoria[i] = punto_intermedio
        direccion = B - C
        for i, t in enumerate(np.linspace(0, 1, int(pasos/2)), start= int(pasos/2)):
            punto_intermedio = C + t * direccion # ec. parametrica de una recta
            trayectoria[i] = punto_intermedio
        return trayectoria.astype(int), pasos
    elif (A[0] > 0 and A[1] >= 0 and B[0] > 0 and B[1] <= 0) or (A[0] > 0 and A[1] <= 0 and B[0] > 0 and B[1] >= 0): # primer y cuarto cuadrante
        pasos = 20
        direccion = B - A
        trayectoria = np.zeros((pasos, 3))
        for i, t in enumerate(np.linspace(0, 1, pasos)): # t varia de 0 a 1
            punto_intermedio = A + t * direccion # ec. parametrica de una recta
            trayectoria[i] = punto_intermedio
        return trayectoria.astype(int), pasos
    elif (A[0] > 0 and A[1] < 0 and B[0] < 0 and B[1] < 0) or (A[0] < 0 and A[1] < 0 and B[0] > 0 and B[1] < 0): # tercer a cuarto cuadrante
        pasos = 30
        if A[0] > 0: # si A esta en el cuarto cuadrante
            C1 = np.array([150, 150, (A[2] + B[2])/2])
            direccion = C1 - A
            trayectoria = np.zeros((pasos, 3))
            for i, t in enumerate(np.linspace(0, 1, int(pasos/2))): # t varia de 0 a 1
                punto_intermedio = A + t * direccion # ec. parametrica de una recta
                trayectoria[i] = punto_intermedio
            C2 = np.array([-150, 150, (A[2] + B[2])/2])
            direccion = C2 - C1
            for i, t in enumerate(np.linspace(0, 1, int(pasos/3)), start= int(pasos/3)):
                punto_intermedio = C1 + t * direccion # ec. parametrica de una recta
                trayectoria[i] = punto_intermedio
            direccion = B - C2
            for i, t in enumerate(np.linspace(0, 1, int(pasos/3)), start= int(2*pasos/3)):
                punto_intermedio = C2 + t * direccion # ec. parametrica de una recta
                trayectoria[i] = punto_intermedio
            return trayectoria.astype(int), pasos
        else: # si A esta en el tercer cuadrante
            C1 = np.array([-150, 150, (A[2] + B[2])/2])
            direccion = C1 - A
            trayectoria = np.zeros((pasos, 3))
            for i, t in enumerate(np.linspace(0, 1, int(pasos/2))): # t varia de 0 a 1
                punto_intermedio = A + t * direccion # ec. parametrica de una recta
                trayectoria[i] = punto_intermedio
            C2 = np.array([150, 150, (A[2] + B[2])/2])
            direccion = C2 - C1
            for i, t in enumerate(np.linspace(0, 1, int(pasos/3)), start= int(pasos/3)):
                punto_intermedio = C1 + t * direccion # ec. parametrica de una recta
                trayectoria[i] = punto_intermedio
            direccion = B - C2
            for i, t in enumerate(np.linspace(0, 1, int(pasos/3)), start= int(2*pasos/3)):
                punto_intermedio = C2 + t * direccion # ec. parametrica de una recta
                trayectoria[i] = punto_intermedio
            return trayectoria.astype(int), pasos
    elif (A[0] > 0 and A[1] < 0 and B[0] < 0 and B[1] >= 0) or (A[0] < 0 and A[1] >= 0 and B[0] > 0 and B[1] < 0): # segundo y cuarto cuadrante
        pasos = 20
        C = np.array([150, 200, (A[2] + B[2])/2])
        direccion = C - A
        trayectoria = np.zeros((pasos, 3))
        for i, t in enumerate(np.linspace(0, 1, int(pasos/2))): # t varia de 0 a 1
            punto_intermedio = A + t * direccion # ec. parametrica de una recta
            trayectoria[i] = punto_intermedio
        direccion = B - C
        for i, t in enumerate(np.linspace(0, 1, int(pasos/2)), start = int(pasos/2)):
            punto_intermedio = C + t * direccion # ec. parametrica de una recta
            trayectoria[i] = punto_intermedio
        return trayectoria.astype(int), pasos
    elif (A[0] > 0 and A[1] >= 0 and B[0] < 0 and B[1] < 0) or (A[0] < 0 and A[1] <= 0 and B[0] > 0 and B[1] >= 0): # primer y tercer cuadrante
        pasos = 20
        C = np.array([-150, 200, (A[2] + B[2])/2])
        direccion = C - A
        trayectoria = np.zeros((pasos, 3))
        for i, t in enumerate(np.linspace(0, 1, int(pasos/2))): # t varia de 0 a 1
            punto_intermedio = A + t * direccion # ec. parametrica de una recta
            trayectoria[i] = punto_intermedio
        direccion = B - C
        for i, t in enumerate(np.linspace(0, 1, int(pasos/2)), start= int(pasos/2)):
            punto_intermedio = C + t * direccion # ec. parametrica de una recta
            trayectoria[i] = punto_intermedio
        return trayectoria.astype(int), pasos
    else: # para los demas casos, ambos en un cuadrante, 2y3 o 1y4
        pasos = 10
        direccion = B - A
        trayectoria = np.zeros((pasos, 3))
        for i, t in enumerate(np.linspace(0, 1, pasos)): # t varia de 0 a 1
            punto_intermedio = A + t * direccion # ec. parametrica de una recta
            trayectoria[i] = punto_intermedio
        return trayectoria.astype(int), pasos     
    
def TrayCia():
    # Centro de la circunferencia
    centro = np.array([0, 200])

    # Radio de la circunferencia
    radio = 50

    # Número de puntos en la trayectoria
    num_puntos = 30

    # Ángulos desde 0 a 2pi con pasos igualmente espaciados
    angulos = np.linspace(0, 2 * np.pi, num_puntos)

    # Coordenadas x e y de la trayectoria circular
    x_tray = centro[0] + radio * np.cos(angulos)
    y_tray = centro[1] + radio * np.sin(angulos)
    z_tray = np.linspace(-50, -100, num_puntos)

    # Crear el array de NumPy con las coordenadas
    trayectoria = np.column_stack((x_tray, y_tray, z_tray))

    return trayectoria.astype(int), num_puntos

def puntos_en_area(A, B, radio1, radio2):
    # Calcular la distancia al cuadrado desde el origen hasta cada punto
    dist1 = A[0]**2 + A[1]**2
    dist2 = B[0]**2 + B[1]**2
    
    # Calcular el cuadrado del radio
    radio1 = radio1**2
    radio2 = radio2**2
    
    theta1 = np.arctan2(A[1], A[0])
    print(theta1)
    theta2 = np.arctan2(B[1], B[0])
    print(theta2)
    if (theta1 <= 0 and theta1 <= -np.pi/2):
        theta1 = theta1 + theta1 + 2 * np.pi
    
    if (theta2 <= 0 and theta2 <= -np.pi/2):
        theta2 = theta1 + theta2 + 2 * np.pi
    
    print(theta1)
    print(theta2)
    
    theta_min = -np.pi / 3
    print(theta_min)
    theta_max = 4 * np.pi / 3
    print(theta_max)
    
    # Verificar si los puntos están dentro del area de trabajo
    if (radio2 <= dist1 <= radio1 and radio2 <= dist2 <= radio1 and # dentro de los radios 1 y 2
        A[2] > -220 and B[2] > -220 and
        theta_min <= theta1 <= theta_max and
        theta_min <= theta2 <= theta_max): # Eje z
        return True
    else:
        return False
    # if (dist1 <= radio1 and dist2 <= radio1 and
    #     dist1 >= radio2 and dist2 >= radio2 and
    #     A[2] > -220 and B[2] > -220 and 
    #     (A[0] > 160 or A[0] < -140) and 
    #     (B[0] > 160 or B[0] < -140) and 
    #     (A[1] > -np.sqrt(radio1)*np.sin(88*np.pi/180) or B[1] > -np.sqrt(radio1)*np.sin(88*np.pi/180))):                              
    #     return True
    # else:
    #     return True

def appCineI(tray, pasos):
    global theta1, theta2, phi
    Coord_art = np.zeros((pasos, 3))
    for i in range(len(tray)):
        inverseKinematics(tray[i, 0], tray[i, 1])
        Coord_art[i] = np.array([theta1, theta2, phi])
    Coord_art = Coord_art.astype(int)
    return Coord_art

def enviar_Trama(Coord_art, tray, gripper, text_display):
    global Ardu, data
    for i in range(len(tray)):
        if i == len(tray) - 1: # el ultimo valor pone en modo RUN con data[1] = 1
            data = f"1,1,{Coord_art[i, 0]},{Coord_art[i, 1]},{Coord_art[i, 2]},{tray[i, 2]},{gripper[i]}\n"
            Ardu.write(data.encode())
        else: # todos los datos enviados se guardan con data[0] = 1
            data = f"1,0,{Coord_art[i, 0]},{Coord_art[i, 1]},{Coord_art[i, 2]},{tray[i, 2]},{gripper[i]}\n"
            Ardu.write(data.encode())
        print(f"Dato enviado {i}: {data.encode()}")
        if i != 0:
            text_display.append(f"{i}:  {data.encode()}")
        else:
            text_display.append(f"Dato enviado {i}: {data.encode()}")
        cadena = Ardu.readline().decode('utf-8').strip() # para esperar a que este listo el arduino
    #Dejamos abierta la comunicacion serial
    
def gripper(angulo, pasos):
    gripper = np.zeros(pasos).astype(int) 
    for i in range(pasos):
        gripper[i] = angulo # cerrar
    gripper[-1] = 100 # abrir cuando finaliza el movimiento
    return gripper


Ardu = serial.Serial('COM8', 115200)
time.sleep(2)

##################################################################### Interfaz
class Ventana(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Gui SCARA - Proyecto 4 - 2024 - Sebas Monje")
        self.setGeometry(100, 100, 800, 600)
        self.setStyleSheet("background-color: #333333; color: white;")
        icon = QIcon('C:/Users/Pc/Documents/Semestre 9/Proyecto 4/scara.ico')
        self.setWindowIcon(icon)
        
        self.lineas = []  # Lista para almacenar las referencias de las líneas

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        layout = QHBoxLayout(central_widget)
        
        # Layout para los controles a la izquierda
        left_layout = QVBoxLayout()
        left_layout.setAlignment(Qt.AlignTop)
        left_layout.setContentsMargins(10, 10, 50, 10)
        left_layout.setSizeConstraint(QVBoxLayout.SetFixedSize)
        


        # Casillas para punto inicial
        left_layout.addWidget(self.create_label("Posicion Inicial (x,y,z):"))
        self.init_point_layout = self.create_point_layout()
        left_layout.addLayout(self.init_point_layout)

        # Casillas para punto final
        left_layout.addWidget(self.create_label("Posicion Final (x,y,z):"))
        self.final_point_layout = self.create_point_layout()
        left_layout.addLayout(self.final_point_layout)

        # Casilla para el gripper
        gripper_layout = QHBoxLayout()
        left_layout.addLayout(gripper_layout)
        gripper_layout.addWidget(self.create_label("Gripper:"))
        self.garra_slider = QSlider(Qt.Horizontal)
        self.garra_slider.setMinimum(110)
        self.garra_slider.setMaximum(180)
        self.garra_slider.setTickInterval(1)
        self.garra_slider.setTickPosition(QSlider.TicksBelow)
        self.garra_slider.setStyleSheet(
            """
            QSlider::groove:horizontal {
                border: 1px solid #3A3939;
                height: 10px;
                background: #201F1F;
                margin: 2px 0;
            }

            QSlider::handle:horizontal {
                background: qradialgradient(cx:0.3, cy:-0.4,
                        fx:0.3, fy:-0.4,
                        radius: 1.35,
                        stop:0 #556B2F, stop:1 #8FBC8F);
                border: 1px solid #3A3939;
                width: 18px;
                height: 18px;
                margin: -5px 0; /* handle is placed by default on the contents rect of the groove. Expand outside the groove */
                border-radius: 9px; /* make the handle circle */
            }

            QSlider::add-page:horizontal {
                background: #575757;
                border-radius: 3px;
            }

            QSlider::sub-page:horizontal {
                background: #575757;
                border-radius: 3px;
            }
            """
        )
        gripper_layout.addWidget(self.garra_slider)
        self.slider_value_label = QLabel("110 a 180")
        gripper_layout.addWidget(self.slider_value_label)
        self.garra_slider.valueChanged.connect(self.slider_value_changed) # Conectar el cambio de valor del slider a un método
        
        agarre_layout = QHBoxLayout() # Se crean los espacios para ubicar los angulos de agarre
        soltar_layout = QHBoxLayout()
        left_layout.addLayout(agarre_layout)
        left_layout.addLayout(soltar_layout)
        
        agarre_layout.addWidget(self.create_label("Ángulo de Agarre Inicial:")) # Widget para el slider de agarre inicial
        self.agarre_slider = QSlider(Qt.Horizontal)
        self.agarre_slider.setMinimum(0)
        self.agarre_slider.setMaximum(180)
        self.agarre_slider.setTickInterval(1)
        self.agarre_slider.setTickPosition(QSlider.TicksRight)
        self.agarre_slider.setValue(90)
        self.agarre_slider.setStyleSheet(
            """
            QSlider::groove:horizontal {
                border: 1px solid #3A3939;
                height: 10px;
                background: #201F1F;
                margin: 2px 0;
            }

            QSlider::handle:horizontal {
                background: qradialgradient(cx:0.3, cy:-0.4,
                        fx:0.3, fy:-0.4,
                        radius: 1.35,
                        stop:0 #556B2F, stop:1 #FFD700);
                border: 1px solid #3A3939;
                width: 18px;
                height: 18px;
                margin: -5px 0; /* handle is placed by default on the contents rect of the groove. Expand outside the groove */
                border-radius: 9px; /* make the handle circle */
            }

            QSlider::add-page:horizontal {
                background: #575757;
                border-radius: 3px;
            }

            QSlider::sub-page:horizontal {
                background: #575757;
                border-radius: 3px;
            }
            """
        )
        agarre_layout.addWidget(self.agarre_slider)
        self.agarre_slider_value_label = QLabel("90")
        agarre_layout.addWidget(self.agarre_slider_value_label)
        # Conectar el cambio de valor del slider a un método
        self.agarre_slider.valueChanged.connect(self.agarre_slider_value_changed)
                                            
        soltar_layout.addWidget(self.create_label("Ángulo de Agarre Final:  ")) # wisget para el slider del agarre final
        self.soltar_slider = QSlider(Qt.Horizontal)
        self.soltar_slider.setMinimum(0)
        self.soltar_slider.setMaximum(180)
        self.soltar_slider.setTickInterval(1)
        self.soltar_slider.setTickPosition(QSlider.TicksRight)
        self.soltar_slider.setValue(90)
        self.soltar_slider.setStyleSheet(
            """
            QSlider::groove:horizontal {
                border: 1px solid #3A3939;
                height: 10px;
                background: #201F1F;
                margin: 2px 0;
            }

            QSlider::handle:horizontal {
                background: qradialgradient(cx:0.3, cy:-0.4,
                        fx:0.3, fy:-0.4,
                        radius: 1.35,
                        stop:0 #556B2F, stop:1 #FFD700);
                border: 1px solid #3A3939;
                width: 18px;
                height: 18px;
                margin: -5px 0; /* handle is placed by default on the contents rect of the groove. Expand outside the groove */
                border-radius: 9px; /* make the handle circle */
            }

            QSlider::add-page:horizontal {
                background: #575757;
                border-radius: 3px;
            }

            QSlider::sub-page:horizontal {
                background: #575757;
                border-radius: 3px;
            }
            """
        )
        soltar_layout.addWidget(self.soltar_slider)
        self.soltar_slider_value_label = QLabel("90")
        soltar_layout.addWidget(self.soltar_slider_value_label)
        # Conectar el cambio de valor del slider a un método
        self.soltar_slider.valueChanged.connect(self.soltar_slider_value_changed)
        
        self.agarre_slider_moved = False # Variables de estado para determinar si 
        self.soltar_slider_moved = False # se cambiaron los valores de estos sliders
        
        self.agarre_slider.sliderMoved.connect(self.on_agarre_slider_moved)
        self.soltar_slider.sliderMoved.connect(self.on_soltar_slider_moved)
        
        """
        agarre_layout.addWidget(self.create_label("Ángulo de agarre inicial:"))
        self.agarre = QLineEdit(self)
        self.agarre.setFixedWidth(40)  # Establece el ancho fijo para cada casilla de texto
        self.agarre.setText("0")
        agarre_layout.addWidget(self.agarre)
        #garre_layout.addStretch()
        
        soltar_layout.addWidget(self.create_label("Ángulo de agarre final:"))
        self.soltar = QLineEdit(self)
        self.soltar.setFixedWidth(40)  # Establece el ancho fijo para cada casilla de texto
        self.soltar.setText("0")
        soltar_layout.addWidget(self.soltar)
        #soltar_layout.addStretch()
        """
        
        left_layout.addStretch()
        
        self.text_display = QTextEdit(self) # Terminal textual
        self.text_display.setReadOnly(True)
        self.text_display.setStyleSheet("QTextEdit {"
                                        "font-family: 'Courier New', monospace;"
                                        "font-size: 12pt;"
                                        "color: white;"
                                        "}")
        left_layout.addWidget(self.text_display)

        # Botones
        button_layout = QHBoxLayout()
        left_layout.addLayout(button_layout)

        boton_enviar = QPushButton("Enviar", self)  # Creacion de los botones
        button_layout.addWidget(boton_enviar)
        boton_enviar.setStyleSheet("QPushButton {"
                                    "background-color: #4CAF50;"
                                    "color: black;"
                                    "border-radius: 10px;"
                                    "border: 2px solid #4CAF50;"
                                    "padding: 5px 10px;"
                                    "font-family: 'Courier New', monospace;"
                                    "}"
                                    "QPushButton:hover {"
                                    "background-color: #45a049;"
                                    "}"
                                    "QPushButton:pressed {"
                                    "background-color: #39803c;"
                                    "}")
        boton_enviar.clicked.connect(self.enviarParametros)
        
        boton_reestablecer = QPushButton("Reestablecer", self)
        button_layout.addWidget(boton_reestablecer)
        boton_reestablecer.setStyleSheet("QPushButton {"
                                          "background-color: #FFD700;"
                                          "color: black;"
                                          "border-radius: 10px;"
                                          "border: 2px solid #FFD700;"
                                          "padding: 5px 10px;"
                                          "font-family: 'Courier New', monospace;"
                                          "}"
                                          "QPushButton:hover {"
                                          "background-color: #e6ac00;"
                                          "}"
                                          "QPushButton:pressed {"
                                          "background-color: #cc9900;"
                                          "}")
        boton_reestablecer.clicked.connect(self.reestablecer)
        
        boton_parada = QPushButton("Parada", self)
        button_layout.addWidget(boton_parada)
        boton_parada.setStyleSheet("QPushButton {"
                                    "background-color: #FF6347;"
                                    "color: black;"
                                    "border-radius: 10px;"
                                    "border: 2px solid #FF6347;"
                                    "padding: 5px 10px;"
                                    "font-family: 'Courier New', monospace;"
                                    "}"
                                    "QPushButton:hover {"
                                    "background-color: #e57373;"
                                    "}"
                                    "QPushButton:pressed {"
                                    "background-color: #f44336;"
                                    "}")
        boton_parada.clicked.connect(self.STOP)

        layout.addLayout(left_layout)

################################# Gráfico 3D
        self.fig = plt.figure()
        self.fig.patch.set_facecolor((0.2, 0.2, 0.2, 1))
        self.fig.subplots_adjust(left=0, right=1, bottom=0, top=1)

        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_facecolor((0.2, 0.2, 0.2, 0.5))
        self.ax.grid(True, color='white')  # Color blanco para el grid
        self.ax.xaxis.set_tick_params(labelcolor='white')  # Color blanco para los labels del eje X
        self.ax.yaxis.set_tick_params(labelcolor='white')  # Color blanco para los labels del eje Y
        self.ax.zaxis.set_tick_params(labelcolor='white')  # Color blanco para los labels del eje Z
        self.ax.set_xlabel('X', color='white')
        self.ax.set_ylabel('Y', color='white')
        self.ax.set_zlabel('Z', color='white')
        
        theta = np.linspace(-np.pi/3, 4*np.pi/3, 100)
        z = np.linspace(-220, 0, 100)
        theta, z = np.meshgrid(theta, z)
        ## Delimitacion del area de trabajo
        radio_exterior = L1 + L2
        radio_interior = L1 - L2 * np.cos(15 * np.pi / 180)
        x_exterior = radio_exterior * np.cos(theta)
        y_exterior = radio_exterior * np.sin(theta)
        x_interior = radio_interior * np.cos(theta)
        y_interior = radio_interior * np.sin(theta)
        #y_exterior[y_exterior < 0] = 0
        #y_interior[y_interior < 0] = 0
        self.ax.plot_surface(x_exterior, y_exterior, z, alpha=0.5, color=(0.5569, 0.5569, 0.2196, 1.0)) # grafica del area de trabajo
        self.ax.plot_surface(x_interior, y_interior, z, alpha=0.5, color=(0.118, 0.565, 1.0, 1.0))
        angles = [-60, 240]
        for angle in angles:
            rad = np.deg2rad(angle)
            x_plane = np.linspace(radio_interior * np.cos(rad), radio_exterior * np.cos(rad), 100)
            y_plane = np.linspace(radio_interior * np.sin(rad), radio_exterior * np.sin(rad), 100)
            X_plane, Z_plane = np.meshgrid(x_plane, z[:, 0])
            Y_plane, Z_plane = np.meshgrid(y_plane, z[:, 0])
            self.ax.plot_surface(X_plane, Y_plane, Z_plane, alpha=0.5, color=(0.4, 0.8039, 0.6666, 1.0))

        self.canvas = FigureCanvas(self.fig)
        self.canvas.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        layout.addWidget(self.canvas, 1) 
        
        self.canvas.setStyleSheet("background-color: rgba(0.2, 0.2, 0.2, 0.5);")
        
        # Habilitar la herramienta de navegación
        self.toolbar = NavigationToolbar(self.canvas, self)
        left_layout.addWidget(self.toolbar)
        left_layout.setAlignment(self.toolbar, Qt.AlignBottom)
        self.toolbar.actions()[4].setVisible(False)
        self.toolbar.actions()[5].setVisible(False)
        self.toolbar.actions()[6].setVisible(False)
        self.toolbar.actions()[7].setVisible(False)
        
        # Ajustar tamaño del gráfico
        self.canvas.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.canvas.updateGeometry()
        
        self.canvas.mpl_connect('scroll_event', self.on_wheel_event)
        
        left_layout.addStretch()
        frame = QFrame(self)
        frame.setStyleSheet("background-color: #CCCCCC;")  # Establecer el color de fondo
        image_label = QLabel(frame)
        pixmap = QPixmap('C:/Users/Pc/Pictures/Varios/Fiuna1.png')  # Cargar la imagen desde un archivo
        image_label.setPixmap(pixmap)
        frame_layout = QVBoxLayout()
        frame_layout.addWidget(image_label)
        frame.setLayout(frame_layout)
        left_layout.addWidget(frame)
        


        self.show()

##################################################### Funciones de la Interfaz
    def on_agarre_slider_moved(self):
        self.agarre_slider_moved = True

    def on_soltar_slider_moved(self):
        self.soltar_slider_moved = True

    def on_wheel_event(self, event):
        zoom_factor = 1.1 if event.step > 0 else 0.9 # Obtener el factor de zoom basado en la dirección del scroll
        xlim = self.ax.get_xlim() # obtencion de limites
        ylim = self.ax.get_ylim()
        zlim = self.ax.get_zlim()
        x_center = np.mean(xlim) # centrado del punto de acercamiento
        y_center = np.mean(ylim)
        z_center = np.mean(zlim)
        new_x_range = (xlim[0] - x_center) * zoom_factor, (xlim[1] - x_center) * zoom_factor # calculo del rango
        new_y_range = (ylim[0] - y_center) * zoom_factor, (ylim[1] - y_center) * zoom_factor
        new_z_range = (zlim[0] - z_center) * zoom_factor, (zlim[1] - z_center) * zoom_factor
        self.ax.set_xlim(x_center + np.array(new_x_range)) # configurar nuevos limites
        self.ax.set_ylim(y_center + np.array(new_y_range))
        self.ax.set_zlim(z_center + np.array(new_z_range))
        self.canvas.draw() # Redibujar la figura con los nuevos límites
        
    def slider_value_changed(self):
        # Actualizar la etiqueta con el nuevo valor del slider
        self.slider_value_label.setText(str(self.garra_slider.value()))
    
    def agarre_slider_value_changed(self):
        # Actualizar la etiqueta con el nuevo valor del slider
        self.agarre_slider_value_label.setText(str(self.agarre_slider.value()))
    
    def soltar_slider_value_changed(self):
        # Actualizar la etiqueta con el nuevo valor del slider
        self.soltar_slider_value_label.setText(str(self.soltar_slider.value()))

    def create_label(self, text):
        label = QLabel(text, self)
        label.setStyleSheet("font-size: 14px; font-family: 'Courier New'; margin-bottom: 5px;")
        return label
    
    def create_point_layout(self):
        layout = QHBoxLayout()
        for _ in range(3):
            edit = QLineEdit(self)
            edit.setFixedWidth(40)  # Establece el ancho fijo para cada casilla de texto
            layout.addWidget(edit)
        return layout

########################################## Funciones de control en la interfaz
    def enviarParametros(self):
        x1_val = int(self.init_point_layout.itemAt(0).widget().text()) # Obtencion de datos de la interfaz
        y1_val = int(self.init_point_layout.itemAt(1).widget().text())
        z1_val = int(self.init_point_layout.itemAt(2).widget().text())
        x2_val = int(self.final_point_layout.itemAt(0).widget().text())
        y2_val = int(self.final_point_layout.itemAt(1).widget().text())
        z2_val = int(self.final_point_layout.itemAt(2).widget().text())
        ang_agarre = int(self.agarre_slider.value())
        ang_soltar = int(self.soltar_slider.value())
        gripper_val = int(self.garra_slider.value())
    
        A = np.array([x1_val, y1_val, z1_val])
        B = np.array([x2_val, y2_val, z2_val])
        if puntos_en_area(A, B, L1 + L2, L1 - L2 * np.cos(15*np.pi/180)): # Radio interior cia de radio 96mm, radio exterior 364mm
            self.text_display.clear()
            tray, pasos = TrayRecta(A, B)
            # tray, pasos = TrayCia() # trayectoria circular de ejemplo
            x = tray[:, 0]
            y = tray[:, 1]
            z = tray[:, 2]
            linea, = self.ax.plot(x, y, z, color='green', linewidth=2, label='Trayectoria') # Agregar los datos al gráfico
            self.lineas.append(linea) # se guarda la referencia de la linea
            self.canvas.draw() # Actualizar el gráfico en el canvas
            
            """

            fig = go.Figure(data=[go.Scatter3d(x=x, y=y, z=z, mode='lines', marker=dict(color='green'))])
            fig.update_layout(scene=dict(xaxis=dict(title='X'),
                                         yaxis=dict(title='Y'),
                                         zaxis=dict(title='Z')),
                              title='Trayectoria en Linea Recta',
                              margin=dict(l=0, r=0, b=0, t=40))
            fig.show()
            """
            
            Gripper = gripper(gripper_val, pasos)
            Coord_art = appCineI(tray, pasos)
            
            if self.agarre_slider_moved: # Para modificar los angulos de agarre inicial y final
                Coord_art[0, 2] = Coord_art[0, 2] + ang_agarre 
                self.agarre_slider_moved = False
                enviar_Trama(Coord_art, tray, Gripper, self.text_display)
            elif self.soltar_slider_moved:
                Coord_art[-1, 2] = Coord_art[-1, 2] + ang_soltar
                self.agarre_slider_moved = False
                enviar_Trama(Coord_art, tray, Gripper, self.text_display)
            elif self.agarre_slider_moved and self.soltar_slider_moved:
                Coord_art[0, 2] = Coord_art[0, 2] + ang_agarre 
                Coord_art[-1, 2] = Coord_art[-1, 2] + ang_soltar
                self.agarre_slider_moved = False
                self.agarre_slider_moved = False
                enviar_Trama(Coord_art, tray, Gripper, self.text_display)
            else:
                enviar_Trama(Coord_art, tray, Gripper, self.text_display)
        else:
            self.text_display.append("Punto/s fuera del area de trabajo.")
            print("Punto/s fuera del area de trabajo.")

    def STOP(self):
        global Ardu, data
        data = "Parar"
        Ardu.write(data.encode())
        self.text_display.append("PROCESO DETENIDO")
    
    def reestablecer(self):
        global Ardu, data
        data = "2,0,0,0,0,0,0\n"
        Ardu.write(data.encode())
        print("reestableciendo")
        Ardu.close()
        # Limpiar las casillas de texto de init_point_layout
        self.init_point_layout.itemAt(0).widget().clear()
        self.init_point_layout.itemAt(1).widget().clear()
        self.init_point_layout.itemAt(2).widget().clear()     
        # Limpiar las casillas de texto de final_point_layout
        self.final_point_layout.itemAt(0).widget().clear()
        self.final_point_layout.itemAt(1).widget().clear()
        self.final_point_layout.itemAt(2).widget().clear()
        for linea in self.lineas: # eliminar lineas dibujadas
            linea.remove()
        self.lineas.clear()
        self.canvas.draw() # Actualizar el gráfico en el canvas
        self.text_display.clear()
        time.sleep(15)
        Ardu = serial.Serial('COM8', 115200) # Se reestablece la comunicacion
        time.sleep(2)


######################################################################### Main
if __name__ == "__main__":
    app = QApplication(sys.argv)
    ventana = Ventana()
    sys.exit(app.exec_())

