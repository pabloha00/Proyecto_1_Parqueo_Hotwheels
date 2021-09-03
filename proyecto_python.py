'''------------------------------------------------------------------------------
Autores: Pablo Herrarte y Andy Bonilla
Programa: proyecto 1
Creado: 21 de agosto de 2021    
Descripcion: un laboratoria bien fumado tbh pero chilero
intefaz gráfica para el proyecto de parqueo para hotwheels
-------------------------------------------------------------------------------'''

'''------------------------------------------------------------------------------
-------------------------IMPORTAR LIBRERIAS--------------------------------------
------------------------------------------------------------------------------'''
import builtins
import tkinter as tk            #se importa libreria de GUI
from tkinter import *
import serial, time             #se importa libreria serial y cuenta de tiempo
from Adafruit_IO import Client, RequestError, Feed

'''------------------------------------------------------------------------------
-----------------------DEFINICION DE PUERTO SERIAL-------------------------------
------------------------------------------------------------------------------'''
port1=serial.Serial()             #declarar puerto serial y braudeaje
port1.port='COM1'                 #se dice el puerto a usar
port1.baudrate = 9600             #set Baud rate to 9600
port1.bytesize = 8                # Number of data bits = 8
port1.parity   ='N'               # No parity
port1.stopbits = 1                # Number of Stop bits = 1
port1.open()                      #apertura del puerto serial 
port1.timeout = 3
#---------INICIALIZACION DE COMUNICACION CON ADAFRUIT
ADAFRUIT_IO_USERNAME = "anbo_one"
ADAFRUIT_IO_KEY = "aio_Whyv195VLRm3pU5OiSIZiMDV4IRl"
aio = Client(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)     #parametros
digital_feed0 = aio.feeds('parqueos')
digital_feed1 = aio.feeds('parqueo1')
digital_feed2 = aio.feeds('parqueo2')
digital_feed3 = aio.feeds('parqueo3')
time.sleep(4)
v0 = 3
v1 = 1
v2 = 1
v3 = 1
P0 = 3
P1 = 1
P2 = 1
P3 = 1
PK0 = 3
PK1 = 1
PK2 = 1
PK3 = 1

while TRUE:
    port1.flushInput()
    port1.read_until(b'\n', 9)
    todo = port1.read_until(b'\n', 8)
    parqueos = todo.split(b',')
    print(todo)
    print(parqueos)

    if len(parqueos) == 4:
        if len(parqueos[0]) == 1:
            parqueos_total = int(parqueos[0])
        if len(parqueos[1]) == 1:
            parqueo_1 = int(parqueos[1])
        if len(parqueos[2]) == 1:
            parqueo_2 = int(parqueos[2])
        if len(parqueos[3]) == 2:
            parqueo_3 = int(parqueos[3])
    
    else:
        port1.read_until(b'\n', 8)
        todo = port1.read_until(b'\n', 8)

    #print(cc, parqueos_total, parqueo_1, parqueo_2, parqueo_3)
    if v0==parqueos_total:
        P0 = parqueos_total

    if v1==parqueo_1:
        P1 = parqueo_1

    if v2==parqueo_2:
        P2 = parqueo_2

    if v3==parqueo_3:
        P3 = parqueo_3        
    '''------------------------------------------------------------------------------
    -------------------INTERCAMBIO DE DATOS CON ADAFRUIT-----------------------------
    ------------------------------------------------------------------------------'''
    if P0 != PK0:
        #---------SE MANDAN VALORES DE CANTIDAD DE PARQUEOS DESDE PIC PARA ADAFRUIT
        aio.send_data(digital_feed0.key, parqueos_total)
        time.sleep(2)

        #---------SE MANDA SI ESTA OCUPADO EL PARQUEO1
    if P1 != PK1:
        aio.send_data(digital_feed1.key, parqueo_1)
        time.sleep(2)

        #---------SE MANDA SI ESTA OCUPADO EL PARQUEO2
    if P2 != PK2:
        aio.send_data(digital_feed2.key, parqueo_2)
        time.sleep(2)

        #---------SE MANDA SI ESTA OCUPADO EL PARQUEO3
    if P3 != PK3:
        aio.send_data(digital_feed3.key, parqueo_3)
        time.sleep(2)
        
    v0=parqueos_total
    v1=parqueo_1
    v2=parqueo_2
    v3=parqueo_3
    PK0 = P0
    PK1 = P1
    PK2 = P2
    PK3 = P3