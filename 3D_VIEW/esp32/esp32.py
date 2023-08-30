from PIL import Image
import requests
from io import BytesIO
import os, sys
import datetime
import os
import random
import time

numero_random = random.randint(1, 1000)

nombre_carpeta = "carpeta"+str(numero_random)
ruta_carpeta = os.path.join(os.getcwd(), nombre_carpeta)

try:
    os.mkdir(ruta_carpeta)
except FileExistsError:
    print(f"La carpeta {nombre_carpeta} ya existe en {ruta_carpeta}.")

datetime_object = datetime.datetime.now()
print(datetime_object)
d1 = str(datetime_object)

#Cambiar la direccion IP segun su configuracion

url = "http://172.20.10.2/capture"

for i in range(0,50):
    response = requests.get(url)
    img = Image.open(BytesIO(response.content))

    try:
        img.save(nombre_carpeta+"\\result"+str(i+10)+".jpg")
        time.sleep(5)
        print(i)
    except IOError:
        print("cannot convert", infile)

datetime_object = datetime.datetime.now()
print(datetime_object)