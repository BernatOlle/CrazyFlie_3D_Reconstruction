import urllib.request
import os
import time

class Take:
    def __init__(self,name) -> None:
        self.i = 000
        nombre_carpeta = "carpeta_"+str(name)
        self.folder = os.path.join(os.getcwd(), nombre_carpeta)
        try:
            os.mkdir(self.folder)
        except FileExistsError:
            print(f"La carpeta {nombre_carpeta} ya existe en {self.folder}.")
            
        
    def take_photo(self):
        url = "http://172.20.10.2/capture"
        nombre_archivo = "photo"+str(self.i)+".jpg"
        ruta_completa = os.path.join(self.folder, nombre_archivo)
        try:
            urllib.request.urlretrieve(url, ruta_completa)
            print(f"La foto se ha guardado correctamente como {nombre_archivo}")
            self.i += 1 
        except Exception as e:
            print(f"Error al capturar la foto: {str(e)}")
 
def main():       
    take = Take("Bernat")
    for i in range(50):
        take.take_photo()
        time.sleep(0.2)