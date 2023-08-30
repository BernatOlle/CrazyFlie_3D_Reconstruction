import socket
import uuid



class Network:
    def __init__(self):
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server = "172.20.10.5"
        self.port = 12345
        self.unique_id = uuid.uuid4() 
        self.addr = (self.server, self.port)
        self.p = self.connect()

    def connect(self):
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server = "172.20.10.5"
        self.port = 12345
        self.addr = (self.server, self.port)
        try:
            self.client.connect(self.addr)
            #print("Connection")
            self.client.recv(2048).decode()
        except:
            pass

    def send(self, data):
        try:
            self.client.send(str.encode("take"))
            file_path = self.client.recv(2048).decode()
            photo_data = b''  # crea una variable vacía para almacenar los datos de la foto
            while True:  # ciclo infinito que recibe los datos de la foto en fragmentos de 1024 bytes
                
                data = self.client.recv(1024)  # recibe datos de 1024 bytes del emisor
               
                if len(data) == 0:
                    break
                photo_data += data  # agrega los datos recibidos al final de la variable photo_data
            print("Transmited "+str(file_path))
        except socket.error as e:
            print(e)
        
        # Escribe los datos de la foto en un archivo
        with open('photos/'+file_path, 'wb') as f: # crea un archivo nuevo llamado "foto_recibida.jpg" y lo abre en modo escritura binaria
            f.write(photo_data) # escribe los datos de la foto en el archivo
        self.client.close()

    def take(self):
        self.connect()
        self.send("take")

if __name__ == '__main__':
    s = Network()
    s.take()
