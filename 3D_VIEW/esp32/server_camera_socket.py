import socket
from _thread import *
from take_photo import Photo

server = "localhost"
port = 12345

camera = Photo()

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
    s.bind((server, port))  # connectes el socket amb el servidor i el port
except socket.error as e:
    str(e)

s.listen()  # escolta fins que es connecten dos clients
print("Waiting for a connection, Server Started")

connected = set()

def threaded_client(conn):
    conn.send(str.encode("Conected to the server"))
    while True:
        try:
            data = conn.recv(4096).decode()
            if data == "take":
                print("Taking photo")
                filename = camera.take_photo()
                image = open(filename, 'rb')
                image_data = image.read()
                conn.sendall(image_data)
                camera.delete_photo(filename)
                print("All sended")
                conn.sendall('Transmited')
        except:
            break

    print("Lost connectiom")
    conn.close()

while True:
    conn, addr = s.accept()
    print("Connected to: ", addr)
    start_new_thread(threaded_client, (conn,))
