import threading

# Definir la función para el primer hilo
def hilo1():
    print("Hilo 1 iniciado")
    # Lógica del hilo 1 aquí

# Definir la función para el segundo hilo
def hilo2():
    print("Hilo 2 iniciado")
    # Lógica del hilo 2 aquí

# Crear los hilos
t1 = threading.Thread(target=hilo1)
t2 = threading.Thread(target=hilo2)

# Iniciar los hilos
t1.start()
t2.start()

# Esperar a que los hilos terminen
t1.join()
t2.join()

# Se llega aquí cuando ambos hilos han terminado
print("Ambos hilos han terminado")