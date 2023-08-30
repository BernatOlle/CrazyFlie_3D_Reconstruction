#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>

#define SERVER_IP "192.168.1.148"
#define PORT 5000
#define BUFFER_SIZE 1024

int main() {
    int client_fd;
    struct sockaddr_in server_addr;
    char message[BUFFER_SIZE];

    // Crear el socket del cliente
    if ((client_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        perror("Error al crear el socket");
        exit(EXIT_FAILURE);
    }

    // Configurar la dirección del servidor
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
    server_addr.sin_port = htons(PORT);

    // Conectar al servidor
    if (connect(client_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) == -1) {
        perror("Error en la operación connect");
        exit(EXIT_FAILURE);
    }

    printf("Conectado al servidor.\n");

    while (1) {
        printf("Escribe un mensaje para el servidor (escribe 'exit' para salir): ");
        fgets(message, BUFFER_SIZE, stdin);

        if (strcmp(message, "exit\n") == 0) {
            break;
        }

        // Enviar el mensaje al servidor
        if (send(client_fd, message, strlen(message), 0) == -1) {
            perror("Error en la operación send");
            exit(EXIT_FAILURE);
        }
    }

    // Cerrar el socket
    close(client_fd);

    return 0;
}
