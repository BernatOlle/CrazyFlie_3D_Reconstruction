#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>

#define PORT 5000
#define BUFFER_SIZE 1024

int main() {
    int server_fd, client_fd;
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_addr_len = sizeof(client_addr);
    char buffer[BUFFER_SIZE] = {0};

    // Crear el socket del servidor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        perror("Error al crear el socket");
        exit(EXIT_FAILURE);
    }

    // Configurar la dirección del servidor
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(PORT);

    // Enlazar el socket a la dirección y puerto
    if (bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) == -1) {
        perror("Error en la operación bind");
        exit(EXIT_FAILURE);
    }

    // Escuchar por conexiones entrantes
    if (listen(server_fd, 5) == -1) {
        perror("Error en la operación listen");
        exit(EXIT_FAILURE);
    }

    printf("Esperando conexiones...\n");

    // Aceptar la conexión entrante
    if ((client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &client_addr_len)) == -1) {
        perror("Error en la operación accept");
        exit(EXIT_FAILURE);
    }

    printf("Cliente conectado.\n");

    // Leer datos del cliente y mostrarlos
    while (1) {
        int num_bytes = read(client_fd, buffer, BUFFER_SIZE);
        if (num_bytes <= 0) {
            printf("Cliente desconectado.\n");
            break;
        }

        buffer[num_bytes] = '\0';  // Asegurar que la cadena esté terminada con null
        printf("Cliente: %s", buffer);
    }

    // Cerrar los sockets
    close(client_fd);
    close(server_fd);

    return 0;
}
