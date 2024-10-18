#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <time.h>

#define CAN_MODE 0xDC
#define DL_MODE 0xA8
#define DL2_MODE 0xD1
#define GET_MODE 0x81
#define GET_HEADER 0xAA
#define GET_LATEST 0xAB
#define READ_DATA 0xAC
#define END_READ 0xAD
#define RESET_DATA 0xAF
#define WAIT_TIME 0xBA
#define MAX_RETRIES 10
#define DATASET_SIZE 61
#define LATEST_SIZE 56

#define DIGITAL_ON 1
#define DIGITAL_OFF 0
#define SPEED_ACTIVE 0x80
#define SPEED_MASK 0x1F
#define INT16_POSITIVE_MASK 0xFFFF
#define SIGN_BIT 0x8000
#define POSITIVE_VALUE_MASK 0x0FFF
#define TYPE_MASK 0x7000
#define TYPE_NONE 0x0000
#define TYPE_DIGITAL 0x1000
#define TYPE_TEMP 0x2000
#define TYPE_VOLUME 0x3000
#define TYPE_RADIATION 0x4000
#define TYPE_RAS 0x7000
#define RAS_POSITIVE_MASK 0x01FF
#define INT32_MASK 0xFFFFFFFF
#define INT32_SIGN 0x80000000

typedef struct BlnetConnection
{
    int sock;
    int count;
    unsigned char mode;
    int addressInc;
    int addressEnd;
    int actualSize;
    int fetchSize;
    int canFrames;
    in_addr_t address;
    in_port_t port;
} BLNETConn;

typedef struct BLNETParser
{
    double collector;
    double buffer_bottom;
    double buffer_top;
    double circulation;
    double return_flow;
    int digital[9];
    double speed[1];
    double energy;
    double power[2];
} BLNETData;

void parse_blnet_data(BLNETData *parsed_data, unsigned char *data, size_t data_len);

void connect_to_bootloader(BLNETConn *conn)
{
    if (conn->sock == -1)
    {
        conn->sock = socket(AF_INET, SOCK_STREAM, 0);
        if (conn->sock == -1)
        {
            perror("Could not create socket");
            exit(1);
        }

        struct sockaddr_in server;
        server.sin_family = AF_INET;
        server.sin_addr.s_addr = conn->address;
        server.sin_port = conn->port;

        if (connect(conn->sock, (struct sockaddr *)&server, sizeof(server)) < 0)
        {
            close(conn->sock);
            perror("Connection failed");
            exit(1);
        }
        else
        {
            fprintf(stdout, "Successfully connected to BL-NET\n");
        }
    }
}

void disconnect_from_bootloader(BLNETConn *conn)
{
    if (conn->sock != -1)
    {
        close(conn->sock);
        conn->sock = -1;
    }
}

void send_command(int sock, const unsigned char *cmd, int length)
{
    if (send(sock, cmd, length, 0) < 0)
    {
        perror("Send failed");
        exit(1);
    }
}

int receive_response(int sock, unsigned char *buffer, int length)
{
    int received = recv(sock, buffer, length, 0);
    if (received < 0)
    {
        perror("Receive failed");
        exit(1);
    }
    return received;
}

int checksum(const unsigned char *data, int length)
{
    if (length == 1)
        return 1;
    unsigned int sum = 0;
    for (int i = 0; i < length - 1; i++)
        sum += data[i];
    return (sum % 256) == data[length - 1];
}

void query(BLNETConn *conn, const unsigned char *cmd, int cmd_len, unsigned char *response, int resp_len)
{
    send_command(conn->sock, cmd, cmd_len);
    receive_response(conn->sock, response, resp_len);

    if (!checksum(response, resp_len))
    {
        fprintf(stderr, "Checksum error\n");
        exit(1);
    }
}

void get_latest(BLNETConn *conn, BLNETData *data)
{
    unsigned char cmd[] = {GET_LATEST};
    unsigned char response[conn->actualSize];

    for (int i = 0; i < MAX_RETRIES; i++)
    {
        query(conn, cmd, sizeof(cmd), response, sizeof(response));
        if (response[0] != WAIT_TIME)
        {
            printf("Got latest data\n");

            for (size_t i = 0; i < conn->actualSize; i++)
                printf("%d ", response[i]);
            printf("\n");

            parse_blnet_data(data, response, conn->actualSize);

            for (size_t i = 0; i < 5; i++)
                printf("%f\n", *(&(data->collector) + sizeof(double) * i));
            printf("%f\n", data->energy);
            return;
        }
    }
    fprintf(stderr, "Max retries reached\n");
    exit(1);
}

void get_mode(BLNETConn *conn)
{
    unsigned char cmd[] = {GET_MODE};
    unsigned char response[1];

    query(conn, cmd, sizeof(cmd), response, sizeof(response));
}

// Helper function to calculate the value
double calculate_value(int value, double multiplier, int positive_mask, int signbit)
{
    int result = value & positive_mask;
    if (value & signbit)
        result = -((result ^ positive_mask) + 1);
    return result * multiplier;
}

// Convert analog sensor data
double convert_analog(short value)
{
    int mask = value & TYPE_MASK;
    if (mask == TYPE_TEMP)
        return calculate_value(value, 0.1, POSITIVE_VALUE_MASK, SIGN_BIT);
    else if (mask == TYPE_VOLUME)
        return calculate_value(value, 4, POSITIVE_VALUE_MASK, SIGN_BIT);
    else if (mask == TYPE_DIGITAL)
        return (value & SIGN_BIT) ? 1 : 0;
    else if (mask == TYPE_RAS)
        return calculate_value(value, 0.1, RAS_POSITIVE_MASK, SIGN_BIT);
    else
        return calculate_value(value, 1, POSITIVE_VALUE_MASK, SIGN_BIT);
}

// Convert digital sensor data
int convert_digital(int value, int position)
{
    return (value & (1 << position)) ? DIGITAL_ON : DIGITAL_OFF;
}

// Convert speed sensor data
double convert_speed(int value)
{
    if (value & SPEED_ACTIVE)
        return -1;
    else
        return value & SPEED_MASK;
}

// Convert energy data
double convert_energy(int mwh, int kwh)
{
    return mwh * 1000 + calculate_value(kwh, 0.1, INT16_POSITIVE_MASK, SIGN_BIT);
}

// Convert power data
double convert_power(int value, int active, int position)
{
    if (active & position)
        return calculate_value(value, 1 / 2560.0, INT32_MASK, INT32_SIGN);
    else
        return -1;
}

// Parse binary data
void parse_blnet_data(BLNETData *parsed_data, unsigned char *data, size_t data_len)
{

    // Extract relevant parts of the data
    unsigned short analog[6];
    int digital;
    int speed;
    int active;
    unsigned long *power = (unsigned long *)malloc(sizeof(unsigned long));
    unsigned short *kwh = (unsigned short *)malloc(sizeof(unsigned short)), *mwh = (unsigned short *)malloc(sizeof(unsigned short));

    memcpy(analog, data + sizeof(unsigned char), 6 * sizeof(unsigned short));
    // memcpy(&digital, data + 32, sizeof(int));
    // memcpy(&speed, data + 36, sizeof(int));
    // memcpy(&active, data + 26, sizeof(int));
    memcpy(power, data + 34, sizeof(unsigned long));
    memcpy(kwh, data + 38, sizeof(unsigned short));
    memcpy(mwh, data + 40, sizeof(unsigned short));

    for (int i = 0; i < 5; i++)
        *(&(parsed_data->collector) + sizeof(double) * i) = convert_analog(analog[i]);

    // for (int i = 0; i < 9; i++)
    //     parsed_data->digital[i] = convert_digital(digital, i);

    // parsed_data->speed[0] = convert_speed(speed);

    parsed_data->energy = convert_energy(*mwh, *kwh);
    // parsed_data->power[i] = convert_power(power[i], active, i);

    free(power);
    free(kwh);
    free(mwh);
}

void send_data(BLNETData *data)
{
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock == -1)
    {
        perror("Could not create socket");
        exit(1);
    }

    struct sockaddr_in loxone;
    loxone.sin_family = AF_INET;
    loxone.sin_addr.s_addr = inet_addr("192.168.90.55");
    loxone.sin_port = htons(7000);
    char msg[255];
    for (size_t i = 0; i < 5; i++)
    {
        snprintf(msg + strlen(msg), (254 - strlen(msg)), "sensor_%d=%.1f;", i, *(&(data->collector) + sizeof(double) * i));
    }
    snprintf(msg + strlen(msg), (254 - strlen(msg)), "energy=%.1f", data->energy);
    fprintf(stdout, "%s\n", msg);

    if (sendto(sock, msg, strlen(msg) + 1, 0, (struct sockaddr *)&loxone, sizeof(loxone)) < 0)
    {
        perror("Sending data failed");
        close(sock);
        exit(1);
    }
    else
    {
        close(sock);
        fprintf(stdout, "Successfully sent data to Loxone\n");
    }
}

int main(int argc, char *argv[])
{
    // if (argc != 3)
    // {
    //     fprintf(stderr, "Usage: %s <IP address> <port>\n", argv[0]);
    //     exit(1);
    // }

    // const char *ip_address = argv[1];
    // int port = atoi(argv[2]);
    // if (port > UINT16_MAX)
    // {
    //     fprintf(stderr, "<port> must be < 65536\n");
    //     exit(1);
    // }

    BLNETConn conn;
    conn.sock = -1;
    conn.count = -1;
    conn.actualSize = 57;
    conn.fetchSize = 65;
    conn.canFrames = 1;
    conn.address = inet_addr("192.168.90.151");
    conn.port = htons(40000);

    while (1)
    {
        // Connect to bootloader using the IP address and port provided as arguments
        connect_to_bootloader(&conn);
        conn.actualSize = 55;
        get_mode(&conn);
        BLNETData data;
        get_latest(&conn, &data);
        send_data(&data);
        disconnect_from_bootloader(&conn);
        sleep(10);
    }

    return 0;
}