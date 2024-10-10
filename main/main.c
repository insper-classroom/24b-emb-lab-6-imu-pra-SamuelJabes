#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "mpu6050.h"

#include <Fusion.h>

#define SAMPLE_PERIOD (0.01f)

#define UART_ID uart0

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;

QueueHandle_t xQueueMouse;

typedef struct {
    int8_t x;
    int8_t y;
} mouse_data_t;

static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}

void mpu6050_task(void *p) {
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    mpu6050_reset();
    int16_t acceleration[3], gyro[3], temp;

    // Inicializar o Fusion
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    while(1) {
        mpu6050_read_raw(acceleration, gyro, &temp);

        // Converter os dados para as unidades necessárias
        FusionVector gyroscope = {
            .axis.x = gyro[0] / 131.0f,  // Conversão para graus/s
            .axis.y = gyro[1] / 131.0f,
            .axis.z = gyro[2] / 131.0f,
        };

        FusionVector accelerometer = {
            .axis.x = acceleration[0] / 16384.0f,  // Conversão para g
            .axis.y = acceleration[1] / 16384.0f,
            .axis.z = acceleration[2] / 16384.0f,
        };

        // Atualizar os dados de fusão de sensores (sem magnetômetro)
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

        // Obter a orientação (roll, pitch e yaw)
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        // Eixo X
        int x = (int) (euler.angle.pitch) * (-1);
        uart_putc_raw(UART_ID, 0);
        uart_putc_raw(UART_ID, (x >> 8) & 0xFF);
        uart_putc_raw(UART_ID, x & 0xFF);
        uart_putc_raw(UART_ID, -1);

        // Eixo Y

        int y = (int) (euler.angle.roll) * (-1);
        uart_putc_raw(UART_ID, 1);
        uart_putc_raw(UART_ID, (y >> 8) & 0xFF);
        uart_putc_raw(UART_ID, y & 0xFF);
        uart_putc_raw(UART_ID, -1);

        // printf("Acc. X = %d, Y = %d, Z = %d - ", acceleration[0], acceleration[1], acceleration[2]);
        // printf("Gyro. X = %d, Y = %d, Z = %d - ", gyro[0], gyro[1], gyro[2]);
        // printf("Temp. = %f\n\n\n", (temp / 340.0) + 36.53);

        int acc;

        if (acceleration[1] < 0) {
            acc = -1 * acceleration[1];
        } else {
            acc = acceleration[1];
        }

        
        if (acc > 17000){
            // printf("Mouse click detectado!\n");
            uart_putc_raw(UART_ID, 2);
            uart_putc_raw(UART_ID, 0);
            uart_putc_raw(UART_ID, ((acc >> 8) & 0xFF));
            uart_putc_raw(UART_ID, -1);
        }

    
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

void main() {
    stdio_init_all();

    xQueueMouse = xQueueCreate(10, sizeof(mouse_data_t));

    xTaskCreate(mpu6050_task, "mpu6050_task", 4096, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true);
}
