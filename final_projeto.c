// Projeto_Final: BH1750 + AHT10 + OLED 128x64 + ESP8266 + aplicativo android + protocolo MQTT
// Autor: Moisés Lourenço / Felipe 
// Data: 24/01/2026
// Descrição: A caixa de comunicação alternativa Lê os sensores de luminosidade, temperatura e umidade, bem como a interação da criança com a mesma
//           e envia todas estas informações a um servidor que fica disponivel ao cuidador em um dispositivo móvel android via aplicativo. e mostra as 
//           informações no display OLED 128x64.

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "ssd1306.h"
#include "bh1750.h"


// I2C BH1750 (i2c0)
#define I2C_PORT_SENSOR i2c0       
#define SDA_SENSOR 0
#define SCL_SENSOR 1

// I2C SSD1306 (i2c1)   
#define I2C_PORT_OLED i2c1
#define SDA_OLED 14
#define SCL_OLED 15

int main() {
    stdio_init_all();
    // I2C BH1750
    i2c_init(I2C_PORT_SENSOR, 100 * 1000);
    gpio_set_function(SDA_SENSOR, GPIO_FUNC_I2C);
    gpio_set_function(SCL_SENSOR, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_SENSOR);
    gpio_pull_up(SCL_SENSOR);
    bh1750_init(I2C_PORT_SENSOR);
    sleep_ms(200);

    // I2C SSD1306
    i2c_init(I2C_PORT_OLED, 400000);
    gpio_set_function(SDA_OLED, GPIO_FUNC_I2C);
    gpio_set_function(SCL_OLED, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_OLED);
    gpio_pull_up(SCL_OLED);
    
    ssd1306_init(I2C_PORT_OLED);
    // Tela inicial
    ssd1306_clear();
    ssd1306_draw_string(18, 0, "Servo e BH1750");
    ssd1306_show();
    
    while (true) {
        // Lê luminosidade
        float lux = bh1750_read_lux(I2C_PORT_SENSOR);
        if (lux < 0) {
            printf("BH1750 read failed\n");
        } else {
            printf("Lux: %.1f\n", lux);
        }
      
        ssd1306_clear();
        ssd1306_draw_string(18, 0, " BH1750");
        char line1[24];
        snprintf(line1, sizeof(line1), "Lux: %.1f", lux);
        ssd1306_draw_string(6, 20, line1);
        ssd1306_show();
        sleep_ms(1200);
   }
}
