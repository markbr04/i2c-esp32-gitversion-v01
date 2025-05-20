#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

#define I2C_MASTER_SCL_IO               19       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO               18      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM                  I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ              100000 /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE       0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE       0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS           5000
#define LOG_LOCAL_LEVEL                 ESP_LOG_VERBOSE             //Alineación marginal desde linux

#define ACS37800_SENSOR_ADDR            0x60        /*!< Address of the MPU9250 sensor */
                                                    /*Cambiando la dirección del sensor*/

#define DATA_SIZE                       4

static const char *TAG = "ejemplo";

uint8_t multi_data[4];

/**
 * @brief i2c master initialization
 */
static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ACS37800_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}


static esp_err_t acs37800_master_probe(i2c_master_bus_handle_t bus_handle, uint16_t address, int xfer_timeout_ms)
{
    esp_err_t ret = i2c_master_probe(bus_handle, ACS37800_SENSOR_ADDR, I2C_MASTER_TIMEOUT_MS);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Dispositivo encontrado en la direccion: 0x%02X", ACS37800_SENSOR_ADDR);
    } else {
        ESP_LOGI(TAG, "Dispositivo no encontrado en la direccion: 0x%02X", ACS37800_SENSOR_ADDR);
    }
    return ret;
}

static esp_err_t read_acs37800(i2c_master_dev_handle_t dev_handle, i2c_operation_job_t *i2c_operation, size_t operation_list_num, int xfer_timeout_ms)
{   
    uint8_t address1 = 0x28;
    
    i2c_operation_job_t operations[] = {
    
    { .command = I2C_MASTER_CMD_START },
    { .command = I2C_MASTER_CMD_WRITE, .write = { .ack_check = false, .data = (uint8_t *) &address1, .total_bytes = 1 } },
    { .command = I2C_MASTER_CMD_STOP },
};
    esp_err_t ret = i2c_master_execute_defined_operations(dev_handle, operations, sizeof(operations)/sizeof(i2c_operation_job_t), I2C_MASTER_TIMEOUT_MS);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Lectura exitosa del sensor ACS37800");
        for (int i = 0; i < sizeof(multi_data); i++)
        {
            printf(" 0x%02X", multi_data[i]);
        }
        
    } else {
        ESP_LOGE(TAG, "Error al leer el sensor ACS37800");
    }
    return ret;
}

    
void app_main(void)
{


    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");

    acs37800_master_probe(bus_handle, ACS37800_SENSOR_ADDR, I2C_MASTER_TIMEOUT_MS);
    read_acs37800(dev_handle, NULL, 0, I2C_MASTER_TIMEOUT_MS);

    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}
