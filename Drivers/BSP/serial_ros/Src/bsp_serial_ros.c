#include "bsp_serial_ros.h"
#include "usart.h"

#define DMA_RX_BUFFER_SIZE 256
static uint8_t dma_rx_buffer[DMA_RX_BUFFER_SIZE];
static uint16_t rd_ptr = 0;

void serial_ros_bsp_init(void) {
    // Start DMA reception in circular mode
    HAL_UART_Receive_DMA(&huart1, dma_rx_buffer, DMA_RX_BUFFER_SIZE);
}

bool serial_ros_bsp_send(const uint8_t *data, uint16_t length) {
    if (HAL_UART_Transmit_DMA(&huart1, (uint8_t*)data, length) == HAL_OK) {
        return true;
    }
    return false;
}

bool serial_ros_bsp_read_byte(uint8_t *byte) {
    uint16_t wr_ptr = DMA_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);

    if (rd_ptr != wr_ptr) {
        *byte = dma_rx_buffer[rd_ptr];
        rd_ptr = (rd_ptr + 1) % DMA_RX_BUFFER_SIZE;
        return true;
    }
    return false;
}
