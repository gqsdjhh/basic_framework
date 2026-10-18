#ifndef BSP_RC_H
#define BSP_RC_H

#include <stdint.h>
#include "main.h"

#define DEVICE_USART_CNT 3     // C板至多分配3个串口
#define USART_RXBUFF_LIMIT 256 // 如果协议需要更大的buff,请修改这里
#define USART_TXBUFF_SIZE 512  // 每个串口实例发送环形缓冲区的大小,若经常出现队列满的日志,请修改这里

// 模块回调函数,用于解析协议
typedef void (*usart_module_callback)();

/* 发送模式枚举 */
typedef enum
{
    USART_TRANSFER_NONE=0,
    USART_TRANSFER_BLOCKING,
    USART_TRANSFER_IT,
    USART_TRANSFER_DMA,
} USART_TRANSFER_MODE;

// 串口实例结构体,每个module都要包含一个实例.
// 由于串口是独占的点对点通信,所以不需要考虑多个module同时使用一个串口的情况,因此不用加入id;当然也可以选择加入,这样在bsp层可以访问到module的其他信息
typedef struct
{
    uint8_t recv_buff[USART_RXBUFF_LIMIT]; // 预先定义的最大buff大小,如果太小请修改USART_RXBUFF_LIMIT
    uint8_t recv_buff_size;                // 模块接收一包数据的大小
    UART_HandleTypeDef *usart_handle;      // 实例对应的usart_handle
    usart_module_callback module_callback; // 解析收到的数据的回调函数

    /* 发送环形缓冲区相关,IT/DMA模式下USARTSend会先将数据入队,由发送完成中断链式驱动发送 */
    uint8_t tx_buff[USART_TXBUFF_SIZE];    // 发送环形缓冲区(队列)
    uint8_t tx_staging[USART_TXBUFF_SIZE]; // 连续化暂存区,供HAL IT/DMA发送直接读取
    volatile uint16_t tx_front;            // 环形缓冲区队头,指向最旧的数据
    volatile uint16_t tx_back;             // 环形缓冲区队尾,指向写入位置
    volatile uint16_t tx_count;            // 环形缓冲区中排队的字节数,用于区分满/空
    volatile uint16_t tx_in_flight;        // 已交给HAL但尚未发送完成的字节数,0表示空闲
    USART_TRANSFER_MODE tx_mode;           // 最近一次调用的发送模式,链式续发时使用
} USARTInstance;

/* usart 初始化配置结构体 */
typedef struct
{
    uint8_t recv_buff_size;                // 模块接收一包数据的大小
    UART_HandleTypeDef *usart_handle;      // 实例对应的usart_handle
    usart_module_callback module_callback; // 解析收到的数据的回调函数
} USART_Init_Config_s;

/**
 * @brief 注册一个串口实例,返回一个串口实例指针
 *
 * @param init_config 传入串口初始化结构体
 */
USARTInstance *USARTRegister(USART_Init_Config_s *init_config);

/**
 * @brief 启动串口服务,需要传入一个usart实例.一般用于lost callback的情况(使用串口的模块daemon)
 *
 * @param _instance
 */
void USARTServiceInit(USARTInstance *_instance);


/**
 * @brief 通过调用该函数可以发送一帧数据,需要传入一个usart实例,发送buff以及这一帧的长度
 * @note IT/DMA模式下不再直接调用HAL,而是先将数据拷贝至发送环形缓冲区中,由发送完成中断链式驱动发送,
 *       因此短时间内连续调用此接口不会丢帧.
 * @note 队列满时会丢弃最旧的数据(可能截断此前排队帧的头部)并打印错误日志,请控制发送频率或增大USART_TXBUFF_SIZE.
 * @note BLOCKING模式不经过队列直接阻塞发送,请勿与同一实例的IT/DMA模式混用.
 * 
 * @param _instance 串口实例
 * @param send_buf 待发送数据的buffer
 * @param send_size how many bytes to send
 */
void USARTSend(USARTInstance *_instance, uint8_t *send_buf, uint16_t send_size,USART_TRANSFER_MODE mode);

/**
 * @brief 判断串口发送是否已完成(队列为空且没有在途数据)
 * @note 发送接口本身是非阻塞的,该函数仅在需要确保上一帧完全发出总线时使用
 *
 * @param _instance 要判断的串口实例
 * @return uint8_t ready 1, busy 0
 */
uint8_t USARTIsReady(USARTInstance *_instance);

#endif
