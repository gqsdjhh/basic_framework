# bsp_usart

<p align='right'>neozng1@hnu.edu.cn</p>

> 已实现发送环形缓冲区以解决短时间内连续调用`USARTSend()`发生丢帧的问题。IT/DMA模式不再直接调用HAL,而是将数据入队后由发送完成中断链式驱动发送(参考bsp_can的发送FIFO实现);BLOCKING模式不经过队列,直接阻塞发送。

## 使用说明

若你需要构建新的基于串口的module，首先需要拥有一个`usart_instance`的指针用于操作串口对象。

需要调用`USARTRegister()`并传入配置好的`USART_Init_Config_s`(包含接收包长度、串口硬件句柄、协议解析回调)，函数会返回该module拥有的`USARTInstance`指针。

若要发送数据，调用`USARTSend()`。注意buffsize务必小于你创建的buff的大小，否则造成指针越界后果未知。

串口硬件收到数据时，会将其存入`usart_instance.recv_buff[]`中，当收到完整一包数据，会调用设定的回调函数`module_callback`（即你注册时提供的解析函数）。在此函数中，你可以通过`usart_instance.recv_buff[]`访问串口收到的数据。

## 代码结构

.h文件内包括了外部接口和类型定义,以及模块对应的宏。c文件内为私有函数和外部接口的定义。

## 类型定义

```c
#define DEVICE_USART_CNT 3     // C板至多分配3个串口
#define USART_RXBUFF_LIMIT 256 // 如果协议需要更大的buff,请修改这里
#define USART_TXBUFF_SIZE 512  // 每个串口实例发送环形缓冲区的大小,若经常出现队列满的日志,请修改这里

// 模块回调函数,用于解析协议
typedef void (*usart_module_callback)();

/* 发送模式枚举 */
typedef enum
{
    USART_TRANSFER_NONE = 0,
    USART_TRANSFER_BLOCKING,
    USART_TRANSFER_IT,
    USART_TRANSFER_DMA,
} USART_TRANSFER_MODE;

/* 串口实例结构体,每个module都要包含一个实例 */
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
```

- `DEVICE_USART_CNT`是开发板上可用的串口数量。

- `USART_RXBUFF_LIMIT`是串口单次接收的数据长度上限，暂时设为256，如果需要更大的buffer容量，修改该值。

- `USART_TXBUFF_SIZE`是每个串口实例发送环形缓冲区的大小（字节），队列满时会丢弃最旧的数据，若日志中频繁出现`USART Send FIFO full`，说明总线吞吐不足或缓冲区太小，需要修改该值。

- `usart_module_callback()`是模块提供给串口接收中断回调函数使用的协议解析函数指针。对于每个需要串口的模块，需要定义一个这样的函数用于解包数据。

- 每定义一个`usart_instance`，就代表一个串口的**实例**（对象）。一个串口实例内有接收buffer，单个数据包的大小，该串口对应的`HAL handle`（代表其使用的串口硬件具体是哪一个）以及用于解包数据的回调函数。发送相关的`tx_buff`/`tx_staging`/索引等成员是**bsp层私有实现，module代码不需要关心**，只在理解bsp_usart.c内部逻辑时使用。

## 外部接口

```c
USARTInstance *USARTRegister(USART_Init_Config_s *init_config);
void USARTServiceInit(USARTInstance *_instance);
void USARTSend(USARTInstance *_instance, uint8_t *send_buf, uint16_t send_size, USART_TRANSFER_MODE mode);
uint8_t USARTIsReady(USARTInstance *_instance);
```

`USARTRegister`是用于初始化串口对象的接口，调用时传入初始化配置结构体，注册成功后会自动开启DMA接收。`USARTServiceInit`用于手动重启某个实例的接收服务，一般用于lost callback的情况(使用串口的模块daemon)。

`USARTSend()`是模块通过其拥有的串口对象发送数据的接口，调用时传入的参数为串口实例指针，发送缓存，此次要发送的数据长度（8-bit\*n)以及发送模式：

- `USART_TRANSFER_BLOCKING`：直接调用`HAL_UART_Transmit()`阻塞发送，**不经过发送队列**，请勿与同一实例的IT/DMA模式混用。
- `USART_TRANSFER_IT`/`USART_TRANSFER_DMA`：数据先被拷贝至发送环形缓冲区中，随后立即触发一次发送尝试；若上一帧未发送完成，数据留在队列中，由发送完成中断(`HAL_UART_TxCpltCallback`)链式驱动续发，即"入队→发送→发送完成中断→继续出队发送"。

`USARTIsReady()`用于判断发送队列是否已排空（无在途数据且队列为空）。由于`USARTSend()`本身是非阻塞的，该函数仅在需要确保上一帧完全发出总线时使用（如发送后立刻切换串口配置的场景）。

## 私有函数和变量

在.c文件内设为static的函数和变量

```c
static uint8_t idx;
static USARTInstance *usart_instance[DEVICE_USART_CNT];
```

这是bsp层管理所有串口实例的入口。

```c
static uint16_t USARTTxPending(const USARTInstance *_instance)
static void USARTTxEnqueue(USARTInstance *_instance, const uint8_t *send_buf, uint16_t send_size)
static void USARTTxStart(USARTInstance *_instance)
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
```

- `USARTTxPending()`通过`tx_count`获取环形缓冲区中排队的数据量（不包含已交给HAL的在途数据）；独立计数避免了`tx_front == tx_back`时满/空无法区分的问题。

- `USARTTxEnqueue()`是发送队列的入队函数，在临界区内操作：数据按回绕分两段拷贝至队尾；当队列空余空间不足时，`tx_front`前移丢弃最旧的数据（与bsp_can行为一致），因此**被丢弃的字节可能截断此前排队帧的头部**。入队数据为拷贝，调用者可以立即复用`send_buf`。由于在途数据存放在`tx_staging`中，丢弃操作永远不会破坏正在发送的帧。

- `USARTTxStart()`是出队发送函数：在临界区内将环形缓冲区中的数据按回绕分两段连续化拷贝到`tx_staging`（拷贝完成后环形区清空，数据所有权移交staging），并按最近一次记录的`tx_mode`调用`HAL_UART_Transmit_IT/DMA()`。若上一帧仍在途(`tx_in_flight`非0)或HAL未就绪则直接返回，数据留在队列中等待下一次触发。

- `HAL_UARTEx_RxEventCallback()`在发生**IDLE中断**或**单次DMA传输中断**后会被调用（说明收到了完整的一包数据），随后在里面根据中断来源，调用拥有产生了该中断的模块的协议解析函数进行数据解包。

- `HAL_UART_ErrorCallback()`在串口传输出错的时候会被调用，重新开启接收；此外，若发送DMA出错，HAL会将`gState`置回READY并调用此回调，而TxCplt回调不会再触发，此时函数会复位`tx_in_flight`并继续驱动发送队列，防止发送永久停滞。

- `HAL_UART_TxCpltCallback()`是发送完成中断回调，每当一个IT/DMA发送完成（`gState`已被HAL置回READY），就复位`tx_in_flight`并调用`USARTTxStart()`续发队列中的下一段数据，形成链式驱动。

**并发保护**：入队发生在任务上下文，出队续发发生在发送完成中断上下文，存在竞态，因此队列操作使用BASEPRI关闭优先级5及以上的中断进行保护（与bsp_can相同，对外部中断实时性有轻微影响）。发送队列的临界区内包含最多`USART_TXBUFF_SIZE`字节的memcpy，请注意自定义缓冲区大小时不要设置得过大。

## 注意事项

- 发送流程是非阻塞的：即使总线上没有接收方，发送队列也不会把任务卡死；但此时数据会不断累积，最终丢弃最旧的数据并打印`USART Send FIFO full`日志。
- 队列满时的丢帧策略是丢弃最旧的字节，可能造成此前排队的帧内容不完整，若某module的发送频率超过串口吞吐，请控制发送频率或增大`USART_TXBUFF_SIZE`。
- 单帧长度超过`USART_TXBUFF_SIZE`的发送请求会被直接拒绝并打印错误日志。
- 同一实例混用BLOCKING与IT/DMA模式会导致发送时序错乱（gState检查会延迟队列发送并打印警告），请不要这样做。
- 若同一实例的多个调用点使用不同的IT/DMA模式，队列续发时以最近一次调用的模式为准（last-call-wins）。
- 不要在其他地方重定义`HAL_UART_TxCpltCallback()`，否则会覆盖本模块的发送队列驱动。
