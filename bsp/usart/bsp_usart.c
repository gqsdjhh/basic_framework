/**
 * @file bsp_usart.c
 * @author neozng
 * @brief  串口bsp层的实现
 * @version beta
 * @date 2022-11-01
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "bsp_usart.h"
#include "bsp_log.h"
#include "stdlib.h"
#include "memory.h"

/* usart service instance, modules' info would be recoreded here using USARTRegister() */
/* usart服务实例,所有注册了usart的模块信息会被保存在这里 */
static uint8_t idx;
static USARTInstance *usart_instance[DEVICE_USART_CNT] = {NULL};

/**
 * @brief 启动串口服务,会在每个实例注册之后自动启用接收,当前实现为DMA接收,后续可能添加IT和BLOCKING接收
 *
 * @todo 串口服务会在每个实例注册之后自动启用接收,当前实现为DMA接收,后续可能添加IT和BLOCKING接收
 *       可能还要将此函数修改为extern,使得module可以控制串口的启停
 *
 * @param _instance instance owned by module,模块拥有的串口实例
 */
void USARTServiceInit(USARTInstance *_instance)
{
    HAL_UARTEx_ReceiveToIdle_DMA(_instance->usart_handle, _instance->recv_buff, _instance->recv_buff_size);
    // 关闭dma half transfer中断防止两次进入HAL_UARTEx_RxEventCallback()
    // 这是HAL库的一个设计失误,发生DMA传输完成/半完成以及串口IDLE中断都会触发HAL_UARTEx_RxEventCallback()
    // 我们只希望处理第一种和第三种情况,因此直接关闭DMA半传输中断
    __HAL_DMA_DISABLE_IT(_instance->usart_handle->hdmarx, DMA_IT_HT);
}

USARTInstance *USARTRegister(USART_Init_Config_s *init_config)
{
    if (idx >= DEVICE_USART_CNT) // 超过最大实例数
        while (1)
            LOGERROR("[bsp_usart] USART exceed max instance count!");

    for (uint8_t i = 0; i < idx; i++) // 检查是否已经注册过
        if (usart_instance[i]->usart_handle == init_config->usart_handle)
            while (1)
                LOGERROR("[bsp_usart] USART instance already registered!");

    USARTInstance *instance = (USARTInstance *)malloc(sizeof(USARTInstance));
    memset(instance, 0, sizeof(USARTInstance));

    instance->usart_handle = init_config->usart_handle;
    instance->recv_buff_size = init_config->recv_buff_size;
    instance->module_callback = init_config->module_callback;

    usart_instance[idx++] = instance;
    USARTServiceInit(instance);
    return instance;
}

/* ---------------- 发送环形缓冲区私有实现 ---------------- */

/**
 * @brief 计算环形缓冲区中排队的数据量(不包含已交给HAL发送的在途数据)
 *
 * @param _instance 串口实例
 * @return uint16_t 排队的数据字节数
 */
static uint16_t USARTTxPending(const USARTInstance *_instance)
{
    return _instance->tx_count;
}

/**
 * @brief 将一帧数据入队到发送环形缓冲区.队列空间不足时丢弃最旧的数据(与bsp_can行为一致)
 *
 * @note 调用前需保证send_size <= USART_TXBUFF_SIZE,否则drop计算将存在下溢风险
 *
 * @param _instance 串口实例
 * @param send_buf 待发送数据的buffer
 * @param send_size 数据长度
 */
static void USARTTxEnqueue(USARTInstance *_instance, const uint8_t *send_buf, uint16_t send_size)
{
    uint32_t saved_baspri = __get_BASEPRI();
    __set_BASEPRI(0x50U); // 临界区开始,暂时关闭优先级5及以上的中断,防止与发送完成中断竞态

    uint16_t pending = USARTTxPending(_instance);
    if (USART_TXBUFF_SIZE - pending < send_size)
    { // 队列满,丢弃最旧的数据(只丢弃排队中的数据,在途数据位于staging中不受影响)
        uint16_t drop = send_size - (USART_TXBUFF_SIZE - pending);
        _instance->tx_front = (_instance->tx_front + drop) % USART_TXBUFF_SIZE;
        _instance->tx_count -= drop;
    }

    // 拷贝数据至队尾,回绕时分两段写入
    uint16_t first = USART_TXBUFF_SIZE - _instance->tx_back;
    if (first > send_size)
        first = send_size;
    memcpy(&_instance->tx_buff[_instance->tx_back], send_buf, first);
    if (send_size > first)
        memcpy(_instance->tx_buff, send_buf + first, send_size - first);
    _instance->tx_back = (_instance->tx_back + send_size) % USART_TXBUFF_SIZE;
    _instance->tx_count += send_size;

    __set_BASEPRI(saved_baspri); // 临界区结束,开启中断
}

/**
 * @brief 尝试将队列中的数据连续化拷贝到staging后启动一次HAL发送
 *        由USARTSend()和HAL_UART_TxCpltCallback()调用,形成"入队->发送->发送完成中断->续发"的链式驱动
 *
 * @param _instance 串口实例
 */
static void USARTTxStart(USARTInstance *_instance)
{
    uint32_t saved_baspri = __get_BASEPRI();
    __set_BASEPRI(0x50U); // 临界区开始

    if (_instance->tx_in_flight)
    { // 上一帧还未发送完成,等待发送完成中断续发
        __set_BASEPRI(saved_baspri);
        return;
    }
    if (_instance->usart_handle->gState != HAL_UART_STATE_READY)
    { // HAL发送未就绪(正常流程不应发生,仅可能由混用BLOCKING模式导致),保持数据等待下次触发
        LOGWARNING("[bsp_usart] USART HAL not ready, TX deferred. gState [%d]", _instance->usart_handle->gState);
        __set_BASEPRI(saved_baspri);
        return;
    }

    uint16_t pending = USARTTxPending(_instance);
    if (pending == 0)
    { // 队列为空,没有数据可以发送
        __set_BASEPRI(saved_baspri);
        return;
    }

    // 将环形缓冲区中的数据连续化拷贝至staging,回绕时分两段拷贝
    uint16_t first = USART_TXBUFF_SIZE - _instance->tx_front;
    if (first > pending)
        first = pending;
    memcpy(_instance->tx_staging, &_instance->tx_buff[_instance->tx_front], first);
    if (pending > first)
        memcpy(&_instance->tx_staging[first], _instance->tx_buff, pending - first);

    _instance->tx_front = _instance->tx_back; // 数据所有权移交staging,环形缓冲区清空
    _instance->tx_count = 0;
    _instance->tx_in_flight = pending;

    USART_TRANSFER_MODE mode = _instance->tx_mode; // 快照发送模式,供链式续发使用

    __set_BASEPRI(saved_baspri); // 临界区结束,开启中断

    HAL_StatusTypeDef ret = HAL_ERROR;
    if (mode == USART_TRANSFER_DMA)
        ret = HAL_UART_Transmit_DMA(_instance->usart_handle, _instance->tx_staging, pending);
    else if (mode == USART_TRANSFER_IT)
        ret = HAL_UART_Transmit_IT(_instance->usart_handle, _instance->tx_staging, pending);

    if (ret != HAL_OK)
    { // 正常流程不应失败,staging中的数据已无法找回,打印错误并复位状态等待后续发送
        _instance->tx_in_flight = 0;
        LOGERROR("[bsp_usart] USART TX start failed! HAL ret [%d]", ret);
    }
}

/* @todo 当前仅进行了形式上的封装,后续要进一步考虑是否将module的行为与bsp完全分离 */
void USARTSend(USARTInstance *_instance, uint8_t *send_buf, uint16_t send_size, USART_TRANSFER_MODE mode)
{
    switch (mode)
    {
    case USART_TRANSFER_BLOCKING:
        HAL_UART_Transmit(_instance->usart_handle, send_buf, send_size, 100);
        break;
    case USART_TRANSFER_IT:
    case USART_TRANSFER_DMA:
        if (send_size == 0)
            break;
        if (send_size > USART_TXBUFF_SIZE)
        { // 单帧数据量超过队列容量,必然无法发送
            LOGERROR("[bsp_usart] USART send size [%d] exceeds TX FIFO size [%d], frame dropped!", send_size, USART_TXBUFF_SIZE);
            break;
        }
        _instance->tx_mode = mode; // 记录本次发送模式,链式续发时使用
        if (USART_TXBUFF_SIZE - USARTTxPending(_instance) < send_size)
            // 快速检查用于日志提示,实际丢弃在队列临界区内完成
            LOGERROR("[bsp_usart] USART Send FIFO full! dropping oldest data. size [%d]", send_size);
        USARTTxEnqueue(_instance, send_buf, send_size); // 数据入队
        USARTTxStart(_instance);                        // 触发一次发送,防止队列发送停止
        break;
    default:
        while (1)
            ; // illegal mode! check your code context! 检查定义instance的代码上下文,可能出现指针越界
        break;
    }
}

/* 判断发送队列是否已排空(无在途数据且队列为空) */
uint8_t USARTIsReady(USARTInstance *_instance)
{
    if (_instance->tx_in_flight == 0 && USARTTxPending(_instance) == 0)
        return 1;
    else
        return 0;
}

/**
 * @brief 每次dma/idle中断发生时，都会调用此函数.对于每个uart实例会调用对应的回调进行进一步的处理
 *        例如:视觉协议解析/遥控器解析/裁判系统解析
 *
 * @note  通过__HAL_DMA_DISABLE_IT(huart->hdmarx,DMA_IT_HT)关闭dma half transfer中断防止两次进入HAL_UARTEx_RxEventCallback()
 *        这是HAL库的一个设计失误,发生DMA传输完成/半完成以及串口IDLE中断都会触发HAL_UARTEx_RxEventCallback()
 *        我们只希望处理，因此直接关闭DMA半传输中断第一种和第三种情况
 *
 * @param huart 发生中断的串口
 * @param Size 此次接收到的总数居量,暂时没用
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    for (uint8_t i = 0; i < idx; ++i)
    { // find the instance which is being handled
        if (huart == usart_instance[i]->usart_handle)
        { // call the callback function if it is not NULL
            if (usart_instance[i]->module_callback != NULL)
            {
                usart_instance[i]->module_callback();
                memset(usart_instance[i]->recv_buff, 0, Size); // 接收结束后清空buffer,对于变长数据是必要的
            }
            HAL_UARTEx_ReceiveToIdle_DMA(usart_instance[i]->usart_handle, usart_instance[i]->recv_buff, usart_instance[i]->recv_buff_size);
            __HAL_DMA_DISABLE_IT(usart_instance[i]->usart_handle->hdmarx, DMA_IT_HT);
            return; // break the loop
        }
    }
}

/**
 * @brief 当串口发送/接收出现错误时,会调用此函数,此时这个函数要做的就是重新启动接收
 *
 * @note  最常见的错误:奇偶校验/溢出/帧错误
 *
 * @param huart 发生错误的串口
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < idx; ++i)
    {
        if (huart == usart_instance[i]->usart_handle)
        {
            /* 若发生的是发送DMA错误,HAL会将gState置回READY并调用本回调,而TxCplt回调不会再触发,
               此时tx_in_flight会残留导致发送队列永久停滞,需要复位并继续发送队列(gState为READY用于区分RX错误) */
            if (usart_instance[i]->tx_in_flight != 0 && usart_instance[i]->usart_handle->gState == HAL_UART_STATE_READY)
            {
                LOGWARNING("[bsp_usart] USART TX aborted by error, resume send queue. instance idx [%d]", i);
                usart_instance[i]->tx_in_flight = 0;
                USARTTxStart(usart_instance[i]);
            }
            HAL_UARTEx_ReceiveToIdle_DMA(usart_instance[i]->usart_handle, usart_instance[i]->recv_buff, usart_instance[i]->recv_buff_size);
            __HAL_DMA_DISABLE_IT(usart_instance[i]->usart_handle->hdmarx, DMA_IT_HT);
            LOGWARNING("[bsp_usart] USART error callback triggered, instance idx [%d]", i);
            return;
        }
    }
}

/**
 * @brief 在串口发送完成中断回调中触发发送队列的续发,形成链式驱动
 *
 * @note  对HAL的回调函数的重定义(原定义为__weak),IT和DMA模式发送完成时都会被调用
 *
 * @param huart 发生发送完成中断的串口
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < idx; ++i)
    {
        if (huart == usart_instance[i]->usart_handle)
        {
            usart_instance[i]->tx_in_flight = 0; // 在途数据发送完成
            USARTTxStart(usart_instance[i]);     // 队列非空则继续发送
            return; // break the loop
        }
    }
}