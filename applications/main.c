/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-05-29     RT-Thread    first version
 */

#include <board.h>
#include <rtthread.h>
#include <stm32f1xx.h>
#include <rtdevice.h>
#include "drv_common.h"
#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/*主机ID定义*/
#define MASTER_ID   0x110
/*各电机ID定义*/
#define MOTOR1_ID   0x141
#define MOTOR2_ID   0x142
#define MOTOR3_ID   0x143
#define MOTOR4_ID   0x144
#define MOTOR5_ID   0x145
#define MOTOR6_ID   0x146
#define MOTOR7_ID   0x147
#define MOTOR8_ID   0x148

/*电机控制指令定义*/
#define TORQUE_CURRENT_CMD 0xA1
#define STATE_QUEST_CMD    0x9C
#define MOTOR_ON_CMD       0x81
#define MOTOR_OFF_CMD      0x88

#define LED0_PIN GET_PIN(C,6)
#define LED1_PIN GET_PIN(C,7)
#define KEY0_PIN 40
#define KEY1_PIN 41

/* RS485A串口名称和收发控制引脚定义*/
#define RS485A_UART_NAME       "uart3"
#define RS485A_RE_PIN GET_PIN(B,1)

/* CAN 设备名称定义 */
#define CAN_DEV_NAME       "can1"

/* 串口接收消息结构*/
struct rs485_serial_rx_msg
{
    rt_device_t dev;
    rt_size_t size;
};

/* 用于CAN接收消息的信号量 */
static struct rt_semaphore can_rx_sem;
/* CAN设备句柄 */
static rt_device_t can_dev;
/*电机启停的信号量*/
static struct rt_semaphore motor_on_sem, motor_off_sem;
/* 串口设备句柄 */
static rt_device_t rs485_serial_device_handle;
/* 消息队列控制块 */
static struct rt_messagequeue rs485_serial_rx_mq;
/* 串口配置结构体 */
struct serial_configure rs485_serial_config = RT_SERIAL_CONFIG_DEFAULT;
struct rt_can_msg can_tx_msg;
/*向上位机发送数据数组*/
uint8_t serial_tx_buffer[35];

/* 串口接收数据回调函数 */
static rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
    struct rs485_serial_rx_msg msg;
    rt_err_t result;
    msg.dev = dev;
    msg.size = size;

    /* 向消息队列发送消息 */
    result = rt_mq_send(&rs485_serial_rx_mq, &msg, sizeof(msg));
    if (result == -RT_EFULL)
    {
        /* 消息队列满 */
        rt_kprintf("message queue full！\n");
    }
    return result;
}

/* 串口处理接收数据线程入口 */
static void rs485_serial_thread_entry(void *parameter)
{
    rt_size_t size;
    struct rs485_serial_rx_msg msg;
    struct rt_can_msg can_tx_msg = { 0 };
    rt_err_t result;
    rt_uint32_t rx_length;
    rt_uint8_t check_sum, check_index;
    static char rx_buffer[RT_SERIAL_RB_BUFSZ + 1];

    while (1)
    {
        rt_memset(&msg, 0, sizeof(msg));
        /* 从消息队列中读取消息*/
        result = rt_mq_recv(&rs485_serial_rx_mq, &msg, sizeof(msg), RT_WAITING_FOREVER);
        if (result == RT_EOK)
        {
            //rt_kprintf("%d\n", msg.size);
            /* 从串口读取数据*/

            rx_length = rt_device_read(msg.dev, 0, rx_buffer, msg.size);

//            rt_kprintf("%02x\n", rx_buffer[0]);
//            rt_kprintf("%02x\n", rx_buffer[1]);
//            rt_kprintf("%02x\n", rx_buffer[2]);
//            rt_kprintf("%02x\n", rx_buffer[3]);
//            rt_kprintf("%02x\n", rx_buffer[4]);
//            rt_kprintf("%02x\n", rx_buffer[5]);
//            rt_kprintf("%02x\n", rx_buffer[6]);
//            rt_kprintf("%02x\n", rx_buffer[7]);
//            rt_kprintf("%02x\n", rx_buffer[8]);
//            rt_kprintf("%02x\n", rx_buffer[9]);
//            rt_kprintf("%02x\n", rx_buffer[10]);
//            rt_kprintf("%02x\n", rx_buffer[11]);
//            rt_kprintf("%02x\n", rx_buffer[12]);
//            rt_kprintf("%02x\n", rx_buffer[13]);
//            rt_kprintf("%02x\n", rx_buffer[14]);
//            rt_kprintf("%02x\n", rx_buffer[15]);
//            rt_kprintf("%02x\n", rx_buffer[16]);
//            rt_kprintf("%02x\n", rx_buffer[17]);
//            rt_kprintf("%02x\n", rx_buffer[18]);
//            rt_kprintf("%02x\n", rx_buffer[19]);
//            rt_kprintf("%02x\n", rx_buffer[20]);
//            rt_kprintf("%02x\n", rx_buffer[21]);
            if ((rx_buffer[0] == 0x5A) && (rx_buffer[18] == 0xA5))
            {
                //rt_kprintf("recevice success\n");
                //判断指令类型，若不为0xFF，则是转矩控制指令,否则为状态查询指令
                if ((rx_buffer[1] != 0xFF) && (rx_buffer[17] != 0xFF))
                {
                    check_sum = 0;
                    for (check_index = 1; check_index < 17; check_index++)
                    {
                        check_sum += rx_buffer[check_index];
                    }
                    if ((check_sum != rx_buffer[17]) && (rx_buffer[17] != 0x00))
                    {
                        rt_kprintf("check_sum wrong\n");
                        rt_kprintf("%02x\n", check_sum);
                    }
                    else
                    {
                        //rt_kprintf("check_sum right\n");

                        /* 向电机发送转矩电流控制数据 */
                        can_tx_msg.ide = RT_CAN_STDID; /* 标准格式 */
                        can_tx_msg.rtr = RT_CAN_DTR; /* 数据帧 */
                        can_tx_msg.len = 8; /* 数据长度为 8 */
                        can_tx_msg.data[0] = TORQUE_CURRENT_CMD;
                        can_tx_msg.data[1] = 0x00;
                        can_tx_msg.data[2] = 0x00;
                        can_tx_msg.data[3] = 0x00;
                        can_tx_msg.data[6] = 0x00;
                        can_tx_msg.data[7] = 0x00;

                        /* CAN向电机1发送数据 */
                        can_tx_msg.id = MOTOR1_ID;
                        can_tx_msg.data[4] = rx_buffer[1];
                        can_tx_msg.data[5] = rx_buffer[2];
                        /* 发送一帧 CAN 数据 */
                        size = rt_device_write(can_dev, 0, &can_tx_msg, sizeof(can_tx_msg));

                        /* CAN向电机2发送数据 */
                        can_tx_msg.id = MOTOR2_ID;
                        can_tx_msg.data[4] = rx_buffer[3];
                        can_tx_msg.data[5] = rx_buffer[4];
                        /* 发送一帧 CAN 数据 */
                        size = rt_device_write(can_dev, 0, &can_tx_msg, sizeof(can_tx_msg));

                        /* CAN向电机3发送数据 */
                        can_tx_msg.id = MOTOR3_ID;
                        can_tx_msg.data[4] = rx_buffer[5];
                        can_tx_msg.data[5] = rx_buffer[6];
                        /* 发送一帧 CAN 数据 */
                        size = rt_device_write(can_dev, 0, &can_tx_msg, sizeof(can_tx_msg));

                        /* CAN向电机4发送数据 */
                        can_tx_msg.id = MOTOR4_ID;
                        can_tx_msg.data[4] = rx_buffer[7];
                        can_tx_msg.data[5] = rx_buffer[8];
                        /* 发送一帧 CAN 数据 */
                        size = rt_device_write(can_dev, 0, &can_tx_msg, sizeof(can_tx_msg));

                        /* CAN向电机5发送数据 */
                        can_tx_msg.id = MOTOR5_ID;
                        can_tx_msg.data[4] = rx_buffer[9];
                        can_tx_msg.data[5] = rx_buffer[10];
                        /* 发送一帧 CAN 数据 */
                        size = rt_device_write(can_dev, 0, &can_tx_msg, sizeof(can_tx_msg));

                        /* CAN向电机6发送数据 */
                        can_tx_msg.id = MOTOR6_ID;
                        can_tx_msg.data[4] = rx_buffer[11];
                        can_tx_msg.data[5] = rx_buffer[12];
                        /* 发送一帧 CAN 数据 */
                        size = rt_device_write(can_dev, 0, &can_tx_msg, sizeof(can_tx_msg));

                        /* CAN向电机7发送数据 */
                        can_tx_msg.id = MOTOR7_ID;
                        can_tx_msg.data[4] = rx_buffer[13];
                        can_tx_msg.data[5] = rx_buffer[14];
                        /* 发送一帧 CAN 数据 */
                        size = rt_device_write(can_dev, 0, &can_tx_msg, sizeof(can_tx_msg));

                        /* CAN向电机8发送数据 */
                        can_tx_msg.id = MOTOR8_ID;
                        can_tx_msg.data[4] = rx_buffer[15];
                        can_tx_msg.data[5] = rx_buffer[16];
                        /* 发送一帧 CAN 数据 */
                        size = rt_device_write(can_dev, 0, &can_tx_msg, sizeof(can_tx_msg));
                        for (check_index = 1; check_index < 33; check_index++)
                        {
                            serial_tx_buffer[33] += serial_tx_buffer[check_index];
                        }
                        rt_pin_write(RS485A_RE_PIN, PIN_HIGH);/*使485处于发送模式*/
                        rt_device_write(rs485_serial_device_handle, 0, serial_tx_buffer, sizeof(serial_tx_buffer));
                        rt_pin_write(RS485A_RE_PIN, PIN_LOW);/*使485处于接收模式*/
                    }
                }
                else
                {
                    /* 向电机发送状态查询指令 */
                    can_tx_msg.ide = RT_CAN_STDID; /* 标准格式 */
                    can_tx_msg.rtr = RT_CAN_DTR; /* 数据帧 */
                    can_tx_msg.len = 8; /* 数据长度为 8 */
                    can_tx_msg.data[0] = STATE_QUEST_CMD;/* 指令类型为状态查询 */
                    can_tx_msg.data[1] = 0x00;
                    can_tx_msg.data[2] = 0x00;
                    can_tx_msg.data[3] = 0x00;
                    can_tx_msg.data[4] = 0x00;
                    can_tx_msg.data[5] = 0x00;
                    can_tx_msg.data[6] = 0x00;
                    can_tx_msg.data[7] = 0x00;

                    /* CAN向电机1发送数据 */
                    can_tx_msg.id = MOTOR1_ID;
                    /* 发送一帧 CAN 数据 */
                    size = rt_device_write(can_dev, 0, &can_tx_msg, sizeof(can_tx_msg));

                    /* CAN向电机2发送数据 */
                    can_tx_msg.id = MOTOR2_ID;
                    /* 发送一帧 CAN 数据 */
                    size = rt_device_write(can_dev, 0, &can_tx_msg, sizeof(can_tx_msg));

                    /* CAN向电机3发送数据 */
                    can_tx_msg.id = MOTOR3_ID;
                    /* 发送一帧 CAN 数据 */
                    size = rt_device_write(can_dev, 0, &can_tx_msg, sizeof(can_tx_msg));

                    /* CAN向电机4发送数据 */
                    can_tx_msg.id = MOTOR4_ID;
                    /* 发送一帧 CAN 数据 */
                    size = rt_device_write(can_dev, 0, &can_tx_msg, sizeof(can_tx_msg));

                    /* CAN向电机5发送数据 */
                    can_tx_msg.id = MOTOR5_ID;
                    /* 发送一帧 CAN 数据 */
                    size = rt_device_write(can_dev, 0, &can_tx_msg, sizeof(can_tx_msg));

                    /* CAN向电机6发送数据 */
                    can_tx_msg.id = MOTOR6_ID;
                    /* 发送一帧 CAN 数据 */
                    size = rt_device_write(can_dev, 0, &can_tx_msg, sizeof(can_tx_msg));

                    /* CAN向电机7发送数据 */
                    can_tx_msg.id = MOTOR7_ID;
                    /* 发送一帧 CAN 数据 */
                    size = rt_device_write(can_dev, 0, &can_tx_msg, sizeof(can_tx_msg));

                    /* CAN向电机8发送数据 */
                    can_tx_msg.id = MOTOR8_ID;
                    /* 发送一帧 CAN 数据 */
                    size = rt_device_write(can_dev, 0, &can_tx_msg, sizeof(can_tx_msg));

                    //通过485向上位机发送数据
                    for (check_index = 1; check_index < 33; check_index++)
                    {
                        serial_tx_buffer[33] += serial_tx_buffer[check_index];
                    }
                    rt_pin_write(RS485A_RE_PIN, PIN_HIGH);/*使485处于发送模式*/
                    rt_device_write(rs485_serial_device_handle, 0, serial_tx_buffer, sizeof(serial_tx_buffer));
                    rt_pin_write(RS485A_RE_PIN, PIN_LOW);/*使485处于接收模式*/
                }
            }
        }
    }
}

/* rs485串口接收数据线程初始化 */
static rt_err_t uart_dma_thread_init()
{
    rt_err_t ret = RT_EOK;
    static char msg_pool[256];

    /* 初始化消息队列 */
    rt_mq_init(&rs485_serial_rx_mq, "rx_mq", msg_pool, /* 存放消息的缓冲区 */
    sizeof(struct rs485_serial_rx_msg), /* 一条消息的最大长度 */
    sizeof(msg_pool), /* 存放消息的缓冲区大小 */
    RT_IPC_FLAG_FIFO); /* 如果有多个线程等待，按照先来先得到的方法分配消息 */

    /* 收发控制引脚使能并拉低 */
    rt_pin_mode(RS485A_RE_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(RS485A_RE_PIN, PIN_LOW);

    /* 查找串口设备 */
    rs485_serial_device_handle = rt_device_find(RS485A_UART_NAME);
    if (!rs485_serial_device_handle)
    {
        rt_kprintf("find %s failed!\n", RS485A_UART_NAME);
        return RT_ERROR;
    }

    /* 修改串口配置参数 */
    rs485_serial_config.baud_rate = BAUD_RATE_115200;        //修改波特率为 115200
    rs485_serial_config.data_bits = DATA_BITS_8;           //数据位 8
    rs485_serial_config.stop_bits = STOP_BITS_1;           //停止位 1
    rs485_serial_config.bufsz = 128;                   //修改缓冲区 buff size 为 128
    rs485_serial_config.parity = PARITY_NONE;           //无奇偶校验位

    /* 控制串口设备。通过控制接口传入命令控制字，与控制参数 */
    rt_device_control(rs485_serial_device_handle, RT_DEVICE_CTRL_CONFIG, &rs485_serial_config);

    /* 以 DMA接收及轮询发送方式打开串口设备 */
    rt_device_open(rs485_serial_device_handle, RT_DEVICE_FLAG_DMA_RX);

    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(rs485_serial_device_handle, uart_input);

    /* 创建 serial 线程，优先级6 */
    rt_thread_t serial_thread_handle = rt_thread_create("rs485_serial", rs485_serial_thread_entry, RT_NULL, 2048, 6,
            10);
    /* 创建成功则启动线程 */
    if (serial_thread_handle != RT_NULL)
    {
        rt_thread_startup(serial_thread_handle);
    }
    else
    {
        ret = RT_ERROR;
    }

    return ret;
}

/* CAN接收数据回调函数 */
static rt_err_t can_rx_call(rt_device_t dev, rt_size_t size)
{
    /* CAN 接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
    rt_sem_release(&can_rx_sem);

    return RT_EOK;
}

/* CAN接收数据处理线程 */
static void can_rx_thread_entry(void *parameter)
{
    struct rt_can_msg rxmsg = { 0 };

    serial_tx_buffer[0] = 0x5A;
    serial_tx_buffer[34] = 0xA5;
    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(can_dev, can_rx_call);
    /*不使用硬件过滤表*/
#ifdef RT_CAN_USING_HDR
    struct rt_can_filter_item items[5] =
    {
        RT_CAN_FILTER_ITEM_INIT(0x100, 0, 0, 1, 0x700, RT_NULL, RT_NULL), /* std,match ID:0x100~0x1ff，hdr 为 - 1，设置默认过滤表 */
        RT_CAN_FILTER_ITEM_INIT(0x300, 0, 0, 1, 0x700, RT_NULL, RT_NULL), /* std,match ID:0x300~0x3ff，hdr 为 - 1 */
        RT_CAN_FILTER_ITEM_INIT(0x211, 0, 0, 1, 0x7ff, RT_NULL, RT_NULL), /* std,match ID:0x211，hdr 为 - 1 */
        RT_CAN_FILTER_STD_INIT(0x486, RT_NULL, RT_NULL), /* std,match ID:0x486，hdr 为 - 1 */
        {   0x555, 0, 0, 1, 0x7ff, 7,} /* std,match ID:0x555，hdr 为 7，指定设置 7 号过滤表 */
    };
    struct rt_can_filter_config cfg =
    {   5, 1, items}; /* 一共有 5 个过滤表 */
    /* 设置硬件过滤表 */
    res = rt_device_control(can_dev, RT_CAN_CMD_SET_FILTER, &cfg);
    RT_ASSERT(res == RT_EOK);
#endif

    while (1)
    {
        /* hdr 值为 - 1，表示直接从 uselist 链表读取数据 */
        rxmsg.hdr = -1;
        /* 阻塞等待接收信号量 */
        rt_sem_take(&can_rx_sem, RT_WAITING_FOREVER);
        /* 从 CAN 读取一帧数据 */
        rt_device_read(can_dev, 0, &rxmsg, sizeof(rxmsg));
        /* 打印数据 ID 及内容 */
//        rt_kprintf("ID:%x", rxmsg.id);
//        for (i = 0; i < 8; i++)
//        {
//            rt_kprintf("%2x", rxmsg.data[i]);
//        }
        //rt_kprintf("\n");
        // rt_kprintf("state_quest\n");
        /*根据id号判断是哪个电机，并把相应的值放进数据发送数组中*/
        /*由于控制指令和查询状态指令的返回值格式相同，这里不做区分*/
        if (rxmsg.data[0] == STATE_QUEST_CMD || rxmsg.data[0] == TORQUE_CURRENT_CMD)/*判断是不是状态查询帧*/
        {
            switch (rxmsg.id)
            {
            case MOTOR1_ID:
                serial_tx_buffer[1] = rxmsg.data[2];
                serial_tx_buffer[2] = rxmsg.data[3];
                serial_tx_buffer[17] = rxmsg.data[6];
                serial_tx_buffer[18] = rxmsg.data[7];
                break;
            case MOTOR2_ID:
                serial_tx_buffer[3] = rxmsg.data[2];
                serial_tx_buffer[4] = rxmsg.data[3];
                serial_tx_buffer[19] = rxmsg.data[6];
                serial_tx_buffer[20] = rxmsg.data[7];
                break;
            case MOTOR3_ID:
                serial_tx_buffer[5] = rxmsg.data[2];
                serial_tx_buffer[6] = rxmsg.data[3];
                serial_tx_buffer[21] = rxmsg.data[6];
                serial_tx_buffer[22] = rxmsg.data[7];
                break;
            case MOTOR4_ID:
                serial_tx_buffer[7] = rxmsg.data[2];
                serial_tx_buffer[8] = rxmsg.data[3];
                serial_tx_buffer[23] = rxmsg.data[6];
                serial_tx_buffer[24] = rxmsg.data[7];
                break;
            case MOTOR5_ID:
                serial_tx_buffer[9] = rxmsg.data[2];
                serial_tx_buffer[10] = rxmsg.data[3];
                serial_tx_buffer[25] = rxmsg.data[6];
                serial_tx_buffer[26] = rxmsg.data[7];
                break;
            case MOTOR6_ID:
                serial_tx_buffer[11] = rxmsg.data[2];
                serial_tx_buffer[12] = rxmsg.data[3];
                serial_tx_buffer[27] = rxmsg.data[6];
                serial_tx_buffer[28] = rxmsg.data[7];
                break;
            case MOTOR7_ID:
                serial_tx_buffer[13] = rxmsg.data[2];
                serial_tx_buffer[14] = rxmsg.data[3];
                serial_tx_buffer[29] = rxmsg.data[6];
                serial_tx_buffer[30] = rxmsg.data[7];
                break;
            case MOTOR8_ID:/*收到电机8数据，说明所有数据均发送完成*/
                serial_tx_buffer[15] = rxmsg.data[2];
                serial_tx_buffer[16] = rxmsg.data[3];
                serial_tx_buffer[31] = rxmsg.data[6];
                serial_tx_buffer[32] = rxmsg.data[7];
                break;
            default:
                break;
            }
        }

    }
}
/* CAN接收数据处理初始化 */
static rt_err_t can_rx_thread_init()
{
    rt_err_t res;
    rt_thread_t thread;

    /* 查找 CAN 设备 */
    can_dev = rt_device_find(CAN_DEV_NAME);
    if (!can_dev)
    {
        rt_kprintf("find %s failed!\n", CAN_DEV_NAME);
        return RT_ERROR;
    }

    /* 初始化 CAN 接收信号量 */
    rt_sem_init(&can_rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);

    /* 以中断接收及发送方式打开 CAN 设备 */
    res = rt_device_open(can_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
    res = rt_device_control(can_dev, RT_CAN_CMD_SET_BAUD, (void *) CAN1MBaud);
    res = rt_device_control(can_dev, RT_CAN_CMD_SET_MODE, (void *) RT_CAN_MODE_NORMAL);
    RT_ASSERT(res == RT_EOK);
    /* 创建数据接收线程,优先级为5 */
    thread = rt_thread_create("can_rx", can_rx_thread_entry, RT_NULL, 2048, 5, 10);
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    else
    {
        res = RT_ERROR;
    }
    return res;
}

//led灯初始化函数
static void led_init(void)
{
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(LED1_PIN, PIN_MODE_OUTPUT);
}
//KEY0电机启动中断函数
void motor_on_irq(void *args)
{
    rt_sem_release(&motor_on_sem);
    rt_pin_write(LED0_PIN, PIN_HIGH);
}
//KEY1电机关闭中断函数
void motor_off_irq(void *args)
{
    rt_sem_release(&motor_off_sem);
    rt_pin_write(LED0_PIN, PIN_HIGH);
}
//电机启动底半处理
static void motor_on_thread_entry()
{

    rt_size_t size;
    struct rt_can_msg can_motor_on_msg;
    /* 电机开启指令初始化 */
    can_motor_on_msg.ide = RT_CAN_STDID; /* 标准格式 */
    can_motor_on_msg.rtr = RT_CAN_DTR; /* 数据帧 */
    can_motor_on_msg.len = 8; /* 数据长度为 8 */
    can_motor_on_msg.data[0] = MOTOR_ON_CMD;/* 指令类型为状态查询 */
    can_motor_on_msg.data[1] = 0x00;
    can_motor_on_msg.data[2] = 0x00;
    can_motor_on_msg.data[3] = 0x00;
    can_motor_on_msg.data[4] = 0x00;
    can_motor_on_msg.data[5] = 0x00;
    can_motor_on_msg.data[6] = 0x00;
    can_motor_on_msg.data[7] = 0x00;
    while (1)
    {
        /* 阻塞等待接收信号量 */
        rt_sem_take(&motor_on_sem, RT_WAITING_FOREVER);
        //rt_kprintf("power up motor!\n");
        /* CAN向电机1发送数据 */
        can_motor_on_msg.id = MOTOR1_ID;
        /* 发送一帧 CAN 数据 */
        size = rt_device_write(can_dev, 0, &can_motor_on_msg, sizeof(can_motor_on_msg));

        /* CAN向电机2发送数据 */
        can_motor_on_msg.id = MOTOR2_ID;
        /* 发送一帧 CAN 数据 */
        size = rt_device_write(can_dev, 0, &can_motor_on_msg, sizeof(can_motor_on_msg));

        /* CAN向电机3发送数据 */
        can_motor_on_msg.id = MOTOR3_ID;
        /* 发送一帧 CAN 数据 */
        size = rt_device_write(can_dev, 0, &can_motor_on_msg, sizeof(can_motor_on_msg));

        /* CAN向电机4发送数据 */
        can_motor_on_msg.id = MOTOR4_ID;
        /* 发送一帧 CAN 数据 */
        size = rt_device_write(can_dev, 0, &can_motor_on_msg, sizeof(can_motor_on_msg));

        /* CAN向电机5发送数据 */
        can_motor_on_msg.id = MOTOR5_ID;
        /* 发送一帧 CAN 数据 */
        size = rt_device_write(can_dev, 0, &can_motor_on_msg, sizeof(can_motor_on_msg));

        /* CAN向电机6发送数据 */
        can_motor_on_msg.id = MOTOR6_ID;
        /* 发送一帧 CAN 数据 */
        size = rt_device_write(can_dev, 0, &can_motor_on_msg, sizeof(can_motor_on_msg));

        /* CAN向电机7发送数据 */
        can_motor_on_msg.id = MOTOR7_ID;
        /* 发送一帧 CAN 数据 */
        size = rt_device_write(can_dev, 0, &can_motor_on_msg, sizeof(can_motor_on_msg));

        /* CAN向电机8发送数据 */
        can_motor_on_msg.id = MOTOR8_ID;
        /* 发送一帧 CAN 数据 */
        size = rt_device_write(can_dev, 0, &can_motor_on_msg, sizeof(can_motor_on_msg));
    }
}
//电机停止底半处理
static void motor_off_thread_entry()
{

    rt_size_t size;
    struct rt_can_msg can_motor_off_msg;
    /* 电机关闭指令初始化 */
    can_motor_off_msg.ide = RT_CAN_STDID; /* 标准格式 */
    can_motor_off_msg.rtr = RT_CAN_DTR; /* 数据帧 */
    can_motor_off_msg.len = 8; /* 数据长度为 8 */
    can_motor_off_msg.data[0] = MOTOR_OFF_CMD;/* 指令类型为状态查询 */
    can_motor_off_msg.data[1] = 0x00;
    can_motor_off_msg.data[2] = 0x00;
    can_motor_off_msg.data[3] = 0x00;
    can_motor_off_msg.data[4] = 0x00;
    can_motor_off_msg.data[5] = 0x00;
    can_motor_off_msg.data[6] = 0x00;
    can_motor_off_msg.data[7] = 0x00;

    while (1)
    {
        /* 阻塞等待接收信号量 */
        rt_sem_take(&motor_off_sem, RT_WAITING_FOREVER);
        //rt_kprintf("power off motor!\n");
        /* CAN向电机1发送数据 */
        can_motor_off_msg.id = MOTOR1_ID;
        /* 发送一帧 CAN 数据 */
        size = rt_device_write(can_dev, 0, &can_motor_off_msg, sizeof(can_motor_off_msg));

        /* CAN向电机2发送数据 */
        can_motor_off_msg.id = MOTOR2_ID;
        /* 发送一帧 CAN 数据 */
        size = rt_device_write(can_dev, 0, &can_motor_off_msg, sizeof(can_motor_off_msg));

        /* CAN向电机3发送数据 */
        can_motor_off_msg.id = MOTOR3_ID;
        /* 发送一帧 CAN 数据 */
        size = rt_device_write(can_dev, 0, &can_motor_off_msg, sizeof(can_motor_off_msg));

        /* CAN向电机4发送数据 */
        can_motor_off_msg.id = MOTOR4_ID;
        /* 发送一帧 CAN 数据 */
        size = rt_device_write(can_dev, 0, &can_motor_off_msg, sizeof(can_motor_off_msg));

        /* CAN向电机5发送数据 */
        can_motor_off_msg.id = MOTOR5_ID;
        /* 发送一帧 CAN 数据 */
        size = rt_device_write(can_dev, 0, &can_motor_off_msg, sizeof(can_motor_off_msg));

        /* CAN向电机6发送数据 */
        can_motor_off_msg.id = MOTOR6_ID;
        /* 发送一帧 CAN 数据 */
        size = rt_device_write(can_dev, 0, &can_motor_off_msg, sizeof(can_motor_off_msg));

        /* CAN向电机7发送数据 */
        can_motor_off_msg.id = MOTOR7_ID;
        /* 发送一帧 CAN 数据 */
        size = rt_device_write(can_dev, 0, &can_motor_off_msg, sizeof(can_motor_off_msg));

        /* CAN向电机8发送数据 */
        can_motor_off_msg.id = MOTOR8_ID;
        /* 发送一帧 CAN 数据 */
        size = rt_device_write(can_dev, 0, &can_motor_off_msg, sizeof(can_motor_off_msg));
    }
}
//电机启停线程初始化函数
static rt_err_t motor_on_off_thread_init(void)
{
    rt_err_t res_on, res_off, res;
    rt_thread_t thread_on, thread_off;
    res = RT_EOK;

    /* 按键0引脚为输入模式 */
    rt_pin_mode(KEY0_PIN, PIN_MODE_INPUT_PULLUP);
    /* 绑定中断，上升沿模式，回调函数名为motor_on */
    rt_pin_attach_irq(KEY0_PIN, PIN_IRQ_MODE_FALLING, motor_on_irq, RT_NULL);
    rt_pin_irq_enable(KEY0_PIN, PIN_IRQ_ENABLE);
    /* 按键1引脚为输入模式 */
    rt_pin_mode(KEY1_PIN, PIN_MODE_INPUT_PULLUP);
    /* 绑定中断，上升沿模式，回调函数名为motor_on */
    rt_pin_attach_irq(KEY1_PIN, PIN_IRQ_MODE_FALLING, motor_off_irq, RT_NULL);
    rt_pin_irq_enable(KEY1_PIN, PIN_IRQ_ENABLE);
    /* 初始化电机启动信号量 */
    rt_sem_init(&motor_on_sem, "motor_on_sem", 0, RT_IPC_FLAG_FIFO);
    /* 初始化电机启动信号量 */
    rt_sem_init(&motor_off_sem, "motor_off_sem", 0, RT_IPC_FLAG_FIFO);

    /* 创建电机启动线程,优先级为4 */
    thread_on = rt_thread_create("motor_on", motor_on_thread_entry, RT_NULL, 1024, 4, 10);
    if (thread_on != RT_NULL)
    {
        rt_thread_startup(thread_on);
    }
    else
    {
        res_on = RT_ERROR;
    }
    /* 创建电机停止线程,优先级为4 */
    thread_off = rt_thread_create("motor_off", motor_off_thread_entry, RT_NULL, 1024, 4, 10);
    if (thread_off != RT_NULL)
    {
        rt_thread_startup(thread_off);
    }
    else
    {
        res_off = RT_ERROR;
    }
    if ((res_on == RT_ERROR) || (res_off == RT_ERROR))
    {
        res = RT_ERROR;
    }
    return res;
}

int main(void)
{

    int count = 1;
    /* LED灯初始化 */
    led_init();

    //电机启停线程初始化
    if (motor_on_off_thread_init() == RT_EOK)
    {
        LOG_D("motor_on_off_init_success!");
    }
    else
    {
        LOG_D("motor_on_off_init_failed!");
    }
    //CAN接收线程初始化
    if (can_rx_thread_init() == RT_EOK)
    {
        LOG_D("can_rx_init_success!");
    }
    else
    {
        LOG_D("can_rx_thread_init_failed!");
    }

    //rs485A串口接收数据线程初始化
    if (uart_dma_thread_init() == RT_EOK)
    {
        LOG_D("uart_thread_init_success!");
    }
    else
    {
        LOG_D("uart_thread_init_failed!");
    }

    rt_pin_write(LED0_PIN, PIN_LOW);
    /*系统运行指示*/
    while (count++)
    {

        rt_pin_write(LED1_PIN, PIN_LOW);
        rt_thread_mdelay(1000);
        rt_pin_write(LED1_PIN, PIN_HIGH);
        rt_thread_mdelay(1000);
    }
    return RT_EOK;
}

