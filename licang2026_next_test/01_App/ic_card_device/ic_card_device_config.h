/**
 * @file    ic_card_device_config.h
 * @brief   当前比赛读球设备的地址、数据块和超时策略。
 *
 * 这些值属于“本项目怎样使用读卡器”，不是厂家串口协议本身，因此不能
 * 放进平台无关协议Core。模块重新配置地址或赛事更换数据块时只改本文件。
 */

#ifndef IC_CARD_DEVICE_CONFIG_H
#define IC_CARD_DEVICE_CONFIG_H

#define IC_CARD_DEVICE_ADDRESS           0x20U
#define IC_CARD_DEVICE_DATA_BLOCK        0x01U
#define IC_CARD_DEVICE_QUEUE_TIMEOUT_MS  0U
#define IC_CARD_DEVICE_READ_TIMEOUT_MS   500U

#endif /* IC_CARD_DEVICE_CONFIG_H */
