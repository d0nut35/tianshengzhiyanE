/**
 * @file    cat_log.h
 * @brief   基于 SEGGER RTT 的彩色分级日志（全工程统一后端）
 * @author  xmprocat（原版），haoyu（修正与适配）
 * @note    - 全局总开关 LOG_DEBUG：1 编入并经 RTT 输出，0 全部编译为空、零开销
 *          - 后端走 J-Link RTT，不占用串口；用 RTT Viewer / J-Scope 查看
 *          - 模块日志请在各 .c 顶部按 [模块名] 再封一层（见各驱动 .c）
 */

#ifndef CAT_LOG_H
#define CAT_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "SEGGER_RTT.h"

/* 全局总开关：1=经 RTT 输出，0=所有日志宏编译为空（可被 -D 覆盖） */
#ifndef LOG_DEBUG
#define LOG_DEBUG   1
#endif

#if LOG_DEBUG

/* 统一输出原型：设色 + 级别标签 + 正文 + 复位色 + 换行 */
#define LOG_PROTO(label, color, fmt, ...)               \
        SEGGER_RTT_printf(0, "%s%s" fmt "%s\r\n",       \
                          (color), (label),             \
                          ##__VA_ARGS__,                \
                          RTT_CTRL_RESET)

/* 清屏 */
#define LOG_CLEAR()     SEGGER_RTT_WriteString(0, RTT_CTRL_CLEAR)

/* 无级别、无颜色：原样打印 */
#define LOG(fmt, ...)   LOG_PROTO("", "", fmt, ##__VA_ARGS__)

/* 分级彩色日志 */
#define LOGD(fmt, ...)  LOG_PROTO("[D] ", RTT_CTRL_TEXT_BRIGHT_BLUE,   fmt, ##__VA_ARGS__)
#define LOGI(fmt, ...)  LOG_PROTO("[I] ", RTT_CTRL_TEXT_BRIGHT_GREEN,  fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...)  LOG_PROTO("[W] ", RTT_CTRL_TEXT_BRIGHT_YELLOW, fmt, ##__VA_ARGS__)
#define LOGE(fmt, ...)  LOG_PROTO("[E] ", RTT_CTRL_TEXT_BRIGHT_RED,    fmt, ##__VA_ARGS__)

#else  /* 关闭：全部编译为空，参数不求值、不进固件 */

#define LOG_CLEAR()         do {} while (0)
#define LOG(fmt, ...)       do {} while (0)
#define LOGD(fmt, ...)      do {} while (0)
#define LOGI(fmt, ...)      do {} while (0)
#define LOGW(fmt, ...)      do {} while (0)
#define LOGE(fmt, ...)      do {} while (0)

#endif /* LOG_DEBUG */

#ifdef __cplusplus
}
#endif

#endif /* CAT_LOG_H */
