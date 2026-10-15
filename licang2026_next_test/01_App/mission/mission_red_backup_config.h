/* 红方备用模式专属调参。单位：坐标/距离mm，速度mm/s，角度deg，时间ms。
 * 第一版独立复制91b53e9正式红方参数；修改本文件不改变正式红蓝点表。
 * 只在起点KEY1锁定红方且PD15按下时使用，禁止运行中切换。
 */
#ifndef MISSION_RED_BACKUP_CONFIG_H
#define MISSION_RED_BACKUP_CONFIG_H

/* 开头圆盘机：导航点与找线后标定点分别调整。 */
#define RED_BACKUP_PLATFORM_X_MM       525
#define RED_BACKUP_PLATFORM_Y_MM       4300
#define RED_BACKUP_PLATFORM_YAW_DEG     180.0f
#define RED_BACKUP_PLATFORM_V_MMS       500.0f
#define RED_BACKUP_PLATFORM_W_DPS       30.0f
#define RED_BACKUP_PLATFORM_FIX_X_MM    473
#define RED_BACKUP_PLATFORM_FIX_Y_MM    4300

/* 仓库入口：A*导航预停点、白线校准点、额外IMU角度补偿。 */
#define RED_BACKUP_DEPOT_X_MM           440
#define RED_BACKUP_DEPOT_Y_MM           2276
#define RED_BACKUP_DEPOT_YAW_DEG         180.0f
#define RED_BACKUP_DEPOT_V_MMS          500.0f
#define RED_BACKUP_DEPOT_W_DPS          60.0f
#define RED_BACKUP_DEPOT_FIX_X_MM       403
#define RED_BACKUP_DEPOT_FIX_Y_MM       2208
#define RED_BACKUP_DEPOT_YAW_OFS_DEG    (-0.32f)

/* 仓库线尾校准与D1~D4横移：D号不随积木数字改变。 */
#define RED_BACKUP_DEPOT_LINE_SENSOR    6U
#define RED_BACKUP_DEPOT_LINE_Y_MM      2144
#define RED_BACKUP_DEPOT_HIGH_COUNT     5U
#define RED_BACKUP_DEPOT_LOCATE_MS      20000U
#define RED_BACKUP_DEPOT_SHIFT_VY_MMS   (-100.0f)
#define RED_BACKUP_DEPOT_POLL_MS        10U
#define RED_BACKUP_DEPOT_D1_Y_MM        2308
#define RED_BACKUP_DEPOT_D2_Y_MM        2507
#define RED_BACKUP_DEPOT_D3_Y_MM        2706
#define RED_BACKUP_DEPOT_D4_Y_MM        2896

/* 回家：与原红方相同的直线导航。 */
#define RED_BACKUP_HOME_X_MM           1215
#define RED_BACKUP_HOME_Y_MM           440
#define RED_BACKUP_HOME_YAW_DEG        180.0f
#define RED_BACKUP_HOME_V_MMS          500.0f
#define RED_BACKUP_HOME_W_DPS          30.0f

/* 备用单段总保护，不改变正式模式期限；不得大于Mission现有30000ms阶段期限。 */
#define RED_BACKUP_OPERATION_MS        30000U
#endif
