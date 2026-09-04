/**
 * @file    mecanum_chassis.c
 * @brief   麦克纳姆底盘运动学/里程计/路径跟随（纯逻辑，平台无关）
 * @author  haoyu
 * @note    - 仅依赖标准库与 map.h；轮速经注入回调以 rps 交出
 *          - 导航整定为本文件私有宏；几何由适配层经 inst 注入
 *          - 角度一律 rad、长度一律 mm；yaw 有效性由适配层保证
 */

#include "mecanum_chassis.h"

#include <math.h>
#include <string.h>

#ifndef MEC_PI
#define MEC_PI              3.14159265358979f /* 圆周率 */
#endif

/* ---- 导航/路径整定（场景标定，纯逻辑私有，沿用原可用版本默认值） ---- */
#define MEC_POS_TOL_MM      20.0f      /* 点到点位置容差，mm */
#define MEC_YAW_TOL_RAD     0.0349066f /* 航向到位容差，约 2°，rad */
#define MEC_LIN_STEP        10.0f      /* 线速度每周期步长，mm/s */
#define MEC_ANG_STEP        0.3f      /* 角速度每周期步长，rad/s */
#define MEC_LIN_DECEL       300.0f     /* 线减速度，mm/s^2 */
#define MEC_ANG_DECEL       4.0f       /* 角减速度，rad/s^2 */
#define MEC_LOOK_DIST_MM    250.0f     /* 路径前瞻距离，mm */
#define MEC_CUR_DIST_MM     100.0f     /* 路径当前点判定距离，mm */
#define MEC_FINAL_DIST_MM   1000.0f    /* 终点减速段长度，mm */
#define MEC_REACH_MM        10.0f      /* 路径终点到达阈值，mm */
#define MEC_YAW_KP          10.0f      /* 路径航向 PID 比例增益 */
#define MEC_YAW_KI          0.0f        /* 路径航向 PID 积分增益 */
#define MEC_YAW_KD          0.00f      /* 路径航向 PID 微分增益 */
#define MEC_YAW_I_LIM       0.10f      /* 路径航向积分限幅，rad*s */
#define MEC_YAW_OUT_LIM     1.0f      /* 路径航向输出限幅，rad/s */
#define MEC_CTRL_DT_S       0.02f      /* 路径航向 PID 控制周期，s */
#define MEC_DIR_FILTER      0.05f      /* 运动方向角低通滤波系数 */
#define MEC_MIN_RADIUS_MM   0.01f      /* set_radius 最小有效半径，mm */
#define MEC_ODOM_X_GAIN     0.99f      /* 横移里程修正系数（实测标定） */
#define MEC_INITED          1U         /* 标志位置位值 */

static float mec_norm_angle(float a);
static float mec_move_to(float cur, float target, float step);
static float mec_mm_to_rps(const mecanum_t *self, float mm_s);
static float mec_dist_sq(float x, float y, const map_point_t *p);
static uint32_t mec_path_sig(const map_point_t *path, uint16_t len);
static uint16_t mec_cur_index(const mecanum_t *self, const map_point_t *path,
                              uint16_t len, uint16_t cur);
static uint16_t mec_look_index(const mecanum_t *self, const map_point_t *path,
                               uint16_t len, uint16_t cur);
static float mec_yaw_pid(mecanum_t *self, float yaw_e);
static chassis_status_t mec_check(const mecanum_t *self);

/**
 * @brief  把角度归一化到 [-PI, PI]
 * @param  a 输入角，rad
 * @retval 归一化角，rad
 */
static float mec_norm_angle(float a)
{
    float r = a; /* 归一化中间量 */

    while (r > MEC_PI) {
        r -= (2.0f * MEC_PI);
    }
    while (r < (-MEC_PI)) {
        r += (2.0f * MEC_PI);
    }
    return r;
}

/**
 * @brief  以限定步长把当前值逼近目标值
 * @param  cur    当前值
 * @param  target 目标值
 * @param  step   单步上限（>0）
 * @retval 逼近后的值
 */
static float mec_move_to(float cur, float target, float step)
{
    float d = 0.0f; /* 当前到目标的差值 */

    if (step <= 0.0f) {
        return target;
    }
    d = target - cur;
    if (d > step) {
        return cur + step;
    }
    if (d < (-step)) {
        return cur - step;
    }
    return target;
}

/**
 * @brief  轮线速度 mm/s 换算为轮转速 rps
 * @param  self 底盘对象
 * @param  mm_s 轮线速度，mm/s
 * @retval 轮转速，rps
 */
static float mec_mm_to_rps(const mecanum_t *self, float mm_s)
{
    /* 轮周长 = PI * 直径 */
    return mm_s / (MEC_PI * self->geom.wheel_dia_mm) * self->geom.correction;
}

/**
 * @brief  里程计点到路径点的距离平方
 * @param  x 里程计 x，mm    @param  y 里程计 y，mm
 * @param  p 路径点
 * @retval 距离平方，mm^2
 */
static float mec_dist_sq(float x, float y, const map_point_t *p)
{
    float ex = (float)p->x_mm - x; /* x 误差，mm */
    float ey = (float)p->y_mm - y; /* y 误差，mm */

    return (ex * ex) + (ey * ey);
}

/**
 * @brief  为路径点数组生成 FNV 轻量签名，用于识别新路径
 * @param  path 路径点数组
 * @param  len  路径点数
 * @retval 路径签名
 */
static uint32_t mec_path_sig(const map_point_t *path, uint16_t len)
{
    uint16_t i = 0U;              /* 遍历下标 */
    uint32_t sig = 2166136261UL;  /* FNV 偏移基 */
    uint32_t v = 0UL;             /* 参与散列的坐标值 */

    sig ^= (uint32_t)len;
    sig *= 16777619UL;
    for (i = 0U; i < len; i++) {
        v = (uint32_t)(uint16_t)path[i].x_mm;
        sig = (sig ^ v) * 16777619UL;
        v = (uint32_t)(uint16_t)path[i].y_mm;
        sig = (sig ^ v) * 16777619UL;
    }
    return sig;
}

/**
 * @brief  推进当前路径点下标（落在判定圆内的最远点）
 * @param  self 底盘对象  @param  path 路径数组
 * @param  len  路径点数  @param  cur  上次当前点下标
 * @retval 更新后的当前点下标
 */
static uint16_t mec_cur_index(const mecanum_t *self, const map_point_t *path,
                              uint16_t len, uint16_t cur)
{
    uint16_t i = 0U;          /* 扫描下标 */
    uint16_t out = cur;       /* 更新后的当前点 */
    float th = MEC_CUR_DIST_MM * MEC_CUR_DIST_MM; /* 判定距离平方 */

    if (out >= len) {
        out = 0U;
    }
    for (i = out; i < len; i++) {
        if (mec_dist_sq(self->odom.x_mm, self->odom.y_mm, &path[i]) < th) {
            out = i;
        }
    }
    return out;
}

/**
 * @brief  选取前瞻点下标（距里程计 ≥ 前瞻距离的首个点）
 * @param  self 底盘对象  @param  path 路径数组
 * @param  len  路径点数  @param  cur  当前点下标
 * @retval 前瞻点下标
 */
static uint16_t mec_look_index(const mecanum_t *self, const map_point_t *path,
                               uint16_t len, uint16_t cur)
{
    uint16_t i = 0U;          /* 扫描下标 */
    uint16_t start = cur;     /* 前瞻搜索起点 */
    float th = MEC_LOOK_DIST_MM * MEC_LOOK_DIST_MM; /* 前瞻距离平方 */

    if (start < (uint16_t)(len - 1U)) {
        start++;
    }
    for (i = start; i < len; i++) {
        if (mec_dist_sq(self->odom.x_mm, self->odom.y_mm, &path[i]) >= th) {
            return i;
        }
    }
    return (uint16_t)(len - 1U);
}

/**
 * @brief  路径航向 PID：输入航向误差，输出角速度指令（带积分/输出限幅）
 * @param  self  底盘对象
 * @param  yaw_e 航向误差，rad
 * @retval 角速度指令，rad/s
 */
static float mec_yaw_pid(mecanum_t *self, float yaw_e)
{
    float d = 0.0f;   /* 误差微分，rad/s */
    float out = 0.0f; /* PID 原始输出 */

    if (self->path_pid_on != MEC_INITED) {
        self->path_yaw_i = 0.0f;
        self->path_yaw_e = yaw_e;
        self->path_pid_on = MEC_INITED;
    }
    /* 容差内清积分并停转，记误差防微分跳变 */
    if (fabsf(yaw_e) <= MEC_YAW_TOL_RAD) {
        self->path_yaw_i = 0.0f;
        self->path_yaw_e = yaw_e;
        return 0.0f;
    }
    self->path_yaw_i += yaw_e * MEC_CTRL_DT_S;
    self->path_yaw_i = fminf(MEC_YAW_I_LIM, fmaxf(-MEC_YAW_I_LIM, self->path_yaw_i));
    d = (yaw_e - self->path_yaw_e) / MEC_CTRL_DT_S;
    out = (MEC_YAW_KP * yaw_e) + (MEC_YAW_KI * self->path_yaw_i) + (MEC_YAW_KD * d);
    self->path_yaw_e = yaw_e;
    /* 底盘正角速度与里程计 yaw 正向相反，故取负后限幅 */
    return fminf(MEC_YAW_OUT_LIM, fmaxf(-MEC_YAW_OUT_LIM, -out));
}

/**
 * @brief  校验对象指针与初始化状态
 * @param  self 底盘对象
 * @retval CHASSIS_OK / CHASSIS_ERR_PARAM / CHASSIS_ERR_INIT
 */
static chassis_status_t mec_check(const mecanum_t *self)
{
    if (self == NULL) {
        return CHASSIS_ERR_PARAM;
    }
    if (self->is_inited != MEC_INITED) {
        return CHASSIS_ERR_INIT;
    }
    return CHASSIS_OK;
}

chassis_status_t mecanum_inst(mecanum_t *self, const mecanum_geom_t *geom,
                              chassis_motors_fn_t pf, void *ctx)
{
    if ((self == NULL) || (geom == NULL) || (pf == NULL)) {
        return CHASSIS_ERR_PARAM;
    }
    if ((geom->half_base_mm <= 0.0f) || (geom->half_shaft_mm <= 0.0f) ||
        (geom->wheel_dia_mm <= 0.0f) || (geom->correction <= 0.0f) ||
        (geom->enc_ppr <= 0.0f)) {
        return CHASSIS_ERR_PARAM;
    }
    (void)memset(self, 0, sizeof(*self)); /* 清零全部状态 */
    self->geom = *geom;
    self->pf = pf;
    self->motor_ctx = ctx;
    self->is_inited = MEC_INITED;
    return CHASSIS_OK;
}

chassis_status_t mecanum_set_vel(mecanum_t *self, float vx, float vy, float wz)
{
    chassis_status_t ret = mec_check(self); /* 校验结果 */
    float rot = 0.0f; /* 自转等效线速度，mm/s */
    float lf = 0.0f;  /* 左前轮线速度，mm/s */
    float lr = 0.0f;  /* 左后轮线速度，mm/s */
    float rf = 0.0f;  /* 右前轮线速度，mm/s */
    float rr = 0.0f;  /* 右后轮线速度，mm/s */

    if (ret != CHASSIS_OK) {
        return ret;
    }
    /* 运动学逆解，轮序 lf,lr,rf,rr */
    rot = wz * (self->geom.half_base_mm + self->geom.half_shaft_mm);
    lf = vy + vx - rot;
    lr = vy - vx - rot;
    rf = vy - vx + rot;
    rr = vy + vx + rot;
    return self->pf(self->motor_ctx, mec_mm_to_rps(self, lf),
                    mec_mm_to_rps(self, lr), mec_mm_to_rps(self, rf),
                    mec_mm_to_rps(self, rr));
}

chassis_status_t mecanum_set_radius(mecanum_t *self, float linear, float r,
                                    bool insitu)
{
    chassis_status_t ret = mec_check(self); /* 校验结果 */
    float wz = 0.0f; /* 由线速度与半径换算的角速度，rad/s */

    if (ret != CHASSIS_OK) {
        return ret;
    }
    /* 半径过小退化为停车，避免角速度发散 */
    if (fabsf(r) <= MEC_MIN_RADIUS_MM) {
        return mecanum_stop(self);
    }
    wz = linear / r;
    if (insitu) {
        return mecanum_set_vel(self, 0.0f, 0.0f, wz);
    }
    return mecanum_set_vel(self, 0.0f, linear, wz);
}

chassis_status_t mecanum_stop(mecanum_t *self)
{
    chassis_status_t ret = mec_check(self); /* 校验结果 */

    if (ret != CHASSIS_OK) {
        return ret;
    }
    self->nav_lin_cmd = 0.0f;
    self->nav_ang_cmd = 0.0f;
    return self->pf(self->motor_ctx, 0.0f, 0.0f, 0.0f, 0.0f);
}

chassis_status_t mecanum_odom(mecanum_t *self, mecanum_delta_t delta,
                              float yaw_rad, float dt_s)
{
    chassis_status_t ret = mec_check(self); /* 校验结果 */
    float circ = 0.0f;    /* 轮周长，mm */
    float s_lf = 0.0f;    /* 左前轮位移，mm */
    float s_lr = 0.0f;    /* 左后轮位移，mm */
    float s_rf = 0.0f;    /* 右前轮位移，mm */
    float s_rr = 0.0f;    /* 右后轮位移，mm */
    float dxb = 0.0f;     /* 车体系 x 增量，mm */
    float dyb = 0.0f;     /* 车体系 y 增量，mm */
    float ny = 0.0f;      /* 归一化真实 yaw，rad */
    float dyaw = 0.0f;    /* 周期 yaw 增量，rad */
    float mid = 0.0f;     /* 积分用中值 yaw，rad */

    if (ret != CHASSIS_OK) {
        return ret;
    }
    if (dt_s <= 0.0f) {
        return CHASSIS_ERR_PARAM;
    }
    /* 脉冲增量换算为四轮位移 */
    circ = MEC_PI * self->geom.wheel_dia_mm / self->geom.correction;
    s_lf = (float)delta.lf * circ / self->geom.enc_ppr;
    s_lr = (float)delta.lr * circ / self->geom.enc_ppr;
    s_rf = (float)delta.rf * circ / self->geom.enc_ppr;
    s_rr = (float)delta.rr * circ / self->geom.enc_ppr;
    dxb = (s_lf - s_lr - s_rf + s_rr) * 0.25f * MEC_ODOM_X_GAIN;
    dyb = (s_lf + s_lr + s_rf + s_rr) * 0.25f;

    ny = mec_norm_angle(yaw_rad);
    if (self->odom_inited != MEC_INITED) {
        /* 首帧仅对齐基准，不产生角速度 */
        self->last_yaw = ny;
        self->odom.yaw_rad = ny;
        self->odom.w = 0.0f;
        self->odom_inited = MEC_INITED;
        mid = ny;
    } else {
        dyaw = mec_norm_angle(ny - self->last_yaw);
        mid = mec_norm_angle(self->last_yaw + (dyaw * 0.5f));
        self->odom.yaw_rad = ny;
        self->odom.w = dyaw / dt_s;
        self->last_yaw = ny;
    }
    /* 车体系增量旋转到地图系并积分 */
    self->odom.x_mm += (cosf(mid) * dxb) + (sinf(mid) * dyb);
    self->odom.y_mm += (-sinf(mid) * dxb) + (cosf(mid) * dyb);
    self->odom.vx = dxb / dt_s;
    self->odom.vy = dyb / dt_s;
    return CHASSIS_OK;
}

chassis_status_t mecanum_navigate(mecanum_t *self, float tx_mm, float ty_mm,
                                  float t_yaw, float t_v, float t_w,
                                  uint8_t *finished)
{
    chassis_status_t ret = mec_check(self); /* 校验结果 */
    float ex = 0.0f;      /* 地图系 x 误差，mm */
    float ey = 0.0f;      /* 地图系 y 误差，mm */
    float dist = 0.0f;    /* 位置误差，mm */
    float eyaw = 0.0f;    /* 航向误差，rad */
    float lin_t = 0.0f;   /* 目标线速度，mm/s */
    float ang_t = 0.0f;   /* 目标角速度，rad/s */
    float dir = 0.0f;     /* 地图系运动方向角，rad */
    float bvx = 0.0f;     /* 车体系 x 速度，mm/s */
    float bvy = 0.0f;     /* 车体系 y 速度，mm/s */
    float cmd_w = 0.0f;   /* 下发角速度，rad/s */

    if (ret != CHASSIS_OK) {
        return ret;
    }
    ex = tx_mm - self->odom.x_mm;
    ey = ty_mm - self->odom.y_mm;
    dist = sqrtf((ex * ex) + (ey * ey));
    eyaw = mec_norm_angle(t_yaw - self->odom.yaw_rad);
    /* 按减速距离生成平滑目标速度并限步逼近 */
    lin_t = fminf(t_v, sqrtf(2.0f * MEC_LIN_DECEL *
                             fmaxf(dist - MEC_POS_TOL_MM, 0.0f)));
    ang_t = fminf(t_w, sqrtf(2.0f * MEC_ANG_DECEL *
                             fmaxf(fabsf(eyaw) - MEC_YAW_TOL_RAD, 0.0f)));
    self->nav_lin_cmd = mec_move_to(self->nav_lin_cmd, lin_t, MEC_LIN_STEP);
    self->nav_ang_cmd = mec_move_to(self->nav_ang_cmd, ang_t, MEC_ANG_STEP);

    if ((self->nav_lin_cmd > 0.0f) && (dist > 0.0f)) {
        /* 沿目标方向分解，再旋到车体系 */
        dir = atan2f(ex, ey);
        bvx = (cosf(self->odom.yaw_rad) * self->nav_lin_cmd * sinf(dir)) -
              (sinf(self->odom.yaw_rad) * self->nav_lin_cmd * cosf(dir));
        bvy = (sinf(self->odom.yaw_rad) * self->nav_lin_cmd * sinf(dir)) +
              (cosf(self->odom.yaw_rad) * self->nav_lin_cmd * cosf(dir));
    }
    if (eyaw < 0.0f) {
        cmd_w = self->nav_ang_cmd;
    } else if (eyaw > 0.0f) {
        cmd_w = -self->nav_ang_cmd;
    } else {
        cmd_w = 0.0f;
    }
    if (finished != NULL) {
        /* 位置 + 航向双容差判到位 */
        *finished = (uint8_t)(((dist < MEC_POS_TOL_MM) &&
                     (fabsf(eyaw) < MEC_YAW_TOL_RAD)) ? 1U : 0U);
    }
    return mecanum_set_vel(self, bvx, bvy, cmd_w);
}

chassis_status_t mecanum_follow(mecanum_t *self, const map_point_t *path,
                                uint16_t len, float cruise_v, float t_yaw,
                                uint8_t *finished)
{
    chassis_status_t ret = CHASSIS_ERR; /* 校验结果 */
    uint32_t sig = 0UL;       /* 输入路径签名 */
    uint16_t cur = 0U;        /* 当前点下标 */
    uint16_t look = 0U;       /* 前瞻点下标 */
    uint16_t i = 0U;          /* 剩余路程累加下标 */
    float rest = 0.0f;        /* 到终点剩余路程，mm */
    float yaw_e = 0.0f;       /* 终点航向误差，rad */
    float ex = 0.0f;          /* 前瞻点 x 误差，mm */
    float ey = 0.0f;          /* 前瞻点 y 误差，mm */
    float lin_t = 0.0f;       /* 目标线速度，mm/s */
    float dir_t = 0.0f;       /* 目标运动方向角，rad */
    float bvx = 0.0f;         /* 车体系 x 速度，mm/s */
    float bvy = 0.0f;         /* 车体系 y 速度，mm/s */

    if (finished == NULL) {
        return CHASSIS_ERR_PARAM;
    }
    ret = mec_check(self);
    if (ret != CHASSIS_OK) {
        return ret;
    }
    if ((path == NULL) || (len == 0U) || (cruise_v < 0.0f)) {
        return CHASSIS_ERR_PARAM;
    }
    *finished = 0U;

    /* 新路径则重置跟随状态机 */
    sig = mec_path_sig(path, len);
    if ((self->path_active != MEC_INITED) || (sig != self->path_sig)) {
        self->path_dir = 0.0f;
        self->path_look = 0U;
        self->path_index = 0U;
        self->path_sig = sig;
        self->path_active = MEC_INITED;
        self->nav_lin_cmd = 0.0f;
        self->nav_ang_cmd = 0.0f;
        self->path_pid_on = 0U;
    }

    /* 当前点、前瞻点均只前不退 */
    cur = mec_cur_index(self, path, len, self->path_index);
    if (cur < self->path_index) {
        cur = self->path_index;
    } else {
        self->path_index = cur;
    }
    look = mec_look_index(self, path, len, cur);
    if (look < self->path_look) {
        look = self->path_look;
    } else {
        self->path_look = look;
    }

    /* 剩余路程 = 到前瞻点 + 各段折线长 */
    rest = sqrtf(mec_dist_sq(self->odom.x_mm, self->odom.y_mm, &path[look]));
    for (i = look; i < (uint16_t)(len - 1U); i++) {
        ex = (float)path[i + 1U].x_mm - (float)path[i].x_mm;
        ey = (float)path[i + 1U].y_mm - (float)path[i].y_mm;
        rest += sqrtf((ex * ex) + (ey * ey));
    }
    if (rest < MEC_REACH_MM) {
        /* 位置已到但航向未到位：原地补转，避免短路径转不完 */
        yaw_e = mec_norm_angle(t_yaw - self->odom.yaw_rad);
        if (fabsf(yaw_e) >= MEC_YAW_TOL_RAD) {
            self->nav_lin_cmd = 0.0f;
            self->nav_ang_cmd = mec_yaw_pid(self, yaw_e);
            return mecanum_set_vel(self, 0.0f, 0.0f, self->nav_ang_cmd);
        }
        /* 到达终点：复位状态并停车 */
        self->path_active = 0U;
        self->nav_ang_cmd = 0.0f;
        self->path_pid_on = 0U;
        *finished = 1U;
        return mecanum_set_vel(self, 0.0f, 0.0f, 0.0f);
    }

    /* 终点段限速，平滑线速度并夹到巡航上限 */
    if (rest <= MEC_FINAL_DIST_MM) {
        lin_t = fminf(cruise_v, sqrtf(2.0f * MEC_LIN_DECEL *
                                      fmaxf(rest - MEC_REACH_MM, 0.0f)));
    } else {
        lin_t = cruise_v;
    }
    self->nav_lin_cmd = mec_move_to(self->nav_lin_cmd, lin_t, MEC_LIN_STEP);
    self->nav_lin_cmd = fminf(self->nav_lin_cmd, cruise_v);

    /* 航向 PID 给角速度 */
    self->nav_ang_cmd = mec_yaw_pid(self,
                                    mec_norm_angle(t_yaw - self->odom.yaw_rad));

    ex = (float)path[look].x_mm - self->odom.x_mm;
    ey = (float)path[look].y_mm - self->odom.y_mm;
    if ((self->nav_lin_cmd > 0.0f) && ((fabsf(ex) + fabsf(ey)) > 0.0f)) {
        /* 前瞻方向低通滤波后旋到车体系 */
        dir_t = atan2f(ex, ey);
        self->path_dir = mec_norm_angle(self->path_dir +
            (MEC_DIR_FILTER * mec_norm_angle(dir_t - self->path_dir)));
        bvx = (cosf(self->odom.yaw_rad) * self->nav_lin_cmd * sinf(self->path_dir)) -
              (sinf(self->odom.yaw_rad) * self->nav_lin_cmd * cosf(self->path_dir));
        bvy = (sinf(self->odom.yaw_rad) * self->nav_lin_cmd * sinf(self->path_dir)) +
              (cosf(self->odom.yaw_rad) * self->nav_lin_cmd * cosf(self->path_dir));
    }
    return mecanum_set_vel(self, bvx, bvy, self->nav_ang_cmd);
}

chassis_status_t mecanum_get_odom(const mecanum_t *self, chassis_odom_t *out)
{
    if ((self == NULL) || (out == NULL)) {
        return CHASSIS_ERR_PARAM;
    }
    if (self->is_inited != MEC_INITED) {
        return CHASSIS_ERR_INIT;
    }
    *out = self->odom;
    return CHASSIS_OK;
}

chassis_status_t mecanum_reset_odom(mecanum_t *self)
{
    chassis_status_t ret = mec_check(self); /* 校验结果 */

    if (ret != CHASSIS_OK) {
        return ret;
    }
    (void)memset(&self->odom, 0, sizeof(self->odom));
    self->odom_inited = 0U;
    self->last_yaw = 0.0f;
    self->nav_lin_cmd = 0.0f;
    self->nav_ang_cmd = 0.0f;
    self->path_active = 0U;
    return CHASSIS_OK;
}

chassis_status_t mecanum_set_pose(mecanum_t *self, float x_mm, float y_mm,
                                  float yaw_rad)
{
    chassis_status_t ret = mec_check(self); /* 校验结果 */

    if (ret != CHASSIS_OK) {
        return ret;
    }
    self->odom.x_mm = x_mm;
    self->odom.y_mm = y_mm;
    self->odom.yaw_rad = mec_norm_angle(yaw_rad);
    self->odom.vx = 0.0f;
    self->odom.vy = 0.0f;
    self->odom.w = 0.0f;
    self->last_yaw = self->odom.yaw_rad;
    self->odom_inited = MEC_INITED; /* 已显式定位，视为已对齐 */
    return CHASSIS_OK;
}
