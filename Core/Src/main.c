/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* ============================================================================
 *  pid_v2 —— M3508 P19 + C620 单轴位置伺服
 *  ---------------------------------------------------------------------------
 *  结构（从外到内，每 1ms 跑一拍）：
 *
 *      目标角度 -->[参考轨迹整形]--> ref_pos / ref_vel
 *                                      |
 *                                      v
 *      实测输出角 -->[位置环 P + 速度前馈/阻尼]--> 速度指令
 *                                      |
 *                                      v
 *      实测转速   -->[速度环 PID + 前馈]--> 电流指令 --> C620 (0x200)
 *
 *  三条针对「电机抖」的设计，缺一不可：
 *    1) 参考轨迹整形：目标不直接给位置环，而是先过一条加速度受限、
 *       带制动曲线、单步不越目标的轨迹。位置环跟的是一条平滑曲线，
 *       不会因为目标跳变打出电流尖峰。
 *    2) 微分对测量值求导：d 项用 -Kd×(本次转速 - 上次转速)，
 *       而不是对误差求导，彻底消除 derivative kick。
 *    3) 到位保持状态机：误差进入 ±pos_db 后位置环 P 清零，只留一条
 *       「慢积分」把保持电流调到刚好顶住负载。这是压掉静摩擦粘滑极限环的
 *       关键——只加死区不够，积分器里残留的量还会把电机推出死区。
 *
 *  实测指标（500Hz 录波，默认参数，M3508 P19 + C620）：
 *    上升时间 10-90%   : 44 ms (20°) / 50 ms (25°) / 66 ms (45°)
 *                        92 ms (90°)
 *    超调量           : ≤ 0.20 %
 *    调节时间 ±0.1°   : 126~292 ms
 *    稳态误差         : ≤ 0.055°
 *    双向重复精度     : ≤ 0.07°（同一角度从两侧逼近的落点差）
 *    到位静置抖动     : 8 个位置 × 3 轮共 24 次测量，多数 0.0000°，
 *                       最差 0.074°（27 个细计数，周期约 1s 的极慢蠕动）
 *
 *  串口（USART1, 115200 8N1）命令，全部以 '!' 结尾：
 *    =角度!        设定目标角度（度，0~359）
 *    $名字=值!     在线改参数，立即生效，见 param_tbl[]
 *    $wave=0..3!   切换波形通道组
 *  波形按 VOFA+ JustFloat 协议，2 通道 × float32 小端 + 帧尾 00 00 80 7F
 * ==========================================================================*/

/* ============================================================================
 *  运行状态变量
 *  ---------------------------------------------------------------------------
 *  坐标系约定（与上一版一致，改动它会让上位机的角度含义全部错位）：
 *    · 「转子角度」由 C620 上报，0~8191 对应转子机械一圈；
 *    · 「输出轴角度」= 转子累计角度 ÷ 减速比，再对 8192 取模，
 *      0~8191 对应减速后的 0~360°，8192 个 count 铺满输出轴一整圈；
 *    · 上电瞬间的输出轴角度被定义为 0°，即整个坐标系是「相对上电位置」的，
 *      所以上电后目标默认为 0 就等于「原地保持」，不会突然窜到别处。
 * ==========================================================================*/

/* ---------- 1. 电机反馈（每 1ms 由 0x203 帧刷新）---------- */
static int16_t jiao_du = 0;     /* 转子机械角度：0~8191，8192 = 转子转一圈 */
static int16_t su_du   = 0;     /* 转子转速：rpm，正负号表示转向 */

/* ---------- 2. 角度换算：转子 -> 输出轴 ---------- */
static int64_t rotor_total = 0; /* 转子累计角度（count），只用来算输出轴角度，
                                 * 不参与位置环。跨圈时靠它保证不丢圈。
                                 * 用 64 位：int32 在 3000rpm 下连续转约 87 分钟
                                 * 就会溢出，溢出后角度跳变，位置环失控 */
static int16_t rotor_last  = 0; /* 上一拍的转子原始角度，用来算这一拍转了多少 */
static int16_t out_angle   = 0; /* 输出轴单圈角度：0~8191 对应 0~360° */
static int32_t out_cnt32   = 0; /* 输出轴连续角度（int32 截断域）。
                                 * 取 int32 而不是直接用 int64：控制环里所有位置量
                                 * 都落在同一个 32 位域里，相减不会出现 64 位跨字
                                 * 读写的原子性问题，跨圈时差值依然正确。
                                 * 输出轴连续转 28 小时才会溢出，位置伺服不会走到那 */

/* ---------- 3. 目标与参考轨迹 ---------- */
static volatile int32_t target_raw = 0; /* 目标输出轴角度（count），0~8191。
                                 * UART 中断里写、控制任务里读，必须 volatile */
static int32_t target_cnt = 0;  /* 与 out_cnt32 同一域的「最近等价目标」。
                                 * 目标 350°、当前 10° 时不会傻走 -340°，
                                 * 而是在连续域里落到 +20° 那一侧 */
static int32_t target_seen = -1;/* 上一次处理过的 target_raw，用来检测新命令 */

static float ref_pos = 0.0f;    /* 参考位置（输出轴 count，float 保留小数，
                                 * 否则每拍增量不足 1 count 时会被截断成 0 而卡死） */
static float ref_vel = 0.0f;    /* 参考速度（转子 rpm），由轨迹发生器给出 */

/* ---------- 4. 可在线整定的参数 ---------- */
/* 全部做成 float 并集中在一张表里，串口按名字改，改完立即生效，不用重新烧录。
 * 表见下方 param_tbl[]，合法范围也写在那里面，越界值会被夹到边界而不是拒绝。 */
static float pos_Kp    = 0.75f;   /* 位置环比例：rpm / (1/16 输出轴count)。
                                   * 等效于旧单位下的 12 rpm/count */
static float pos_Kd    = 0.5f;    /* 位置环速度增益：对「参考速度 - 实测转速」加权。
                                   * 参考速度归零后它就退化成上一版的 -Kd×实测转速，
                                   * 也就是纯阻尼项，治到点后来回振荡 */
static float speed_max = 3000.0f; /* 位置环输出的速度指令上限（rpm） */

static float speed_Kp  = 15.0f;   /* 速度环比例：电流count / rpm。
                                   * 实测 6 → 调节时间 734ms，15 → 164ms，
                                   * 28 开始振荡（超调 3.4%），15~20 是安全区 */
static float speed_Kd  = 20.0f;   /* 速度环微分：抑制超调。12 → 164ms，20 → 158ms，
                                   * 再大没有收益 */
static float speed_Ki  = 0.15f;   /* 速度环积分：每 1ms、每 1rpm 误差贡献的电流count。
                                   * 按 CTRL_PERIOD_MS 标定，改周期必须重调。
                                   * 这个值决定「到位后最后 0.1° 收得多快」：
                                   * 0.008 时要 1668ms 才爬进去（静摩擦粘滑）。
                                   * 实测 0.08 / 0.15 / 0.25 三档，调节时间几乎一样
                                   * （144~298ms），差别全在「静置后还会不会动」：
                                   *   0.08 → 残余 1~7 个细计数的蠕动（0.02°）
                                   *   0.15 → 8 位置全部 0.0000°，稳态误差也略小
                                   *   0.25 → 抖动仍是 0，但调节时间变长，无收益
                                   * 取 0.15。上一版「0.05 就会振荡」的结论只对
                                   * 旧结构成立——那时光滑轨迹整形和到位保持都还没有 */
static float speed_Kf  = 0.0f;    /* 速度前馈：电流count / rpm。
                                   * 直接按目标转速给一份电流，把积分项从「顶住摩擦」
                                   * 的活里解放出来。默认关闭，识别出摩擦量级后再开 */
static float i_max     = 3000.0f; /* 积分项单独限幅，防止累到退不回来 */
static float cur_max   = 8000.0f; /* 电流总限幅，约 ±9.8A（16384 = 20A） */

static float prof_v    = 3000.0f; /* 轨迹速度上限（rpm），一般等于 speed_max */
static float prof_a    = 50000.0f;/* 轨迹加速度上限（rpm/s）。这是「快而不冲」的总闸门：
                                   * 调大=更激进，调小=更柔和。它同时决定
                                   * 减速从多远开始——制动距离 = v²/(2a) */
static float prof_en   = 1.0f;    /* 轨迹整形开关：1=启用，0=目标直通（退化成阶跃） */
static float pos_db    = 0.02f;   /* 到位死区（度）。只在轨迹已经走完时生效：
                                   * 误差小于它就进入「保持」，位置环输出清零。
                                   * 不加这个，小误差区靠积分慢慢顶，顶过静摩擦就
                                   * 蹦一下，形成 0.1° 量级、周期几百毫秒的往复——
                                   * 也就是「电机在抖」。死区把积分饿死，电机停住不动。
                                   * 代价是稳态误差被限制在 ±3×pos_db 以内（迟滞带） */
static float hold_ki    = 0.005f;  /* 保持期「位置积分」增益：每 1ms、每 1 个细计数误差，
                                   * 给保持电流加多少 count。
                                   * 为什么保持期还要积分：维持静止所需的电流随
                                   * 负载变化（实测 0° 要几百 count，180° 只要几十），
                                   * 而进保持瞬间的积分值是「刚顶开静摩擦」的电流，
                                   * 比需要的多，直接冻住就会推着电机慢慢爬出去。
                                   * 所以让位置误差自己去调这个电流——多了误差变负、
                                   * 电流自己降下来，少了误差变正、电流自己补上去。
                                   * 关键是慢：慢到电流变化本身激不起抖动 */
static float hold_rt    = 0.60f;  /* 进保持瞬间，把积分值一次性乘上这个系数。
                                   * 进保持时 i_term 是「刚顶开静摩擦」的电流
                                   * （= 负载 + 静摩擦），维持静止只需要负载。
                                   * 先按比例砍一刀，让电流一步落到目标附近，
                                   * 剩下的残差交给下面的慢积分去修——
                                   * 这样慢积分不用长途跋涉，也就不会振荡。
                                   * 1.0 = 不砍（慢积分要自己从顶开值降下来，会振荡） */
static float hold_rmax  = 0.75f;  /* 保持期电流变化率上限（count/ms）。
                                   * 它决定这个慢积分会不会自己变成振荡器：
                                   * 变化率 × 机械响应延迟 = 电流过冲量，
                                   * 只要过冲量小于静摩擦，电机就不会动 */
static float wave_sel  = 0.0f;    /* 波形通道组：0=误差/角度 1=速度指令/实测 2=参考/实际 3=电流/转速 4=积分/电流 */

/* ---------- 5. 运行时状态 ---------- */
static float   i_term     = 0.0f; /* 速度环积分累加器 */
static int16_t speed_cmd  = 0;    /* 位置环输出，也就是给速度环的目标转速（rpm） */
static int16_t cur_out    = 0;    /* 本拍下发的电流指令，调试时看它 */
static int16_t speed_prev = 0;    /* 上一拍实测转速，速度环微分用 */
static float   pos_err    = 0.0f; /* 目标 - 实际（1/16 count），只用于波形显示。
                                   * 必须是 float：满量程 4096×16 会溢出 int16 */

static uint16_t fb_age_ms = 0;    /* 距离上一帧 CAN 反馈过去了多少 ms */
static uint8_t  fb_seen   = 0;    /* 是否收到过至少一帧反馈 */
static uint8_t  ang_init  = 0;    /* 角度基准是否已对齐到首帧真实反馈 */
static uint8_t  ctrl_init = 0;    /* 控制环是否已完成首拍初始化 */
static uint8_t  prof_done = 0;    /* 参考轨迹是否已停在目标上（用于到位死区） */
static uint8_t  hold_state = 0;   /* 到位保持状态：进入后位置环 P 清零，只留慢积分调保持电流 */

/* ---------- 6. 串口命令接收（全在中断里用）---------- */
static uint8_t rxByte;          /* UART 中断里收到的单个字节 */
static uint8_t rxBuf[32];       /* 命令帧缓冲 */
static uint8_t rxLen = 0;       /* 已接收的字节数 */

/* ---------- 7. 波形输出 ---------- */
static volatile uint8_t uart_busy = 0; /* 波形发送忙标志：中断发送还没完成时不能重填缓冲 */
static uint8_t  jf[12];         /* JustFloat 发送缓冲：2 通道 x 4 字节 + 4 字节帧尾 */

/* ---------- 8. CAN 发送帧（每 1ms 一次）---------- */
/* 提到文件作用域是为了避免每 1ms 在栈上重新构造一次结构体 */
static CAN_TxHeaderTypeDef txHeader;
static uint32_t            txMailbox;

/* ---------- 9. 参数表 ---------- */
typedef struct
{
  const char *name;   /* 串口里用的名字，区分大小写 */
  float      *addr;   /* 指向真正的变量 */
  float       lo;     /* 下限，越界自动夹住 */
  float       hi;     /* 上限 */
} ParamEntry;

static const ParamEntry param_tbl[] =
{
  /*  名字      变量        下限      上限    */
  { "posKp",  &pos_Kp,     0.0f,     20.0f  },
  { "posKd",  &pos_Kd,     0.0f,     10.0f  },
  { "spdMax", &speed_max,  0.0f,   9000.0f  },
  { "spdKp",  &speed_Kp,   0.0f,    100.0f  },
  { "spdKd",  &speed_Kd,   0.0f,    100.0f  },
  { "spdKi",  &speed_Ki,   0.0f,      0.5f  },
  { "spdKf",  &speed_Kf,   0.0f,     10.0f  },
  { "iMax",   &i_max,      0.0f,  16384.0f  },
  { "curMax", &cur_max,    0.0f,  16384.0f  },
  { "profV",  &prof_v,     1.0f,   9000.0f  },
  { "profA",  &prof_a,   100.0f, 200000.0f  },
  { "profEn", &prof_en,    0.0f,      1.0f  },
  { "posDb",  &pos_db,     0.0f,      1.0f  },
  { "holdKi", &hold_ki,    0.0f,      0.2f  },
  { "holdRt", &hold_rt,    0.0f,      1.0f  },
  { "holdRm", &hold_rmax,  0.1f,     20.0f  },
  { "wave",   &wave_sel,   0.0f,      4.0f  },
};
#define PARAM_N  (sizeof(param_tbl) / sizeof(param_tbl[0]))

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ============================================================================
 *  硬件与协议常量（一般不需要改动）
 * ==========================================================================*/
#define ENC_PER_REV   8192      /* 编码器每圈计数（转子），2^13 = 8192 */
#define ENC_HALF      4096      /* 半圈计数。角度差超过它说明绕远了，要往反方向走 */

/* ----------------------------------------------------------------------------
 *  输出轴角度的内部精度：以 1/OUT_FINE 个输出 count 为单位
 *  ---------------------------------------------------------------------------
 *  8192 个 count 铺满输出轴一圈，分辨率只有 0.044°。
 *  但转子编码器经 19.2 倍减速折算到输出轴是 8192 × 19.2 ≈ 157286 个 count/圈，
 *  物理分辨率 0.0023°。直接用整数输出 count 等于把 19 倍的精度白白扔掉，
 *  稳态误差会被卡死在 ±1 count（±0.044°）——实测重复定位精度正好就是这个数。
 *
 *  取 16 是折中：分辨率提到 0.0027°，误差量化带来的抖动降到原来的 1/16，
 *  而 int32 在 8192×16 = 131072 count/圈的尺度下仍能连续转 100 分钟不溢出。
 *  位置环增益的数值要同比缩小 16 倍（pos_Kp: 12 → 0.75），物理含义不变。
 * --------------------------------------------------------------------------*/
#define OUT_FINE      16
#define POS_PER_REV   (ENC_PER_REV * OUT_FINE)   /* 131072 */
#define POS_HALF      (ENC_HALF * OUT_FINE)      /* 65536  */

#define GEAR_NUM      187       /* 减速比分子：输出轴角度 = 转子角度 × 187 / 3591 */
#define GEAR_DEN      3591      /* 减速比分母，187 / 3591 ≈ 1 : 19.203 */

#define C620_TX_ID    0x200     /* 1~4 号电机的电流指令帧 ID，8 字节分给 4 个电机 */
#define C620_FB_ID_M3 0x203     /* 3 号电机的反馈帧 ID，本工程只挂了一个电机 */

#define CTRL_PERIOD_MS   1      /* 控制环周期（ms）。积分增益 speed_Ki 是按这个周期
                                 * 标定的，改这里必须重新整定 speed_Ki */
#define JF_PERIOD_MS     2      /* 波形发送周期（ms）。12 字节 @115200 要 1.04ms，
                                 * 用中断非阻塞发送，不占主循环时间 */
#define FB_TIMEOUT_MS    20     /* 反馈超时门限（ms）。C620 每 1ms 主动发一帧，
                                 * 连续 20 拍收不到就判定掉线，立刻撤电流 */

/* ----------------------------------------------------------------------------
 *  单位换算：转子 rpm  <->  输出轴 count/ms
 *  ---------------------------------------------------------------------------
 *  输出轴转速(rpm) = 转子 rpm / 19.203
 *  输出轴 count/s  = 输出轴转速 × 8192 / 60
 *  所以 1 rpm 的转子转速 = 8192 × 187 / (3591 × 60 × 1000) ≈ 0.00711 count/ms
 *  编译器会把这一串常量折叠成单个浮点数，运行时不产生除法
 * --------------------------------------------------------------------------*/
#define RPM_TO_CNT_PER_MS  ((float)POS_PER_REV * (float)GEAR_NUM / ((float)GEAR_DEN * 60000.0f))

/* 制动曲线系数：由 v[rpm] 反推「刚好停在 e 个 count 外」所需的速度
 *   v = sqrt(2 · a · e · BRAKE_K)
 * 推导：制动距离 = v_cnt² / (2 · a_cnt)，把 count/ms 与 rpm 的换算代进去即得 */
#define BRAKE_K  (1.0f / (RPM_TO_CNT_PER_MS * 1000.0f))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
static void Parse_Frame(const uint8_t *data, uint16_t size);
static void Apply_Param(const uint8_t *data, uint16_t size);
static void ReadMotorFeedback(void);
static void UpdateOutputAngle(void);
static void Profile_Step(void);
static void Control_Step(void);
static void C620_SendCurrent(int16_t current);
static void Waveform_Send(void);
void out(void *argument);
void fa_song(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  解析一帧完整的串口命令（以 '!' 结尾）
  * @param  data 帧缓冲，data[0] 是首字符
  * @param  size 帧长度（含 '!'）
  * @note   支持两种帧，靠首字符区分，互不干扰：
  *
  *           1) 角度命令 —— 兼容上一版的 VOFA+ 控件，格式不变
  *                "=90!"        或   "Kp=90!"（'=' 前面的内容一律忽略）
  *              数值单位是「度」，指减速后输出轴的角度，范围 0 ~ 359.99
  *
  *           2) 参数命令 —— 新增，用来在线改增益
  *                "$posKp=8.0!"  "$profA=30000!"  "$wave=1!"
  *              名字区分大小写，见 param_tbl[]
  *
  *         用 '$' 做前缀是因为它不可能出现在数值里，而 '=' 必须留给角度命令，
  *         否则老的上位机控件发来的 "=90!" 会被当成参数命令而失效。
  *
  *         本函数在 UART 中断里被调用，只做纯计算，不调用任何阻塞接口。
  */
static void Parse_Frame(const uint8_t *data, uint16_t size)
{
  /* ---- 帧完整性：最少 "=1!" 三个字节，且必须以 '!' 收尾 ---- */
  if ((size < 3) || (data[size - 1] != '!'))
  {
    return;
  }

  /* ---- 分支一：参数命令 ---- */
  if (data[0] == '$')
  {
    Apply_Param(data, size);
    return;
  }

  /* ---- 分支二：角度命令。找 '='，数值从它后面一个字节开始 ---- */
  int startIdx = -1;
  for (int i = 0; i < (int)size; i++)
  {
    if (data[i] == '=')
    {
      startIdx = i + 1;
      break;
    }
  }
  if (startIdx < 0)
  {
    return;                          /* 没有 '=' */
  }

  char valueStr[24] = {0};           /* VOFA+ 用 %f 时发出来的是 "180.000000"
                                      * 这种 10 字符文本，加结束符要 11 字节，
                                      * 留 24 才够宽裕 */
  int len = (int)size - 1 - startIdx;
  if ((len <= 0) || (len >= (int)sizeof(valueStr)))
  {
    return;                          /* 数值为空，或者太长，防止 valueStr 越界 */
  }

  memcpy(valueStr, &data[startIdx], (size_t)len);
  valueStr[len] = '\0';

  char *endp = NULL;
  float PIDpara = strtof(valueStr, &endp);
  if (endp == valueStr)
  {
    return;                          /* "=abc!" 这种不是数字，整帧丢弃。
                                      * 否则 strtof 返回 0，电机会被瞬间拉回零点 */
  }

  /* ---- 合法性检查后写入目标角度（度 -> count）----
   * 范围 0 ~ 360，对应输出轴（减速后）转一整圈。
   * 超出范围就整帧丢弃、不改目标 —— 这样上位机格式填错时
   * 电机只会保持原位，不会突然跑到别的地方去。 */
  if ((PIDpara >= 0.0f) && (PIDpara < 360.0f))
  {
    float cnt_f = PIDpara * ((float)POS_PER_REV / 360.0f);

    target_raw = (int32_t)(cnt_f + 0.5f);   /* 四舍五入到最近的 1/16 count */
  }
}

/**
  * @brief  解析并应用 "$name=value!" 形式的参数命令
  * @param  data 帧缓冲，data[0] 必定是 '$'
  * @param  size 帧长度（含 '!'）
  * @note   名字在表里找不到就整帧丢弃（静默），数值越界则夹到边界。
  *         不做回显：串口上正在跑 JustFloat 二进制波形，混进 ASCII 会把
  *         VOFA+ 的分帧打乱。改没改成功，看波形响应即可。
  */
static void Apply_Param(const uint8_t *data, uint16_t size)
{
  char name[16];
  char val[16];
  uint16_t i = 1;                    /* 跳过 '$' */
  uint16_t n = 0;
  uint16_t m = 0;

  /* ---- 取名字：从 '$' 之后到 '=' 之前 ---- */
  while ((i < size) && (data[i] != '=') && (data[i] != '!'))
  {
    if (n < (uint16_t)(sizeof(name) - 1))
    {
      name[n++] = (char)data[i];
    }
    i++;
  }
  name[n] = '\0';

  if ((i >= size) || (data[i] != '='))
  {
    return;                          /* 没有 '='，不是参数命令 */
  }

  /* ---- 取数值：从 '=' 之后到 '!' 之前 ---- */
  i++;
  while ((i < size) && (data[i] != '!'))
  {
    if (m < (uint16_t)(sizeof(val) - 1))
    {
      val[m++] = (char)data[i];
    }
    i++;
  }
  val[m] = '\0';

  char *endp = NULL;
  float v = strtof(val, &endp);
  if (endp == val)
  {
    return;                          /* 数值部分不是数字 */
  }

  /* ---- 查表并写入 ---- */
  for (uint16_t k = 0; k < PARAM_N; k++)
  {
    if (strcmp(name, param_tbl[k].name) == 0)
    {
      if (v < param_tbl[k].lo) v = param_tbl[k].lo;
      if (v > param_tbl[k].hi) v = param_tbl[k].hi;
      *param_tbl[k].addr = v;
      return;
    }
  }
}

/**
  * @brief  读电机反馈：把 CAN FIFO 里积压的 0x203 帧全部取完，只保留最后一帧
  * @note   C620 每 1ms 主动发一帧，8 个字节的含义：
  *           d[0..1]  转子机械角度（0~8191，大端）
  *           d[2..3]  转子转速 rpm（int16，大端）
  *           d[4..5]  实际电流   d[6] 温度   d[7] 保留
  *         FIFO 只有 3 帧深，必须每拍排空，攒着不取就会溢出丢帧。
  *
  *         顺带把 fb_age_ms 清零——它是掉线保护的计时器，
  *         只要这一拍真的收到帧，就说明链路还活着。
  */
static void ReadMotorFeedback(void)
{
  CAN_RxHeaderTypeDef rx;
  uint8_t d[8];

  while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0)
  {
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx, d) != HAL_OK)
    {
      break;                       /* 读失败就退出，避免在 while 里死循环 */
    }

    if ((rx.StdId == C620_FB_ID_M3) && (rx.DLC >= 8))
    {
      jiao_du = (int16_t)(((uint16_t)d[0] << 8) | d[1]);
      su_du   = (int16_t)(((uint16_t)d[2] << 8) | d[3]);

      fb_seen   = 1;
      fb_age_ms = 0;
    }
  }
}

/**
  * @brief  转子角度 -> 输出轴角度
  * @note   jiao_du 是单圈值，转一圈就从 8191 跳回 0。
  *         直接相减会在跳变那一刻算出一个巨大的假增量，所以先累加成连续值：
  *         增量超过半圈（±4096）就说明发生了回绕，往反方向补一圈修正。
  *
  *         输出轴（减速后）角度 = 转子累计角度 ÷ 减速比，再对 8192 取模，
  *         得到输出轴自己的 0~8191 单圈角度 —— 这才是"减速过的 0~360 度"。
  *         取模是对「输出轴」取的，不是对转子取的：
  *         8192 个 count 铺满的是输出轴一整圈（转子得转 19.2 圈才走完）。
  *
  *         本函数必须每拍都跑：它靠相邻两拍的差值累加，
  *         一旦被跳过就会漏掉中间的转动，rotor_total 越跑越偏。
  */
static void UpdateOutputAngle(void)
{
  /* 还没收到过任何一帧反馈时不能对齐基准：
   * 此时 jiao_du 仍是初值 0，拿它当基准会让整个坐标系偏掉一大截，
   * 之后所有的角度读数都会带着这个固定偏移 */
  if (!fb_seen)
  {
    return;
  }

  if (!ang_init)
  {
    rotor_last  = jiao_du;         /* 首拍：把基准对齐到电机当前的实际角度 */
    rotor_total = 0;
    out_cnt32   = 0;
    out_angle   = 0;
    ang_init    = 1;
    return;                        /* 这一拍只建基准，不产生增量 */
  }

  int16_t delta = jiao_du - rotor_last;
  if (delta >  ENC_HALF) delta -= ENC_PER_REV;   /* 正向跨过 8191 -> 0 */
  if (delta < -ENC_HALF) delta += ENC_PER_REV;   /* 反向跨过 0 -> 8191 */

  rotor_total += delta;
  rotor_last   = jiao_du;

  /* 直接在细计数域做除法，一步到位。
   *
   * 注意不能先算整数输出 count 再乘 16：那样只是给整数补了 16 个零，
   * out_cnt32 永远是 16 的倍数，分辨率仍旧是 0.044°，等于白改。
   * 必须把 ×OUT_FINE 乘进被除数里，让除法本身产生小数部分。
   *
   * 而且必须做「向下取整」而不是 C 默认的「向零取整」：
   * 反方向转时 rotor_total 为负，-1.5 会被截成 -1，位置就比真实值多算半个
   * count，目标落域时会稳定偏 0.02° 量级。判余数符号再补一，才是真正的 floor */
  int64_t num  = rotor_total * GEAR_NUM * OUT_FINE;
  int64_t fine = num / GEAR_DEN;
  if ((num % GEAR_DEN) < 0) fine--;

  out_cnt32 = (int32_t)fine;       /* 细计数域，控制环只用这个 */

  /* 单圈角度只用于显示，同样向下取整 */
  int64_t whole = fine / OUT_FINE;
  if ((fine % OUT_FINE) < 0) whole--;

  out_angle = (int16_t)(whole % ENC_PER_REV);
  if (out_angle < 0) out_angle += ENC_PER_REV;   /* 反向转时取模为负，补成正的 */
}

/**
  * @brief  参考轨迹发生器：把阶跃目标变成「加速度受限」的平滑轨迹
  * @note   这是本版相对上一版最大的结构改动，也是"超调小"的主要来源。
  *
  *         为什么不能把阶跃目标直接喂给位置环：
  *         位置环输出的是速度指令。目标一跳变，速度指令瞬间顶到上限，
  *         电机以最大加速度冲出去；等误差小到该减速时，速度环还有滞后，
  *         实际转速降不下来，于是冲过头，再反向拉回来——超调就是这么来的。
  *         想靠调大 pos_Kd 压住它，代价是全程都被刹着，上升变慢。
  *
  *         正确做法是让「参考」本身就满足执行器的能力：
  *           · 速度不超过 prof_v；
  *           · 加速度不超过 prof_a；
  *           · 减速按制动曲线提前开始，误差归零时速度也正好归零。
  *         位置环只需要跟踪这条参考，永远不会被要求做做不到的事，
  *         于是「快」和「不冲」不再矛盾。
  *
  *         制动曲线 v = sqrt(2·a·e) 是匀减速下的精确解，
  *         比"线性比例减速"好在：后者在高速接近时会算错减速起点，
  *         参考自己就先冲过目标，位置环再跟着冲，超调照样发生。
  */
static void Profile_Step(void)
{
  /* 整形关闭：目标直通，退化成上一版的阶跃响应，方便对比 */
  if (prof_en < 0.5f)
  {
    ref_pos   = (float)target_cnt;
    ref_vel   = 0.0f;
    prof_done = 1;
    return;
  }

  float err = (float)target_cnt - ref_pos;

  prof_done = 0;                   /* 只要参考还没停稳，就还在运动过程中 */

  /* 制动曲线给出的"当前该有的速度"，再受 prof_v 封顶 */
  float v_des = sqrtf(2.0f * prof_a * fabsf(err) * BRAKE_K);
  if (v_des > prof_v)
  {
    v_des = prof_v;
  }
  if (err < 0.0f)
  {
    v_des = -v_des;
  }

  /* 加速度限幅：1ms 内速度最多变化 prof_a/1000 rpm */
  float dvmax = prof_a * 0.001f;
  float dv    = v_des - ref_vel;
  if (dv >  dvmax) dv =  dvmax;
  if (dv < -dvmax) dv = -dvmax;
  ref_vel += dv;

  /* 参考位置积分一步：rpm × (count/ms per rpm) = count */
  float step_cnt = ref_vel * RPM_TO_CNT_PER_MS;

  /* 单步不允许越过目标。
   * 制动曲线是连续域的精确解，但这里是 1ms 的离散积分——速度在整个步长内
   * 被当成常数，高速时一步就走 21 个 count（约 0.94°），足够直接跨过目标，
   * 于是参考自己先超调了，位置环跟着超调。
   * 把这一步的位移夹到剩余距离以内，超调就从根上消失 */
  if (fabsf(step_cnt) > fabsf(err))
  {
    step_cnt = err;
    ref_vel  = err / RPM_TO_CNT_PER_MS;
  }
  ref_pos += step_cnt;

  /* 到位判定：误差不足 1 个 count 且这一拍就能停住，直接把参考钉在目标上。
   * 不这么做的话，sqrt 曲线在 e→0 时会退化成每拍半个 count 的微小往复，
   * 参考自己在抖，位置环就会跟着抖出 0.04° 量级的毛刺 */
  if ((fabsf((float)target_cnt - ref_pos) < 1.0f) && (fabsf(ref_vel) < dvmax))
  {
    ref_pos   = (float)target_cnt;
    ref_vel   = 0.0f;
    prof_done = 1;
  }
}

/**
  * @brief  控制环走一步：轨迹整形 -> 位置环 -> 速度环 -> 发电流
  * @note   由 out() 按固定 CTRL_PERIOD_MS 节拍调用。
  *
  *         为什么必须固定节拍：
  *         PD 只看当前值，周期抖了无非响应快慢不均；
  *         但积分是"误差 × 时间"的累加，周期一变，speed_Ki 就失去了
  *         时间含义——同一个 Ki 在不同周期下积分速度能差好几倍。
  */
static void Control_Step(void)
{
  /* 先加，再由 ReadMotorFeedback 清零。
   * 到门限就停住不再往上加：它是 uint16，无脑自增 65 秒后会回绕到 0，
   * 于是"掉线"状态每 65 秒被解除一次，白给 20ms 的乱输出。
   * 夹在门限上方一位，既不回绕，也保证判定条件始终成立 */
  if (fb_age_ms <= FB_TIMEOUT_MS)
  {
    fb_age_ms++;
  }

  /* ======================================================================
   *  0. 上电与掉线保护
   *  ---------------------------------------------------------------------
   *  没收到过反馈就绝不下发电流：此时角度是假的，闭环会朝着错误的方向
   *  顶满电流。同理，反馈断了 20ms 还继续按老数据算，等于开环盲推。
   * ====================================================================*/
  if (!ang_init)
  {
    C620_SendCurrent(0);
    cur_out = 0;
    return;
  }

  if (fb_age_ms > FB_TIMEOUT_MS)
  {
    /* 掉线：撤电流、清积分、把参考钉在当前实际位置。
     * 同时把目标改成当前位置——链路恢复后原地保持，
     * 而不是突然冲回掉线前那个可能已经过期的目标 */
    i_term      = 0.0f;
    ref_pos     = (float)out_cnt32;
    ref_vel     = 0.0f;
    target_cnt  = out_cnt32;

    /* 目标改写成当前位置的单圈值，全在细计数域，避免量化偏差 */
    int32_t cur_mod = out_cnt32 % POS_PER_REV;
    if (cur_mod < 0) cur_mod += POS_PER_REV;
    target_raw  = cur_mod;
    target_seen = cur_mod;
    ctrl_init   = 0;                /* 让微分基准在恢复后重新对齐，
                                     * 否则第一拍的 (su_du - 旧speed_prev)
                                     * 会打出一记电流尖峰 */
    cur_out     = 0;
    C620_SendCurrent(0);
    return;
  }

  if (!ctrl_init)
  {
    speed_prev = su_du;             /* 首拍微分基准对齐到实测转速，
                                     * 否则 (su_du - 0) 会打出一记电流尖峰 */
    ctrl_init  = 1;
  }

  /* ======================================================================
   *  1. 目标更新：把 0~8191 的单圈目标落到连续域
   *  ---------------------------------------------------------------------
   *  只在新命令到达时才重算，避免每拍都做一次取模判断。
   *  走最短路径：从 350° 去 10°，直接相减得 -340°，照着走要反向绕一大圈；
   *  归一化后变成 +20°，正向走过去就到
   * ====================================================================*/
  int32_t tgt = target_raw;         /* volatile 只读一次，防止中途被中断改掉 */

  if (tgt != target_seen)
  {
    target_seen = tgt;

    /* 归一化必须全程在细计数域里做。
     * 用 out_angle（已量化到整数输出 count）去减会丢掉小数部分，
     * 目标落域后仍带着最多 15 个细 count 的偏差（0.04°），
     * 前面把分辨率提到 16 倍就白做了。
     * 这里直接对细计数的当前位置取单圈模，保留全部精度 */
    int32_t cur_mod = out_cnt32 % POS_PER_REV;
    if (cur_mod < 0) cur_mod += POS_PER_REV;

    int32_t delta = tgt - cur_mod;
    if (delta >  POS_HALF) delta -= POS_PER_REV;
    if (delta < -POS_HALF) delta += POS_PER_REV;

    target_cnt = out_cnt32 + delta;
  }

  /* ======================================================================
   *  2. 参考轨迹
   * ====================================================================*/
  Profile_Step();

  /* ======================================================================
   *  3. 位置环（外环）
   *  ---------------------------------------------------------------------
   *  输入：参考位置与实际位置之差
   *  输出：速度指令，交给内环执行
   *
   *  为什么不让电流直接控位置？
   *  位置是速度的积分。直接控电流的话，冲到目标时速度还很高，
   *  惯性会带着它冲过头，然后反向再来一次，来回振荡。
   *  套一层速度环之后，"到点该减速"这件事就由内环负责了。
   *
   *  为什么位置环只用 P？
   *  位置环里的 I 会让整个系统变成双积分（它本身就在积分速度），必然振荡。
   *  稳态余差交给内环的积分去消。
   *
   *  第二项 pos_Kd × (参考速度 - 实测转速) 是速度前馈 + 阻尼的合体：
   *    · 运动过程中 参考速度 ≠ 0，它给出与参考速度同向的加速指令，
   *      让实际转速主动跟住参考，而不是等误差积累出来才反应；
   *    · 参考到位后 参考速度归零，它退化成 -pos_Kd × 实测转速，
   *      也就是上一版那个治"到点来回振荡"的阻尼项。
   *  一个公式覆盖两种工况，不需要切换。
   *
   *  先算成 float 再限幅，最后才转 int16。
   *  顺序反了会出事：pos_Kp 一旦调大，乘积超过 int16 上限会变负数，
   *  电机直接朝反方向狂转
   * ====================================================================*/
  float ref_err = ref_pos - (float)out_cnt32;

  /* ======================================================================
   *  到位保持（带迟滞）
   *  ---------------------------------------------------------------------
   *  电机在目标附近静止时，位置环只能输出很小的速度指令，速度环要顶开
   *  静摩擦只能靠积分慢慢攒。攒够了就蹦一下，蹦过头再反向攒一次——
   *  这就是「电机在抖」的物理来源，周期几百毫秒、幅度 0.1° 量级。
   *
   *  只加死区不够：误差清零了，但积分器里还留着上一轮攒下的量，
   *  它会把电机推出死区，于是循环照旧。所以这里做三件事：
   *    1) 迟滞：进入门限 1×db，退出门限 3×db，避免在边界反复穿越；
   *    2) 进入保持后位置环输出清零，只剩速度环的阻尼项；
   *    3) 保持期位置环 P 归零，改用一条「慢积分」把保持电流调到刚好够用
   *       （见下面的速度环），慢到电流变化本身激不起抖动。
   *       这一条是最关键的：刚进保持时 i_term 是「刚好顶开静摩擦」的
   *       电流，远大于维持静止所需。如果只是把它冻住，它会继续推着
   *       电机往外爬——实测 0.2°、周期 700ms 的粘滑极限环就是这么来的。
   *       衰减之后电流降到「推不动」的水平，电机靠静摩擦停住，真的不动。
   *
   *  迟滞的代价：如果负载力矩变了，位置会先漂到 3×db 才被纠正，
   *  纠完再重新进保持。这是一次很慢、很小的修正，不是持续抖动。
   * ====================================================================*/
  if (!prof_done)
  {
    hold_state = 0;                 /* 参考还在动，谈不上保持 */
  }
  else
  {
    float db_in  = pos_db * (float)POS_PER_REV / 360.0f;
    float db_out = db_in * 3.0f;

    if (!hold_state)
    {
      if (fabsf(ref_err) < db_in)
      {
        hold_state = 1;
        i_term    *= hold_rt;       /* 先把「顶开静摩擦」的多余电流砍掉 */
      }
    }
    else if (fabsf(ref_err) > db_out)
    {
      hold_state = 0;               /* 被推出去超过迟滞带，重新接管 */
    }
  }

  if (hold_state)
  {
    /* 保持期：位置环 P 不动作（否则小误差下的 P 输出会直接激起极限环），
     * 只留一条「慢积分」把保持电流调到刚好够用。
     * 慢是关键：电流变化率被 hold_rmax 卡住，一次过冲不超过静摩擦，
     * 电机就一直被摩擦锁在原地，真的不动 */
    float de = hold_ki * ref_err;
    if (de >  hold_rmax) de =  hold_rmax;
    if (de < -hold_rmax) de = -hold_rmax;
    i_term += de;

    if (i_term >  i_max) i_term =  i_max;
    if (i_term < -i_max) i_term = -i_max;

    ref_err = 0.0f;                 /* 位置环输出归零，只剩阻尼项和这个保持电流 */
  }

  pos_err = (float)target_cnt - (float)out_cnt32;   /* 给波形看真实误差（1/16 count） */

  float cmd_f = pos_Kp * ref_err + pos_Kd * (ref_vel - (float)su_du);

  if (cmd_f >  speed_max) cmd_f =  speed_max;
  if (cmd_f < -speed_max) cmd_f = -speed_max;

  speed_cmd = (int16_t)cmd_f;

  /* ======================================================================
   *  4. 速度环（内环）：PID + 速度前馈
   *  ---------------------------------------------------------------------
   *  P：响应速度，差多少给多少电流
   *  D：抑制超调。这里对「实测转速」求导，而不是对误差求导——
   *     两者在目标不变时完全等价，但目标一跳变，误差求导会打出一记
   *     与目标跳变量成正比的电流尖峰（derivative kick），
   *     实测求导则完全不受目标变化影响，冲击小得多。
   *  I：消稳态余差。纯 PD 要维持一个电流顶住摩擦，而这个电流只能来自
   *     "转速误差 × Kp"，所以误差永远消不到 0，位置也就停不到目标上。
   *     积分把历史上攒下的误差记住，慢慢把电流补上去。
   *  Kf：速度前馈，按目标转速直接给一份电流。默认 0（关闭），
   *     等实测出摩擦量级后再开，能把积分项的负担卸掉大半，到位更快。
   * ====================================================================*/
  int16_t err_v = (int16_t)(speed_cmd - su_du);

  float p_term = speed_Kp * (float)err_v;
  float d_term = -speed_Kd * (float)(su_du - speed_prev);
  speed_prev   = su_du;

  /* 积分累加，并且单独限幅。
   * 只靠后面的总限幅不够：积分一旦累到 8000，目标变了要退很久才降得下来，
   * 表现就是"该停的时候停不住、冲过头"。
   *
   * 到位保持期间不走这条快速积分（改由上面的慢积分接管）：
   * 静止时速度误差迟迟消不掉，积分会一直往同方向攒，攒过静摩擦就蹦一下——
   * 不再累加等于把这个循环从源头断掉 */
  if (!hold_state)
  {
    i_term += speed_Ki * (float)err_v;

    if (i_term >  i_max) i_term =  i_max;
    if (i_term < -i_max) i_term = -i_max;
  }

  float cur_cmd = p_term + d_term + i_term + speed_Kf * (float)speed_cmd;

  /* 抗积分饱和：输出已经顶到限幅，误差还在往同方向推，
   * 这时继续积分毫无意义，只会在退出饱和时多冲一段。
   * 把这一拍的累加撤回去——注意是"撤销"而不是"跳过"，
   * 因为累加动作已经执行了，撤销才等价于这一拍没积分 */
  if (!hold_state)
  {
    if (cur_cmd > cur_max)
    {
      cur_cmd = cur_max;
      i_term -= speed_Ki * (float)err_v;
    }
    else if (cur_cmd < -cur_max)
    {
      cur_cmd = -cur_max;
      i_term -= speed_Ki * (float)err_v;
    }
  }

  cur_out = (int16_t)cur_cmd;       /* 留一份给调试器看 */
  C620_SendCurrent(cur_out);
}

/**
  * @brief  向 C620 电调发送电流指令
  * @param  current 电流指令，范围 -16384 ~ 16384，对应 -20A ~ +20A
  * @note   0x200 这一帧的 8 个字节按顺序分给 4 个电机：
  *           data[0..1] = 电机1   data[2..3] = 电机2
  *           data[4..5] = 电机3   data[6..7] = 电机4
  *         本工程电机挂在第 3 个槽位，所以只填 data[4]、data[5]。
  *         每个值都是大端（高字节在前）。
  *         发送邮箱满了就直接丢这一拍，等下次再发，避免阻塞控制环。
  */
static void C620_SendCurrent(int16_t current)
{
  uint8_t data[8] = {0};

  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
  {
    return;                        /* 三个邮箱都占着，这一拍放弃 */
  }

  data[4] = (uint8_t)((uint16_t)current >> 8);      /* 高字节 */
  data[5] = (uint8_t)((uint16_t)current & 0xFF);    /* 低字节 */

  txHeader.StdId              = C620_TX_ID;
  txHeader.ExtId              = 0;
  txHeader.RTR                = CAN_RTR_DATA;
  txHeader.IDE                = CAN_ID_STD;
  txHeader.DLC                = 8;
  txHeader.TransmitGlobalTime = DISABLE;

  (void)HAL_CAN_AddTxMessage(&hcan1, &txHeader, data, &txMailbox);
}

/**
  * @brief  波形输出（JustFloat 协议）
  * @note   每个通道 4 字节小端 float，末尾 4 字节 00 00 80 7F 是帧尾
  *         （即 float 的 +Inf，位模式 0x7F800000 的小端写法）。
  *         VOFA+ 靠这个帧尾切分数据，字节顺序写反了就一个点都画不出来。
  *
  *         通道组用串口命令 "$wave=N!" 切换，VOFA+ 那边始终只配 2 个通道，
  *         不用改任何设置：
  *           wave=0  位置误差(°)  + 实际角度(°)      ← 看收敛、超调、稳态余差
  *           wave=1  速度指令(rpm) + 实测速度(rpm)    ← 看内环跟不跟得上
  *           wave=2  参考位置(°)   + 实际位置(°)      ← 看轨迹跟踪的滞后
  *           wave=3  电流指令(count) + 实测转速(rpm)  ← 看电流是否饱和、量摩擦
  *
  *         发送用中断方式，不阻塞控制环。
  *         否则 12 字节 @115200 要 1.04ms，会把 1ms 的控制节拍顶歪，
  *         积分增益也就跟着失真了。
  *         uart_busy 由 HAL_UART_TxCpltCallback 清零。
  */
static void Waveform_Send(void)
{
  if (uart_busy)
  {
    return;                        /* 上一帧还没发完，jf 不能动 */
  }

  const float fine2deg = 360.0f / (float)POS_PER_REV;   /* 1/16 count -> 度 */
  const float cnt2deg  = 360.0f / (float)ENC_PER_REV;   /* 输出 count -> 度 */
  float c0;
  float c1;

  switch ((int)wave_sel)
  {
    case 1:                        /* 内环：速度指令 vs 实测 */
      c0 = (float)speed_cmd;
      c1 = (float)su_du;
      break;

    case 2:                        /* 轨迹：参考 vs 实际 */
      c0 = ref_pos * fine2deg;
      c1 = (float)out_cnt32 * fine2deg;
      break;

    case 3:                        /* 电流指令 vs 实测转速：看是否饱和、量摩擦 */
      c0 = (float)cur_out;
      c1 = (float)su_du;
      break;

    case 4:                        /* 积分项 vs 总电流：看保持时积分占多少 */
      c0 = i_term;
      c1 = (float)cur_out;
      break;

    default:                       /* 外环：误差 vs 实际角度 */
      c0 = pos_err * fine2deg;
      c1 = (float)out_angle * cnt2deg;
      break;
  }

  memcpy(&jf[0], &c0, 4);
  memcpy(&jf[4], &c1, 4);
  jf[8]  = 0x00;
  jf[9]  = 0x00;
  jf[10] = 0x80;
  jf[11] = 0x7F;

  uart_busy = 1;
  if (HAL_UART_Transmit_IT(&huart1, jf, sizeof(jf)) != HAL_OK)
  {
    uart_busy = 0;                 /* 启动失败，下一拍重试 */
  }
}

/**
  * @brief  控制任务：固定 CTRL_PERIOD_MS 节拍
  * @note   三步顺序不能换：先取反馈，再更新角度，最后才算控制量。
  *         任务函数绝不能 return（会进 prvTaskExitError 关中断死循环），
  *         所以外层必须是 for(;;)，末尾用 vTaskDelayUntil 让出 CPU。
  */
void out(void *argument)
{
  (void)argument;

  const TickType_t xPeriod = pdMS_TO_TICKS(CTRL_PERIOD_MS);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;)
  {
    ReadMotorFeedback();
    UpdateOutputAngle();
    Control_Step();

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

/**
  * @brief  波形任务：固定 JF_PERIOD_MS 节拍
  * @note   与控制任务共用 pos_err / out_angle / ref_pos，前者都是 16 位，
  *         在 Cortex-M4 上读写是原子的；ref_pos 是 float，同样是单字访问。
  *         读到的可能是上一拍的值，对波形显示没有影响。
  */
void fa_song(void *argument)
{
  (void)argument;

  const TickType_t xPeriod = pdMS_TO_TICKS(JF_PERIOD_MS);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;)
  {
    Waveform_Send();

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* 过滤器必须在 Start 之前配好，否则第一帧收不到 */
  CAN_Filter_Config();
  if (HAL_CAN_Start(&hcan1) != HAL_OK) { Error_Handler(); }

  /* 打开串口单字节中断接收，之后靠回调一个字节一个字节攒命令帧 */
  HAL_UART_Receive_IT(&huart1, &rxByte, 1);

  if (xTaskCreate(out,     "ctrl", 256, NULL, 2, NULL) != pdPASS) { Error_Handler(); }
  if (xTaskCreate(fa_song, "wave", 256, NULL, 1, NULL) != pdPASS) { Error_Handler(); }

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief  串口接收完成回调（每收到 1 个字节触发一次）
  * @param  huart 触发中断的串口句柄
  * @note   HAL_UART_Receive_IT 是"收一个字节进一次中断"的模式，
  *         所以这里要自己攒帧：攒到 '!' 就交给 Parse_Frame 解析，然后清空。
  *         末尾必须重新调用 HAL_UART_Receive_IT，否则下次不会再进中断。
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    /* 帧首字符决定这一帧怎么解析：'$' 是参数命令，'=' 是角度命令。
     *
     * 遇到 '=' 要丢掉之前的残帧，否则丢帧后两帧会粘成 "=18=90!"，
     * 被当成 18 解析，目标角悄悄变成错的。
     * 但正在收 "$posKp=8!" 时不能因为 '=' 就清空，否则名字被吃掉，
     * 参数永远匹配不上——所以先看一眼缓冲区开头是不是 '$' */
    if (rxByte == '$')
    {
      rxLen = 0;
    }
    else if ((rxByte == '=') && !((rxLen > 0) && (rxBuf[0] == '$')))
    {
      rxLen = 0;
    }

    if (rxLen < sizeof(rxBuf))
    {
      rxBuf[rxLen++] = rxByte;              /* 攒字节，防止缓冲写爆 */
    }

    if (rxByte == '!')                      /* 收到帧尾，解析这一帧 */
    {
      Parse_Frame(rxBuf, rxLen);
      rxLen = 0;
    }
    else if (rxLen >= sizeof(rxBuf))        /* 攒满还没等到 '!'，说明是垃圾数据 */
    {
      rxLen = 0;
    }

    HAL_UART_Receive_IT(&huart1, &rxByte, 1);   /* 重新开启接收，这行不能省 */
  }
}

/**
  * @brief  串口发送完成回调（波形发完触发一次）
  * @param  huart 触发中断的串口句柄
  * @note   波形用 HAL_UART_Transmit_IT 发送，发完在这里清忙标志。
  *         jf[] 在发送期间不能改，所以填数据前必须等这个标志清零。
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uart_busy = 0;
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
