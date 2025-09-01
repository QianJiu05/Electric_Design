// #include "main.h"
/* 系统常量定义 */
#define TS_PLL          0.00005f     // 50μs 采样周期 (对应 20kHz 执行频率)
#define TWO_PI          6.28318530718f  // 精确的 2π 值
#define TWO_PI_OVER_3   2.0943951023931953f  // 2π/3 (120度)
#define SQRT_3_OVER_2   0.8660254037844386f   // √3/2
#define CLARK_GAIN      0.6666666666666666f   // 2/3 (Clark变换系数)

/* PI 控制器参数 */
#define PI_GAIN_KP      0.1f         // PI控制器比例增益
#define PI_GAIN_KI      5.0f        // PI控制器积分增益

/* 系统状态结构 */
typedef struct {
    /* VCO 部分 */
    float vco_phase;         // VCO 相位累加器 (弧度)
    float vco_freq;          // VCO 当前频率 (Hz)

    /* PI 控制器部分 */
    float integrator;        // PI 积分器状态

    /* 输入/输出缓冲 */
    float vabc[3];           // 三相电压测量值缓存
} System_State;

/* 外部接口 */
typedef struct {
    /* 输入信号 */
    float freq_sp;           // 频率设定点 (Hz)
    float volt_ref;          // 参考电压幅值 (V)
    float vabc[3];           // 三相电压测量值 (V)

    /* 输出信号 */
    float abc[3];            // 三相调制波输出 (占空比: -1.0~1.0)
    float theta;             // 生成相位 (rad)
    float freq_actual;       // 实际频率 (Hz)
} System_IO;

typedef struct {
    double sample_time;      // 采样时间（0表示连续模式）
    double fundamental_freq; // 信号基频(Hz)
    double period;           // 信号周期(1/fundamental_freq)
    double initial_cond;     // 初始条件
    double *buffer;          // 离散模式下的缓冲区
    int buffer_size;         // 缓冲区大小
    int current_index;       // 当前缓冲区索引
    double integral;         // 连续模式下的积分值
    double last_time;        // 上次更新时间
} RMS_Calculator;


void system_init(void);
void control_step(void);
void system_step(void);
RMS_Calculator* RMS_Init(double sample_time, double fundamental_freq, double initial_cond);
// 释放RMS计算器
void RMS_Free(RMS_Calculator *rms);
// 更新RMS值（连续模式）
double RMS_Update_Continuous(RMS_Calculator *rms, double input, double current_time);
// 更新RMS值（离散模式）
double RMS_Update_Discrete(RMS_Calculator *rms, double input);
// 主RMS更新函数（根据模式自动选择）
double RMS_Update(RMS_Calculator *rms, double input,double current_time );
