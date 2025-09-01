#include <stdio.h>
#include <math.h>
#include <stdlib.h>

// RMS计算器结构体
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

// 初始化RMS计算器
RMS_Calculator* RMS_Init(double sample_time, double fundamental_freq, double initial_cond) {
    RMS_Calculator *rms = (RMS_Calculator*)malloc(sizeof(RMS_Calculator));
    rms->sample_time = sample_time;
    rms->fundamental_freq = fundamental_freq;
    rms->period = 1.0 / fundamental_freq;
    rms->initial_cond = initial_cond;
    rms->last_time = 0.0;
    
    if (sample_time > 0) {
        // 离散模式：分配缓冲区
        rms->buffer_size = (int)(rms->period / sample_time + 0.5);
        rms->buffer = (double*)malloc(rms->buffer_size * sizeof(double));
        
        // 用初始条件填充缓冲区
        for (int i = 0; i < rms->buffer_size; i++) {
            rms->buffer[i] = initial_cond;
        }
        rms->current_index = 0;
    } else {
        // 连续模式：初始化积分值
        rms->buffer = NULL;
        rms->buffer_size = 0;
        rms->integral = initial_cond * rms->period; // 初始积分值
    }
    
    return rms;
}

// 释放RMS计算器
void RMS_Free(RMS_Calculator *rms) {
    if (rms->buffer != NULL) {
        free(rms->buffer);
    }
    free(rms);
}

// 更新RMS值（连续模式）
double RMS_Update_Continuous(RMS_Calculator *rms, double input, double current_time) {
    double delta_time = current_time - rms->last_time;
    
    // 计算新的积分值（梯形法积分）
    double input_sq = input * input;
    double last_input_sq = rms->initial_cond * rms->initial_cond;
    rms->integral += (input_sq + last_input_sq) * delta_time / 2.0;
    
    // 减去超出周期的部分（移动窗口）
    if (current_time > rms->period) {
        double oldest_time = current_time - rms->period;
        // 这里简化处理，实际实现可能需要更精确的旧值计算
        rms->integral -= last_input_sq * delta_time;
    }
    
    rms->last_time = current_time;
    rms->initial_cond = input;
    
    return sqrt(rms->integral / rms->period);
}

// 更新RMS值（离散模式）
double RMS_Update_Discrete(RMS_Calculator *rms, double input) {
    // 将输入平方存入缓冲区
    rms->buffer[rms->current_index] = input * input;
    
    // 计算缓冲区中所有值的平均
    double sum = 0.0;
    for (int i = 0; i < rms->buffer_size; i++) {
        sum += rms->buffer[i];
    }
    double mean = sum / rms->buffer_size;
    
    // 更新索引（循环缓冲区）
    rms->current_index = (rms->current_index + 1) % rms->buffer_size;
    
    return sqrt(mean);
}

// 主RMS更新函数（根据模式自动选择）
double RMS_Update(RMS_Calculator *rms, double input, double current_time) {
    if (rms->sample_time > 0) {
        return RMS_Update_Discrete(rms, input);
    } else {
        return RMS_Update_Continuous(rms, input, current_time);
    }
}

// 测试函数
int main() {
    // 示例1：离散模式RMS计算（50Hz信号，采样率1ms）
    printf("离散模式RMS计算:\n");
    RMS_Calculator *rms_discrete = RMS_Init(0.00001, 50.0, 0.0);
    
    // 生成一个测试信号（50Hz正弦波，幅值1.0）
    for (int i = 0; i < 10000; i++) {
        double t = i * 0.00001;
        double signal = 5*sin(2 * M_PI * 50.0 * t);
        double rms = RMS_Update(rms_discrete, signal, t);
        
        if (i % 10 == 0) {
            printf("时间: %.3fs, 信号值: %.3f, RMS值: %.3f\n", t, signal, rms);
        }
    }
    RMS_Free(rms_discrete);
    
    // 示例2：连续模式RMS计算（50Hz信号）
    printf("\n连续模式RMS计算:\n");
    RMS_Calculator *rms_continuous = RMS_Init(0.0, 50.0, 0.0);
    
    // 生成一个测试信号（50Hz正弦波，幅值1.0）
    for (int i = 0; i < 100; i++) {
        double t = i * 0.001;
        double signal = sin(2 * M_PI * 50.0 * t);
        double rms = RMS_Update(rms_continuous, signal, t);
        
        if (i % 10 == 0) {
            printf("时间: %.3fs, 信号值: %.3f, RMS值: %.3f\n", t, signal, rms);
        }
    }
    RMS_Free(rms_continuous);
    
    return 0;
}