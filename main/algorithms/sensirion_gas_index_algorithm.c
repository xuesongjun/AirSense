/*
 * Copyright (c) 2020, Sensirion AG
 * All rights reserved.
 */

#include "sensirion_gas_index_algorithm.h"
#include <math.h>

#define ALGORITHM_TYPE_VOC 0
#define ALGORITHM_TYPE_NOX 1

#define F16(x) ((float)(x))

static const float VOC_SRAW_MINIMUM = 20000.0f;
static const float NOX_SRAW_MINIMUM = 10000.0f;
static const float VOC_GATING_THRESHOLD = 340.0f;
static const float NOX_GATING_THRESHOLD = 30.0f;
static const float TAU_MEAN_HOURS = 12.0f;
static const float GATING_MAX_DURATION_MINUTES = 180.0f;
static const float SAMPLING_INTERVAL = 3.0f;  // 主循环每3秒采样一次
static const float INITIAL_BLACKOUT = 45.0f;  // 45秒初始化 = 15次采样

static void _init_instances(GasIndexAlgorithmParams* params) {
    params->mUptime = F16(0.0f);
    params->mSraw = F16(0.0f);
    params->mGas_Index = F16(0.0f);
}

static void _reset_algorithm(GasIndexAlgorithmParams* params) {
    params->mUptime = F16(0.0f);
    params->mSraw = F16(0.0f);
    params->mGas_Index = F16(0.0f);
}

void GasIndexAlgorithm_init(GasIndexAlgorithmParams* params, int32_t algorithm_type) {
    params->mIndex = algorithm_type;

    if (algorithm_type == ALGORITHM_TYPE_VOC) {
        params->mSraw_Minimum = VOC_SRAW_MINIMUM;
        params->mGating_Threshold = VOC_GATING_THRESHOLD;
    } else {
        params->mSraw_Minimum = NOX_SRAW_MINIMUM;
        params->mGating_Threshold = NOX_GATING_THRESHOLD;
    }

    params->mTau_Mean_Hours = TAU_MEAN_HOURS;
    params->mGating_Max_Duration_Minutes = GATING_MAX_DURATION_MINUTES;

    _init_instances(params);
}

void GasIndexAlgorithm_reset(GasIndexAlgorithmParams* params) {
    _reset_algorithm(params);
}

void GasIndexAlgorithm_process(GasIndexAlgorithmParams* params, int32_t sraw, int32_t* gas_index) {
    // 室内空气质量监测使用绝对阈值映射，不使用自适应基线
    //
    // VOC 典型值参考 (Sensirion SGP41):
    //   干净空气: 25000-30000
    //   轻度污染: 30000-35000
    //   中度污染: 35000-40000
    //   重度污染: 40000+
    //
    // NOx 典型值参考:
    //   干净空气: 14000-16000
    //   轻度污染: 16000-20000
    //   中度污染: 20000-25000
    //   重度污染: 25000+

    if (sraw <= 0 || sraw >= 65000) {
        // 无效数据
        *gas_index = 1;
        return;
    }

    // 增加运行时间计数
    params->mUptime += SAMPLING_INTERVAL;

    // 初始化期间(45秒)返回1，避免传感器预热期间的不稳定值
    if (params->mUptime <= INITIAL_BLACKOUT) {
        *gas_index = 1;
        return;
    }

    // 根据算法类型选择映射范围
    float baseline;      // 优秀空气质量基准值 (index = 100)
    float good_max;      // 良好空气质量上限 (index = 200)
    float moderate_max;  // 中等空气质量上限 (index = 300)
    float poor_max;      // 较差空气质量上限 (index = 400)

    if (params->mIndex == ALGORITHM_TYPE_VOC) {
        // VOC 映射
        baseline = 27000.0f;      // 典型干净室内空气
        good_max = 32000.0f;      // 轻度污染
        moderate_max = 37000.0f;  // 中度污染
        poor_max = 42000.0f;      // 重度污染
    } else {
        // NOx 映射
        baseline = 15000.0f;      // 典型干净室内空气
        good_max = 18000.0f;      // 轻度污染
        moderate_max = 22000.0f;  // 中度污染
        poor_max = 27000.0f;      // 重度污染
    }

    // 分段线性映射到 1-500
    float index_value;
    float sraw_f = (float)sraw;

    if (sraw_f <= baseline) {
        // 优秀: 1-100
        index_value = 1.0f + (sraw_f / baseline) * 99.0f;
    } else if (sraw_f <= good_max) {
        // 良好: 100-200
        index_value = 100.0f + ((sraw_f - baseline) / (good_max - baseline)) * 100.0f;
    } else if (sraw_f <= moderate_max) {
        // 中等: 200-300
        index_value = 200.0f + ((sraw_f - good_max) / (moderate_max - good_max)) * 100.0f;
    } else if (sraw_f <= poor_max) {
        // 较差: 300-400
        index_value = 300.0f + ((sraw_f - moderate_max) / (poor_max - moderate_max)) * 100.0f;
    } else {
        // 很差: 400-500
        index_value = 400.0f + ((sraw_f - poor_max) / (poor_max * 0.2f)) * 100.0f;
    }

    // 限制范围 1-500
    if (index_value < 1.0f) {
        index_value = 1.0f;
    } else if (index_value > 500.0f) {
        index_value = 500.0f;
    }

    params->mGas_Index = index_value;
    *gas_index = (int32_t)(index_value + 0.5f);
}
