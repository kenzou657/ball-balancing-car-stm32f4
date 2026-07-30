#ifndef __TRACK_IR_H
#define __TRACK_IR_H

#include "sys.h"

/*
 * 9 路红外循迹模块
 * 引脚来自 DOC/侧边引脚复用.md 的中央排针方案。
 * 若实际接线变化，只需修改本文件中的 TRACK_IRx_PORT / TRACK_IRx_PIN。
 */

#define TRACK_IR_NUM                9

/* 黑线有效电平：0=黑线输出低电平，1=黑线输出高电平 */
#define TRACK_BLACK_LEVEL           0

/* 多路同时触发时认为可能是 A/B 点、横线、宽线标志 */
#define TRACK_WIDE_COUNT_TH         5

/* 连续丢线计数上限由上层控制模块决定，这里只负责累计 */
#define TRACK_LOST_COUNT_MAX        60000u

/* 输入 GPIO 上下拉配置：多数红外模块为推挽输出，默认上拉更稳妥 */
#define TRACK_GPIO_PUPD             GPIO_PuPd_UP

/* IR0..IR8 从左到右排列 */
#define TRACK_IR0_PORT              GPIOC
#define TRACK_IR0_PIN               GPIO_Pin_11
#define TRACK_IR1_PORT              GPIOA
#define TRACK_IR1_PIN               GPIO_Pin_7
#define TRACK_IR2_PORT              GPIOC
#define TRACK_IR2_PIN               GPIO_Pin_5
#define TRACK_IR3_PORT              GPIOC
#define TRACK_IR3_PIN               GPIO_Pin_3
#define TRACK_IR4_PORT              GPIOC
#define TRACK_IR4_PIN               GPIO_Pin_2
#define TRACK_IR5_PORT              GPIOC
#define TRACK_IR5_PIN               GPIO_Pin_1
#define TRACK_IR6_PORT              GPIOC
#define TRACK_IR6_PIN               GPIO_Pin_0
#define TRACK_IR7_PORT              GPIOC
#define TRACK_IR7_PIN               GPIO_Pin_10
#define TRACK_IR8_PORT              GPIOD
#define TRACK_IR8_PIN               GPIO_Pin_15

#define TRACK_IR0_CLK               RCC_AHB1Periph_GPIOC
#define TRACK_IR1_CLK               RCC_AHB1Periph_GPIOA
#define TRACK_IR2_CLK               RCC_AHB1Periph_GPIOC
#define TRACK_IR3_CLK               RCC_AHB1Periph_GPIOC
#define TRACK_IR4_CLK               RCC_AHB1Periph_GPIOC
#define TRACK_IR5_CLK               RCC_AHB1Periph_GPIOC
#define TRACK_IR6_CLK               RCC_AHB1Periph_GPIOC
#define TRACK_IR7_CLK               RCC_AHB1Periph_GPIOC
#define TRACK_IR8_CLK               RCC_AHB1Periph_GPIOD

typedef struct
{
    uint16_t raw_mask;          /* 原始电平掩码，bit0=IR0，bit8=IR8，1 表示 GPIO 读到高电平 */
    uint16_t line_mask;         /* 黑线有效掩码，bit0=IR0，bit8=IR8，1 表示检测到黑线 */
    uint8_t active_count;       /* 检测到黑线的通道数量 */
    uint8_t line_valid;         /* 是否检测到黑线 */
    uint8_t wide_line;          /* 是否达到宽线/横线候选阈值 */
    float line_error;           /* 加权平均线偏差，左负右正 */
    float last_line_error;      /* 最近一次有效线偏差 */
    uint16_t lost_count;        /* 连续丢线计数 */
    uint16_t wide_line_count;   /* 连续宽线候选计数 */
} TrackIrState_t;

extern TrackIrState_t TrackIrState;

void Track_IR_Init(void);
void Track_IR_Reset(TrackIrState_t *state);
void Track_IR_Update(TrackIrState_t *state);
uint16_t Track_IR_ReadRawMask(void);
uint16_t Track_IR_ReadLineMask(void);
float Track_IR_CalcLineError(uint16_t line_mask, uint8_t *active_count);
uint8_t Track_IR_IsWideLine(const TrackIrState_t *state, uint8_t stable_count);

#endif
