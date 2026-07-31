#include "track_ir.h"

TrackIrState_t TrackIrState;

static const int8_t track_ir_weight[TRACK_IR_NUM] = {-4, -3, -2, -1, 0, 1, 2, 3, 4};

static uint16_t Track_IR_RemapLineMask(uint16_t line_mask)
{
    uint16_t remap_mask = 0;

    line_mask &= 0x01FFu;

    if(line_mask & (1u << 3)) remap_mask |= (1u << 0);
    if(line_mask & (1u << 7)) remap_mask |= (1u << 1);
    if(line_mask & (1u << 6)) remap_mask |= (1u << 2);
    if(line_mask & (1u << 5)) remap_mask |= (1u << 3);
    if(line_mask & (1u << 4)) remap_mask |= (1u << 4);
    if(line_mask & (1u << 8)) remap_mask |= (1u << 5);
    if(line_mask & (1u << 2)) remap_mask |= (1u << 6);
    if(line_mask & (1u << 1)) remap_mask |= (1u << 7);
    if(line_mask & (1u << 0)) remap_mask |= (1u << 8);

    return remap_mask;
}

static void Track_IR_GPIO_Config(GPIO_TypeDef *port, uint16_t pin)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = TRACK_GPIO_PUPD;
    GPIO_Init(port, &GPIO_InitStructure);
}

void Track_IR_Init(void)
{
    RCC_AHB1PeriphClockCmd(TRACK_IR0_CLK | TRACK_IR1_CLK | TRACK_IR2_CLK |
                           TRACK_IR3_CLK | TRACK_IR4_CLK | TRACK_IR5_CLK |
                           TRACK_IR6_CLK | TRACK_IR7_CLK | TRACK_IR8_CLK,
                           ENABLE);

    Track_IR_GPIO_Config(TRACK_IR0_PORT, TRACK_IR0_PIN);
    Track_IR_GPIO_Config(TRACK_IR1_PORT, TRACK_IR1_PIN);
    Track_IR_GPIO_Config(TRACK_IR2_PORT, TRACK_IR2_PIN);
    Track_IR_GPIO_Config(TRACK_IR3_PORT, TRACK_IR3_PIN);
    Track_IR_GPIO_Config(TRACK_IR4_PORT, TRACK_IR4_PIN);
    Track_IR_GPIO_Config(TRACK_IR5_PORT, TRACK_IR5_PIN);
    Track_IR_GPIO_Config(TRACK_IR6_PORT, TRACK_IR6_PIN);
    Track_IR_GPIO_Config(TRACK_IR7_PORT, TRACK_IR7_PIN);
    Track_IR_GPIO_Config(TRACK_IR8_PORT, TRACK_IR8_PIN);

    Track_IR_Reset(&TrackIrState);
}

void Track_IR_Reset(TrackIrState_t *state)
{
    if(state == 0)
    {
        return;
    }

    state->raw_mask = 0;
    state->line_mask = 0;
    state->active_count = 0;
    state->line_valid = 0;
    state->wide_line = 0;
    state->line_error = 0.0f;
    state->last_line_error = 0.0f;
    state->lost_count = 0;
    state->wide_line_count = 0;
}

uint16_t Track_IR_ReadRawMask(void)
{
    uint16_t mask = 0;

    if(GPIO_ReadInputDataBit(TRACK_IR0_PORT, TRACK_IR0_PIN) != Bit_RESET) mask |= (1u << 0);
    if(GPIO_ReadInputDataBit(TRACK_IR1_PORT, TRACK_IR1_PIN) != Bit_RESET) mask |= (1u << 1);
    if(GPIO_ReadInputDataBit(TRACK_IR2_PORT, TRACK_IR2_PIN) != Bit_RESET) mask |= (1u << 2);
    if(GPIO_ReadInputDataBit(TRACK_IR3_PORT, TRACK_IR3_PIN) != Bit_RESET) mask |= (1u << 3);
    if(GPIO_ReadInputDataBit(TRACK_IR4_PORT, TRACK_IR4_PIN) != Bit_RESET) mask |= (1u << 4);
    if(GPIO_ReadInputDataBit(TRACK_IR5_PORT, TRACK_IR5_PIN) != Bit_RESET) mask |= (1u << 5);
    if(GPIO_ReadInputDataBit(TRACK_IR6_PORT, TRACK_IR6_PIN) != Bit_RESET) mask |= (1u << 6);
    if(GPIO_ReadInputDataBit(TRACK_IR7_PORT, TRACK_IR7_PIN) != Bit_RESET) mask |= (1u << 7);
    if(GPIO_ReadInputDataBit(TRACK_IR8_PORT, TRACK_IR8_PIN) != Bit_RESET) mask |= (1u << 8);

    return mask;
}

uint16_t Track_IR_ReadLineMask(void)
{
    uint16_t raw_mask = Track_IR_ReadRawMask();
    uint16_t line_mask;

#if TRACK_BLACK_LEVEL
    line_mask = raw_mask & 0x01FFu;
#else
    line_mask = (~raw_mask) & 0x01FFu;
#endif

    return Track_IR_RemapLineMask(line_mask);
}

float Track_IR_CalcLineError(uint16_t line_mask, uint8_t *active_count)
{
    uint8_t i;
    uint8_t count = 0;
    int16_t weighted_sum = 0;

    line_mask &= 0x01FFu;

    for(i = 0; i < TRACK_IR_NUM; i++)
    {
        if(line_mask & (1u << i))
        {
            weighted_sum += track_ir_weight[i];
            count++;
        }
    }

    if(active_count != 0)
    {
        *active_count = count;
    }

    if(count == 0)
    {
        return 0.0f;
    }

    return (float)weighted_sum / (float)count;
}

void Track_IR_Update(TrackIrState_t *state)
{
    uint8_t active_count = 0;

    if(state == 0)
    {
        return;
    }

    state->raw_mask = Track_IR_ReadRawMask();
#if TRACK_BLACK_LEVEL
    state->line_mask = state->raw_mask & 0x01FFu;
#else
    state->line_mask = (~state->raw_mask) & 0x01FFu;
#endif
    state->line_mask = Track_IR_RemapLineMask(state->line_mask);

    state->line_error = Track_IR_CalcLineError(state->line_mask, &active_count);
    state->active_count = active_count;
    state->line_valid = (active_count > 0) ? 1 : 0;
    state->wide_line = (active_count >= TRACK_WIDE_COUNT_TH) ? 1 : 0;

    if(state->line_valid)
    {
        state->last_line_error = state->line_error;
        state->lost_count = 0;
    }
    else if(state->lost_count < TRACK_LOST_COUNT_MAX)
    {
        state->lost_count++;
    }

    if(state->wide_line)
    {
        if(state->wide_line_count < TRACK_LOST_COUNT_MAX)
        {
            state->wide_line_count++;
        }
    }
    else
    {
        state->wide_line_count = 0;
    }
}

uint8_t Track_IR_IsWideLine(const TrackIrState_t *state, uint8_t stable_count)
{
    if(state == 0)
    {
        return 0;
    }

    if(stable_count == 0)
    {
        return state->wide_line;
    }

    return (state->wide_line_count >= stable_count) ? 1 : 0;
}
