/*
	Copyright 2016 - 2022 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "mcpwm.h"
#include "mc_interface.h"
#include "utils_math.h"
#include "utils_sys.h"

// Structs
typedef struct {
	volatile bool updated;
	volatile unsigned int top;
	volatile unsigned int duty;
	volatile unsigned int val_sample;
	volatile unsigned int curr1_sample;
	volatile unsigned int curr2_sample;
#ifdef HW_HAS_3_SHUNTS
	volatile unsigned int curr3_sample;
#endif
} mc_timer_struct;

// Private variables
static volatile int comm_step; // Range [1 6]
static volatile int direction;
static volatile float dutycycle_set;
static volatile float dutycycle_now;
static volatile float m_pll_speed;
static volatile mc_timer_struct timer_struct;
static volatile int curr_samp_volt; // Use the voltage-synchronized samples for this current sample
static int invert_counter = 0;
static volatile mc_configuration *conf;
static volatile bool dccal_done;
static volatile bool init_done = false;

// Private functions
static void set_duty_cycle_hl(float dutyCycle);
static void set_duty_cycle_ll(float dutyCycle);
static void set_duty_cycle_hw(float dutyCycle);
static void stop_pwm_ll(void);
static void stop_pwm_hw(void);
static void full_brake_ll(void);
static void full_brake_hw(void);
static void set_next_comm_step(int next_step);
static void update_adc_sample_pos(mc_timer_struct *timer_tmp);
static void commutate(int steps);
static void set_next_timer_settings(mc_timer_struct *settings);
static void update_timer_attempt(void);
static void set_switching_frequency(float frequency);
static void do_dc_cal(void);

// Defines
#define IS_DETECTING()			(state == MC_STATE_DETECTING)

// Threads
static THD_WORKING_AREA(timer_thread_wa, 512);
static THD_FUNCTION(timer_thread, arg);
static volatile bool timer_thd_stop;

void mcpwm_init(volatile mc_configuration *configuration) {
	utils_sys_lock_cnt();

	init_done= false;

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

	conf = configuration;

	// Initialize variables
	comm_step = 1;
	direction = 1;
	dutycycle_set = 0.0;
	dutycycle_now = 0.0;
	curr_samp_volt = 0;
	invert_counter = 0;
	dccal_done = false;
	m_pll_speed = 0.0;

	TIM_DeInit(TIM1);
	TIM_DeInit(TIM8);
	TIM1->CNT = 0;
	TIM8->CNT = 0;

	// TIM1 clock enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	// Time Base configuration
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = SYSTEM_CORE_CLOCK / (int)conf->m_dc_f_sw;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	// Channel 1, 2 and 3 Configuration in PWM mode
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = TIM1->ARR / 2;

#ifndef INVERTED_TOP_DRIVER_INPUT
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // gpio high = top fets on
#else
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
#endif
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

#ifndef INVERTED_BOTTOM_DRIVER_INPUT
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;  // gpio high = bottom fets on
#else
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
#endif
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

	// Automatic Output enable, Break, dead time and lock configuration
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	TIM_BDTRInitStructure.TIM_DeadTime = conf_general_calculate_deadtime(HW_DEAD_TIME_NSEC, SYSTEM_CORE_CLOCK);
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;

	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
	TIM_CCPreloadControl(TIM1, ENABLE);
	TIM_ARRPreloadConfig(TIM1, ENABLE);

	/*
	 * ADC!
	 */
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	// Clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_ADC3, ENABLE);

	dmaStreamAllocate(STM32_DMA_STREAM(STM32_DMA_STREAM_ID(2, 4)),
			5,
			(stm32_dmaisr_t)mcpwm_adc_int_handler,
			(void *)0);

	// DMA for the ADC
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC_Value;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC->CDR;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = HW_ADC_CHANNELS;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream4, &DMA_InitStructure);

	// DMA2_Stream0 enable
	DMA_Cmd(DMA2_Stream4, ENABLE);

	// Enable transfer complete interrupt
	DMA_ITConfig(DMA2_Stream4, DMA_IT_TC, ENABLE);

	// ADC Common Init
	// Note that the ADC is running at 42MHz, which is higher than the
	// specified 36MHz in the data sheet, but it works.
	ADC_CommonInitStructure.ADC_Mode = ADC_TripleMode_RegSimult;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	// Channel-specific settings
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Falling;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T8_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = HW_ADC_NBR_CONV;

	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = 0;
	ADC_Init(ADC2, &ADC_InitStructure);
	ADC_Init(ADC3, &ADC_InitStructure);

	// Enable Vrefint channel
	ADC_TempSensorVrefintCmd(ENABLE);

	// Enable DMA request after last transfer (Multi-ADC mode)
	ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);

	// Injected channels for current measurement at end of cycle
	ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_CC4);
	ADC_ExternalTrigInjectedConvConfig(ADC2, ADC_ExternalTrigInjecConv_T8_CC2);
#ifdef HW_HAS_3_SHUNTS
	ADC_ExternalTrigInjectedConvConfig(ADC3, ADC_ExternalTrigInjecConv_T8_CC3);
#endif
	ADC_ExternalTrigInjectedConvEdgeConfig(ADC1, ADC_ExternalTrigInjecConvEdge_Falling);
	ADC_ExternalTrigInjectedConvEdgeConfig(ADC2, ADC_ExternalTrigInjecConvEdge_Falling);
#ifdef HW_HAS_3_SHUNTS
	ADC_ExternalTrigInjectedConvEdgeConfig(ADC3, ADC_ExternalTrigInjecConvEdge_Falling);
#endif
	ADC_InjectedSequencerLengthConfig(ADC1, HW_ADC_INJ_CHANNELS);
	ADC_InjectedSequencerLengthConfig(ADC2, HW_ADC_INJ_CHANNELS);
#ifdef HW_HAS_3_SHUNTS
	ADC_InjectedSequencerLengthConfig(ADC3, HW_ADC_INJ_CHANNELS);
#endif

	hw_setup_adc_channels();

	// Interrupt
	ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);
	nvicEnableVector(ADC_IRQn, 6);

	// Enable ADC1
	ADC_Cmd(ADC1, ENABLE);

	// Enable ADC2
	ADC_Cmd(ADC2, ENABLE);

	// Enable ADC3
	ADC_Cmd(ADC3, ENABLE);

	// ------------- Timer8 for ADC sampling ------------- //
	// Time Base configuration
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 500;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM8, ENABLE);
	TIM_CCPreloadControl(TIM8, ENABLE);

	// PWM outputs have to be enabled in order to trigger ADC on CCx
	TIM_CtrlPWMOutputs(TIM8, ENABLE);

	// TIM1 Master and TIM8 slave
	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);
	TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);
	TIM_SelectInputTrigger(TIM8, TIM_TS_ITR0);
	TIM_SelectSlaveMode(TIM8, TIM_SlaveMode_Reset);

	// Enable TIM1 and TIM8
	TIM_Cmd(TIM1, ENABLE);
	TIM_Cmd(TIM8, ENABLE);

	// Main Output Enable
	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	// ADC sampling locations
	stop_pwm_hw();
	mc_timer_struct timer_tmp;
	timer_tmp.top = TIM1->ARR;
	timer_tmp.duty = TIM1->ARR / 2;
	update_adc_sample_pos(&timer_tmp);
	set_next_timer_settings(&timer_tmp);

	utils_sys_unlock_cnt();

	CURRENT_FILTER_ON();
	CURRENT_FILTER_ON_M2();

	// Calibrate current offset
	ENABLE_GATE();
	DCCAL_OFF();
	do_dc_cal();

	// Start threads
	timer_thd_stop = false;
	chThdCreateStatic(timer_thread_wa, sizeof(timer_thread_wa), NORMALPRIO, timer_thread, NULL);

	init_done = true;
}

void mcpwm_deinit(void) {
	if (!init_done) {
		return;
	}

	init_done = false;

	timer_thd_stop = true;

	while (timer_thd_stop) {
		chThdSleepMilliseconds(1);
	}

	TIM_DeInit(TIM1);
	TIM_DeInit(TIM8);
	ADC_DeInit();
	DMA_DeInit(DMA2_Stream4);
	nvicDisableVector(ADC_IRQn);
	dmaStreamRelease(STM32_DMA_STREAM(STM32_DMA_STREAM_ID(2, 4)));
}

bool mcpwm_init_done(void) {
	return init_done;
}

void mcpwm_set_configuration(volatile mc_configuration *configuration) {
	// Stop everything first to be safe
	stop_pwm_ll();

	utils_sys_lock_cnt();
	conf = configuration;
	utils_sys_unlock_cnt();
}

static void do_dc_cal(void) {
	static volatile int curr0_sum;
	static volatile int curr1_sum;
#ifdef HW_HAS_3_SHUNTS
	static volatile int curr2_sum;
#endif
	static volatile int curr_start_samples;
	static volatile int curr0_offset;
	static volatile int curr1_offset;
#ifdef HW_HAS_3_SHUNTS
	static volatile int curr2_offset;
#endif

	DCCAL_ON();

	// Wait max 5 seconds
	int cnt = 0;
	while(IS_DRV_FAULT()){
		chThdSleepMilliseconds(1);
		cnt++;
		if (cnt > 5000) {
			break;
		}
	};

	chThdSleepMilliseconds(1000);
	curr0_sum = 0;
	curr1_sum = 0;

#ifdef HW_HAS_3_SHUNTS
	curr2_sum = 0;
#endif

	curr_start_samples = 0;
	while(curr_start_samples < 4000) {};
	curr0_offset = curr0_sum / curr_start_samples;
	curr1_offset = curr1_sum / curr_start_samples;

#ifdef HW_HAS_3_SHUNTS
	curr2_offset = curr2_sum / curr_start_samples;
#endif

	DCCAL_OFF();
	dccal_done = true;
}


/**
 * Use duty cycle control. Absolute values less than MCPWM_MIN_DUTY_CYCLE will
 * stop the motor.
 *
 * @param dutyCycle
 * The duty cycle to use.
 */
void mcpwm_set_duty(float dutyCycle) {
	set_duty_cycle_hl(dutyCycle);
}

/**
 * Use duty cycle control. Absolute values less than MCPWM_MIN_DUTY_CYCLE will
 * stop the motor.
 *
 * WARNING: This function does not use ramping. A too large step with a large motor
 * can destroy hardware.
 *
 * @param dutyCycle
 * The duty cycle to use.
 */
void mcpwm_set_duty_noramp(float dutyCycle) {
	if (state != MC_STATE_RUNNING) {
		set_duty_cycle_hl(dutyCycle);
	} else {
		dutycycle_set = dutyCycle;
		dutycycle_now = dutyCycle;
		set_duty_cycle_ll(dutyCycle);
	}
}


/**
 * Calculate the current RPM of the motor. This is a signed value and the sign
 * depends on the direction the motor is rotating in. Note that this value has
 * to be divided by half the number of motor poles.
 *
 * @return
 * The RPM value.
 */
float mcpwm_get_rpm(void) {
	return RADPS2RPM_f(m_pll_speed);
}

mc_state mcpwm_get_state(void) {
	return state;
}

/**
 * Calculate the KV (RPM per volt) value for the motor. This function has to
 * be used while the motor is moving. Note that the return value has to be
 * divided by half the number of motor poles.
 *
 * @return
 * The KV value.
 */
float mcpwm_get_kv(void) {
	return rpm_now / (mc_interface_get_input_voltage_filtered() * fabsf(dutycycle_now));
}



/**
 * Get the motor current. The sign of this value will
 * represent whether the motor is drawing (positive) or generating
 * (negative) current.
 *
 * @return
 * The motor current.
 */
float mcpwm_get_tot_current(void) {
	return last_current_sample;
}


/**
 * Switch off all FETs.
 */
void mcpwm_stop_pwm(void) {
	control_mode = CONTROL_MODE_NONE;
	stop_pwm_ll();
}

static void stop_pwm_ll(void) {
	state = MC_STATE_OFF;
	ignore_iterations = MCPWM_CMD_STOP_TIME;
	stop_pwm_hw();
}

static void stop_pwm_hw(void) {
#ifdef HW_HAS_DRV8313
	DISABLE_BR();
#endif

	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive);
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);

	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

	set_switching_frequency(conf->m_bldc_f_sw_max);
}

static void full_brake_ll(void) {
	state = MC_STATE_FULL_BRAKE;
	ignore_iterations = MCPWM_CMD_STOP_TIME;
	full_brake_hw();
}

static void full_brake_hw(void) {
#ifdef HW_HAS_DRV8313
	ENABLE_BR();
#endif

	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive);
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);

	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);

	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);

	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

	set_switching_frequency(conf->m_bldc_f_sw_max);
}

/**
 * High-level duty cycle setter. Will set the ramping goal of the duty cycle.
 * If motor is not running, it will be started in different ways depending on
 * whether it is moving or not.
 *
 * @param dutyCycle
 * The duty cycle in the range [-MCPWM_MAX_DUTY_CYCLE MCPWM_MAX_DUTY_CYCLE]
 * If the absolute value of the duty cycle is less than MCPWM_MIN_DUTY_CYCLE,
 * the motor phases will be shorted to brake the motor.
 */
static void set_duty_cycle_hl(float dutyCycle) {
	utils_truncate_number(&dutyCycle, -conf->l_max_duty, conf->l_max_duty);

	if (state == MC_STATE_DETECTING) {
		stop_pwm_ll();
		return;
	}

	dutycycle_set = dutyCycle;

	if (state != MC_STATE_RUNNING) {
		if (fabsf(dutyCycle) >= conf->l_min_duty) {
			// dutycycle_now is updated by the back-emf detection. If the motor already
			// is spinning, it will be non-zero.
			if (fabsf(dutycycle_now) < conf->l_min_duty) {
				dutycycle_now = SIGN(dutyCycle) * conf->l_min_duty;
			}

			set_duty_cycle_ll(dutycycle_now);
		} else {
			// In case the motor is already spinning, set the state to running
			// so that it can be ramped down before the full brake is applied.
			if (fabsf(dutycycle_now) > 0.1) {
				state = MC_STATE_RUNNING;
			} else {
				full_brake_ll();
			}
		}
	}
}

/**
 * Low-level duty cycle setter. Will update the state of the application
 * and the motor direction accordingly.
 *
 * This function should be used with care. Ramping together with current
 * limiting should be used.
 *
 * @param dutyCycle
 * The duty cycle in the range [-MCPWM_MAX_DUTY_CYCLE MCPWM_MAX_DUTY_CYCLE]
 * If the absolute value of the duty cycle is less than MCPWM_MIN_DUTY_CYCLE,
 * the motor will be switched off.
 */
static void set_duty_cycle_ll(float dutyCycle) {
	if (dutyCycle >= conf->l_min_duty) {
		direction = 1;
	} else if (dutyCycle <= -conf->l_min_duty) {
		dutyCycle = -dutyCycle;
		direction = 0;
	}

	if (dutyCycle < conf->l_min_duty) {
		float max_erpm_fbrake;
		max_erpm_fbrake = conf->l_max_erpm_fbrake;

		switch (state) {
		case MC_STATE_RUNNING:
			if (fabsf(rpm_now) > max_erpm_fbrake) {
				dutyCycle = conf->l_min_duty;
			} else {
				full_brake_ll();
				return;
			}
			break;

		case MC_STATE_DETECTING:
			stop_pwm_ll();
			return;
			break;

		default:
			return;
		}
	} else if (dutyCycle > conf->l_max_duty) {
		dutyCycle = conf->l_max_duty;
	}

	set_duty_cycle_hw(dutyCycle);

	state = MC_STATE_RUNNING;
	set_next_comm_step(comm_step);
	commutate(1);
}

/**
 * Lowest level (hardware) duty cycle setter. Will set the hardware timer to
 * the specified duty cycle and update the ADC sampling positions.
 *
 * @param dutyCycle
 * The duty cycle in the range [MCPWM_MIN_DUTY_CYCLE  MCPWM_MAX_DUTY_CYCLE]
 * (Only positive)
 */
static void set_duty_cycle_hw(float dutyCycle) {
	mc_timer_struct timer_tmp;

	utils_sys_lock_cnt();
	timer_tmp = timer_struct;
	utils_sys_unlock_cnt();

	utils_truncate_number(&dutyCycle, conf->l_min_duty, conf->l_max_duty);

	timer_tmp.top = SYSTEM_CORE_CLOCK / (int)conf->m_dc_f_sw;

	timer_tmp.duty = (uint16_t)((float)timer_tmp.top * dutyCycle);

	update_adc_sample_pos(&timer_tmp);
	set_next_timer_settings(&timer_tmp);
}


static THD_FUNCTION(timer_thread, arg) {
	(void)arg;

	chRegSetThreadName("mcpwm timer");

	float amp;
	float min_s;
	float max_s;

	for(;;) {
		if (timer_thd_stop) {
			timer_thd_stop = false;
			return;
		}

		if (state == MC_STATE_OFF) {
			// Track the motor back-emf and follow it with dutycycle_now. Also track
			// the direction of the motor.
			amp = filter_run_fir_iteration((float*)amp_fir_samples,
					(float*)amp_fir_coeffs, AMP_FIR_TAPS_BITS, amp_fir_index);

			// Direction tracking
			if (amp > 0) {
				direction = 0;
			} else {
				direction = 1;
				amp = -amp;
			}
			dutycycle_now = amp / (float)ADC_Value[ADC_IND_VIN_SENS];
			utils_truncate_number((float*)&dutycycle_now, -conf->l_max_duty, conf->l_max_duty);
		} 

		chThdSleepMilliseconds(1);
	}
}

void mcpwm_adc_inj_int_handler(void) {
	uint32_t t_start = timer_time_now();

	int curr0 = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
	int curr1 = ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_1);

	int curr0_2 = ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_2);
	int curr1_2 = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_2);

#ifdef HW_HAS_3_SHUNTS
	int curr2 = ADC_GetInjectedConversionValue(ADC3, ADC_InjectedChannel_1);
#endif

#ifdef INVERTED_SHUNT_POLARITY
	curr0 = 4095 - curr0;
	curr1 = 4095 - curr1;

	curr0_2 = 4095 - curr0_2;
	curr1_2 = 4095 - curr1_2;
#ifdef HW_HAS_3_SHUNTS
	curr2 = 4095 - curr2;
#endif
#endif

	float curr0_currsamp = curr0;
	float curr1_currsamp = curr1;
#ifdef HW_HAS_3_SHUNTS
	float curr2_currsamp = curr2;
#endif

	if (curr_samp_volt & (1 << 0)) {
		curr0 = GET_CURRENT1();
	}

	if (curr_samp_volt & (1 << 1)) {
		curr1 = GET_CURRENT2();
	}

#ifdef HW_HAS_3_SHUNTS
	if (curr_samp_volt & (1 << 2)) {
		curr2 = GET_CURRENT3();
	}
#endif

	curr0_sum += curr0;
	curr1_sum += curr1;
#ifdef HW_HAS_3_SHUNTS
	curr2_sum += curr2;
#endif

	curr_start_samples++;

	curr0 -= curr0_offset;
	curr1 -= curr1_offset;
	curr0_2 -= curr0_offset;
	curr1_2 -= curr1_offset;

#ifdef HW_HAS_3_SHUNTS
	curr2_currsamp -= curr2_offset;
	curr2 -= curr2_offset;
#endif

	ADC_curr_norm_value[0] = curr0;
	ADC_curr_norm_value[1] = curr1;

#ifdef HW_HAS_3_SHUNTS
	ADC_curr_norm_value[2] = curr2;
#else
	ADC_curr_norm_value[2] = -(ADC_curr_norm_value[0] + ADC_curr_norm_value[1]);
#endif

	float curr_tot_sample = 0;
	if (conf->motor_type == MOTOR_TYPE_DC) {
		if (direction) {
#ifdef HW_HAS_3_SHUNTS
			curr_tot_sample = -(GET_CURRENT3() - curr2_offset);
#else
			curr_tot_sample = -(GET_CURRENT2() - curr1_offset);
#endif
	} else {
		curr_tot_sample = -(GET_CURRENT1() - curr0_offset);
	}

	last_current_sample = curr_tot_sample * FAC_CURRENT;

	// Filter out outliers
	if (fabsf(last_current_sample) > (conf->l_abs_current_max * 1.2)) {
		last_current_sample = SIGN(last_current_sample) * conf->l_abs_current_max * 1.2;
	}

	filter_add_sample((float*) current_fir_samples, last_current_sample,
			CURR_FIR_TAPS_BITS, (uint32_t*) &current_fir_index);
	last_current_sample_filtered = filter_run_fir_iteration(
			(float*) current_fir_samples, (float*) current_fir_coeffs,
			CURR_FIR_TAPS_BITS, current_fir_index);

	last_inj_adc_isr_duration = timer_seconds_elapsed_since(t_start);
}

/*
 * New ADC samples ready. Do commutation!
 */
void mcpwm_adc_int_handler(void *p, uint32_t flags) {
	(void)p;
	(void)flags;

	uint32_t t_start = timer_time_now();

	// Set the next timer settings if an update is far enough away
	update_timer_attempt();

	// Reset the watchdog
	timeout_feed_WDT(THREAD_MCPWM);

	const float input_voltage = GET_INPUT_VOLTAGE();
	int ph1, ph2, ph3;

	static int direction_before = 1;
	if (!(state == MC_STATE_RUNNING && direction == direction_before)) {
		has_commutated = 0;
	}
	direction_before = direction;


	if (state == MC_STATE_RUNNING && !has_commutated) {
		set_next_comm_step(comm_step);
		commutate(0);
	}
	

	const float current_nofilter = mcpwm_get_tot_current();
	const float current_in_nofilter = current_nofilter * fabsf(dutycycle_now);

	if (state == MC_STATE_RUNNING && has_commutated) {
		// Compensation for supply voltage variations
		const float voltage_scale = 20.0 / input_voltage;
		float ramp_step = conf->m_duty_ramp_step / (conf->m_dc_f_sw / 1000.0);
		float ramp_step_no_lim = ramp_step;

		float dutycycle_now_tmp = dutycycle_now;

		utils_step_towards((float*)&dutycycle_now_tmp, dutycycle_set, ramp_step);

		static int limit_delay = 0;

		// Apply limits in priority order
		if (current_nofilter > conf->lo_current_max) {
			utils_step_towards((float*) &dutycycle_now, 0.0,
					ramp_step_no_lim * fabsf(current_nofilter - conf->lo_current_max) * conf->m_current_backoff_gain);
			limit_delay = 1;
		} else if (current_nofilter < conf->lo_current_min) {
			utils_step_towards((float*) &dutycycle_now, direction ? conf->l_max_duty : -conf->l_max_duty,
					ramp_step_no_lim * fabsf(current_nofilter - conf->lo_current_min) * conf->m_current_backoff_gain);
			limit_delay = 1;
		} else if (current_in_nofilter > conf->lo_in_current_max) {
			utils_step_towards((float*) &dutycycle_now, 0.0,
					ramp_step_no_lim * fabsf(current_in_nofilter - conf->lo_in_current_max) * conf->m_current_backoff_gain);
			limit_delay = 1;
		} else if (current_in_nofilter < conf->lo_in_current_min) {
			utils_step_towards((float*) &dutycycle_now, direction ? conf->l_max_duty : -conf->l_max_duty,
					ramp_step_no_lim * fabsf(current_in_nofilter - conf->lo_in_current_min) * conf->m_current_backoff_gain);
			limit_delay = 1;
		}

		if (limit_delay > 0) {
			limit_delay--;
		} else {
			dutycycle_now = dutycycle_now_tmp;
		}

		// When the set duty cycle is in the opposite direction, make sure that the motor
		// starts again after stopping completely
		if (fabsf(dutycycle_now) < conf->l_min_duty) {
			if (dutycycle_set >= conf->l_min_duty) {
				dutycycle_now = conf->l_min_duty;
			} else if (dutycycle_set <= -conf->l_min_duty) {
				dutycycle_now = -conf->l_min_duty;
			}
		}

		set_duty_cycle_ll(dutycycle_now);
	}

	mc_interface_mc_timer_isr(false);

	last_adc_isr_duration = timer_seconds_elapsed_since(t_start);
}


bool mcpwm_is_dccal_done(void) {
	return dccal_done;
}


static void update_adc_sample_pos(mc_timer_struct *timer_tmp) {
	volatile uint32_t duty = timer_tmp->duty;
	volatile uint32_t top = timer_tmp->top;
	volatile uint32_t val_sample = timer_tmp->val_sample;
	volatile uint32_t curr1_sample = timer_tmp->curr1_sample;
	volatile uint32_t curr2_sample = timer_tmp->curr2_sample;

#ifdef HW_HAS_3_SHUNTS
	volatile uint32_t curr3_sample = timer_tmp->curr3_sample;
#endif

	if (duty > (uint32_t)((float)top * conf->l_max_duty)) {
		duty = (uint32_t)((float)top * conf->l_max_duty);
	}

	curr_samp_volt = 0;

	if (conf->motor_type == MOTOR_TYPE_DC) {
		curr1_sample = top - 10; // Not used anyway
		curr2_sample = top - 10;
#ifdef HW_HAS_3_SHUNTS
		curr3_sample = top - 10;
#endif

	if (duty > 1000) {
		val_sample = duty / 2;
	} else {
		val_sample = duty + 800;
		curr_samp_volt = (1 << 0) | (1 << 1) | (1 << 2);
	}

	timer_tmp->val_sample = val_sample;
	timer_tmp->curr1_sample = curr1_sample;
	timer_tmp->curr2_sample = curr2_sample;
#ifdef HW_HAS_3_SHUNTS
	timer_tmp->curr3_sample = curr3_sample;
#endif
}


static void commutate(int steps) {

	if (conf->motor_type == MOTOR_TYPE_DC) {
		comm_step += steps;
		while (comm_step > 6) {
			comm_step -= 6;
		}
		while (comm_step < 1) {
			comm_step += 6;
		}

		if (!(state == MC_STATE_RUNNING)) {
			return;
		}

		set_next_comm_step(comm_step);
	}

	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
	has_commutated = 1;

	mc_timer_struct timer_tmp;

	utils_sys_lock_cnt();
	timer_tmp = timer_struct;
	utils_sys_unlock_cnt();

	update_adc_sample_pos(&timer_tmp);
	set_next_timer_settings(&timer_tmp);
}

static void set_next_timer_settings(mc_timer_struct *settings) {
	utils_sys_lock_cnt();
	timer_struct = *settings;
	timer_struct.updated = false;
	utils_sys_unlock_cnt();

	update_timer_attempt();
}

/**
 * Try to apply the new timer settings. This is really not an elegant solution, but for now it is
 * the best I can come up with.
 */
static void update_timer_attempt(void) {
	utils_sys_lock_cnt();

	// Set the next timer settings if an update is far enough away
	if (!timer_struct.updated && TIM1->CNT > 10 && TIM1->CNT < (TIM1->ARR - 500)) {
		// Disable preload register updates
		TIM1->CR1 |= TIM_CR1_UDIS;
		TIM8->CR1 |= TIM_CR1_UDIS;

		// Set the new configuration
		TIM1->ARR = timer_struct.top;
		TIM1->CCR1 = timer_struct.duty;
		TIM1->CCR2 = timer_struct.duty;
		TIM1->CCR3 = timer_struct.duty;
		TIM8->CCR1 = timer_struct.val_sample;
		TIM1->CCR4 = timer_struct.curr1_sample;
		TIM8->CCR2 = timer_struct.curr2_sample;
#ifdef HW_HAS_3_SHUNTS
		TIM8->CCR3 = timer_struct.curr3_sample;
#endif

		// Enables preload register updates
		TIM1->CR1 &= ~TIM_CR1_UDIS;
		TIM8->CR1 &= ~TIM_CR1_UDIS;
		timer_struct.updated = true;
	}

	utils_sys_unlock_cnt();
}

static void set_switching_frequency(float frequency) {
	mc_timer_struct timer_tmp;

	utils_sys_lock_cnt();
	timer_tmp = timer_struct;
	utils_sys_unlock_cnt();

	timer_tmp.top = SYSTEM_CORE_CLOCK / (int)frequency;
	update_adc_sample_pos(&timer_tmp);
	set_next_timer_settings(&timer_tmp);
}

static void set_next_comm_step(int next_step) {
    static bool invert_duty_cycle = false;

    if (conf->motor_type == MOTOR_TYPE_DC) {
        if (invert_counter == 5) {
            invert_duty_cycle = !invert_duty_cycle;
            invert_counter = 0;
        } else {
            invert_counter++;
        }

        // 0
        TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_Inactive);
        TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);

        if (invert_duty_cycle) {
            // Inverted duty cycle
            if (direction) {
                // +
                TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
                TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
                TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);

                // -
                TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_Inactive);
                TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
                TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
            } else {
                // +
                TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
                TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
                TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);

                // -
                TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_Inactive);
                TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
                TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
            }
        } else {
            // Regular duty cycle
            if (direction) {
                // +
                TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
                TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
                TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);

                // -
                TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_Inactive);
                TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
                TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
            } else {
                // +
                TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
                TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
                TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);

                // -
                TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_Inactive);
                TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
                TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
            	}
        	}
    return;
	}
}