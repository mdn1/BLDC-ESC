/*
 * user_app.hpp
 *
 *  Created on: Jul 9, 2020
 *      Author: nevol
 */

#ifndef INC_USER_APP_HPP_
#define INC_USER_APP_HPP_

#include "main.h"
#include "stm32f4xx_hal.h"

enum MOTOR_ACTIVE_PHASE
{
	C_B = 0,
	A_B,
	A_C,
	B_C,
	B_A,
	C_A
};

typedef enum
{
	STOP = 0,
	OPEN_LOOP,
	CLOSED_LOOP
}MOTOR_STATE;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern uint32_t duty;

class outputs
{
public:
	void Motor_Control_Routine();
	void Trigger_outputs(uint32_t all_bemf[]);


private:
	void Motor_Alignment();
	void Open_Loop();
	void Closed_Loop();
	void SetPhase(MOTOR_ACTIVE_PHASE active_phase, uint32_t pwm_duty);
	void Read_Bemf(uint32_t all_bemf[]);
	bool Check_zero_corssing_point();
};



#endif /* INC_USER_APP_HPP_ */
