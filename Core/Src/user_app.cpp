/*
 * user_app.c++
 *
 *  Created on: Jul 9, 2020
 *      Author: nevol
 */
#include "user_app.hpp"

uint16_t motor_control_phase_delay = 1000;
uint32_t counter=0;
uint32_t bemfA=0;
uint32_t bemfB=0;
uint32_t bemfC=0;
int32_t sum_bemf=0;
int32_t delta_bemf=0;
int32_t last_delta_bemf=0;
uint32_t open_loop_counter = 0;

Motor_Active_Phase phase_to_activate = C_B;

//This function is called every 10µs=0,000010s
void outputs::Trigger_outputs(uint16_t duty, uint32_t all_bemf[])
{
	motor_control_phase_delay = 1*duty;

	if (counter >= motor_control_phase_delay)
	{
		open_loop_counter ++;

		Read_Bemf(all_bemf);
		switch (phase_to_activate)
		{
			case C_B:
				outputs::SetPhase(phase_to_activate);
				delta_bemf = bemfA - sum_bemf;
				if (outputs::Check_zero_corssing_point()) {phase_to_activate = A_B;}
				break;
			case A_B:
				outputs::SetPhase(phase_to_activate);
				delta_bemf = bemfC - sum_bemf;
				if (outputs::Check_zero_corssing_point()) {phase_to_activate = A_C;}
				break;
			case A_C:
				outputs::SetPhase(phase_to_activate);
				delta_bemf = bemfB - sum_bemf;
				if (outputs::Check_zero_corssing_point()) {phase_to_activate = B_C;}
				break;
			case B_C:
				outputs::SetPhase(phase_to_activate);
				delta_bemf = bemfA - sum_bemf;
				if (outputs::Check_zero_corssing_point()) {phase_to_activate = B_A;}
				break;
			case B_A:
				outputs::SetPhase(phase_to_activate);
				delta_bemf = bemfC - sum_bemf;
				if (outputs::Check_zero_corssing_point()) {phase_to_activate = C_A;}
				break;
			case C_A:
				outputs::SetPhase(phase_to_activate);
				delta_bemf = bemfB - sum_bemf;
				if (outputs::Check_zero_corssing_point()) {phase_to_activate = C_B;}
				break;
		}

		counter=0;
	}else {counter++;}
}

void outputs::Read_Bemf(uint32_t all_bemf[])
{
	bemfA = all_bemf[0];
	bemfB = all_bemf[1];
	bemfC = all_bemf[2];
	sum_bemf= (bemfA + bemfB + bemfC)/3;
}


/*
HIGH	C	A	A	B	B	C
LOW		B	B	C	C	A	A
BEMF	A	C	B	A	C	B
*/
void outputs::SetPhase(Motor_Active_Phase active_phase1)
{
	switch (phase_to_activate)
	{
	case C_B:
		HAL_GPIO_WritePin(Q1_GPIO_Port, Q1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Q2_GPIO_Port, Q2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Q3_GPIO_Port, Q3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Q4_GPIO_Port, Q4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Q5_GPIO_Port, Q5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Q6_GPIO_Port, Q6_Pin, GPIO_PIN_SET);
		//phase_to_activate = A_B;
		break;
	case A_B:
		HAL_GPIO_WritePin(Q1_GPIO_Port, Q1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Q2_GPIO_Port, Q2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Q3_GPIO_Port, Q3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Q4_GPIO_Port, Q4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Q5_GPIO_Port, Q5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Q6_GPIO_Port, Q6_Pin, GPIO_PIN_RESET);
		//phase_to_activate = A_C;
		break;
	case A_C:
		HAL_GPIO_WritePin(Q1_GPIO_Port, Q1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Q2_GPIO_Port, Q2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Q3_GPIO_Port, Q3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Q4_GPIO_Port, Q4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Q5_GPIO_Port, Q5_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Q6_GPIO_Port, Q6_Pin, GPIO_PIN_RESET);
		//phase_to_activate = B_C;
		break;
	case B_C:
		HAL_GPIO_WritePin(Q1_GPIO_Port, Q1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Q2_GPIO_Port, Q2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Q3_GPIO_Port, Q3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Q4_GPIO_Port, Q4_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Q5_GPIO_Port, Q5_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Q6_GPIO_Port, Q6_Pin, GPIO_PIN_RESET);
		//phase_to_activate = B_A;
		break;
	case B_A:
		HAL_GPIO_WritePin(Q1_GPIO_Port, Q1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Q2_GPIO_Port, Q2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Q3_GPIO_Port, Q3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Q4_GPIO_Port, Q4_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Q5_GPIO_Port, Q5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Q6_GPIO_Port, Q6_Pin, GPIO_PIN_RESET);
		//phase_to_activate = C_A;
		break;
	case C_A:
		HAL_GPIO_WritePin(Q1_GPIO_Port, Q1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Q2_GPIO_Port, Q2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Q3_GPIO_Port, Q3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Q4_GPIO_Port, Q4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Q5_GPIO_Port, Q5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Q6_GPIO_Port, Q6_Pin, GPIO_PIN_SET);
		//phase_to_activate = C_B;
		break;
//	case C_B:
//		HAL_GPIO_WritePin(Q1_GPIO_Port, Q1_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(Q2_GPIO_Port, Q2_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(Q3_GPIO_Port, Q3_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(Q4_GPIO_Port, Q4_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(Q5_GPIO_Port, Q5_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(Q6_GPIO_Port, Q6_Pin, GPIO_PIN_RESET);
//		phase_to_activate = A_B;
//		break;
//	case A_B:
//		HAL_GPIO_WritePin(Q1_GPIO_Port, Q1_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(Q2_GPIO_Port, Q2_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(Q3_GPIO_Port, Q3_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(Q4_GPIO_Port, Q4_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(Q5_GPIO_Port, Q5_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(Q6_GPIO_Port, Q6_Pin, GPIO_PIN_SET);
//		phase_to_activate = A_C;
//		break;
//	case A_C:
//		HAL_GPIO_WritePin(Q1_GPIO_Port, Q1_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(Q2_GPIO_Port, Q2_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(Q3_GPIO_Port, Q3_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(Q4_GPIO_Port, Q4_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(Q5_GPIO_Port, Q5_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(Q6_GPIO_Port, Q6_Pin, GPIO_PIN_SET);
//		phase_to_activate = B_C;
//		break;
//	case B_C:
//		HAL_GPIO_WritePin(Q1_GPIO_Port, Q1_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(Q2_GPIO_Port, Q2_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(Q3_GPIO_Port, Q3_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(Q4_GPIO_Port, Q4_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(Q5_GPIO_Port, Q5_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(Q6_GPIO_Port, Q6_Pin, GPIO_PIN_SET);
//		phase_to_activate = B_A;
//		break;
//	case B_A:
//		HAL_GPIO_WritePin(Q1_GPIO_Port, Q1_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(Q2_GPIO_Port, Q2_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(Q3_GPIO_Port, Q3_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(Q4_GPIO_Port, Q4_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(Q5_GPIO_Port, Q5_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(Q6_GPIO_Port, Q6_Pin, GPIO_PIN_SET);
//		phase_to_activate = C_A;
//		break;
//	case C_A:
//		HAL_GPIO_WritePin(Q1_GPIO_Port, Q1_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(Q2_GPIO_Port, Q2_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(Q3_GPIO_Port, Q3_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(Q4_GPIO_Port, Q4_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(Q5_GPIO_Port, Q5_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(Q6_GPIO_Port, Q6_Pin, GPIO_PIN_RESET);
//		phase_to_activate = C_B;
//		break;

	}
}

bool outputs::Check_zero_corssing_point()
{
	if (open_loop_counter >=3000)
	{

		//Zero cross from - to +
		if (last_delta_bemf <= 0)
		{
			if (delta_bemf > 0)
			{
				last_delta_bemf = delta_bemf; //save the last delta
				return true;
			}
		}

		//Zero cross from + to -
		if (last_delta_bemf > 0)
		{
			if (delta_bemf < 0)
			{
				last_delta_bemf = delta_bemf;
				return true;
			  }
		 }

		return false; //no crossing point reached
	}
	else
	{
		return true;
	}
}
