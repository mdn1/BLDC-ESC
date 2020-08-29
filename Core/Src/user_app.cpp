/*
 * user_app.c++
 *
 *  Created on: Jul 9, 2020
 *      Author: nevol
 */
#include "user_app.hpp"

uint16_t motor_control_phase_delay = 20;
uint32_t counter=0;
uint32_t alignment_counter=0;
uint32_t bemfA=0;
uint32_t bemfB=0;
uint32_t bemfC=0;
int32_t sum_bemf=0;
int32_t delta_bemf=0;
int32_t last_delta_bemf=0;
uint32_t open_loop_counter = 0;

MOTOR_STATE motor_state = STOP;
MOTOR_ACTIVE_PHASE active_phase = C_B;

void outputs::Motor_Control_Routine()
{
	switch (motor_state)
	{
	case STOP:
		Motor_Alignment();
		break;
	case OPEN_LOOP:
		Open_Loop();
		break;
	case CLOSED_LOOP:
		break;
	}
}

//This function is called every 100µs=0,000 1s
void outputs::Motor_Alignment()
{
	if (alignment_counter >= 0 && alignment_counter <=1 )
	{
		SetPhase(C_B, 50);
		alignment_counter ++;
	}
	else if(alignment_counter > 1 && alignment_counter < 20000)
	{
		alignment_counter++;
	}
	else if (alignment_counter == 20000)
	{
		SetPhase((MOTOR_ACTIVE_PHASE)(active_phase + 1), 50);
		alignment_counter ++;
	}
	else if (alignment_counter > 20000 && alignment_counter < 30000)
	{
		alignment_counter ++;
	}else
	{
		alignment_counter = 0;
		motor_state = OPEN_LOOP;
	}

}

//This function is called every 10µs=0,000010s
void outputs::Trigger_outputs(uint32_t all_bemf[])
{
	motor_control_phase_delay = 1;

	if (counter >= motor_control_phase_delay)
	{
		open_loop_counter ++;
		//Closed_Loop();
		Read_Bemf(all_bemf);

		counter=0;
	}
	else
	{
		counter++;
	}
}


void outputs::Open_Loop()
{
	if (counter >= motor_control_phase_delay)
		{
		switch (active_phase)
				{
					case C_B:
						outputs::SetPhase(A_B, duty);
						break;
					case A_B:
						outputs::SetPhase(A_C, duty);
						break;
					case A_C:
						outputs::SetPhase(B_C, duty);
						break;
					case B_C:
						outputs::SetPhase(B_A, duty);
						break;
					case B_A:
						outputs::SetPhase(C_A, duty);
						break;
					case C_A:
						outputs::SetPhase(C_B, duty);
						break;
				}
			counter=0;
		}
		else
		{
			counter++;
		}
}


/*
void outputs::Closed_Loop()
{
	switch (active_phase)
	{
		case C_B:
			outputs::SetPhase(active_phase);
			delta_bemf = bemfA - sum_bemf;
			if (outputs::Check_zero_corssing_point()) {active_phase = A_B;}
			break;
		case A_B:
			outputs::SetPhase(active_phase);
			delta_bemf = bemfC - sum_bemf;
			if (outputs::Check_zero_corssing_point()) {active_phase = A_C;}
			break;
		case A_C:
			outputs::SetPhase(active_phase);
			delta_bemf = bemfB - sum_bemf;
			if (outputs::Check_zero_corssing_point()) {active_phase = B_C;}
			break;
		case B_C:
			outputs::SetPhase(active_phase);
			delta_bemf = bemfA - sum_bemf;
			if (outputs::Check_zero_corssing_point()) {active_phase = B_A;}
			break;
		case B_A:
			outputs::SetPhase(active_phase);
			delta_bemf = bemfC - sum_bemf;
			if (outputs::Check_zero_corssing_point()) {active_phase = C_A;}
			break;
		case C_A:
			outputs::SetPhase(active_phase);
			delta_bemf = bemfB - sum_bemf;
			if (outputs::Check_zero_corssing_point()) {active_phase = C_B;}
			break;
	}
}
*/

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
void outputs::SetPhase(MOTOR_ACTIVE_PHASE phase_to_activate, uint32_t pwm_duty)
{
	switch (phase_to_activate)
	{
		case C_B:
			htim3.Instance->CCR1 = 0;
			htim3.Instance->CCR2 = 0;
			htim3.Instance->CCR3 = 100;
			htim3.Instance->CCR4 = 0;
			htim1.Instance->CCR1 = 0;
			htim1.Instance->CCR4 = pwm_duty;
			active_phase = C_B;
			break;
		case A_B:
			htim3.Instance->CCR1 = 0;
			htim3.Instance->CCR2 = pwm_duty;
			htim3.Instance->CCR3 = 100;
			htim3.Instance->CCR4 = 0;
			htim1.Instance->CCR1 = 0;
			htim1.Instance->CCR4 = 0;
			active_phase = A_B;
			break;
		case A_C:
			htim3.Instance->CCR1 = 0;
			htim3.Instance->CCR2 = pwm_duty;
			htim3.Instance->CCR3 = 0;
			htim3.Instance->CCR4 = 0;
			htim1.Instance->CCR1 = 100;
			htim1.Instance->CCR4 = 0;
			active_phase = A_C;
			break;
		case B_C:
			htim3.Instance->CCR1 = 0;
			htim3.Instance->CCR2 = 0;
			htim3.Instance->CCR3 = 0;
			htim3.Instance->CCR4 = pwm_duty;
			htim1.Instance->CCR1 = 100;
			htim1.Instance->CCR4 = 0;
			active_phase = B_C;
			break;
		case B_A:
			htim3.Instance->CCR1 = 100;
			htim3.Instance->CCR2 = 0;
			htim3.Instance->CCR3 = 0;
			htim3.Instance->CCR4 = pwm_duty;
			htim1.Instance->CCR1 = 0;
			htim1.Instance->CCR4 = 0;
			active_phase = B_A;
			break;
		case C_A:
			htim3.Instance->CCR1 = 100;
			htim3.Instance->CCR2 = 0;
			htim3.Instance->CCR3 = 0;
			htim3.Instance->CCR4 = 0;
			htim1.Instance->CCR1 = 0;
			htim1.Instance->CCR4 = pwm_duty;
			active_phase = C_A;
			break;
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
