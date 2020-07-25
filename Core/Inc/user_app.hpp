/*
 * user_app.hpp
 *
 *  Created on: Jul 9, 2020
 *      Author: nevol
 */

#ifndef INC_USER_APP_HPP_
#define INC_USER_APP_HPP_

#include "main.h"

	typedef enum
	{
	  C_B = 0,
	  A_B,
	  A_C,
	  B_C,
	  B_A,
	  C_A
	}Motor_Active_Phase;

class outputs
{
public:
	void Trigger_outputs(uint16_t duty, uint32_t all_bemf[]);
	void Read_Bemf(uint32_t all_bemf[]);

private:
	void SetPhase(Motor_Active_Phase active_phase);
	bool Check_zero_corssing_point();
};



#endif /* INC_USER_APP_HPP_ */
