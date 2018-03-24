/*
 * FeedbackDriver.h
 *
 *  Created on: 2017. nov. 4.
 *      Author: Vityó
 */

#ifndef FEEDBACKDRIVER_H_
#define FEEDBACKDRIVER_H_

#include <infra.h>

#define TIM1_PERIOD_REG 100
#define	TIM1_PWM_DUTY_CYCLE_UNIT ((TIM1_PERIOD_REG+1) / (100.0-1.0))

namespace V {

class FeedbackDriver{
private:
	infra& infra_ref;
	uint8_t brightness;
	float vonalpoz;
	int asd;
public:
	FeedbackDriver(V::infra& infra): infra_ref(infra) {};
	virtual ~FeedbackDriver();
	void setBrightness( uint8_t duty_cycle );
	void debug_feedback();
	void debug_feedback2();
	void driveLeds();
};

} /* namespace V */

#endif /* FEEDBACKDRIVER_H_ */
