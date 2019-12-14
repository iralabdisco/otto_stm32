#ifndef PID_H
#define PID_H

class Pid {
public:
	float kp_;
	float ki_;
	float kd_;
	float previous_error_;
	float error_;
	float error_sum_;
	float setpoint_;
	int min_;
	int max_;

	Pid(float kp, float ki, float kd, int min, int max){
		this->kp_ = kp;
		this->ki_ = ki;
		this->kd_ = kd;
		this->previous_error_ = 0;
		this->error_ = 0;
		this->error_sum_ = 0;
		this->setpoint_ = 0;
		this->min_ = min;
		this->max_ = max;
	}

	void set(float setpoint){
		this->setpoint_ = setpoint;
	}

	int update(float measure){

		this->error_ = this->setpoint_ - measure;

		//proportional term
		float output = this->error_ * this->kp_;

		//TODO integral term

		//TODO derivative term

		if (output > this->max_){

			output = this->max_;

			//TODO anti-windup (prima capisco cos'è)

		} else if (output < this->min_){

			output = this->min_;

			//TODO anti-windup
		}

		int integer_output = (int) output;

		return integer_output;

	}
};
#endif
