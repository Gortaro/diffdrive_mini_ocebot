#ifndef DIFFDRIVE_MINI_OCEBOT_CONTROLLER_HPP
#define DIFFDRIVE_MINI_OCEBOT_CONTROLLER_HPP

#include <pigpiod_if2.h>
#include <cmath>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"

class Controller
{
    public:
    int pi = 0;
    int left_enc = 0;
    int right_enc = 0;
    int left_motor = 0;
    int right_motor = 0;
    int left_direction = 0;
    int right_direction = 0;

    Controller() = default;

    Controller(int left_enc_pin, int right_enc_pin, int left_motor_pin, int right_motor_pin, int left_dir_pin, int right_dir_pin)
    {
        setup(left_enc_pin, right_enc_pin, left_motor_pin, right_motor_pin, left_dir_pin, right_dir_pin);    
    }

    void setup(int left_enc_pin, int right_enc_pin, int left_motor_pin, int right_motor_pin, int left_dir_pin, int right_dir_pin)
    {
        pi = pigpio_start(NULL, NULL); 

        left_enc = left_enc_pin;
        right_enc = right_enc_pin;

        left_motor = left_motor_pin;
        right_motor = right_motor_pin;

	left_direction = left_dir_pin;
	right_direction = right_dir_pin;

        set_mode(pi, left_enc, PI_INPUT);
        set_mode(pi, right_enc, PI_INPUT);
        
	set_mode(pi, left_motor, PI_OUTPUT);
        set_mode(pi, right_motor, PI_OUTPUT);

	set_PWM_dutycycle(pi, left_motor, 0);
	set_PWM_dutycycle(pi, right_motor, 0);

	set_mode(pi, left_direction, PI_OUTPUT);
	set_mode(pi, right_direction, PI_OUTPUT);
    }

    void register_encoders(int &left_enc, int &right_enc)
    {
	callback_ex(pi, this->left_enc, EITHER_EDGE, read_enc_value, &left_enc);
	callback_ex(pi, this->right_enc, EITHER_EDGE, read_enc_value, &right_enc);
    }

    static void read_enc_value(int /* pi */, unsigned /*gpio*/, unsigned /*level*/, uint32_t /*tick*/, void *encoder)
    {
	(*((int*) encoder))++;
    }

    void set_motor_values(int left, int right)
    {
        int left_direction = (left < 0) ? 1 : 0;
        int right_direction = (right > 0) ? 1 : 0;

        int left_PWM = std::min(abs(left), 30);
        int right_PWM = std::min(abs(right), 30); // Cap max power at ~15%

	int left_current_PWM = get_PWM_dutycycle(pi, left_motor);

        gpio_write(pi, this->left_direction, left_direction);
        gpio_write(pi, this->right_direction, right_direction);

	set_PWM_dutycycle(pi, right_motor, right_PWM);

	while(left_current_PWM != left_PWM)
	{
	    left_current_PWM = get_PWM_dutycycle(pi, left_motor);
	    
	    if(left_current_PWM < left_PWM)
	    {
	        set_PWM_dutycycle(pi, left_motor, left_current_PWM + 1);
	    }
	    else
	    {
		set_PWM_dutycycle(pi, left_motor, left_current_PWM - 1);
	    }

	    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    	}
    }

    void cleanup()
    {
        pigpio_stop(pi);
    }
};

#endif
