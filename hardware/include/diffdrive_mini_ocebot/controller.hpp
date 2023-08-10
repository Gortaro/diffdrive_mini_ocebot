#ifndef DIFFDRIVE_MINI_OCEBOT_CONTROLLER_HPP
#define DIFFDRIVE_MINI_OCEBOT_CONTROLLER_HPP

#include <pigpiod_if2.h>
#include <cmath>
#include <chrono>
#include <thread>

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

	    set_mode(pi, left_direction, PI_OUTPUT);
	    set_mode(pi, right_direction, PI_OUTPUT);
    }

    void read_encoder_values(int &left_enc, int &right_enc)
    {
        //TODO: Figure out how the encoders are coming in
    }

    void set_motor_values(int left, int right)
    {
        int left_direction = (left > 0) ? 1 : 0;
        int right_direction = (right > 0) ? 1 : 0;

        int left_PWM = std::min(abs(left), 255);
        int right_PWM = std::min(abs(right), 255);

        int left_current_PWM = get_PWM_dutycycle(pi, left_motor);
        int right_current_PWM = get_PWM_dutycycle(pi, right_motor);

        gpio_write(pi, this->left_direction, left_direction);
        gpio_write(pi, this->right_direction, right_direction);

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

        while(right_current_PWM != right_PWM)
        {
            right_current_PWM = get_PWM_dutycycle(pi, right_motor);
            
            if(right_current_PWM < right_PWM)
            {
                set_PWM_dutycycle(pi, right_motor, right_current_PWM + 1);
            }
            else
            {
                set_PWM_dutycycle(pi, right_motor, right_current_PWM - 1);
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
