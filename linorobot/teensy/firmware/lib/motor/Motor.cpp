#include "Motor.h"

Controller::Controller(driver motor_driver, int pwm_pin, int motor_pinA, int motor_pinB, int en_pin):
    motor_driver_(motor_driver),
    pwm_pin_(pwm_pin),
    motor_pinA_(motor_pinA),
    motor_pinB_(motor_pinB),
    en_pin_(en_pin)
{
    switch (motor_driver)
    {
        case L298:
            pinMode(pwm_pin_, OUTPUT);
            pinMode(motor_pinA_, OUTPUT);
            pinMode(motor_pinB_, OUTPUT);
            pinMode(en_pin_, OUTPUT);
            //ensure that the motor is in neutral state during bootup
            digitalWrite(en_pin_, LOW);
            analogWrite(pwm_pin_, abs(0));

            break;

    }
}

void Controller::spin(int pwm)
{
    switch (motor_driver_)
    {
        case L298:
            if(pwm > 0)
            {
                digitalWrite(en_pin_, HIGH);
                digitalWrite(motor_pinA_, HIGH);
                digitalWrite(motor_pinB_, LOW);
            }
            else if(pwm < 0)
            {
                digitalWrite(en_pin_, HIGH);                
                digitalWrite(motor_pinA_, LOW);
                digitalWrite(motor_pinB_, HIGH);
            }
            
           /* else
            {
                digitalWrite(en_pin_, LOW);
                
                digitalWrite(motor_pinA_, LOW);
                digitalWrite(motor_pinB_, LOW);
                analogWrite(pwm_pin_, 0);
            }
            */
            
            analogWrite(pwm_pin_, abs(pwm));
            
            break;

    }
}
