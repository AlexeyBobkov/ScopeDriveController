/*
 * SDC_Motor.h
 *
 * Created: 1/20/2019 6:22:28 PM
 *  Author: Alexey
 */ 


#ifndef SDC_MOTOR_H_
#define SDC_MOTOR_H_

///////////////////////////////////////////////////////////////////////////////////////
// motor class
class SDC_Motor
{
public:
    SDC_Motor(uint8_t dirPin, uint8_t speedPin)
        : dirPin_(dirPin), speedPin_(speedPin) {}

    // call once in setup()
    void Setup()
    {
        pinMode(dirPin_, OUTPUT);
        digitalWrite(dirPin_, LOW);
        pinMode(speedPin_, OUTPUT);
        analogWrite(speedPin_, 0);
    }

    // call periodically in loop()
    // returns true if safe to do some long job
    bool Run()
    {
        return true;
    }

    boolean Move(int speed)
    {
        uint8_t direction;
        if(speed > 0)
            direction = HIGH;
        else
        {
            direction = LOW;
            speed = -speed;
        }
        if(speed > 255)
            speed = 255;
        else if (speed < 0)
            speed = 0;

        digitalWrite(dirPin_, direction);
        analogWrite(speedPin_, speed);
        return true;
    }
    boolean Stop()
    {
        return Move(0);
    }

private:
    uint8_t dirPin_, speedPin_; // pins
};

#endif /* SDC_MOTOR_H_ */