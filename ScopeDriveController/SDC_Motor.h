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
    SDC_Motor(double rpm, uint8_t dirPin, uint8_t speedPin)
        : max_speed_(rpm*2*PI/60), dirPin_(dirPin), speedPin_(speedPin), voltage_(12), running_(false) {}

    // call once in setup()
    void Setup();

    // call periodically in loop()
    // returns true if safe to do some long job
    bool Run();

    bool Start (long uspeed,        // start speed, in encoder units/day
                long *upos,         // starting position, in encoder units
                long *ts);          // starting timestamp, in ms (returned by millis())

    bool GetPos(long *upos, long *ts);
    bool SetNextPos(long upos, long ts);
    void Stop();

    // obsolete
    bool Move(int speed);

private:
    double max_speed_;  // rad/sec
    uint8_t dirPin_, speedPin_; // pins
    double voltage_;

    bool running_;
    long upos_;
    long ts_;
    double speed_;      // rad/sec
};

#endif /* SDC_MOTOR_H_ */