#ifndef MultiStepper_h
#define MultiStepper_h

#include <stdlib.h>
#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#include <wiring.h>
#endif

#define MULTISTEPPER_MAX_STEPPERS 10

class AccelStepper;

/////////////////////////////////////////////////////////////////////
class MultiStepper
{
public:
    /// Constructor
    MultiStepper();
    boolean addStepper(AccelStepper& stepper);
    void moveTo(long absolute[]);
    boolean run(String mode);
    void addToPlan(float speeds[], long absolutes[]);
    void recomputePlan();
    boolean executeNextBlock();
    int queueSize();
    int queueCapacity();


    
private:
    AccelStepper* _steppers[MULTISTEPPER_MAX_STEPPERS];
    uint8_t       _num_steppers;

};
#endif
