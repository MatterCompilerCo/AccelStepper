//pseudocode
// Have a circular buffer of steps that consists of 
// [1000,1000,1000]
// [100,1000,1000]
// and so on

// Starting from the back, plan out a trajectory such that:

// 1) Once we hit the last point, velocity is 0
// 2) Every point in the motion array is hit at the same time
// 3) Velocity at the end of one block and start of next block are the same
// 4) we obey max velocity constraints for each one of the steppers


#include "MultiStepper.h"
#include "AccelStepper.h"
#include <circularqueue.h>

struct planBlock{
	float *speeds;
	long *positions;
};

const int BUFFER_SIZE= 20;
CircularQueue<planBlock*,BUFFER_SIZE> buffer;

void printFullPlan() {
	int currentSize = buffer.count();

	for(uint8_t i = 0; i < BUFFER_SIZE; i++) {
		if(i>=currentSize) continue;
	
		planBlock* block = buffer[i];
		Serial.println("====");
		Serial.println(i);
		for(uint8_t j = 0; j < 5; j++) {
			Serial.println("$$$$$");
			Serial.print(*(block->speeds+j));
			Serial.print(" ");
			Serial.println(*(block->positions+j));
		}

	}
}

MultiStepper::MultiStepper()
    : _num_steppers(0)
{
}

boolean MultiStepper::addStepper(AccelStepper& stepper)
{
    if (_num_steppers >= MULTISTEPPER_MAX_STEPPERS)
	return false; // No room for more
    _steppers[_num_steppers++] = &stepper;
    return true;
}

void MultiStepper::addToPlan(float *speeds, long *absolutes) {
    planBlock *block = new planBlock({speeds,absolutes});
    buffer.enqueue(block);
    recomputePlan();

}

//we're always either accelerating, decelerating, or moving at the constant max velocity
void MultiStepper::recomputePlan() {
	int lastIdx = buffer.count()-1;
	if(lastIdx < 0) return;
    
    planBlock* currBlock = buffer[lastIdx];
	long *prevPos;
	long *currPos = currBlock->positions;
	float *currSpeeds = currBlock->speeds;

	if(lastIdx>0) {
		planBlock* prevBlock = buffer[lastIdx-1];
	 	prevPos = prevBlock->positions;

	} else {
		long *zeroArr = new long[5];
		for(int i =0; i < 5; i++) zeroArr[i]=0;
		prevPos = zeroArr;
	}
    // First find the stepper that will take the longest time to move
    float longestTime = 0.0;
	// Serial.println("@@@@@@");
	// for(uint8_t j = 0; j < 5; j++) {
	// 	float time = abs(*(currPos+j)-*(prevPos+j)) / *(currSpeeds+j);
	// 	Serial.print(*(currSpeeds+j));
	// 	Serial.print(" ");
	// 	Serial.print(*(currPos+j));
	// 	Serial.print(" ");
	// 	Serial.println(*(prevPos+j));
	// 	if(time > longestTime) longestTime = time;
	// }


	if(longestTime > 0.0){
		for(uint8_t j = 0; j < 5; j++) {
			float dist = *(currPos+j)-*(prevPos+j);
			float newSpeed = dist / longestTime;
			// Serial.print(lastIdx);
			// Serial.print(" ");
			// Serial.print(j);
			// Serial.print(" ");
			// Serial.println(newSpeed);
			// *(currSpeeds+j) = newSpeed;
		}
	}
	
	Serial.println("PLAN RECOMPUTE SUCCESSFUL.");
	//printFullPlan();
}

boolean MultiStepper::executeNextBlock(){
	Serial.println("EXECUTING NEXT PLAN BLOCK.");
	planBlock* block = buffer.dequeue();
	for(uint8_t i = 0; i < 5; i++)
    _steppers[i]->runTrajPoint(block->speeds[i],block->positions[i]);
	delete[] block->positions;
	delete[] block->speeds;
	delete[] block;
}




boolean MultiStepper::run(String mode) {
	bool nextCommand = true;
	for(uint8_t i = 0; i<5; i++) {
		if ( _steppers[i]->distanceToGo() != 0){
			nextCommand = false;
			_steppers[i]->runSpeed();
		}
	}
	if(nextCommand && mode == "run_program") {
		if(!buffer.isEmpty()) executeNextBlock();
	}
}

int MultiStepper::queueSize(){
	return buffer.count();
}

int MultiStepper::queueCapacity(){
	return BUFFER_SIZE;
}
