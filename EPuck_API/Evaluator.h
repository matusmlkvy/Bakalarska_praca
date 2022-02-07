#ifndef EVALUATOR_H_
#define EVALUATOR_H_

#include "Robot.h"
#include <memory>
#include <atomic>

using namespace EPuck;

// Evaluates reward of the robot
class Evaluator
{
private:
	std::shared_ptr<Robot>	itsRobot;
	Position_t				itsLastPosition;
	double					itsLastSafety;
	double					itsReward;
	bool					itsReady;
	// insert state variables here

	// Internal: updates reward
	static void updateReward(void* sender);

public:
	// Constructs new evaluator
	Evaluator(const std::shared_ptr<Robot>& robot);

	// Returns accumulated reward.
	double reward() const;

	// Weight of distance ran by robot
	double Wd;

	// Weight of safety 
	double Ws;

	// Weight of rotation (penalty).
	double Wr;
};

#endif // STATE_EVALUATOR_H_
