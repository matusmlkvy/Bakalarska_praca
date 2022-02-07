#include "Evaluator.h"
#include <cmath>

using namespace EPuck;

Evaluator::Evaluator(const std::shared_ptr<Robot>& robot) : 
	itsRobot(robot), itsReward(0), Wd(2), Ws(5), Wr(6), itsReady(false) 
{
	// hook to robot event
	itsRobot->setSensorDataReceivedHandler(Evaluator::updateReward, this);
}

void Evaluator::updateReward(void* sender)
{
	Evaluator* obj = (Evaluator*)sender;

	Position_t pos = obj->itsRobot->position();
	Proximity_t prox = obj->itsRobot->proximity();

	// Compute danger of obstacle in the front of the epuck (maximal from frontal sensors)
	int16_t danger = 0;
	if (prox.L_50deg > danger) danger = prox.L_50deg;
	if (prox.L_20deg > danger) danger = prox.L_20deg;
	if (prox.R_20deg > danger) danger = prox.R_20deg;
	if (prox.R_50deg > danger) danger = prox.R_50deg;

	const int16_t DANGER_LIMIT = 300; // Danger which stops the robot by firmware (failsafe)

	double safety = 1 - (double)danger / (double)DANGER_LIMIT;


	if (!obj->itsReady)
	{
		// initial sample
		obj->itsLastPosition = pos;
		obj->itsLastSafety = safety;
		obj->itsReward = 0;
		obj->itsReady = true;
	}

	double dx = (double)(pos.x - obj->itsLastPosition.x);
	double dy = (double)(pos.y - obj->itsLastPosition.y);
	double dpsi = (double)(pos.psi - obj->itsLastPosition.psi);
	dpsi -= 36000 * std::round(dpsi / 36000);

	double ds = std::sqrt(dx * dx + dy * dy);

	obj->itsReward = obj->Wd * ds / 1000 + obj->Ws * (safety - obj->itsLastSafety) - obj->Wr * abs(dpsi) / 36000;
}
