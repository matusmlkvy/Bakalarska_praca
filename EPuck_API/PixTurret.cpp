#include "PixTurret.h"
#include "Robot.h"

namespace EPuck 
{
	PixTurret::PixTurret(Robot& parent) :
		itsPixels(9),
		itsIR(8),
		itsRobot(&parent) {}

	Pixel_t& PixTurret::pixel(int i) 
	{ 
		return this->itsPixels[i];  
	}

	uint8_t& PixTurret::IR(int i)
	{
		return this->itsIR[i];
	}

	void PixTurret::update()
	{
		uint8_t data[9 * 3 + 8];

		// write pixel data
		for (int i = 0; i<9; i++)
		{
			Pixel_t pix = this->itsPixels[i];

			data[i + 0] = pix.R >> 1;
			data[i + 9] = pix.G >> 1;
			data[i + 18] = pix.B >> 1;
		}

		for (int i = 0; i < 8; i++)
			data[i + 27] = this->itsIR[i];

		itsRobot->writeSync(CMD_LED_TURRET, data, 35);
	}
}



