#ifndef PIX_TURRET_H
#define	PIX_TURRET_H

#include <cstdint>
#include <vector>
#include "defs.h"

// Disable enforcing DLL interface for private members
#pragma warning(push)
#pragma warning(disable: 4251)

namespace EPuck {

	struct EPUCK_API Pixel_t
	{
		uint8_t R;
		uint8_t G;
		uint8_t B;

		Pixel_t()
		 : R(0), G(0), B(0) {}

		Pixel_t(uint8_t red, uint8_t green, uint8_t blue)
		 : R(red), G(green), B(blue) {}
	};

	// Declare forward
	class Robot;

	class EPUCK_API PixTurret
	{
	private:
		std::vector<Pixel_t> itsPixels;
		std::vector<uint8_t> itsIR;
		EPuck::Robot*		 itsRobot;

	public:
		// Creates new pixel turret for given robot
		PixTurret(Robot& parent);

		// Returns selected pixel (i = 0 to 8)
		Pixel_t& pixel(int i);

		// Returns selected IR (i = 0 to 7)
		uint8_t& IR(int i);

		// Updates pixel turret (sends data to physical robot).
		// Call manually after changes are done.
		void update();
	};

} // end of namespace

#pragma warning(pop)

#endif