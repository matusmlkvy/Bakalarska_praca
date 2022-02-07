#ifndef PIXEL_H_
#define PIXEL_H_

#include <cstdint>
#include "defs.h"

namespace EPuck
{

#pragma pack(1)
	struct EPUCK_API Pixel565
	{
		uint16_t R : 5;
		uint16_t G : 6;
		uint16_t B : 5;
	};
#pragma pack()

}

#endif
