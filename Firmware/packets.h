/* 
 * File:   packets.h
 * Author: DUSAN
 *
 */

#ifndef PACKETS_H
#define	PACKETS_H

// Outgoing messages
#define MSG_IMAGE       'I'
#define MSG_STATUS      'S'
#define MSG_ACK 		'A'
#define MSG_NEIGHBOUR   'N'
#define MSG_GUMSTIX		'G'

// Incoming commands
#define CMD_LEDS        'L'     // No ACK
#define CMD_WHEELS      'W'     // No ACK
#define CMD_POSITION	'P'     // No ACK
#define CMD_CAMERA      'C'     // With ACK
#define CMD_BEEP        'B'     // With ACK
#define CMD_CONFIG      'A'     // With ACK
#define CMD_TEST		'T'     // With ACK
#define CMD_LED_TURRET  'U'     // With ACK
#define CMD_GUMSTIX		'G'		// No ACK
#define CMD_MOVE        'M'     // With ACK

#ifdef	__cplusplus
#	include <cstdint>
#   pragma pack(1)  // GCC, MSVC style

namespace EPuck {

#   ifdef __cplusplus_cli
        // Managed C++
        using namespace System::Runtime::InteropServices;
#       define STRUCT(x) [StructLayout(LayoutKind::Sequential, CharSet = CharSet::Ansi, Pack = 1)] \
                         public value struct x  // Managed C++
#   else  
        // Native C++
#		ifdef EPUCK_API
#			define STRUCT(x)	struct EPUCK_API x  
#		else
#			define STRUCT(x)	struct x
#		endif
#   endif

#else
#	include <stdint.h>
    // ANSI C
#	define STRUCT(x) typedef struct x x; struct __attribute__ ((packed)) x

#endif

STRUCT(Vector3D_t)
{
	int16_t x;
	int16_t y;
	int16_t z;
};

STRUCT(Wheels_t)
{
	int16_t Left;
	int16_t Right;
};


#define TICKS_PER_METER     7763.6557605802602814089640669519
#define TICKS_PER_TRACK     411.47375531075379491467509554845
#define TICKS_PER_TURN      36000
#define TICKS_PER_RADIAN	5729.5779513082320876798154814105

STRUCT(Position_t)
{
	int32_t	x;     // ticks
	int32_t	y;     // ticks
	int16_t psi;   // ticks
};

STRUCT(Microphones_t)
{
	int16_t Right;
	int16_t Left;
	int16_t Rear;
};

// Camera Settings
STRUCT(CameraSettings_t)
{
	int16_t ColorMode;
	int16_t X0;
	int16_t Y0;
	int16_t Width;
	int16_t Height;
	int16_t ZoomX;
	int16_t ZoomY;
};

// Msg Config packet
STRUCT(ConfigPacket_t)
{
	uint8_t MessageID;
	uint8_t Enable;
};

// Proximity 
STRUCT(Proximity_t)
{
	int16_t R_20deg;
	int16_t R_50deg;
	int16_t R_90deg;
	int16_t R_150deg;
	int16_t L_150deg;
	int16_t L_90deg;
	int16_t L_50deg;
	int16_t L_20deg;
};

// Floor sensor
STRUCT(Floor_t)
{
	int16_t Right;
	int16_t Center;
	int16_t Left;
};

// Neighbour EPuck
STRUCT(Neighbour_t)
{
    uint8_t ID;
    uint8_t Direction;
    int16_t A0;
    int16_t A1;
    int16_t A2;
};

// LEDs
typedef uint16_t LEDs_t;

// Status Packet
STRUCT(StatusPacket_t)
{
	Vector3D_t      Accelerometer;
	Wheels_t        Speed;
	Position_t      Position;
	Floor_t         Floor;
	Proximity_t     Proximity;
	Microphones_t   Microphones;
	LEDs_t	        LEDs;
};

// LEDs packet
typedef LEDs_t LedsPacket_t;

// Wheeels packet
typedef Wheels_t WheelsPacket_t;

// Position packet
typedef Position_t PositionPacket_t;

// Beep packet
typedef uint8_t BeepPacket_t;

// Camera setup packet
typedef CameraSettings_t CameraSetupPacket_t;

// Ack packet
typedef uint8_t AckPacket_t;

// Test packet
typedef uint8_t TestPacket_t;

#undef STRUCT

#ifdef	__cplusplus
} // end of namespace
#	pragma pack()
#endif

#endif	/* PACKETS_H */

