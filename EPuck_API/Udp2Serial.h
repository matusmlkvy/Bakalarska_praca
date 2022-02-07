#ifndef EPUCK_UDP2SERIAL_H_
#define EPUCK_UDP2SERIAL_H_

#include "defs.h"
#include <memory>

// Disable enforcing DLL interface for private members
#pragma warning(push)
#pragma warning(disable: 4251)

namespace EPuck
{
	// Forward declarations for DLL export.
	class SerialPort;
	class UdpClient;

	/* Implements bridge between UDP and serial port. */
	class EPUCK_API Udp2Serial
	{
	public:
		// Constructs bridge to given serial port.
		Udp2Serial(const char* serialPort);

		// Terminates connection and destroys bridge.
		~Udp2Serial();

	private:
		// Handles packet received from serial port.
		static void serialPacketReceived(char msgID, uint16_t msgLen, const uint8_t* data, void* arg);

		// Handles packet received from UDP client.
		static void udpPacketReceived(char msgID, uint16_t msgLen, const uint8_t* data, void* arg);

	private:
		std::unique_ptr<SerialPort>	itsSerialPort;
		std::unique_ptr<UdpClient>	itsUdpClient;
	};

} // end of namespace

#pragma warning(pop)

#endif // EPUCK_UDP2SERIAL_H_
