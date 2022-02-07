#ifndef EPUCK_SERIAL_PORT_H_
#define EPUCK_SERIAL_PORT_H_

#include "Interface.h"

namespace EPuck
{
	// Internal: Implementation class for serial communication.
	class SerialPort : public Interface
	{
	public:
		// Constructs serial port connection with robot
		SerialPort();

		// Destroys and disconnects serial port.
		virtual ~SerialPort();

		// Opens communication line.
		virtual void open(const char* port);

		// Closes communication line.
		virtual void close();

		// Returns true when communication line is opened.
		virtual bool isOpen() const;

		// Returns current baudrate (default is 115200).
		int baudrate() const;

		// Sets baudrate. Call this before open()
		void setBaudrate(int baud);

	protected:
		// Asynchronously writes to comunication interface.
		virtual void asyncWrite(uint8_t* bytes, size_t count);

		// Asynchronously reads from comunication interface.
		virtual void asyncRead(uint8_t* dst, size_t capacity);

	protected:
		// internal serial port
		boost::asio::serial_port	itsPort;
		int							itsBaudrate;
	};

} // end of namespace



#endif // EPUCK_SERIAL_PORT_H_