#ifndef EPUCK_UDP_CLIENT_H_
#define EPUCK_UDP_CLIENT_H_

#define ROBOT_UDP_PORT 2534

#include "Interface.h"

namespace EPuck
{
	// Internal: Implementation class for UDP communication.
	class UdpClient : public Interface
	{
	public:
		// Constructs UDP connection with robot
		UdpClient();

		// Destroys and disconnects UDP client.
		virtual ~UdpClient();

		// Opens communication line. If host is NULL, acts as server.
		virtual void open(const char* host);

		// Closes communication line.
		virtual void close();

		// Returns true when communication line is opened.
		virtual bool isOpen() const;

	protected:
		// Asynchronously writes to comunication interface.
		virtual void asyncWrite(uint8_t* bytes, size_t count);

		// Asynchronously reads from comunication interface.
		virtual void asyncRead(uint8_t* dst, size_t capacity);

	protected:
		// internal UDP socket
		boost::asio::ip::udp::socket itsSocket;
		boost::asio::ip::udp::endpoint itsRemoteEndpoint;
	};

} // end of namespace



#endif // EPUCK_UDP_CLIENT_H_