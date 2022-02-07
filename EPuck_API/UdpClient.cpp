#include "UdpClient.h"	

using namespace boost::asio;
using namespace boost::asio::ip;

namespace EPuck
{
	UdpClient::UdpClient() :
		Interface(),
		itsRemoteEndpoint(),
		itsSocket(itsService, udp::endpoint(udp::v4(), ROBOT_UDP_PORT)) {}


	UdpClient::~UdpClient()
	{
		close();
	}

	void UdpClient::open(const char* host)
	{
		if (host)
		{
			udp::resolver resolver(itsService);
			udp::resolver::query q(udp::v4(), host, std::to_string(ROBOT_UDP_PORT));
			itsRemoteEndpoint = *resolver.resolve(q);
		}
		itsSocket.async_receive_from(buffer(itsCache, Interface::CACHE_SIZE), itsRemoteEndpoint, RxHandler(this));
	}

	void UdpClient::close()
	{
		itsSocket.close();
	}

	bool UdpClient::isOpen() const { return itsSocket.is_open(); }

	void UdpClient::asyncWrite(uint8_t* bytes, size_t count)
	{
		itsSocket.async_send_to(buffer(bytes, count), itsRemoteEndpoint, TxHandler(bytes));
	}

	void UdpClient::asyncRead(uint8_t* dst, size_t capacity)
	{
		itsSocket.async_receive_from(buffer(dst, capacity), itsRemoteEndpoint, RxHandler(this));
	}

}// end of namespace