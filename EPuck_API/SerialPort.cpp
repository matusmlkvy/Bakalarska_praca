#include "SerialPort.h"	

using namespace boost::asio;

namespace EPuck
{
	SerialPort::SerialPort() :
		Interface(),
		itsPort(itsService),
		itsBaudrate(115200) {}

	SerialPort::~SerialPort()
	{
		close();
	}

	void SerialPort::open(const char* port)
	{
		using boost::asio::serial_port;

		itsPort.open(port);

		itsPort.set_option(serial_port::baud_rate(itsBaudrate));
		itsPort.set_option(serial_port::parity(serial_port::parity::none));
		itsPort.set_option(serial_port::stop_bits(serial_port::stop_bits::one));

		itsPort.async_read_some(buffer(itsCache, Interface::CACHE_SIZE), Interface::RxHandler(this));
	}

	void SerialPort::close()
	{
		if (itsPort.is_open())
		{
			itsPort.cancel();
			itsPort.close();
		}
	}

	bool SerialPort::isOpen() const { return itsPort.is_open(); }

	void SerialPort::asyncWrite(uint8_t* bytes, size_t count)
	{
		itsPort.async_write_some(buffer(bytes, count), TxHandler(bytes));
	}

	void SerialPort::asyncRead(uint8_t* dst, size_t capacity)
	{
		itsPort.async_read_some(buffer(dst, capacity), RxHandler(this));
	}

	int SerialPort::baudrate() const {	return itsBaudrate; }

	void SerialPort::setBaudrate(int baud) { itsBaudrate = baud; }

}// end of namespace