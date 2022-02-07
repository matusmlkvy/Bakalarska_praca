#ifndef ROBOT_INTERFACE_H_
#define ROBOT_INTERFACE_H_

#include <ctime>
#include <thread>
#include <mutex>
#include <atomic>
#include <cstdint>
#include <stdexcept>
#include <array>
#include <memory>

#pragma warning(push, 0)
#include <boost/asio.hpp>
#pragma warning(pop)

namespace EPuck 
{
	// Internal: Interface class for robot's internal communication.
	class Interface
	{
	public:
		// Constructor
		Interface();

		// Destructor
		virtual ~Interface();

		// Writes packet asynchronously with given ID to the communication line.
		// Does not wait for the ACK message.
		void writePacket(uint8_t id, const void* data, size_t len);

		// Opens communication line. MUST BE IMPLEMENTED by implementation class.
		virtual void open(const char* host) = 0;

		// Closes communication line. MUST BE IMPLEMENTED by implementation class.
		virtual void close() = 0;

		// Returns true when communication line is opened. MUST BE IMPLEMENTED by implementation class.
		virtual bool isOpen() const = 0;

		// PacketReceived handler type.
		typedef void(*PacketReceivedHandler_t)(char msgID, uint16_t msgLen, const uint8_t* data, void* arg);

		// Sets handler for PacketReceived event.
		void setPacketReceivedHandler(PacketReceivedHandler_t handler, void* arg = NULL);

	protected:
		// Internal: Parses incoming stream stored in cache
		void newParse(std::size_t bytes_received);

		// Internal: Keeps internal io_service busy.
		static void runService(boost::asio::io_service* service);

		// Asynchronously writes to comunication interface (serial / tcp / udp). MUST BE IMPLEMENTED by implementation class.
		virtual void asyncWrite(uint8_t* bytes, size_t count) = 0;

		// Asynchronously reads from comunication interface (serial / tcp / udp). MUST BE IMPLEMENTED by implementation class.
		virtual void asyncRead(uint8_t* dst, size_t capacity) = 0;

	protected:
		static const char SYNC_BYTE = '~';
		static const char START_BYTE = '$';

		// Packet strucutre:
		// SYNC+START(2B)|ID(1B)|LEN(2B)|DATA(LEN)
		static const int HEADER_SIZE = 5;

		// Size of the internal cache.
		static const int CACHE_SIZE = 100;

		// Maximal size of the packet including header.
		static const int MESSAGE_SIZE_MAX = 4096;

		boost::asio::io_service		itsService;
		std::thread					itsThread;
		std::atomic<bool>			itsEnable;

		uint8_t						itsCache[CACHE_SIZE];
		int							itsParseStatus;
		char						itsMsgID;
		uint16_t					itsMsgLength;

		uint8_t						itsData[MESSAGE_SIZE_MAX];
		uint16_t					itsDataCount;

		PacketReceivedHandler_t		itsPacketReceivedFcn;
		void*						itsPacketReceivedArg;
		
		// Handler (function object) class for receivcer
		class RxHandler
		{
		private:
			Interface* itsRobot;
		public:
			RxHandler(Interface* robot) : itsRobot(robot) {}
			void operator()(const boost::system::error_code& error, std::size_t bytes_transferred) { this->itsRobot->newParse(bytes_transferred); }
		};

		// Handler (function object) class for transmitter
		class TxHandler
		{
		private:
			std::shared_ptr<uint8_t> itsBuffer;
			static void Del(uint8_t* ptr) { if (ptr) delete[] ptr; }

		public:
			TxHandler(uint8_t* data) : itsBuffer(data, Del) {}
			void operator()(const boost::system::error_code& error, std::size_t bytes_transferred) {}
		};
	};
} // end of namespace


#endif // ROBOT_INTERFACE_H_
