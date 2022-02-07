#include "Interface.h"
#include "Robot.h"
#include <iostream>

namespace EPuck
{
	Interface::Interface() :
		itsEnable(true),
		itsService(),
		itsThread(Interface::runService, &itsService),
		itsPacketReceivedFcn(NULL),
		itsPacketReceivedArg(NULL)
	{
		itsParseStatus = 0;
		itsDataCount = 0;
	}


	Interface::~Interface()
	{
		itsEnable = false;
		itsService.stop();

		if (itsThread.joinable())
			itsThread.join();
	}

	void Interface::runService(boost::asio::io_service* service)
	{
		// Keep service occupied
		boost::asio::io_service::work work(*service);
		try { service->run(); }
		catch (...) {}
	}

	void Interface::setPacketReceivedHandler(PacketReceivedHandler_t handler, void* arg)
	{
		itsPacketReceivedFcn = handler;
		itsPacketReceivedArg = arg;
	}

	void Interface::newParse(std::size_t bytes_received)
	{
		int state = itsParseStatus;

		for (int i = 0; i < bytes_received; i++)
		{
			uint8_t c = itsCache[i];
			//std::cout << (char)c; // DEBUG

			switch (state)
			{
			case 0:
				// Waiting for SYNC
				if (c == SYNC_BYTE)
					state = 1;
				break;

			case 1:
				// Waiting for START
				if (c == START_BYTE)
					state = 2;
				else
					state = 0;
				break;

			case 2:
				// ID
				this->itsMsgID = c;
				state = 3;
				break;

			case 3:
				// LSB(length)
				this->itsMsgLength = (c & 0xFF);
				state = 4;
				break;

			case 4:
				// MSB(length)
				this->itsMsgLength |= (c << 8);

				if (this->itsMsgLength > 0 && this->itsMsgLength <= Robot::IMAGE_SIZE_MAX)
				{
					this->itsDataCount = 0;
					state = 5;
				}
				else
				{
					// Too large, ignore
					state = 0;
				}
				break;

			case 5:
				// Load data to buffer
				this->itsData[this->itsDataCount++] = c;
				if (this->itsDataCount == this->itsMsgLength)
				{
					if (itsPacketReceivedFcn)
						itsPacketReceivedFcn(itsMsgID, itsMsgLength, itsData, itsPacketReceivedArg);
					state = 0;
				}
				break;

			default:
				state = 0;
				break;
			}
		}

		// Save new state
		this->itsParseStatus = state;

		// Read asynchronously more bytes.
		if (this->itsEnable)
			this->asyncRead(this->itsCache, CACHE_SIZE);
	}

	void Interface::writePacket(uint8_t id, const void* data, size_t len)
	{
		// Create packet
		uint8_t* packet = new uint8_t[HEADER_SIZE + len];
		packet[0] = SYNC_BYTE;
		packet[1] = START_BYTE;
		packet[2] = id;
		packet[3] = (uint8_t)(len & 0xFF);
		packet[4] = (uint8_t)(len >> 8);
		memcpy(packet + HEADER_SIZE, data, len);

		this->asyncWrite(packet, HEADER_SIZE + len);
	}

}// end of namespace