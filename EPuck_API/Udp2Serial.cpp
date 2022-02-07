#include "Udp2Serial.h"
#include "SerialPort.h"
#include "UdpClient.h"

namespace EPuck
{
	Udp2Serial::Udp2Serial(const char* serialPort):
		itsSerialPort(new SerialPort),
		itsUdpClient(new UdpClient)
	{
		itsSerialPort->setPacketReceivedHandler(Udp2Serial::serialPacketReceived, this);
		itsUdpClient->setPacketReceivedHandler(Udp2Serial::udpPacketReceived, this);

		// different baudrate for internal Gumstix <-> epuck serial communication
		itsSerialPort->setBaudrate(230400); 

		itsSerialPort->open(serialPort);
		itsUdpClient->open(NULL);
	}


	Udp2Serial::~Udp2Serial()
	{
		// nothing to do
	}

	void Udp2Serial::serialPacketReceived(char msgID, uint16_t msgLen, const uint8_t* data, void* arg)
	{
		Udp2Serial* obj = (Udp2Serial*)arg;
		obj->itsUdpClient->writePacket(msgID, data, msgLen);
	}

	void Udp2Serial::udpPacketReceived(char msgID, uint16_t msgLen, const uint8_t* data, void* arg)
	{
		Udp2Serial* obj = (Udp2Serial*)arg;
		obj->itsSerialPort->writePacket(msgID, data, msgLen);
	}
}

