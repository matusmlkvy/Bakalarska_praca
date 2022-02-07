#include "Interface.h"
#include "Robot.h"
#include "SerialPort.h"
#include "UdpClient.h"

#include <cmath>

#define DANGER_LIMIT 1000

namespace EPuck
{
	Robot::Robot() :
		itsInterface(),
		itsMutex(),
		itsNeighbours(10),
		itsImage(),
		itsPixTurret(*this),
		itsThread()
	{
		itsCameraSettings.ColorMode = Robot::RGB_565_MODE;
		itsCameraSettings.Width = 40;
		itsCameraSettings.Height = 40;
		itsCameraSettings.X0 = (Robot::IMAGE_WIDTH_MAX - 40 * 8) / 2;
		itsCameraSettings.Y0 = (Robot::IMAGE_HEIGHT_MAX - 40 * 8) / 2;
		itsCameraSettings.ZoomX = 8;
		itsCameraSettings.ZoomY = 8;

		itsSensorsEnabled = false;
		itsCameraEnabled = false;
		itsNeighbourDetectionEnabled = false;
		itsSimulationEnabled = false;

		itsAckReceived = false;

		itsGumstixResponseReceivedHandler = NULL;
		itsGumstixResponseReceivedArg = NULL;

		itsStatus.Accelerometer.x = 0;
		itsStatus.Accelerometer.y = 0;
		itsStatus.Accelerometer.z = 0;

		itsStatus.Floor.Center = 0;
		itsStatus.Floor.Left = 0;
		itsStatus.Floor.Right = 0;

		itsStatus.Position.x = 0;
		itsStatus.Position.y = 0;
		itsStatus.Position.psi = 0;

		itsStatus.Speed.Left = 0;
		itsStatus.Speed.Right = 0;
		
		itsStatus.Proximity.L_150deg = 0;
		itsStatus.Proximity.L_90deg = 0;
		itsStatus.Proximity.L_50deg = 0;
		itsStatus.Proximity.L_20deg = 0;
		itsStatus.Proximity.R_20deg = 0;
		itsStatus.Proximity.R_50deg = 0;
		itsStatus.Proximity.R_90deg = 0;
		itsStatus.Proximity.R_150deg = 0;
	}

	Robot::Robot(const char* host, ConnectionType comType)
		: Robot()
	{
		open(host, comType);
	}

	Robot::~Robot()
	{ /* nothing to do. */
	}

	void Robot::open(const char* host, ConnectionType comType)
	{
		if (itsSimulationEnabled)
			return; // nothing to do

		// Create new base according to the type.
		switch (comType)
		{
		case Robot::Serial:
			itsInterface = std::move(std::unique_ptr<Interface>(new SerialPort));
			break;

		case Robot::UDP:
			itsInterface = std::move(std::unique_ptr<Interface>(new UdpClient));
			break;

		case Robot::TCP:
			// TODO
			break;

		default:
			throw std::invalid_argument("Robot: Unknown connection type.");
		}

		// Hook event handler and open
		itsInterface->setPacketReceivedHandler(Robot::applyMsg, this);
		itsInterface->open(host);
	}

	void Robot::close()
	{
		if (itsSimulationEnabled)
			return; // nothing to do

		if (itsInterface)
			itsInterface->close();
	}

	bool Robot::testConnection()
	{
		if (itsSimulationEnabled)
			return true;

		if (!itsInterface || !itsInterface->isOpen())
			return false;

		TestPacket_t dummy = 0xAA;
		try
		{
			writeSync(CMD_TEST, &dummy, sizeof(TestPacket_t));
			return true;
		}
		catch (Robot::NoResponseException ex)
		{
			return false;
		}
	}

	void Robot::writeSync(uint8_t id, const void* data, size_t len)
	{
		if (itsSimulationEnabled)
			return;

		if (!itsInterface)
			throw std::runtime_error("Robot::writeSync: interface is closed. ");

		itsMutex.lock();

		// Clear flag
		itsAckReceived = false;

		for (int tries = 0; tries < 10; tries++)
		{
			// Write
			writeAsync(id, data, len);

			// Wait for response
			for (int i = 0; i < 100; i++)
			{
				if (itsAckReceived)
				{
					itsMutex.unlock();
					return;
				}

				std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
		}

		itsMutex.unlock();
		throw Robot::NoResponseException();
	}

	void Robot::writeAsync(uint8_t id, const void* data, size_t len)
	{
		if (itsSimulationEnabled)
			return;

		if (!itsInterface)
			throw std::runtime_error("Robot::writeSync: interface is closed. ");

		itsInterface->writePacket(id, data, len);
	}

	void Robot::applyMsg(char msgID, uint16_t msgLen, const uint8_t* data, void* arg)
	{
		Robot* obj = (Robot*)arg;
		switch (msgID)
		{
		case MSG_STATUS:
			if (msgLen == sizeof(StatusPacket_t))
			{
				obj->itsStatus = *((StatusPacket_t*)data);
				obj->itsSensorDataReceived.Fire();
			}
			break;

		case MSG_IMAGE:
			if (msgLen == (int16_t)(obj->itsCameraSettings.Width * obj->itsCameraSettings.Height * sizeof(Pixel565)))
			{
				obj->itsImage.Fill((Pixel565*)data, obj->itsCameraSettings.Width, obj->itsCameraSettings.Height);
				obj->itsImageReceived.Fire();
			}
			break;

		case MSG_NEIGHBOUR:
			if (msgLen % sizeof(Neighbour_t) == 0)
			{
				int count = msgLen / sizeof(Neighbour_t);
				Neighbour_t* src = (Neighbour_t*)data;
				obj->itsNeighbours.assign(src, src + count);

				obj->itsNeighbourDetected.Fire();
			}
			break;

		case MSG_ACK:
			obj->itsAckReceived = true;
			break;

		case MSG_GUMSTIX:
			if (obj->itsGumstixResponseReceivedHandler)
				obj->itsGumstixResponseReceivedHandler(obj->itsGumstixResponseReceivedArg, data, msgLen);
			break;

		default:
			// do nothing
			break;
		}
	}

	bool Robot::isOpen() const { return itsSimulationEnabled || (itsInterface && itsInterface->isOpen()); }

	const Wheels_t& Robot::wheels() const { return itsStatus.Speed; }

	const Position_t& Robot::position() const { return itsStatus.Position; }

	const Vector3D_t& Robot::accel() const { return itsStatus.Accelerometer; }

	const Microphones_t& Robot::sound() const { return itsStatus.Microphones; }

	const Proximity_t& Robot::proximity() const { return itsStatus.Proximity; }

	const Floor_t& Robot::floorSensor() const { return itsStatus.Floor; }

	bool Robot::LED(int i) const
	{
		if (i < 0 || i > 9)
			throw std::out_of_range("LED: index out of bounds.");

		return ((itsStatus.LEDs & (1 << i)) != 0);
	}

	uint16_t Robot::LED() const { return itsStatus.LEDs; }

	bool Robot::sensorsEnabled() const { return itsSensorsEnabled; }

	bool Robot::cameraEnabled() const { return itsCameraEnabled; }

	bool Robot::neighbourDetectionEnabled() const { return itsNeighbourDetectionEnabled; }

	bool Robot::simulationEnabled() const { return itsSimulationEnabled; }

	const CameraSettings_t& Robot::cameraSettings() const { return itsCameraSettings; }

	Image565& Robot::lastImage() { return itsImage; }

	PixTurret& Robot::pixTurret() { return itsPixTurret; }

	Neighbour_t& Robot::neigbour(int k)
	{
		if (k < 0 || k >= itsNeighbours.size())
			throw std::out_of_range("Neighbours: index out of range.");

		return itsNeighbours[k];
	}

	size_t Robot::neighboursCount() const { return itsNeighbours.size(); }

	void Robot::setWheels(const Wheels_t& wheels)
	{
		if (itsSimulationEnabled)
			itsStatus.Speed = wheels;
		else
			writeAsync(CMD_WHEELS, &wheels, sizeof(WheelsPacket_t));
	}

	void Robot::setWheels(int16_t left, int16_t right)
	{
		Wheels_t wh;
		wh.Left = left;
		wh.Right = right;

		if (itsSimulationEnabled)
			itsStatus.Speed = wh;
		else
			writeAsync(CMD_WHEELS, &wh, sizeof(WheelsPacket_t));
	}

	void Robot::setActions(const std::vector<Wheels_t>& actions)
	{
		if (itsSimulationEnabled)
			itsActions = actions;
		else
			writeSync(CMD_MOVE, actions.data(), actions.size() * sizeof(WheelsPacket_t));
	}

	void Robot::setPosition(const  Position_t& position)
	{
		if (itsSimulationEnabled)
			itsStatus.Position = position;
		else
			writeAsync(CMD_POSITION, &position, sizeof(PositionPacket_t));
	}

	void Robot::setCameraSettings(const CameraSettings_t& settings)
	{
		if (settings.ColorMode != Robot::RGB_565_MODE)
			throw std::invalid_argument("Robot: only RGB 565 mode is supported.");

		if (settings.Width * settings.Height * sizeof(Pixel565) > IMAGE_SIZE_MAX)
			throw std::overflow_error("Robot: image too large.");

		if (settings.Width * settings.ZoomX > IMAGE_WIDTH_MAX)
			throw std::overflow_error("Robot: image to wide.");

		if (settings.Height * settings.ZoomY > IMAGE_HEIGHT_MAX)
			throw std::overflow_error("Robot: image to high.");

		if (settings.X0 + settings.Width > IMAGE_WIDTH_MAX || settings.Y0 + settings.Height > IMAGE_HEIGHT_MAX)
			throw std::out_of_range("Robot: image start position out of range.");

		writeSync(CMD_CAMERA, &settings, sizeof(CameraSetupPacket_t));
		itsCameraSettings = settings;
	}

	void Robot::enableSensors()
	{
		if (itsSimulationEnabled)
			return;

		ConfigPacket_t packet;
		packet.MessageID = MSG_STATUS;
		packet.Enable = 1;

		writeSync(CMD_CONFIG, &packet, sizeof(ConfigPacket_t));
		itsSensorsEnabled = true;
	}

	void Robot::disableSensors()
	{
		if (itsSimulationEnabled)
			return;

		ConfigPacket_t packet;
		packet.MessageID = MSG_STATUS;
		packet.Enable = 0;

		writeSync(CMD_CONFIG, &packet, sizeof(ConfigPacket_t));
		itsSensorsEnabled = false;
	}

	void Robot::enableCamera()
	{
		if (itsSimulationEnabled)
			return;

		ConfigPacket_t packet;
		packet.MessageID = MSG_IMAGE;
		packet.Enable = 1;

		writeSync(CMD_CONFIG, &packet, sizeof(ConfigPacket_t));
		itsCameraEnabled = true;
	}

	void Robot::disableCamera()
	{
		if (itsSimulationEnabled)
			return;

		ConfigPacket_t packet;
		packet.MessageID = MSG_IMAGE;
		packet.Enable = 0;

		writeSync(CMD_CONFIG, &packet, sizeof(ConfigPacket_t));
		itsCameraEnabled = false;
	}

	void Robot::enableNeighbourDetection()
	{
		if (itsSimulationEnabled)
			return;

		ConfigPacket_t packet;
		packet.MessageID = MSG_NEIGHBOUR;
		packet.Enable = 1;

		writeSync(CMD_CONFIG, &packet, sizeof(ConfigPacket_t));
		itsNeighbourDetectionEnabled = true;
	}

	void Robot::disableNeighbourDetection()
	{
		if (itsSimulationEnabled)
			return;

		ConfigPacket_t packet;
		packet.MessageID = MSG_NEIGHBOUR;
		packet.Enable = 0;

		writeSync(CMD_CONFIG, &packet, sizeof(ConfigPacket_t));
		itsNeighbourDetectionEnabled = false;
	}

	void Robot::setLED(int i)
	{
		if (i < 0 || i > 9)
			throw std::out_of_range("Robot: LED index out of range.");

		itsStatus.LEDs |= (1 << i);

		if (itsSimulationEnabled)
			return;

		writeAsync(CMD_LEDS, &itsStatus.LEDs, sizeof(LedsPacket_t));
	}

	void Robot::clearLED(int i)
	{
		if (i < 0 || i > 9)
			throw std::out_of_range("Robot: LED index out of range.");

		itsStatus.LEDs &= ~(1 << i);

		if (itsSimulationEnabled)
			return;

		writeAsync(CMD_LEDS, &itsStatus.LEDs, sizeof(LedsPacket_t));
	}

	void Robot::toggleLED(int i)
	{
		if (i < 0 || i > 9)
			throw std::out_of_range("Robot: LED index out of range.");

		itsStatus.LEDs ^= (1 << i);

		if (itsSimulationEnabled)
			return;

		writeAsync(CMD_LEDS, &itsStatus.LEDs, sizeof(LedsPacket_t));
	}

	void Robot::setLED(uint16_t leds)
	{
		itsStatus.LEDs = leds;

		if (itsSimulationEnabled)
			return;

		writeAsync(CMD_LEDS, &leds, sizeof(LedsPacket_t));
	}

	void Robot::setSensorDataReceivedHandler(Handler_t handler, void* arg)
	{
		itsSensorDataReceived.Handler = handler;
		itsSensorDataReceived.Arg = arg;
	}

	void Robot::setImageReceivedHandler(Handler_t handler, void* arg)
	{
		itsImageReceived.Handler = handler;
		itsImageReceived.Arg = arg;
	}


	void Robot::setNeighboursDetectedHandler(Handler_t handler, void* arg)
	{
		itsNeighbourDetected.Handler = handler;
		itsNeighbourDetected.Arg = arg;
	}

	void Robot::setGumstixResponseReceivedHandler(ResponseReceivedHandler_t handler, void* arg)
	{
		itsGumstixResponseReceivedHandler = handler;
		itsGumstixResponseReceivedArg = arg;
	}

	void Robot::Beep(int id)
	{
		if (itsSimulationEnabled)
			return;

		if (id > BEEP_ID_MAX)
			throw std::out_of_range("Robot: beep ID of of range.");

		BeepPacket_t packet = (BeepPacket_t)id;

		writeSync(CMD_BEEP, &packet, sizeof(BeepPacket_t));
	}

	void Robot::sendToGumstix(const void* data, uint16_t len)
	{
		if (itsSimulationEnabled)
			return;

		writeAsync(CMD_GUMSTIX, data, len);
	}

	void Robot::enableSimulation()
	{
		if (itsSimulationEnabled)
			return;

		itsSimulationEnabled = true;
		itsThread = std::thread(&Robot::runSimulation, this);
	}

	void Robot::disableSimulation()
	{
		if (!itsSimulationEnabled)
			return;

		itsSimulationEnabled = false;
		itsThread.join();
	}

	void Robot::bindOccupancyGrid(std::shared_ptr<OccupancyGrid> grid)
	{
		itsGrid = grid;
	}

	void Robot::runSimulation()
	{
		const double dt = 0.1;

		while (itsSimulationEnabled)
		{
			Position_t pos = itsStatus.Position;

			// update position by speed
			double dL = (double)itsStatus.Speed.Left * dt;
			double dR = (double)itsStatus.Speed.Right * dt;

			double dx_loc, dy_loc;
			double dpsi = (dL - dR) / TICKS_PER_TRACK;

			if (std::abs(dpsi) > 1e-5)
			{
				// turning
				double R = (dL + dR) / dpsi / 2.0;
				dx_loc = R * std::sin(dpsi);
				dy_loc = R * (1.0 - std::cos(dpsi));
			}
			else
			{
				// almost straight
				dx_loc = (dL + dR) / 2.0;
				dy_loc = (dL*dL - dR * dR) / TICKS_PER_TRACK / 4.0;
			}

			// rotate to global coordinates
			double psi = (double)itsStatus.Position.psi / TICKS_PER_RADIAN;
			double c = std::cos(psi);
			double s = std::sin(psi);
			double dx_glob = dx_loc * c - dy_loc * s;
			double dy_glob = dy_loc * c + dx_loc * s;

			psi += dpsi;
			psi -= M_2PI * std::round(psi / M_2PI);

			// increment counters
			pos.x += (int32_t)dx_glob;
			pos.y += (int32_t)dy_glob;
			pos.psi = (int16_t)(psi * TICKS_PER_RADIAN);

			
			Proximity_t prox;

			// clear proximity
			prox.L_150deg = 0;
			prox.L_90deg = 0;
			prox.L_50deg = 0;
			prox.L_20deg = 0;

			prox.R_150deg = 0;
			prox.R_90deg = 0;
			prox.R_50deg = 0;
			prox.R_20deg = 0;

			if (itsGrid)
			{
				// get neighbouring occupancy grid cells
				const int32_t r0 = (int32_t)(TICKS_PER_METER * 0.035);
				const int32_t rMax = (int32_t)(TICKS_PER_METER * 0.07);


				for (int32_t dx = -rMax; dx <= rMax; dx += itsGrid->gridSize()/2)
				{
					for (int32_t dy = -rMax; dy <= rMax; dy += itsGrid->gridSize()/2)
					{
						float occ = itsGrid->get(pos.x + dx, pos.y + dy);
						if (occ < 0.2)
							continue;

						double psi = std::atan2((double)dy, (double)dx) * TICKS_PER_RADIAN - (double)pos.psi;
						psi -= TICKS_PER_TURN * std::round(psi / TICKS_PER_TURN);

						int32_t d = (int32_t)std::sqrt(dx * dx + dy * dy) - r0;
						int16_t p = (d > 5) ? (15000 / d) : 3000;
						
						if (psi < -120.0 * TICKS_PER_TURN / 360)
							prox.L_150deg = std::max(prox.L_150deg, p);
						else if (psi < -70.0 * TICKS_PER_TURN / 360)
							prox.L_90deg = std::max(prox.L_90deg, p);
						else if (psi < -35.0 * TICKS_PER_TURN / 360)
							prox.L_50deg = std::max(prox.L_50deg, p);
						else if (psi < 0.0 * TICKS_PER_TURN / 360)
							prox.L_20deg = std::max(prox.L_20deg, p);
						else if (psi < 35.0 * TICKS_PER_TURN / 360)
							prox.R_20deg = std::max(prox.R_20deg, p);
						else if (psi < 70.0 * TICKS_PER_TURN / 360)
							prox.R_50deg = std::max(prox.R_50deg, p);
						else if (psi < 120.0 * TICKS_PER_TURN / 360)
							prox.R_90deg = std::max(prox.R_90deg, p);
						else
							prox.R_150deg = std::max(prox.R_150deg, p);
					}
				}

				if (prox.L_20deg > DANGER_LIMIT
					|| prox.R_20deg > DANGER_LIMIT
					|| prox.L_50deg > DANGER_LIMIT
					|| prox.R_50deg > DANGER_LIMIT)
				{
					// no forward movement
					if (dx_loc > 0)
					{
						pos = itsStatus.Position;
						itsStatus.Speed.Left = 0;
						itsStatus.Speed.Right = 0;
					}
				}

				if (prox.L_150deg > DANGER_LIMIT
					|| prox.R_150deg > DANGER_LIMIT)
				{
					// no backward movement
					if (dx_loc < 0)
					{
						pos = itsStatus.Position;
						itsStatus.Speed.Left = 0;
						itsStatus.Speed.Right = 0;
					}
				}
			}
			
			itsStatus.Position = pos;
			itsStatus.Proximity = prox;

			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}




} // end of namespace
