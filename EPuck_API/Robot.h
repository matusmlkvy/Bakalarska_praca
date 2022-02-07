#ifndef ROBOT_H_
#define ROBOT_H_

#include <string>
#include <memory>
#include <mutex>
#include <atomic>
#include <vector>
#include <thread>

#include "defs.h"
#include "../Firmware/packets.h"
#include "PixTurret.h"
#include "Image565.h"
#include "OccupancyGrid.h"

#ifndef M_PI
#	define M_PI  3.14159265358979323846
#endif


#ifndef M_2PI
#	define M_2PI  6.283185307179586476925
#endif


// Disable enforcing DLL interface for private members
#pragma warning(push)
#pragma warning(disable: 4251)

namespace EPuck 
{
	// forward declaration of implementation class
	class Interface;

	class EPUCK_API Robot
	{
	public:
		typedef void(*Handler_t)(void*);

		typedef void(*ResponseReceivedHandler_t)(void* arg, const void* data, uint16_t len);

		class NoResponseException : public std::runtime_error
		{
		public:
			NoResponseException() : std::runtime_error("Error: EPuck::Robot does not respond.") {}
		};

		enum CameraColorMode
		{
			GREY_SCALE_MODE=0,
			RGB_565_MODE=1,
			YUV_MODE=2
		};

		enum ConnectionType
		{
			Serial = 0,
			UDP = 1,
			TCP = 2
		};

	private:
		// Storage for callback
		struct Event_t
		{
			Handler_t	Handler;
			void*		Arg;

			Event_t() : Handler(NULL), Arg(NULL) {}

			inline void Fire() { if (Handler) Handler(Arg); }
		};

	public:
		// Creates E-puck with no connection.
		Robot();

		// Creates E-puck and opens the connection.
		// If comType is Serial, the host is the port name (e.g. COMx on Windows or /dev/ttySx on Linux).
		// If com Type is TCP or UDP, the host is the IP address.
		Robot(const char* host, ConnectionType comType = Serial);

		// Destructor
		~Robot();

		// Opens E-puck connection at specified location.
		// If comType is Serial, the host is the port name (e.g. COMx on Windows or /dev/ttySx on Linux).
		// If com Type is TCP or UDP, the host is the IP address.
		void open(const char* host, ConnectionType comType = Serial);

		// Closes E-puck connection
		void close();

		// Returns true when connection is open
		bool isOpen() const;

		// Verifies connection and returns true when E-puck is responding
		bool testConnection();

		// Returns speeds of each wheel
		const Wheels_t& wheels() const;

		// Returns position of the robot
		const Position_t& position() const;

		// Returns accelerometer data
		const Vector3D_t& accel() const;

		// Returns volume measured by microphones
		const Microphones_t& sound() const;

		// Returns proximity measured by given sensor
		const Proximity_t& proximity() const;

		// Returns floor sensor reading
		const Floor_t& floorSensor() const;

		// Returns status of the given LED [0 to 9]
		bool LED(int i) const;

		// Returns all LEDs.
		uint16_t LED() const;

		// Returns true when sensor data are enabled.
		bool sensorsEnabled() const;

		// Returns true when camera is enabled.
		bool cameraEnabled() const;

		// Returns true when sonic neighbour detection is enabled.
		bool neighbourDetectionEnabled() const;

		// Returns true when simulation is enabled
		bool simulationEnabled() const;

		// Returns camera settings
		const CameraSettings_t& cameraSettings() const;

		// Returns last image.
		Image565& lastImage();

		// Returns pixel turret object.
		PixTurret& pixTurret();

		// Returns k-th neighbour.
		Neighbour_t& neigbour(int k);

		// Returns count of detected neighbours.
		size_t neighboursCount() const;

		// Sets speed of the wheels
		void setWheels(const Wheels_t& wheels);

		// Sets speed of the wheels
		void setWheels(int16_t left, int16_t right);

		// Sets sequence of actions
		void setActions(const std::vector<Wheels_t>& actions);

		// Sets position of the robot. 
		// Note: This action does not move the robot, only changes reference point.
		void setPosition(const Position_t& position);

		// Changes camera settings.
		void setCameraSettings(const CameraSettings_t& settings);

		// Enables sensor data.
		void enableSensors();

		// Disables sensor data.
		void disableSensors();

		// Enables camera.
		void enableCamera();

		// Disables camera.
		void disableCamera();

		// Enables sonic detection of the neighbours.
		void enableNeighbourDetection();

		// Disables sonic detection of the neighbours.
		void disableNeighbourDetection();

		// Enables simulation mode
		void enableSimulation();

		// Disables simulation
		void disableSimulation();

		// Binds occupancy grid with the robot
		void bindOccupancyGrid(std::shared_ptr<OccupancyGrid> grid);

		// Sets given LED [0 to 9]
		void setLED(int i);

		// Clears given LED [0 to 9]
		void clearLED(int i);

		// Toggles given LED [0 to 9]
		void toggleLED(int i);

		// Sets all LEDs.
		void setLED(uint16_t leds);

		// Beeps with given ID frequency
		void Beep(int id);

		// Writes custom message to Gumstix Overo extension.
		void sendToGumstix(const void* data, uint16_t len);

		// Assigns handler for event fired when new sensor data are received.
		// Note: it is the only thread-safe way to read robot's properties.
		void setSensorDataReceivedHandler(Handler_t handler, void* arg=NULL);
		
		// Assigns handler for event fired when new image is received.
		// Note: its is the only thread-safe way to read recevived image.
		void setImageReceivedHandler(Handler_t handler, void* arg=NULL);

		// Assigns handler for event fired when new neigbours were detected
		void setNeighboursDetectedHandler(Handler_t handler, void* arg=NULL);

		// Assigns handler for event fired when response from gumstix is received.
		void setGumstixResponseReceivedHandler(ResponseReceivedHandler_t handler, void* arg = NULL);

		// Large enough to read whole packet except the image packet
		static const int CACHE_SIZE = 100;

		// Maximal size of the image
		static const int IMAGE_SIZE_MAX = 4096;

		// Width of the camera sensor
		static const int IMAGE_WIDTH_MAX = 640;

		// Height of the camera sensor
		static const int IMAGE_HEIGHT_MAX = 480;

		// Minimal ID
		static const int BEEP_ID_MIN = 1;

		// Maximal ID
		static const int BEEP_ID_MAX = 10;

		friend class PixTurret;
		friend class Interface;

	private:
		// Internal: Handles packetReceived event of the interface.
		// Updates data according to the parsed packet.
		static void applyMsg(char msgID, uint16_t msgLen, const uint8_t* data, void* arg);

		// Internal: writes packet with given ID to the communication line.
		// Waits until ACK message is received, otherwise throws exception.
		void writeSync(uint8_t id, const void* data, size_t len);

		// Internal: Writes packet asynchronously with given ID to the communication line.
		void writeAsync(uint8_t id, const void* data, size_t len);

		// Internal: Runs simulation.
		void runSimulation();

	private:
		std::unique_ptr<Interface>					itsInterface;
		std::mutex									itsMutex;

		std::atomic<bool>							itsAckReceived;

		Image565									itsImage;

		std::vector<Neighbour_t>					itsNeighbours;

		StatusPacket_t								itsStatus;
		CameraSettings_t							itsCameraSettings;
		Event_t										itsSensorDataReceived;
		Event_t										itsImageReceived;
		Event_t										itsNeighbourDetected;

		ResponseReceivedHandler_t					itsGumstixResponseReceivedHandler;
		void*										itsGumstixResponseReceivedArg;

		bool										itsCameraEnabled;
		bool										itsSensorsEnabled;
		bool										itsNeighbourDetectionEnabled;
		volatile bool								itsSimulationEnabled;

		std::vector<Wheels_t>						itsActions;

		PixTurret									itsPixTurret;
		std::shared_ptr<OccupancyGrid>				itsGrid;
		std::thread									itsThread;
	};

}

#pragma warning(pop)

#endif /* ROBOT_H_ */
