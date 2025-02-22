#include "rc.h"

#include <Arduino.h>
#include <ArduinoCRSF.h>

/**
 * @brief Wrapper namespace for the TX12 rc controller.
 * Communication to the controller is implemented using the ArduinoCRSF class.
 *
 */
namespace rc
{

#define RC_BAUDRATE 115200

#define CHANNEL_LEFT_X 4
#define CHANNEL_LEFT_Y 3
#define CHANNEL_RIGHT_X 1
#define CHANNEL_RIGHT_Y 2
#define CHANNEL_SWITCH_E 5
#define CHANNEL_SWITCH_F 6
#define CHANNEL_SWITCH_B 7
#define CHANNEL_SWITCH_C 8
#define CHANNEL_BRAKE_9 9

	ArduinoCRSF rc_controller;
	bool offset_switch_prev = false;

	static uint8_t rxbuf[1024];

	/**
	 * @brief Initializes hardware.  Should be called in the main setup() function.
	 */
	void init(HardwareSerial &serial)
	{
		serial.begin(RC_BAUDRATE);
		serial.addMemoryForRead(rxbuf, sizeof(rxbuf));
		if (!serial)
		{
			while (1)
			{
				Serial.println("RC serial port init failed!");
				delay(1000);
			}
		}
		rc_controller.begin(serial);
	}

	/**
	 * @brief Calls the controller's internal update function.
	 *
	 */
	void update()
	{
		rc_controller.update();
	}
	/**
	 * @brief Calls the controller's internal getLinkStatistics function.
	 *
	 * @return const crsfLinkStatistics_t& A pointer to a struct containing statistics of the connection to the controller.
	 */
	const crsfLinkStatistics_t &link_statistics()
	{
		return *rc_controller.getLinkStatistics();
	}

	/**
	 * @return true The rc controler is connected to the buggy.
	 * @return false The rc controller is not connected to the buggy.
	 */
	bool connected()
	{
		return rc_controller.isLinkUp();
	}

	/**
	 * @brief Effectively, this serves to check that the human holding the rc controller is ready for the buggy to be moving.
	 * This function checks that the A or D button on the controller is held down.
	 *
	 * @return true The person holding the controller is ready for the buggy to start moving.
	 * @return false The person holding the controller is not ready for the buggy to start moving.
	 */
	bool operator_ready()
	{
		//Serial.printf("Channel 9: %d\n",rc_controller.getChannel(CHANNEL_BRAKE_9));
		if (connected())
		{
			return (rc_controller.getChannel(CHANNEL_BRAKE_9) > 1500);
		}
		else
		{
			return false;
		}
	}

#define MAX_RC_STEERING_DEGREES 30.0

	/**
	 * @brief Returns the angle that the steering wheel is pointing to.
	 * Defaults to zero if there is no connection to the controller.
	 *
	 * @return float Units are in degrees.  Zero degrees is straight forward.
	 * Positive values are left of center.  Negative values are right of center.
	 * In other words, CCW is positive.
	 */
	float steering_angle()
	{
		if (connected())
		{
			int raw_width = rc_controller.getChannel(CHANNEL_RIGHT_X);

			// Scaled to -1.0 to 1.0, left positive
			float analog = -1.0 * (raw_width - 1500.0) / 500.0;

			return analog * MAX_RC_STEERING_DEGREES;
		}
		else
		{
			return 0.0;
		}
	}

	/**
	 * @brief Checks if the "use autonomous steering" switch on the controller is on or not.
	 */
	bool use_autonomous_steering()
	{
		bool auto_switch = (rc_controller.getChannel(CHANNEL_SWITCH_E) > 1750);
		return operator_ready() && auto_switch;
	}

	/**
	 * @brief Returns raw steering angle, exactly as recieved from the RC
	 */
	int raw_steering_angle() {
		return rc_controller.getChannel(CHANNEL_RIGHT_X);
	}

	/**
	 * @brief Returns raw autonomous switch value, exactly as recieved from the RC
	 * 
	 * @return Integer value, check use_autonomous_steering() for activation threshold (1750 at time of writing)
	 */
	int raw_auto_switch() {
		return rc_controller.getChannel(CHANNEL_SWITCH_E);
	}

	bool temp_offset_switch()
	{
		bool offset_switch = (rc_controller.getChannel(CHANNEL_SWITCH_F) > 1750);
		bool edge = offset_switch && !offset_switch_prev;
		offset_switch_prev = offset_switch;
		return edge;
	}

} // namespace rc