// MESSAGE BAROMETER_OUTPUTS PACKING

#define MAVLINK_MSG_ID_BAROMETER_OUTPUTS 153

typedef struct __mavlink_barometer_outputs_t
{
 float pressureValue; ///< The pressure value.
 float temperatureValue; ///< The temperature value.
 float altitudeValue; ///< The altitude value.
} mavlink_barometer_outputs_t;

#define MAVLINK_MSG_ID_BAROMETER_OUTPUTS_LEN 12
#define MAVLINK_MSG_ID_153_LEN 12

#define MAVLINK_MSG_ID_BAROMETER_OUTPUTS_CRC 147
#define MAVLINK_MSG_ID_153_CRC 147



#define MAVLINK_MESSAGE_INFO_BAROMETER_OUTPUTS { \
	"BAROMETER_OUTPUTS", \
	3, \
	{  { "pressureValue", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_barometer_outputs_t, pressureValue) }, \
         { "temperatureValue", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_barometer_outputs_t, temperatureValue) }, \
         { "altitudeValue", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_barometer_outputs_t, altitudeValue) }, \
         } \
}


/**
 * @brief Pack a barometer_outputs message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param pressureValue The pressure value.
 * @param temperatureValue The temperature value.
 * @param altitudeValue The altitude value.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_barometer_outputs_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float pressureValue, float temperatureValue, float altitudeValue)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_BAROMETER_OUTPUTS_LEN];
	_mav_put_float(buf, 0, pressureValue);
	_mav_put_float(buf, 4, temperatureValue);
	_mav_put_float(buf, 8, altitudeValue);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BAROMETER_OUTPUTS_LEN);
#else
	mavlink_barometer_outputs_t packet;
	packet.pressureValue = pressureValue;
	packet.temperatureValue = temperatureValue;
	packet.altitudeValue = altitudeValue;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BAROMETER_OUTPUTS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_BAROMETER_OUTPUTS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BAROMETER_OUTPUTS_LEN, MAVLINK_MSG_ID_BAROMETER_OUTPUTS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BAROMETER_OUTPUTS_LEN);
#endif
}

/**
 * @brief Pack a barometer_outputs message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pressureValue The pressure value.
 * @param temperatureValue The temperature value.
 * @param altitudeValue The altitude value.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_barometer_outputs_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float pressureValue,float temperatureValue,float altitudeValue)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_BAROMETER_OUTPUTS_LEN];
	_mav_put_float(buf, 0, pressureValue);
	_mav_put_float(buf, 4, temperatureValue);
	_mav_put_float(buf, 8, altitudeValue);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BAROMETER_OUTPUTS_LEN);
#else
	mavlink_barometer_outputs_t packet;
	packet.pressureValue = pressureValue;
	packet.temperatureValue = temperatureValue;
	packet.altitudeValue = altitudeValue;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BAROMETER_OUTPUTS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_BAROMETER_OUTPUTS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BAROMETER_OUTPUTS_LEN, MAVLINK_MSG_ID_BAROMETER_OUTPUTS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BAROMETER_OUTPUTS_LEN);
#endif
}

/**
 * @brief Encode a barometer_outputs struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param barometer_outputs C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_barometer_outputs_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_barometer_outputs_t* barometer_outputs)
{
	return mavlink_msg_barometer_outputs_pack(system_id, component_id, msg, barometer_outputs->pressureValue, barometer_outputs->temperatureValue, barometer_outputs->altitudeValue);
}

/**
 * @brief Encode a barometer_outputs struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param barometer_outputs C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_barometer_outputs_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_barometer_outputs_t* barometer_outputs)
{
	return mavlink_msg_barometer_outputs_pack_chan(system_id, component_id, chan, msg, barometer_outputs->pressureValue, barometer_outputs->temperatureValue, barometer_outputs->altitudeValue);
}

/**
 * @brief Send a barometer_outputs message
 * @param chan MAVLink channel to send the message
 *
 * @param pressureValue The pressure value.
 * @param temperatureValue The temperature value.
 * @param altitudeValue The altitude value.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_barometer_outputs_send(mavlink_channel_t chan, float pressureValue, float temperatureValue, float altitudeValue)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_BAROMETER_OUTPUTS_LEN];
	_mav_put_float(buf, 0, pressureValue);
	_mav_put_float(buf, 4, temperatureValue);
	_mav_put_float(buf, 8, altitudeValue);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAROMETER_OUTPUTS, buf, MAVLINK_MSG_ID_BAROMETER_OUTPUTS_LEN, MAVLINK_MSG_ID_BAROMETER_OUTPUTS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAROMETER_OUTPUTS, buf, MAVLINK_MSG_ID_BAROMETER_OUTPUTS_LEN);
#endif
#else
	mavlink_barometer_outputs_t packet;
	packet.pressureValue = pressureValue;
	packet.temperatureValue = temperatureValue;
	packet.altitudeValue = altitudeValue;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAROMETER_OUTPUTS, (const char *)&packet, MAVLINK_MSG_ID_BAROMETER_OUTPUTS_LEN, MAVLINK_MSG_ID_BAROMETER_OUTPUTS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAROMETER_OUTPUTS, (const char *)&packet, MAVLINK_MSG_ID_BAROMETER_OUTPUTS_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_BAROMETER_OUTPUTS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_barometer_outputs_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float pressureValue, float temperatureValue, float altitudeValue)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, pressureValue);
	_mav_put_float(buf, 4, temperatureValue);
	_mav_put_float(buf, 8, altitudeValue);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAROMETER_OUTPUTS, buf, MAVLINK_MSG_ID_BAROMETER_OUTPUTS_LEN, MAVLINK_MSG_ID_BAROMETER_OUTPUTS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAROMETER_OUTPUTS, buf, MAVLINK_MSG_ID_BAROMETER_OUTPUTS_LEN);
#endif
#else
	mavlink_barometer_outputs_t *packet = (mavlink_barometer_outputs_t *)msgbuf;
	packet->pressureValue = pressureValue;
	packet->temperatureValue = temperatureValue;
	packet->altitudeValue = altitudeValue;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAROMETER_OUTPUTS, (const char *)packet, MAVLINK_MSG_ID_BAROMETER_OUTPUTS_LEN, MAVLINK_MSG_ID_BAROMETER_OUTPUTS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAROMETER_OUTPUTS, (const char *)packet, MAVLINK_MSG_ID_BAROMETER_OUTPUTS_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE BAROMETER_OUTPUTS UNPACKING


/**
 * @brief Get field pressureValue from barometer_outputs message
 *
 * @return The pressure value.
 */
static inline float mavlink_msg_barometer_outputs_get_pressureValue(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field temperatureValue from barometer_outputs message
 *
 * @return The temperature value.
 */
static inline float mavlink_msg_barometer_outputs_get_temperatureValue(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field altitudeValue from barometer_outputs message
 *
 * @return The altitude value.
 */
static inline float mavlink_msg_barometer_outputs_get_altitudeValue(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a barometer_outputs message into a struct
 *
 * @param msg The message to decode
 * @param barometer_outputs C-struct to decode the message contents into
 */
static inline void mavlink_msg_barometer_outputs_decode(const mavlink_message_t* msg, mavlink_barometer_outputs_t* barometer_outputs)
{
#if MAVLINK_NEED_BYTE_SWAP
	barometer_outputs->pressureValue = mavlink_msg_barometer_outputs_get_pressureValue(msg);
	barometer_outputs->temperatureValue = mavlink_msg_barometer_outputs_get_temperatureValue(msg);
	barometer_outputs->altitudeValue = mavlink_msg_barometer_outputs_get_altitudeValue(msg);
#else
	memcpy(barometer_outputs, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_BAROMETER_OUTPUTS_LEN);
#endif
}
