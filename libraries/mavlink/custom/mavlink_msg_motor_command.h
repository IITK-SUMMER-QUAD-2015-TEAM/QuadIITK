// MESSAGE MOTOR_COMMAND PACKING

#define MAVLINK_MSG_ID_MOTOR_COMMAND 154

typedef struct __mavlink_motor_command_t
{
 uint16_t LFValue; ///< The pwm applied to the left front motor.
 uint16_t RFValue; ///< The pwm applied to the right front motor.
 uint16_t BFValue; ///< The pwm applied to the left back motor.
 uint16_t BRValue; ///< The pwm applied to the right back motor.
} mavlink_motor_command_t;

#define MAVLINK_MSG_ID_MOTOR_COMMAND_LEN 8
#define MAVLINK_MSG_ID_154_LEN 8

#define MAVLINK_MSG_ID_MOTOR_COMMAND_CRC 62
#define MAVLINK_MSG_ID_154_CRC 62



#define MAVLINK_MESSAGE_INFO_MOTOR_COMMAND { \
	"MOTOR_COMMAND", \
	4, \
	{  { "LFValue", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_motor_command_t, LFValue) }, \
         { "RFValue", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_motor_command_t, RFValue) }, \
         { "BFValue", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_motor_command_t, BFValue) }, \
         { "BRValue", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_motor_command_t, BRValue) }, \
         } \
}


/**
 * @brief Pack a motor_command message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param LFValue The pwm applied to the left front motor.
 * @param RFValue The pwm applied to the right front motor.
 * @param BFValue The pwm applied to the left back motor.
 * @param BRValue The pwm applied to the right back motor.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_command_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t LFValue, uint16_t RFValue, uint16_t BFValue, uint16_t BRValue)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MOTOR_COMMAND_LEN];
	_mav_put_uint16_t(buf, 0, LFValue);
	_mav_put_uint16_t(buf, 2, RFValue);
	_mav_put_uint16_t(buf, 4, BFValue);
	_mav_put_uint16_t(buf, 6, BRValue);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN);
#else
	mavlink_motor_command_t packet;
	packet.LFValue = LFValue;
	packet.RFValue = RFValue;
	packet.BFValue = BFValue;
	packet.BRValue = BRValue;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MOTOR_COMMAND;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN, MAVLINK_MSG_ID_MOTOR_COMMAND_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN);
#endif
}

/**
 * @brief Pack a motor_command message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param LFValue The pwm applied to the left front motor.
 * @param RFValue The pwm applied to the right front motor.
 * @param BFValue The pwm applied to the left back motor.
 * @param BRValue The pwm applied to the right back motor.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_command_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t LFValue,uint16_t RFValue,uint16_t BFValue,uint16_t BRValue)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MOTOR_COMMAND_LEN];
	_mav_put_uint16_t(buf, 0, LFValue);
	_mav_put_uint16_t(buf, 2, RFValue);
	_mav_put_uint16_t(buf, 4, BFValue);
	_mav_put_uint16_t(buf, 6, BRValue);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN);
#else
	mavlink_motor_command_t packet;
	packet.LFValue = LFValue;
	packet.RFValue = RFValue;
	packet.BFValue = BFValue;
	packet.BRValue = BRValue;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MOTOR_COMMAND;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN, MAVLINK_MSG_ID_MOTOR_COMMAND_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN);
#endif
}

/**
 * @brief Encode a motor_command struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param motor_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motor_command_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_motor_command_t* motor_command)
{
	return mavlink_msg_motor_command_pack(system_id, component_id, msg, motor_command->LFValue, motor_command->RFValue, motor_command->BFValue, motor_command->BRValue);
}

/**
 * @brief Encode a motor_command struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param motor_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motor_command_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_motor_command_t* motor_command)
{
	return mavlink_msg_motor_command_pack_chan(system_id, component_id, chan, msg, motor_command->LFValue, motor_command->RFValue, motor_command->BFValue, motor_command->BRValue);
}

/**
 * @brief Send a motor_command message
 * @param chan MAVLink channel to send the message
 *
 * @param LFValue The pwm applied to the left front motor.
 * @param RFValue The pwm applied to the right front motor.
 * @param BFValue The pwm applied to the left back motor.
 * @param BRValue The pwm applied to the right back motor.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_motor_command_send(mavlink_channel_t chan, uint16_t LFValue, uint16_t RFValue, uint16_t BFValue, uint16_t BRValue)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MOTOR_COMMAND_LEN];
	_mav_put_uint16_t(buf, 0, LFValue);
	_mav_put_uint16_t(buf, 2, RFValue);
	_mav_put_uint16_t(buf, 4, BFValue);
	_mav_put_uint16_t(buf, 6, BRValue);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_COMMAND, buf, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN, MAVLINK_MSG_ID_MOTOR_COMMAND_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_COMMAND, buf, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN);
#endif
#else
	mavlink_motor_command_t packet;
	packet.LFValue = LFValue;
	packet.RFValue = RFValue;
	packet.BFValue = BFValue;
	packet.BRValue = BRValue;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_COMMAND, (const char *)&packet, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN, MAVLINK_MSG_ID_MOTOR_COMMAND_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_COMMAND, (const char *)&packet, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_MOTOR_COMMAND_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_motor_command_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t LFValue, uint16_t RFValue, uint16_t BFValue, uint16_t BRValue)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint16_t(buf, 0, LFValue);
	_mav_put_uint16_t(buf, 2, RFValue);
	_mav_put_uint16_t(buf, 4, BFValue);
	_mav_put_uint16_t(buf, 6, BRValue);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_COMMAND, buf, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN, MAVLINK_MSG_ID_MOTOR_COMMAND_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_COMMAND, buf, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN);
#endif
#else
	mavlink_motor_command_t *packet = (mavlink_motor_command_t *)msgbuf;
	packet->LFValue = LFValue;
	packet->RFValue = RFValue;
	packet->BFValue = BFValue;
	packet->BRValue = BRValue;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_COMMAND, (const char *)packet, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN, MAVLINK_MSG_ID_MOTOR_COMMAND_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_COMMAND, (const char *)packet, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE MOTOR_COMMAND UNPACKING


/**
 * @brief Get field LFValue from motor_command message
 *
 * @return The pwm applied to the left front motor.
 */
static inline uint16_t mavlink_msg_motor_command_get_LFValue(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field RFValue from motor_command message
 *
 * @return The pwm applied to the right front motor.
 */
static inline uint16_t mavlink_msg_motor_command_get_RFValue(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field BFValue from motor_command message
 *
 * @return The pwm applied to the left back motor.
 */
static inline uint16_t mavlink_msg_motor_command_get_BFValue(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field BRValue from motor_command message
 *
 * @return The pwm applied to the right back motor.
 */
static inline uint16_t mavlink_msg_motor_command_get_BRValue(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Decode a motor_command message into a struct
 *
 * @param msg The message to decode
 * @param motor_command C-struct to decode the message contents into
 */
static inline void mavlink_msg_motor_command_decode(const mavlink_message_t* msg, mavlink_motor_command_t* motor_command)
{
#if MAVLINK_NEED_BYTE_SWAP
	motor_command->LFValue = mavlink_msg_motor_command_get_LFValue(msg);
	motor_command->RFValue = mavlink_msg_motor_command_get_RFValue(msg);
	motor_command->BFValue = mavlink_msg_motor_command_get_BFValue(msg);
	motor_command->BRValue = mavlink_msg_motor_command_get_BRValue(msg);
#else
	memcpy(motor_command, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_MOTOR_COMMAND_LEN);
#endif
}
