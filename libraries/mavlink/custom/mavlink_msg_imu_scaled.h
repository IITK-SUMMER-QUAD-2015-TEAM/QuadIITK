// MESSAGE IMU_SCALED PACKING

#define MAVLINK_MSG_ID_IMU_SCALED 150

typedef struct __mavlink_imu_scaled_t
{
 float Gx; ///< The scaled Gx value in radians per second.
 float Gy; ///< The scaled Gy value in radians per second.
 float Gz; ///< The scaled Gz value in radians per second.
 float Ax; ///< The scaled Ax value in terms of g.
 float Ay; ///< The scaled Ay value in terms of g.
 float Az; ///< The scaled Az value in terms of g.
 float Mx; ///< The scaled Mx value in terms of g.
 float My; ///< The scaled My value in terms of g.
 float Mz; ///< The scaled Mz value in terms of g.
} mavlink_imu_scaled_t;

#define MAVLINK_MSG_ID_IMU_SCALED_LEN 36
#define MAVLINK_MSG_ID_150_LEN 36

#define MAVLINK_MSG_ID_IMU_SCALED_CRC 3
#define MAVLINK_MSG_ID_150_CRC 3



#define MAVLINK_MESSAGE_INFO_IMU_SCALED { \
	"IMU_SCALED", \
	9, \
	{  { "Gx", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_imu_scaled_t, Gx) }, \
         { "Gy", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_imu_scaled_t, Gy) }, \
         { "Gz", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_imu_scaled_t, Gz) }, \
         { "Ax", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_imu_scaled_t, Ax) }, \
         { "Ay", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_imu_scaled_t, Ay) }, \
         { "Az", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_imu_scaled_t, Az) }, \
         { "Mx", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_imu_scaled_t, Mx) }, \
         { "My", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_imu_scaled_t, My) }, \
         { "Mz", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_imu_scaled_t, Mz) }, \
         } \
}


/**
 * @brief Pack a imu_scaled message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param Gx The scaled Gx value in radians per second.
 * @param Gy The scaled Gy value in radians per second.
 * @param Gz The scaled Gz value in radians per second.
 * @param Ax The scaled Ax value in terms of g.
 * @param Ay The scaled Ay value in terms of g.
 * @param Az The scaled Az value in terms of g.
 * @param Mx The scaled Mx value in terms of g.
 * @param My The scaled My value in terms of g.
 * @param Mz The scaled Mz value in terms of g.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_imu_scaled_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float Gx, float Gy, float Gz, float Ax, float Ay, float Az, float Mx, float My, float Mz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_IMU_SCALED_LEN];
	_mav_put_float(buf, 0, Gx);
	_mav_put_float(buf, 4, Gy);
	_mav_put_float(buf, 8, Gz);
	_mav_put_float(buf, 12, Ax);
	_mav_put_float(buf, 16, Ay);
	_mav_put_float(buf, 20, Az);
	_mav_put_float(buf, 24, Mx);
	_mav_put_float(buf, 28, My);
	_mav_put_float(buf, 32, Mz);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IMU_SCALED_LEN);
#else
	mavlink_imu_scaled_t packet;
	packet.Gx = Gx;
	packet.Gy = Gy;
	packet.Gz = Gz;
	packet.Ax = Ax;
	packet.Ay = Ay;
	packet.Az = Az;
	packet.Mx = Mx;
	packet.My = My;
	packet.Mz = Mz;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IMU_SCALED_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_IMU_SCALED;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_IMU_SCALED_LEN, MAVLINK_MSG_ID_IMU_SCALED_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_IMU_SCALED_LEN);
#endif
}

/**
 * @brief Pack a imu_scaled message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param Gx The scaled Gx value in radians per second.
 * @param Gy The scaled Gy value in radians per second.
 * @param Gz The scaled Gz value in radians per second.
 * @param Ax The scaled Ax value in terms of g.
 * @param Ay The scaled Ay value in terms of g.
 * @param Az The scaled Az value in terms of g.
 * @param Mx The scaled Mx value in terms of g.
 * @param My The scaled My value in terms of g.
 * @param Mz The scaled Mz value in terms of g.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_imu_scaled_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float Gx,float Gy,float Gz,float Ax,float Ay,float Az,float Mx,float My,float Mz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_IMU_SCALED_LEN];
	_mav_put_float(buf, 0, Gx);
	_mav_put_float(buf, 4, Gy);
	_mav_put_float(buf, 8, Gz);
	_mav_put_float(buf, 12, Ax);
	_mav_put_float(buf, 16, Ay);
	_mav_put_float(buf, 20, Az);
	_mav_put_float(buf, 24, Mx);
	_mav_put_float(buf, 28, My);
	_mav_put_float(buf, 32, Mz);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IMU_SCALED_LEN);
#else
	mavlink_imu_scaled_t packet;
	packet.Gx = Gx;
	packet.Gy = Gy;
	packet.Gz = Gz;
	packet.Ax = Ax;
	packet.Ay = Ay;
	packet.Az = Az;
	packet.Mx = Mx;
	packet.My = My;
	packet.Mz = Mz;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IMU_SCALED_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_IMU_SCALED;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_IMU_SCALED_LEN, MAVLINK_MSG_ID_IMU_SCALED_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_IMU_SCALED_LEN);
#endif
}

/**
 * @brief Encode a imu_scaled struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param imu_scaled C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_imu_scaled_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_imu_scaled_t* imu_scaled)
{
	return mavlink_msg_imu_scaled_pack(system_id, component_id, msg, imu_scaled->Gx, imu_scaled->Gy, imu_scaled->Gz, imu_scaled->Ax, imu_scaled->Ay, imu_scaled->Az, imu_scaled->Mx, imu_scaled->My, imu_scaled->Mz);
}

/**
 * @brief Encode a imu_scaled struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param imu_scaled C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_imu_scaled_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_imu_scaled_t* imu_scaled)
{
	return mavlink_msg_imu_scaled_pack_chan(system_id, component_id, chan, msg, imu_scaled->Gx, imu_scaled->Gy, imu_scaled->Gz, imu_scaled->Ax, imu_scaled->Ay, imu_scaled->Az, imu_scaled->Mx, imu_scaled->My, imu_scaled->Mz);
}

/**
 * @brief Send a imu_scaled message
 * @param chan MAVLink channel to send the message
 *
 * @param Gx The scaled Gx value in radians per second.
 * @param Gy The scaled Gy value in radians per second.
 * @param Gz The scaled Gz value in radians per second.
 * @param Ax The scaled Ax value in terms of g.
 * @param Ay The scaled Ay value in terms of g.
 * @param Az The scaled Az value in terms of g.
 * @param Mx The scaled Mx value in terms of g.
 * @param My The scaled My value in terms of g.
 * @param Mz The scaled Mz value in terms of g.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_imu_scaled_send(mavlink_channel_t chan, float Gx, float Gy, float Gz, float Ax, float Ay, float Az, float Mx, float My, float Mz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_IMU_SCALED_LEN];
	_mav_put_float(buf, 0, Gx);
	_mav_put_float(buf, 4, Gy);
	_mav_put_float(buf, 8, Gz);
	_mav_put_float(buf, 12, Ax);
	_mav_put_float(buf, 16, Ay);
	_mav_put_float(buf, 20, Az);
	_mav_put_float(buf, 24, Mx);
	_mav_put_float(buf, 28, My);
	_mav_put_float(buf, 32, Mz);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_SCALED, buf, MAVLINK_MSG_ID_IMU_SCALED_LEN, MAVLINK_MSG_ID_IMU_SCALED_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_SCALED, buf, MAVLINK_MSG_ID_IMU_SCALED_LEN);
#endif
#else
	mavlink_imu_scaled_t packet;
	packet.Gx = Gx;
	packet.Gy = Gy;
	packet.Gz = Gz;
	packet.Ax = Ax;
	packet.Ay = Ay;
	packet.Az = Az;
	packet.Mx = Mx;
	packet.My = My;
	packet.Mz = Mz;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_SCALED, (const char *)&packet, MAVLINK_MSG_ID_IMU_SCALED_LEN, MAVLINK_MSG_ID_IMU_SCALED_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_SCALED, (const char *)&packet, MAVLINK_MSG_ID_IMU_SCALED_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_IMU_SCALED_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_imu_scaled_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float Gx, float Gy, float Gz, float Ax, float Ay, float Az, float Mx, float My, float Mz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, Gx);
	_mav_put_float(buf, 4, Gy);
	_mav_put_float(buf, 8, Gz);
	_mav_put_float(buf, 12, Ax);
	_mav_put_float(buf, 16, Ay);
	_mav_put_float(buf, 20, Az);
	_mav_put_float(buf, 24, Mx);
	_mav_put_float(buf, 28, My);
	_mav_put_float(buf, 32, Mz);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_SCALED, buf, MAVLINK_MSG_ID_IMU_SCALED_LEN, MAVLINK_MSG_ID_IMU_SCALED_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_SCALED, buf, MAVLINK_MSG_ID_IMU_SCALED_LEN);
#endif
#else
	mavlink_imu_scaled_t *packet = (mavlink_imu_scaled_t *)msgbuf;
	packet->Gx = Gx;
	packet->Gy = Gy;
	packet->Gz = Gz;
	packet->Ax = Ax;
	packet->Ay = Ay;
	packet->Az = Az;
	packet->Mx = Mx;
	packet->My = My;
	packet->Mz = Mz;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_SCALED, (const char *)packet, MAVLINK_MSG_ID_IMU_SCALED_LEN, MAVLINK_MSG_ID_IMU_SCALED_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_SCALED, (const char *)packet, MAVLINK_MSG_ID_IMU_SCALED_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE IMU_SCALED UNPACKING


/**
 * @brief Get field Gx from imu_scaled message
 *
 * @return The scaled Gx value in radians per second.
 */
static inline float mavlink_msg_imu_scaled_get_Gx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field Gy from imu_scaled message
 *
 * @return The scaled Gy value in radians per second.
 */
static inline float mavlink_msg_imu_scaled_get_Gy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field Gz from imu_scaled message
 *
 * @return The scaled Gz value in radians per second.
 */
static inline float mavlink_msg_imu_scaled_get_Gz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field Ax from imu_scaled message
 *
 * @return The scaled Ax value in terms of g.
 */
static inline float mavlink_msg_imu_scaled_get_Ax(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field Ay from imu_scaled message
 *
 * @return The scaled Ay value in terms of g.
 */
static inline float mavlink_msg_imu_scaled_get_Ay(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field Az from imu_scaled message
 *
 * @return The scaled Az value in terms of g.
 */
static inline float mavlink_msg_imu_scaled_get_Az(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field Mx from imu_scaled message
 *
 * @return The scaled Mx value in terms of g.
 */
static inline float mavlink_msg_imu_scaled_get_Mx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field My from imu_scaled message
 *
 * @return The scaled My value in terms of g.
 */
static inline float mavlink_msg_imu_scaled_get_My(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field Mz from imu_scaled message
 *
 * @return The scaled Mz value in terms of g.
 */
static inline float mavlink_msg_imu_scaled_get_Mz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Decode a imu_scaled message into a struct
 *
 * @param msg The message to decode
 * @param imu_scaled C-struct to decode the message contents into
 */
static inline void mavlink_msg_imu_scaled_decode(const mavlink_message_t* msg, mavlink_imu_scaled_t* imu_scaled)
{
#if MAVLINK_NEED_BYTE_SWAP
	imu_scaled->Gx = mavlink_msg_imu_scaled_get_Gx(msg);
	imu_scaled->Gy = mavlink_msg_imu_scaled_get_Gy(msg);
	imu_scaled->Gz = mavlink_msg_imu_scaled_get_Gz(msg);
	imu_scaled->Ax = mavlink_msg_imu_scaled_get_Ax(msg);
	imu_scaled->Ay = mavlink_msg_imu_scaled_get_Ay(msg);
	imu_scaled->Az = mavlink_msg_imu_scaled_get_Az(msg);
	imu_scaled->Mx = mavlink_msg_imu_scaled_get_Mx(msg);
	imu_scaled->My = mavlink_msg_imu_scaled_get_My(msg);
	imu_scaled->Mz = mavlink_msg_imu_scaled_get_Mz(msg);
#else
	memcpy(imu_scaled, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_IMU_SCALED_LEN);
#endif
}
