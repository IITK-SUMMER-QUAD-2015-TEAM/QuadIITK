// MESSAGE IMU_SCALED_FILTERED PACKING

#define MAVLINK_MSG_ID_IMU_SCALED_FILTERED 151

typedef struct __mavlink_imu_scaled_filtered_t
{
 float Gx_f; ///< The scaled and filtered Gx value in radians per second.
 float Gy_f; ///< The scaled and filtered Gy value in radians per second.
 float Gz_f; ///< The scaled and filtered Gz value in radians per second.
 float Ax_f; ///< The scaled and filtered Ax value in terms of g.
 float Ay_f; ///< The scaled and filtered Ay value in terms of g.
 float Az_f; ///< The scaled and filtered Az value in terms of g.
 float Mx_f; ///< The scaled and filtered Mx value in terms of g.
 float My_f; ///< The scaled and filtered My value in terms of g.
 float Mz_f; ///< The scaled and filtered Mz value in terms of g.
} mavlink_imu_scaled_filtered_t;

#define MAVLINK_MSG_ID_IMU_SCALED_FILTERED_LEN 36
#define MAVLINK_MSG_ID_151_LEN 36

#define MAVLINK_MSG_ID_IMU_SCALED_FILTERED_CRC 254
#define MAVLINK_MSG_ID_151_CRC 254



#define MAVLINK_MESSAGE_INFO_IMU_SCALED_FILTERED { \
	"IMU_SCALED_FILTERED", \
	9, \
	{  { "Gx_f", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_imu_scaled_filtered_t, Gx_f) }, \
         { "Gy_f", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_imu_scaled_filtered_t, Gy_f) }, \
         { "Gz_f", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_imu_scaled_filtered_t, Gz_f) }, \
         { "Ax_f", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_imu_scaled_filtered_t, Ax_f) }, \
         { "Ay_f", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_imu_scaled_filtered_t, Ay_f) }, \
         { "Az_f", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_imu_scaled_filtered_t, Az_f) }, \
         { "Mx_f", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_imu_scaled_filtered_t, Mx_f) }, \
         { "My_f", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_imu_scaled_filtered_t, My_f) }, \
         { "Mz_f", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_imu_scaled_filtered_t, Mz_f) }, \
         } \
}


/**
 * @brief Pack a imu_scaled_filtered message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param Gx_f The scaled and filtered Gx value in radians per second.
 * @param Gy_f The scaled and filtered Gy value in radians per second.
 * @param Gz_f The scaled and filtered Gz value in radians per second.
 * @param Ax_f The scaled and filtered Ax value in terms of g.
 * @param Ay_f The scaled and filtered Ay value in terms of g.
 * @param Az_f The scaled and filtered Az value in terms of g.
 * @param Mx_f The scaled and filtered Mx value in terms of g.
 * @param My_f The scaled and filtered My value in terms of g.
 * @param Mz_f The scaled and filtered Mz value in terms of g.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_imu_scaled_filtered_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float Gx_f, float Gy_f, float Gz_f, float Ax_f, float Ay_f, float Az_f, float Mx_f, float My_f, float Mz_f)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_IMU_SCALED_FILTERED_LEN];
	_mav_put_float(buf, 0, Gx_f);
	_mav_put_float(buf, 4, Gy_f);
	_mav_put_float(buf, 8, Gz_f);
	_mav_put_float(buf, 12, Ax_f);
	_mav_put_float(buf, 16, Ay_f);
	_mav_put_float(buf, 20, Az_f);
	_mav_put_float(buf, 24, Mx_f);
	_mav_put_float(buf, 28, My_f);
	_mav_put_float(buf, 32, Mz_f);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IMU_SCALED_FILTERED_LEN);
#else
	mavlink_imu_scaled_filtered_t packet;
	packet.Gx_f = Gx_f;
	packet.Gy_f = Gy_f;
	packet.Gz_f = Gz_f;
	packet.Ax_f = Ax_f;
	packet.Ay_f = Ay_f;
	packet.Az_f = Az_f;
	packet.Mx_f = Mx_f;
	packet.My_f = My_f;
	packet.Mz_f = Mz_f;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IMU_SCALED_FILTERED_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_IMU_SCALED_FILTERED;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_IMU_SCALED_FILTERED_LEN, MAVLINK_MSG_ID_IMU_SCALED_FILTERED_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_IMU_SCALED_FILTERED_LEN);
#endif
}

/**
 * @brief Pack a imu_scaled_filtered message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param Gx_f The scaled and filtered Gx value in radians per second.
 * @param Gy_f The scaled and filtered Gy value in radians per second.
 * @param Gz_f The scaled and filtered Gz value in radians per second.
 * @param Ax_f The scaled and filtered Ax value in terms of g.
 * @param Ay_f The scaled and filtered Ay value in terms of g.
 * @param Az_f The scaled and filtered Az value in terms of g.
 * @param Mx_f The scaled and filtered Mx value in terms of g.
 * @param My_f The scaled and filtered My value in terms of g.
 * @param Mz_f The scaled and filtered Mz value in terms of g.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_imu_scaled_filtered_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float Gx_f,float Gy_f,float Gz_f,float Ax_f,float Ay_f,float Az_f,float Mx_f,float My_f,float Mz_f)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_IMU_SCALED_FILTERED_LEN];
	_mav_put_float(buf, 0, Gx_f);
	_mav_put_float(buf, 4, Gy_f);
	_mav_put_float(buf, 8, Gz_f);
	_mav_put_float(buf, 12, Ax_f);
	_mav_put_float(buf, 16, Ay_f);
	_mav_put_float(buf, 20, Az_f);
	_mav_put_float(buf, 24, Mx_f);
	_mav_put_float(buf, 28, My_f);
	_mav_put_float(buf, 32, Mz_f);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IMU_SCALED_FILTERED_LEN);
#else
	mavlink_imu_scaled_filtered_t packet;
	packet.Gx_f = Gx_f;
	packet.Gy_f = Gy_f;
	packet.Gz_f = Gz_f;
	packet.Ax_f = Ax_f;
	packet.Ay_f = Ay_f;
	packet.Az_f = Az_f;
	packet.Mx_f = Mx_f;
	packet.My_f = My_f;
	packet.Mz_f = Mz_f;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IMU_SCALED_FILTERED_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_IMU_SCALED_FILTERED;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_IMU_SCALED_FILTERED_LEN, MAVLINK_MSG_ID_IMU_SCALED_FILTERED_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_IMU_SCALED_FILTERED_LEN);
#endif
}

/**
 * @brief Encode a imu_scaled_filtered struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param imu_scaled_filtered C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_imu_scaled_filtered_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_imu_scaled_filtered_t* imu_scaled_filtered)
{
	return mavlink_msg_imu_scaled_filtered_pack(system_id, component_id, msg, imu_scaled_filtered->Gx_f, imu_scaled_filtered->Gy_f, imu_scaled_filtered->Gz_f, imu_scaled_filtered->Ax_f, imu_scaled_filtered->Ay_f, imu_scaled_filtered->Az_f, imu_scaled_filtered->Mx_f, imu_scaled_filtered->My_f, imu_scaled_filtered->Mz_f);
}

/**
 * @brief Encode a imu_scaled_filtered struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param imu_scaled_filtered C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_imu_scaled_filtered_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_imu_scaled_filtered_t* imu_scaled_filtered)
{
	return mavlink_msg_imu_scaled_filtered_pack_chan(system_id, component_id, chan, msg, imu_scaled_filtered->Gx_f, imu_scaled_filtered->Gy_f, imu_scaled_filtered->Gz_f, imu_scaled_filtered->Ax_f, imu_scaled_filtered->Ay_f, imu_scaled_filtered->Az_f, imu_scaled_filtered->Mx_f, imu_scaled_filtered->My_f, imu_scaled_filtered->Mz_f);
}

/**
 * @brief Send a imu_scaled_filtered message
 * @param chan MAVLink channel to send the message
 *
 * @param Gx_f The scaled and filtered Gx value in radians per second.
 * @param Gy_f The scaled and filtered Gy value in radians per second.
 * @param Gz_f The scaled and filtered Gz value in radians per second.
 * @param Ax_f The scaled and filtered Ax value in terms of g.
 * @param Ay_f The scaled and filtered Ay value in terms of g.
 * @param Az_f The scaled and filtered Az value in terms of g.
 * @param Mx_f The scaled and filtered Mx value in terms of g.
 * @param My_f The scaled and filtered My value in terms of g.
 * @param Mz_f The scaled and filtered Mz value in terms of g.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_imu_scaled_filtered_send(mavlink_channel_t chan, float Gx_f, float Gy_f, float Gz_f, float Ax_f, float Ay_f, float Az_f, float Mx_f, float My_f, float Mz_f)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_IMU_SCALED_FILTERED_LEN];
	_mav_put_float(buf, 0, Gx_f);
	_mav_put_float(buf, 4, Gy_f);
	_mav_put_float(buf, 8, Gz_f);
	_mav_put_float(buf, 12, Ax_f);
	_mav_put_float(buf, 16, Ay_f);
	_mav_put_float(buf, 20, Az_f);
	_mav_put_float(buf, 24, Mx_f);
	_mav_put_float(buf, 28, My_f);
	_mav_put_float(buf, 32, Mz_f);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_SCALED_FILTERED, buf, MAVLINK_MSG_ID_IMU_SCALED_FILTERED_LEN, MAVLINK_MSG_ID_IMU_SCALED_FILTERED_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_SCALED_FILTERED, buf, MAVLINK_MSG_ID_IMU_SCALED_FILTERED_LEN);
#endif
#else
	mavlink_imu_scaled_filtered_t packet;
	packet.Gx_f = Gx_f;
	packet.Gy_f = Gy_f;
	packet.Gz_f = Gz_f;
	packet.Ax_f = Ax_f;
	packet.Ay_f = Ay_f;
	packet.Az_f = Az_f;
	packet.Mx_f = Mx_f;
	packet.My_f = My_f;
	packet.Mz_f = Mz_f;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_SCALED_FILTERED, (const char *)&packet, MAVLINK_MSG_ID_IMU_SCALED_FILTERED_LEN, MAVLINK_MSG_ID_IMU_SCALED_FILTERED_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_SCALED_FILTERED, (const char *)&packet, MAVLINK_MSG_ID_IMU_SCALED_FILTERED_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_IMU_SCALED_FILTERED_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_imu_scaled_filtered_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float Gx_f, float Gy_f, float Gz_f, float Ax_f, float Ay_f, float Az_f, float Mx_f, float My_f, float Mz_f)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, Gx_f);
	_mav_put_float(buf, 4, Gy_f);
	_mav_put_float(buf, 8, Gz_f);
	_mav_put_float(buf, 12, Ax_f);
	_mav_put_float(buf, 16, Ay_f);
	_mav_put_float(buf, 20, Az_f);
	_mav_put_float(buf, 24, Mx_f);
	_mav_put_float(buf, 28, My_f);
	_mav_put_float(buf, 32, Mz_f);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_SCALED_FILTERED, buf, MAVLINK_MSG_ID_IMU_SCALED_FILTERED_LEN, MAVLINK_MSG_ID_IMU_SCALED_FILTERED_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_SCALED_FILTERED, buf, MAVLINK_MSG_ID_IMU_SCALED_FILTERED_LEN);
#endif
#else
	mavlink_imu_scaled_filtered_t *packet = (mavlink_imu_scaled_filtered_t *)msgbuf;
	packet->Gx_f = Gx_f;
	packet->Gy_f = Gy_f;
	packet->Gz_f = Gz_f;
	packet->Ax_f = Ax_f;
	packet->Ay_f = Ay_f;
	packet->Az_f = Az_f;
	packet->Mx_f = Mx_f;
	packet->My_f = My_f;
	packet->Mz_f = Mz_f;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_SCALED_FILTERED, (const char *)packet, MAVLINK_MSG_ID_IMU_SCALED_FILTERED_LEN, MAVLINK_MSG_ID_IMU_SCALED_FILTERED_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_SCALED_FILTERED, (const char *)packet, MAVLINK_MSG_ID_IMU_SCALED_FILTERED_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE IMU_SCALED_FILTERED UNPACKING


/**
 * @brief Get field Gx_f from imu_scaled_filtered message
 *
 * @return The scaled and filtered Gx value in radians per second.
 */
static inline float mavlink_msg_imu_scaled_filtered_get_Gx_f(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field Gy_f from imu_scaled_filtered message
 *
 * @return The scaled and filtered Gy value in radians per second.
 */
static inline float mavlink_msg_imu_scaled_filtered_get_Gy_f(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field Gz_f from imu_scaled_filtered message
 *
 * @return The scaled and filtered Gz value in radians per second.
 */
static inline float mavlink_msg_imu_scaled_filtered_get_Gz_f(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field Ax_f from imu_scaled_filtered message
 *
 * @return The scaled and filtered Ax value in terms of g.
 */
static inline float mavlink_msg_imu_scaled_filtered_get_Ax_f(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field Ay_f from imu_scaled_filtered message
 *
 * @return The scaled and filtered Ay value in terms of g.
 */
static inline float mavlink_msg_imu_scaled_filtered_get_Ay_f(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field Az_f from imu_scaled_filtered message
 *
 * @return The scaled and filtered Az value in terms of g.
 */
static inline float mavlink_msg_imu_scaled_filtered_get_Az_f(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field Mx_f from imu_scaled_filtered message
 *
 * @return The scaled and filtered Mx value in terms of g.
 */
static inline float mavlink_msg_imu_scaled_filtered_get_Mx_f(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field My_f from imu_scaled_filtered message
 *
 * @return The scaled and filtered My value in terms of g.
 */
static inline float mavlink_msg_imu_scaled_filtered_get_My_f(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field Mz_f from imu_scaled_filtered message
 *
 * @return The scaled and filtered Mz value in terms of g.
 */
static inline float mavlink_msg_imu_scaled_filtered_get_Mz_f(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Decode a imu_scaled_filtered message into a struct
 *
 * @param msg The message to decode
 * @param imu_scaled_filtered C-struct to decode the message contents into
 */
static inline void mavlink_msg_imu_scaled_filtered_decode(const mavlink_message_t* msg, mavlink_imu_scaled_filtered_t* imu_scaled_filtered)
{
#if MAVLINK_NEED_BYTE_SWAP
	imu_scaled_filtered->Gx_f = mavlink_msg_imu_scaled_filtered_get_Gx_f(msg);
	imu_scaled_filtered->Gy_f = mavlink_msg_imu_scaled_filtered_get_Gy_f(msg);
	imu_scaled_filtered->Gz_f = mavlink_msg_imu_scaled_filtered_get_Gz_f(msg);
	imu_scaled_filtered->Ax_f = mavlink_msg_imu_scaled_filtered_get_Ax_f(msg);
	imu_scaled_filtered->Ay_f = mavlink_msg_imu_scaled_filtered_get_Ay_f(msg);
	imu_scaled_filtered->Az_f = mavlink_msg_imu_scaled_filtered_get_Az_f(msg);
	imu_scaled_filtered->Mx_f = mavlink_msg_imu_scaled_filtered_get_Mx_f(msg);
	imu_scaled_filtered->My_f = mavlink_msg_imu_scaled_filtered_get_My_f(msg);
	imu_scaled_filtered->Mz_f = mavlink_msg_imu_scaled_filtered_get_Mz_f(msg);
#else
	memcpy(imu_scaled_filtered, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_IMU_SCALED_FILTERED_LEN);
#endif
}
