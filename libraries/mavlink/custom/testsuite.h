/** @file
 *	@brief MAVLink comm protocol testsuite generated from custom.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef CUSTOM_TESTSUITE_H
#define CUSTOM_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_custom(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_common(system_id, component_id, last_msg);
	mavlink_test_custom(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_imu_scaled(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_imu_scaled_t packet_in = {
		17.0,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0
    };
	mavlink_imu_scaled_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.Gx = packet_in.Gx;
        	packet1.Gy = packet_in.Gy;
        	packet1.Gz = packet_in.Gz;
        	packet1.Ax = packet_in.Ax;
        	packet1.Ay = packet_in.Ay;
        	packet1.Az = packet_in.Az;
        	packet1.Mx = packet_in.Mx;
        	packet1.My = packet_in.My;
        	packet1.Mz = packet_in.Mz;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_imu_scaled_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_imu_scaled_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_imu_scaled_pack(system_id, component_id, &msg , packet1.Gx , packet1.Gy , packet1.Gz , packet1.Ax , packet1.Ay , packet1.Az , packet1.Mx , packet1.My , packet1.Mz );
	mavlink_msg_imu_scaled_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_imu_scaled_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.Gx , packet1.Gy , packet1.Gz , packet1.Ax , packet1.Ay , packet1.Az , packet1.Mx , packet1.My , packet1.Mz );
	mavlink_msg_imu_scaled_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_imu_scaled_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_imu_scaled_send(MAVLINK_COMM_1 , packet1.Gx , packet1.Gy , packet1.Gz , packet1.Ax , packet1.Ay , packet1.Az , packet1.Mx , packet1.My , packet1.Mz );
	mavlink_msg_imu_scaled_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_imu_scaled_filtered(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_imu_scaled_filtered_t packet_in = {
		17.0,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0
    };
	mavlink_imu_scaled_filtered_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.Gx_f = packet_in.Gx_f;
        	packet1.Gy_f = packet_in.Gy_f;
        	packet1.Gz_f = packet_in.Gz_f;
        	packet1.Ax_f = packet_in.Ax_f;
        	packet1.Ay_f = packet_in.Ay_f;
        	packet1.Az_f = packet_in.Az_f;
        	packet1.Mx_f = packet_in.Mx_f;
        	packet1.My_f = packet_in.My_f;
        	packet1.Mz_f = packet_in.Mz_f;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_imu_scaled_filtered_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_imu_scaled_filtered_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_imu_scaled_filtered_pack(system_id, component_id, &msg , packet1.Gx_f , packet1.Gy_f , packet1.Gz_f , packet1.Ax_f , packet1.Ay_f , packet1.Az_f , packet1.Mx_f , packet1.My_f , packet1.Mz_f );
	mavlink_msg_imu_scaled_filtered_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_imu_scaled_filtered_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.Gx_f , packet1.Gy_f , packet1.Gz_f , packet1.Ax_f , packet1.Ay_f , packet1.Az_f , packet1.Mx_f , packet1.My_f , packet1.Mz_f );
	mavlink_msg_imu_scaled_filtered_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_imu_scaled_filtered_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_imu_scaled_filtered_send(MAVLINK_COMM_1 , packet1.Gx_f , packet1.Gy_f , packet1.Gz_f , packet1.Ax_f , packet1.Ay_f , packet1.Az_f , packet1.Mx_f , packet1.My_f , packet1.Mz_f );
	mavlink_msg_imu_scaled_filtered_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_pid_outputs(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_pid_outputs_t packet_in = {
		17.0,45.0,73.0,101.0,129.0,157.0,185.0
    };
	mavlink_pid_outputs_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.rollRateMotorCommand = packet_in.rollRateMotorCommand;
        	packet1.pitchRateMotorCommand = packet_in.pitchRateMotorCommand;
        	packet1.yawRateMotorCommand = packet_in.yawRateMotorCommand;
        	packet1.rollMotorCommand = packet_in.rollMotorCommand;
        	packet1.pitchMotorCommand = packet_in.pitchMotorCommand;
        	packet1.yawMotorCommand = packet_in.yawMotorCommand;
        	packet1.thrustMotorCommand = packet_in.thrustMotorCommand;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pid_outputs_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_pid_outputs_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pid_outputs_pack(system_id, component_id, &msg , packet1.rollRateMotorCommand , packet1.pitchRateMotorCommand , packet1.yawRateMotorCommand , packet1.rollMotorCommand , packet1.pitchMotorCommand , packet1.yawMotorCommand , packet1.thrustMotorCommand );
	mavlink_msg_pid_outputs_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pid_outputs_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.rollRateMotorCommand , packet1.pitchRateMotorCommand , packet1.yawRateMotorCommand , packet1.rollMotorCommand , packet1.pitchMotorCommand , packet1.yawMotorCommand , packet1.thrustMotorCommand );
	mavlink_msg_pid_outputs_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_pid_outputs_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pid_outputs_send(MAVLINK_COMM_1 , packet1.rollRateMotorCommand , packet1.pitchRateMotorCommand , packet1.yawRateMotorCommand , packet1.rollMotorCommand , packet1.pitchMotorCommand , packet1.yawMotorCommand , packet1.thrustMotorCommand );
	mavlink_msg_pid_outputs_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_barometer_outputs(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_barometer_outputs_t packet_in = {
		17.0,45.0,73.0
    };
	mavlink_barometer_outputs_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.pressureValue = packet_in.pressureValue;
        	packet1.temperatureValue = packet_in.temperatureValue;
        	packet1.altitudeValue = packet_in.altitudeValue;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_barometer_outputs_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_barometer_outputs_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_barometer_outputs_pack(system_id, component_id, &msg , packet1.pressureValue , packet1.temperatureValue , packet1.altitudeValue );
	mavlink_msg_barometer_outputs_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_barometer_outputs_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.pressureValue , packet1.temperatureValue , packet1.altitudeValue );
	mavlink_msg_barometer_outputs_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_barometer_outputs_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_barometer_outputs_send(MAVLINK_COMM_1 , packet1.pressureValue , packet1.temperatureValue , packet1.altitudeValue );
	mavlink_msg_barometer_outputs_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_motor_command(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_motor_command_t packet_in = {
		17235,17339,17443,17547
    };
	mavlink_motor_command_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.LFValue = packet_in.LFValue;
        	packet1.RFValue = packet_in.RFValue;
        	packet1.BFValue = packet_in.BFValue;
        	packet1.BRValue = packet_in.BRValue;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_motor_command_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_motor_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_motor_command_pack(system_id, component_id, &msg , packet1.LFValue , packet1.RFValue , packet1.BFValue , packet1.BRValue );
	mavlink_msg_motor_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_motor_command_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.LFValue , packet1.RFValue , packet1.BFValue , packet1.BRValue );
	mavlink_msg_motor_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_motor_command_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_motor_command_send(MAVLINK_COMM_1 , packet1.LFValue , packet1.RFValue , packet1.BFValue , packet1.BRValue );
	mavlink_msg_motor_command_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_pid_setpoints(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_pid_setpoints_t packet_in = {
		17.0,45.0,73.0,101.0,129.0,157.0
    };
	mavlink_pid_setpoints_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.rollRateSetpoint = packet_in.rollRateSetpoint;
        	packet1.pitchRateSetpoint = packet_in.pitchRateSetpoint;
        	packet1.yawRateSetpoint = packet_in.yawRateSetpoint;
        	packet1.rollSetpoint = packet_in.rollSetpoint;
        	packet1.pitchSetpoint = packet_in.pitchSetpoint;
        	packet1.yawSetpoint = packet_in.yawSetpoint;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pid_setpoints_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_pid_setpoints_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pid_setpoints_pack(system_id, component_id, &msg , packet1.rollRateSetpoint , packet1.pitchRateSetpoint , packet1.yawRateSetpoint , packet1.rollSetpoint , packet1.pitchSetpoint , packet1.yawSetpoint );
	mavlink_msg_pid_setpoints_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pid_setpoints_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.rollRateSetpoint , packet1.pitchRateSetpoint , packet1.yawRateSetpoint , packet1.rollSetpoint , packet1.pitchSetpoint , packet1.yawSetpoint );
	mavlink_msg_pid_setpoints_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_pid_setpoints_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pid_setpoints_send(MAVLINK_COMM_1 , packet1.rollRateSetpoint , packet1.pitchRateSetpoint , packet1.yawRateSetpoint , packet1.rollSetpoint , packet1.pitchSetpoint , packet1.yawSetpoint );
	mavlink_msg_pid_setpoints_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_custom(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_imu_scaled(system_id, component_id, last_msg);
	mavlink_test_imu_scaled_filtered(system_id, component_id, last_msg);
	mavlink_test_pid_outputs(system_id, component_id, last_msg);
	mavlink_test_barometer_outputs(system_id, component_id, last_msg);
	mavlink_test_motor_command(system_id, component_id, last_msg);
	mavlink_test_pid_setpoints(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // CUSTOM_TESTSUITE_H
