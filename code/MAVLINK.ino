#define MAV_COMPONENT_ID MAV_COMP_ID_IMU
#define MAV_SYSTEM_ID 96

#define HEARTBEAT_THRESHOLD 1250
#include <mavlink.h>
int systemType = MAV_TYPE_QUADROTOR;
int autopilotType = MAV_AUTOPILOT_GENERIC;
int systemMode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
int systemStatus = MAV_STATE_UNINIT;

int parameterType=MAVLINK_TYPE_FLOAT;

mavlink_param_set_t set;

mavlink_message_t msg;

uint8_t buf[MAVLINK_MAX_PACKET_LEN];
mavlink_status_t status;
void sendHeartbeat(void)
{
  mavlink_msg_heartbeat_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, systemType, autopilotType, systemMode, 0, systemStatus);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}
void sendInformation(void)
{
  //TODO:pack stuff and ship data:
  //mavlink_msg_raw_imu_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, 0, accelRate[XAXIS], accelRate[YAXIS], accelRate[ZAXIS], gyroRate[XAXIS], gyroRate[YAXIS], gyroRate[ZAXIS], 0, 0, 0);
  //uint16_t mavlink_msg_param_value_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,const char *param_id, float param_value, uint8_t param_type, uint16_t param_count, uint16_t param_index)
  //mavlink_msg_attitude_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBoot, kinematicsAngle[XAXIS], kinematicsAngle[YAXIS], kinematicsAngle[ZAXIS], 0, 0, 0);
  //mavlink_msg_vfr_hud_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, 0.0, 0.0, ((int)(trueNorthHeading / M_PI * 180.0) + 360) % 360, (receiverData[THROTTLE]-1000)/10, getBaroAltitude(), 0.0);
  //mavlink_msg_global_position_int_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBoot, currentPosition.latitude, currentPosition.longitude, getGpsAltitude() * 10, (getGpsAltitude() - baroGroundAltitude * 100) * 10 , 0, 0, 0, ((int)(trueNorthHeading / M_PI * 180.0) + 360) % 360);
  //mavlink_msg_raw_pressure_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBoot, readRawPressure(), 0,0, readRawTemperature());
  //mavlink_msg_rc_channels_raw_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBoot, 0, receiverCommand[THROTTLE], receiverCommand[XAXIS], receiverCommand[YAXIS], receiverCommand[ZAXIS], receiverCommand[MODE], receiverCommand[AUX1], receiverCommand[AUX2], receiverCommand[AUX3], rssiRawValue * 2.55);
  //mavlink_msg_sys_status_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, controlSensorsPresent, controlSensorEnabled, controlSensorsHealthy, 0, 0, 0, 0, system_dropped_packets, 0, 0, 0, 0, 0); 
}
void receiveCommunication(void)
{
  while(Serial.available())
  {
    char c=Serial.read();
    if(mavlink_parse_char(MAVLINK_COMM_0,c,&msg,&status))
    {
      switch(msg.msgid)
      {
        case MAVLINK_MSG_ID_HEARTBEAT:
        {
          static uint16_t previousTime=millis();
          uint16_t currentTime=millis();
          if(currentTime-previousTime>HEARTBEAT_THRESHOLD);
              //TODO:broken communication mode
        }break;
        case MAVLINK_MSG_ID_COMMAND_LONG:
        {
           //TODO:Do stuff according to the command received 
        }break;
        default:
          break;
      }
    }
  }
  //numDroppedPackets+=status.packet_rx_drop_count;
}
