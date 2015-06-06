//      MAV_STATE_UNINIT=0, /* Uninitialized system, state is unknown. | */
//	MAV_STATE_BOOT=1, /* System is booting up. | */
//	MAV_STATE_CALIBRATING=2, /* System is calibrating and not flight-ready. | */
//	MAV_STATE_STANDBY=3, /* System is grounded and on standby. It can be launched any time. | */
//	MAV_STATE_ACTIVE=4, /* System is active and might be already airborne. Motors are engaged. | */
//	MAV_STATE_CRITICAL=5, /* System is in a non-normal flight mode. It can however still navigate. | */
//	MAV_STATE_EMERGENCY=6, /* System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down. | */
//	MAV_STATE_POWEROFF=7, /* System just initialized its power-down sequence, will shut down now. | */
//	MAV_STATE_ENUM_END=8, /*  | */

#define MAV_COMPONENT_ID MAV_COMP_ID_IMU
#define MAV_SYSTEM_ID 96

#define HEARTBEAT_THRESHOLD 1250

#include <mavlink.h>
#include "PID.h"


extern int16_t  frontLeftMotorCommand,frontRightMotorCommand,backLeftMotorCommand,backRightMotorCommand;
extern float angleCorrected[3];

extern float motorRollCommand,motorYawCommand,motorPitchCommand;
/*struct parameter
{
  char* parameterID;
  float* parameterValue;
  void setParameter(char* name, float* value); 
};

void parameter::setParameter(char* name, float* value)
{
  parameterID=name;
  parameterValue=value;
}
#define PARAM_LIST_SIZE 6

parameter parameters[PARAM_LIST_SIZE];

void initParameters(void)
{
  uint8_t count=0;
  parameters[count++].setParameter("GxRaw",&gyroRate[XAXIS]);
  parameters[count++].setParameter("GyRaw",&gyroRate[YAXIS]);
  parameters[count++].setParameter("GzRaw",&gyroRate[ZAXIS]);
  
  parameters[count++].setParameter("AxRaw",&accelRate[XAXIS]);
  parameters[count++].setParameter("AyRaw",&accelRate[YAXIS]);
  parameters[count++].setParameter("AzRaw",&accelRate[ZAXIS]);
}*/

#define D3_GYRO (1<<0)
#define D3_ACCEL (1<<1)
#define D3_MAGNET (1<<2)
#define D3_ANGULAR_RATE_CONTROL (1<<10)
#define YAW_POSITION (1<<12)
#define ALTITUDE_CONTROL (1<<13)
#define ATTITUDE_STABILIZE (1<<11)

#define GPS (1<<5)

#define ABSOLUTE_PRESSURE (1<<3)
#define DIFFERENTIAL_PRESSURE (1<<4)

#define MOTORS (1<<15)

const uint32_t controlSystemsPresent = D3_GYRO|D3_ACCEL|D3_ANGULAR_RATE_CONTROL|MOTORS|D3_MAGNET;
const int systemType = MAV_TYPE_QUADROTOR;
const int autopilotType = MAV_AUTOPILOT_GENERIC;
int systemMode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
int systemStatus = MAV_STATE_UNINIT;

//int parameterType=MAVLINK_TYPE_FLOAT;

//mavlink_param_set_t set;

mavlink_message_t msg;

uint8_t buf[MAVLINK_MAX_PACKET_LEN];
mavlink_status_t status;

uint16_t  sysTimeMillis=0;
uint32_t numDroppedPackets=0;

extern PID rollPID,yawPID,pitchPID;

void sendHeartbeat(void)
{
  mavlink_msg_heartbeat_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, systemType, autopilotType, systemMode, 0, systemStatus);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}
void sendInformation(void)
{
  //TODO:pack stuff and ship data:
  updateFlightTime();
  uint16_t len;
  /*mavlink_msg_raw_imu_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, micros(), accelRaw[XAXIS], accelRaw[YAXIS], accelRaw[ZAXIS], gyroRaw[XAXIS]-gyroZero[XAXIS], gyroRaw[YAXIS]-gyroZero[YAXIS], gyroRaw[ZAXIS]-gyroZero[ZAXIS], magnetRaw[XAXIS], magnetRaw[YAXIS], magnetRaw[ZAXIS]);
  len=mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);*/
  
 /* mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID,&msg,sysTimeMillis,rollPID.getSetPoint(),pitchPID.getSetPoint(),yawPID.getSetPoint(),0);
  len=mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);*/
  
  /*mavlink_msg_rc_channels_raw_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, sysTimeMillis, 0, receivers[ROLL].getDiff(), receivers[PITCH].getDiff(), receivers[THROTTLE].getDiff(), receivers[YAW].getDiff(), 0, 0,0,0, 0);
  len=mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
  
  mavlink_msg_sys_status_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, controlSystemsPresent, controlSystemsPresent, controlSystemsPresent, 0, 0, 0, 0, numDroppedPackets, 0, 0, 0, 0, 0);
  len=mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);*/
 
  mavlink_msg_attitude_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, sysTimeMillis, angleCorrected[XAXIS],angleCorrected[YAXIS], angleCorrected[ZAXIS], 0, 0, 0);
  len=mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);

  /*mavlink_msg_imu_scaled_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, gyroRate[XAXIS], gyroRate[YAXIS], gyroRate[ZAXIS], accelRate[XAXIS], accelRate[YAXIS], accelRate[ZAXIS],0,0,0);
  len=mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);*/
  
  mavlink_msg_debug_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg,sysTimeMillis,1,frontLeftMotorCommand);
  len=mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
  
  mavlink_msg_debug_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg,sysTimeMillis,2,frontRightMotorCommand);
  len=mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
  
  mavlink_msg_debug_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg,sysTimeMillis,3,backLeftMotorCommand);
  len=mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
  
  mavlink_msg_debug_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg,sysTimeMillis,4,backRightMotorCommand);
  len=mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
  
  mavlink_msg_debug_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg,sysTimeMillis,5,getThrottle());
  len=mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
  
  mavlink_msg_debug_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg,sysTimeMillis,6,(float)motorRollCommand);
  len=mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
  
  mavlink_msg_debug_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg,sysTimeMillis,7,(float)motorYawCommand);
  len=mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
  
  mavlink_msg_debug_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg,sysTimeMillis,8,(float)motorPitchCommand);
  len=mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
  
  //sendParameterList();
  //static inline uint16_t mavlink_msg_local_position_ned_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,uint32_t time_boot_ms, float x, float y, float z, float vx, float vy, float vz)
  //uint16_t mavlink_msg_param_value_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,const char *param_id, float param_value, uint8_t param_type, uint16_t param_count, uint16_t param_index)
  //mavlink_msg_vfr_hud_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, 0.0, 0.0, ((int)(trueNorthHeading / M_PI * 180.0) + 360) % 360, (receiverData[THROTTLE]-1000)/10, getBaroAltitude(), 0.0);
  //mavlink_msg_global_position_int_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBoot, currentPosition.latitude, currentPosition.longitude, getGpsAltitude() * 10, (getGpsAltitude() - baroGroundAltitude * 100) * 10 , 0, 0, 0, ((int)(trueNorthHeading / M_PI * 180.0) + 360) % 360);
  //mavlink_msg_raw_pressure_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBoot, readRawPressure(), 0,0, readRawTemperature());
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
           uint8_t command=mavlink_msg_command_long_get_command(&msg);
           uint8_t result=MAV_RESULT_UNSUPPORTED;
           //MAV_CMD_PREFLIGHT_CALIBRATION=241, /* Trigger calibration. This command will be only accepted if in pre-flight mode. |Gyro calibration: 0: no, 1: yes| Magnetometer calibration: 0: no, 1: yes| Ground pressure: 0: no, 1: yes| Radio calibration: 0: no, 1: yes| Empty| Empty| Empty|  */
           if(command== MAV_CMD_PREFLIGHT_CALIBRATION)
           {
             if(mavlink_msg_command_long_get_param1(&msg)==1.0)
             {
               calibrateGyro();
               result=MAV_RESULT_ACCEPTED;
             }
             else if(mavlink_msg_command_long_get_param2(&msg)== 1.0)
             {
                computeAccelBias();
                result=MAV_RESULT_ACCEPTED;
             }
           }
           //MAV_CMD_DO_SET_PARAMETER=180, /* Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter. |Parameter number| Parameter value| Empty| Empty| Empty| Empty| Empty|  */
           mavlink_msg_command_ack_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, command, result);
           uint16_t len= mavlink_msg_to_send_buffer(buf, &msg);
           Serial.write(buf,len);
        }break;
        /*case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        {
          sendParameterList();
        } break;
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
        {
          mavlink_param_request_read_t read;
          mavlink_msg_param_request_read_decode(&msg, &read);

          sendParameter(parameters[read.param_index].parameterID,*(parameters[read.param_index].parameterValue),read.param_index);

        } break;*/
        case MAVLINK_MSG_ID_PARAM_SET:
        {
          /*
          if(!isArmed) { 
            mavlink_msg_param_set_decode(&msg, &set);
            key = (char*) set.param_id;
            parameterMatch = findParameter(key);
            parameterChangeIndicator = 0;
          }*/
        } break;
        default:
          break;
      }
    }
  }
  numDroppedPackets+=status.packet_rx_drop_count;
}

void updateFlightTime(void)
{
  static uint32_t previousTime=0;
  uint16_t timeDiff = millis() - previousTime;
  previousTime += timeDiff;
  
  if (isArmed) 
    sysTimeMillis += timeDiff;
}
/*
void sendParameterList(void)
{
  for(uint8_t i=0;i<PARAM_LIST_SIZE;++i)
  {
    sendParameter(parameters[i].parameterID,*(parameters[i].parameterValue),i);
  }
}*/

/*void sendParameter(char* parameterID,float parameterValue,uint8_t index)
{
  mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, (char*)parameterID,parameterValue,0, PARAM_LIST_SIZE, index);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}*/
