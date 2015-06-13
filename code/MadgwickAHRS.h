                                           
//=====================================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

//----------------------------------------------------------------------------------------------------
// Variable declaration
float angleCorrected[3]={0.0,0.0,0.0};
float angle[3]={0.0,0.0,0.0};
float offset[3]={0.0,0.0,0.0};
extern volatile float beta;				// algorithm gain
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
extern float gyroRate[3];
extern float accelRate[3];
extern float magnetVal[3];
extern float filteredAccel[3];
//---------------------------------------------------------------------------------------------------
// Function declarations

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void getAngles();
void calculateAngleOffset();
void correctAngles();
void printAngles();
#endif
//=====================================================================================================
// End of file
//=====================================================================================================
