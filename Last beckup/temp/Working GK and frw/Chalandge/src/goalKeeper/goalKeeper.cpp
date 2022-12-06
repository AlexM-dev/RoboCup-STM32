#include "goalKeeper.h"

GoalKeeper::GoalKeeper(robotMotion &motion,
							 mathematics &mainMath,
							 /*buttonsModule &buttons,*/
							 ballDetection &ballDetect,
							 Gyroscope &gyro,
							 Camera &cam,
							 lowerBoard &lowBoard,
							 Pin &dribblerPin):m_motion(motion),
																				 m_mainMath(mainMath),
																				 /*m_buttons(buttons),*/
																				 m_ballDetect(ballDetect),
																				 m_gyro(gyro),
																				 m_cam(cam),
																				 m_lowBoard(lowBoard),
																				 m_dribbler(dribblerPin)
{
	
}
///////////////////////////////////////////////
//**************goal keeper code***************
void GoalKeeper::start()
{
	if(powerIR > 14 && startOp == false)
	{
		savedAngle = m_gyro.getCurAngle();
		checkAngle = m_gyro.getCurAngle() - 45;
		if(checkAngle <= 0)checkAngle += 360;
		startOp = true;
	}
	if(startOp == true)
	{
		angle = m_gyro.getCurAngle();
	}
	if(startOp == true)
	{
		int32_t corrAngle = angle - checkAngle;
		if(corrAngle > 45)corrAngle = 45;
		if(corrAngle < -180)corrAngle += 360;
		if(corrAngle < 0 && corrAngle > - 90)
		{
			startOp = false;
		}
		else
		{
			dribblingValue = (500 / 45) * corrAngle + 1000; 
			if(dribblingValue > 1000 && dribblingValue <= 2000)m_dribbler.pwm(dribblingValue);
		}
	}
}
//*********************************************
