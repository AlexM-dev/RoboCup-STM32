//#include "strategy.h"
//#define TACTIC1 1
//#define TACTIC2 2

//#define MOTOR1 0
//#define MOTOR2 1
//#define MOTOR3 2
//#define MOTOR4 3

//Strategy::Strategy(robotMotion &motion,
//									 mathematics &mainMath,
//									 /*buttonsModule &buttons,*/
//									 ballDetection &ballDetect,
//									 Gyroscope &gyro,
//									 Camera &cam,
//									 lowerBoard &lowBoard):m_motion(motion),
//																				 m_mainMath(mainMath),
//																				 /*m_buttons(buttons),*/
//																				 m_ballDetect(ballDetect),
//																				 m_gyro(gyro),
//																				 m_cam(cam),
//																				 m_lowBoard(lowBoard)
//{
//	curTactic = TACTIC1;
//	m_motion.robotInit();
//	time_service::init();
//}
//void Strategy::start()
//{
//	int32_t sp1 = 0, sp2 = 0, sp3 = 0, sp4 = 0;
////	int16_t data = 0;
//	int32_t impact = 0;
//	bool avoidOp = false;
//	time_service::startTime();
//	if (curTactic == TACTIC1)
//	{
//		m_gyro.setAngle(0);
//		m_cam.readData();
////		if (m_cam.get() == 255 && m_ballDetect.isThereABall())
////		{
////			//data = Uart::read();
////				avoidOp = true;
////		}
//		if (m_lowBoard.getValue() == 0)
//		{
//			if (avoidOp == true)
//			{
//				time_service::delay_ms(1000);
//				avoidOp = false;	
//				m_gyro.setAngle(180);
//				sp1 = m_mainMath.calculateSpeed(m_gyro.getCurAngle()+90, 1, MOTOR1);
//				sp2 = m_mainMath.calculateSpeed(m_gyro.getCurAngle()+90, 1, MOTOR2);
//				sp3 = m_mainMath.calculateSpeed(m_gyro.getCurAngle()+90, 1, MOTOR3);
//				sp4 = m_mainMath.calculateSpeed(m_gyro.getCurAngle()+90, 1, MOTOR4);
//			}
//			else
//			{
//				impact = m_gyro.getImpact();
//				sp1 = m_mainMath.calculateSpeed(m_mainMath.calculateAngle(), 1, MOTOR1);
//				sp2 = m_mainMath.calculateSpeed(m_mainMath.calculateAngle(), 1, MOTOR2);
//				sp3 = m_mainMath.calculateSpeed(m_mainMath.calculateAngle(), 1, MOTOR3);
//				sp4 = m_mainMath.calculateSpeed(m_mainMath.calculateAngle(), 1, MOTOR4);		
//			}
//		}
//		else 
//		{
//				sp1 = 0;
//				sp2 = 0;
//				sp3 = 0; 
//				sp4 = 0;
//		}
//		
//		m_motion.move(sp1 + impact,sp2 + impact,sp3 + impact,sp4 + impact);
//	}
//	if(curTactic == TACTIC2)
//	{
//		
//	}
//	
//}
