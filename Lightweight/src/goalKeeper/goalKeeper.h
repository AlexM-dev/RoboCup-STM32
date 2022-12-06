#include <project_config.h>
#include <motion_control.h>
#include <mathematics.h>
#include <buttonsModule.h>
#include <camera.h>
#include <lower_board.h>
#include <ballDetection.h>
#include <time_service.h>
class GoalKeeper
{
	public:
		GoalKeeper(robotMotion &motion,
							 mathematics &mainMath,
							 //buttonsModule &buttons,
							 ballDetection &ballDetect,
							 mpu9250 &gyro,
							 Camera &cam,
							 lowerBoard &lowBoard,
							 Pin &dribblerPin);
		void init();
		void start();
		
	private:
		robotMotion m_motion;
		mathematics m_mainMath;
//		buttonsModule m_buttons;
		mpu9250 m_gyro;
		Camera m_cam;
		lowerBoard m_lowBoard;
		uint8_t curTactic;
		volatile int16_t angleIR;
		volatile int16_t powerIR;
		volatile int16_t angle;
		bool startOp;
		volatile int32_t dribblingValue;
		volatile int16_t checkAngle;
		volatile int16_t savedAngle;
		volatile int32_t kek;
		volatile int16_t camValue;
		Pin m_dribbler;
		ballDetection m_ballDetect;
};
