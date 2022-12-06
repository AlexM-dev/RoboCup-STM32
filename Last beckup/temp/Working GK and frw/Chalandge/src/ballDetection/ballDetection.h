#pragma once
#include <project_config.h>
#include <dma.h>
class ballDetection
{
		public:
			ballDetection(Dma &dmaBall);
			bool isThereABall();
		private:
			Dma m_dmaBall;
};
