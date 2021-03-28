#pragma once
#include <deque>
#include <list>
#include <MRS/Include/View.hpp>
#include "Particle.h"
namespace MRS {
	class FastSLAM
	{
	private:
		std::deque<Particle*> particles;
		MRS::Device::View2DPosition origin;
		double current_time;
		//PointMap2D
	public:
		FastSLAM();
		~FastSLAM();
		void Update(double delta_time, MRS::Device::View * view);
		void OutputMap();
		std::string OutputMapString();
		static int GetIndex(int len, double value,double * borders);
		static double Sum(int len, double* values);
		static void VKF(Particle* p, MRS::Device::View2DPosition* object, MRS::Device::DistanceView * sensor_reading,double* pos, double* prob);
	};
}

