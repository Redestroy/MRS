#pragma once
#include "View.hpp"
#include <math.h>
#define PI 3.14159265

// this is deprecated
// while technically more efficient, it lacks trustworthyness
// would be a good place to learn unit tests though

namespace MRS {
	namespace Device {
		class DistanceView : View {
		private:
			double* angle;	// in radians
			double* distance;	// by user defined constant
			double* precision_angle;
			double* precision_distance;

			const unsigned char ANGLE_PTR_OFFSET = 0;
			const unsigned char DISTANCE_PTR_OFFSET = ANGLE_PTR_OFFSET + sizeof(double);
			const unsigned char PRECISION_A_PTR_OFFSET = DISTANCE_PTR_OFFSET + sizeof(double);
			const unsigned char PRECISION_D_PTR_OFFSET = PRECISION_A_PTR_OFFSET + sizeof(double);
		public:
			DistanceView();
			DistanceView(double distance, double direction);
			~DistanceView();

			void SetDistance(double);
			double GetDistance();

			void SetAngle(double);
			double GetAngle();

			void SetPrecisionAngle(double);
			double GetPrecisionAngle();

			void SetPrecisionDistance(double);
			double GetPrecisionDistance();

			double ToCartesianGetX();
			double ToCartesianGetY();
			double ToDegreesGetAngle();
		};
	}
}