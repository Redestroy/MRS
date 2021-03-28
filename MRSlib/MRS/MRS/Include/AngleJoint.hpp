#pragma once
#include "FeedbackPair.hpp"
#include <deque>			//might need to implament seperatly
namespace MRS {
	namespace Device {
		class __declspec(dllexport) AngleJoint : public FeedbackPair {
		private:

		public:
			AngleJoint();
			virtual void SetAngle(float angle) = 0; // Angle in degrees
			virtual void SetAngleRadians(float angle) = 0; // Angle in Radians
			virtual float ReadAngle() = 0;
			virtual void SetRotationalSpeed() = 0;
			virtual void SetMaxTorque() = 0;
			virtual void SetAngleLimits(float min, float max) = 0;
		};
	}
}