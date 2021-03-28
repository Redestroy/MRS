#pragma once
#include "Actuator.hpp"
#include <string>
namespace MRS {
	namespace Device {
		class MovementActuator2D_soccer : public Actuator
		{
		public:
			MovementActuator2D_soccer();
			~MovementActuator2D_soccer();

			virtual bool OnDoAction(Action);
			virtual void Dash(double power, double angle) = 0;
			virtual void Move(double coords_x, double coords_y) = 0;
			std::string * toString();
		private:

		};



	}
}