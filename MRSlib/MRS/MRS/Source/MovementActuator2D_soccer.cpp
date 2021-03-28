#include "../Include/MovementActuator2D_soccer.hpp"
namespace MRS {
	namespace Device {
		
		MovementActuator2D_soccer::MovementActuator2D_soccer()
		{
			SetActuatorType(ActuatorType::MOVE_2D);
		}

		MovementActuator2D_soccer::~MovementActuator2D_soccer()
		{
		}

		bool MovementActuator2D_soccer::OnDoAction(Action action)
		{
			if (action.GetActionType() == ActionType::DASH) {
				Dash((float)action.GetActionParameter(), (float)(action.GetActionParameter()>>32));
				return true;
			}
			if (action.GetActionType() == ActionType::MOVE) {
				Move((float)action.GetActionParameter(), (float)(action.GetActionParameter() >> 32));
				return true;
			}
			return false;
		}

		std::string* MovementActuator2D_soccer::toString()
		{
			return nullptr;
		}
	}
}