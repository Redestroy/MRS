#pragma once
#include "UDPController.h"
#include "SoccerServerCommandDict.h"
#include <MRS/Include/MovementActuator2D_soccer.hpp>
namespace MRS {
	namespace Device {
		class MovementActuator2D_UDPController : public MovementActuator2D_soccer
		{
		private:
			UDPController* udpController_ptr;
			std::string command_template;
			std::string args_template;
			SoccerServerCommandDict * cmd_dict;
			std::deque<std::string> args_dash, args_move;
		public:
			MovementActuator2D_UDPController();
			MovementActuator2D_UDPController(UDPController*);
			MovementActuator2D_UDPController(UDPController*, std::string cmd_format);
			~MovementActuator2D_UDPController();
			void SetUDPController(UDPController*);
			UDPController* GetUDPController();

			void SetCommandDict(SoccerServerCommandDict*);
			SoccerServerCommandDict* GetCommandDict();

			virtual void Dash(double power, double angle);
			virtual void Move(double coords_x, double coords_y);
		};
	}
}
