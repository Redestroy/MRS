#pragma once
#include <string>
#include <stdarg.h>
#include "ProtocolParser.hpp"

//#include "pch.h"
namespace MRS {
	namespace Device {
		class Controller {
		private:
			PROTOCOL_TYPE protocol;
		public:
			virtual void SendCommand(std::string command_template, std::string args) = 0;
			void SendCommand(std::string command_template, std::string arg_type_template, int, ...);
			void SetProtocol(PROTOCOL_TYPE);
			PROTOCOL_TYPE GetProtocol();
		};
	}
}