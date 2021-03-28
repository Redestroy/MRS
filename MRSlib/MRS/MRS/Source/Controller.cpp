#include "../Include/Controller.hpp"
//#include "..\Include\Controller.hpp"
#pragma warning(disable : 4996)
#define _CRT_SECURE_NO_WARNINGS
namespace MRS {
	namespace Device {
		void Controller::SendCommand(std::string command_template, std::string arg_type_template, int num, ...)
		{
			va_list valist;
			va_start(valist, num);
			char buffer[1024];
			vsprintf(buffer, ProtocolParser::GenerateFormatString(protocol, PROTOCOL_TYPE::Cpp_FORMAT, arg_type_template, num), valist);
			std::string command_args = "";
			command_args += buffer;
			SendCommand(command_template, command_args);
			va_end(valist);
		}

		void Controller::SetProtocol(PROTOCOL_TYPE protocol)
		{
			this->protocol = protocol;
		}

		PROTOCOL_TYPE Controller::GetProtocol()
		{
			return protocol;
		}
	}
}