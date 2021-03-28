#pragma once
#include <string>
namespace MRS {
	namespace Device {
		enum class PROTOCOL_TYPE
		{
			// Add mostly used protocol types for parsing
			SIMPLE_MRS,
			Cpp_FORMAT,
			FIPA_ACL,
			XML,
			JSON,
			GCODE
		};

		enum class ARG_TYPE
		{
			// Add mostly used protocol types for parsing
			INTEGER,
			DOUBLE,
			BOOLEAN,
			STRING,
			C_STRING,
			POINTER,
			CHARACTER,
			OBJECT
		};

		class ProtocolParser {
		public:
			static const char* GenerateFormatString(PROTOCOL_TYPE, PROTOCOL_TYPE, std::string cmd_template, int num);
			static std::string GetArgTypeString(PROTOCOL_TYPE input, PROTOCOL_TYPE output, std::string cmd_template, int id);
			static ARG_TYPE GetArgType(PROTOCOL_TYPE input, std::string arg);
			static std::string GetArg(PROTOCOL_TYPE input, std::string cmd_template, int id);
			static std::string GetSeperatorString(PROTOCOL_TYPE input);
			static std::string ArgTypeString(PROTOCOL_TYPE output, ARG_TYPE);

			static std::string ParseArgsIntoString(PROTOCOL_TYPE protocol, std::string cmd_template, std::string args);

			static int find_Nth_of(std::string, char, int);
		};
	}
}
