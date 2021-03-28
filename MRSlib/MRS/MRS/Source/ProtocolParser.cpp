
#include "../Include/ProtocolParser.hpp"
#include "../Include/SIMPLE_MRS_PARSER.hpp"
namespace MRS {
	namespace Device {
		const char* ProtocolParser::GenerateFormatString(PROTOCOL_TYPE input, PROTOCOL_TYPE output, std::string arg_template, int num)
		{
			std::string output_format = "";
			//output_format += GetStartString(output);
			output_format += GetArgTypeString(input, output, arg_template, 0);
			for (int i = 1; i < num; i++) {
				output_format += GetSeperatorString(output);
				output_format += i;  //GetFormatStartString(output);
				output_format += "=";
				output_format += GetArgTypeString(input, output, arg_template, i);
				//output += //GetFormatEndString(output);
			}
			//output_format += GetEndString(output);
			return output_format.c_str();
		}

		std::string ProtocolParser::GetArgTypeString(PROTOCOL_TYPE input, PROTOCOL_TYPE output, std::string cmd_template, int id)
		{
			std::string output_str = "";
			std::string temp = GetArg(input, cmd_template, id);
			
			output_str += ArgTypeString(output, GetArgType(input, temp));
			
			return output_str;
		}
		ARG_TYPE ProtocolParser::GetArgType(PROTOCOL_TYPE input, std::string arg)
		{
			switch (input) {
			case PROTOCOL_TYPE::SIMPLE_MRS:
				// get type char or substring
				switch (arg[0]) {
				case 'i':
					return ARG_TYPE::INTEGER;
				case 'd':
					return ARG_TYPE::DOUBLE;
				case 'b':
					return ARG_TYPE::BOOLEAN;
				case 'c':
					return ARG_TYPE::CHARACTER;
				case 's':
					return ARG_TYPE::C_STRING;
				case 'o':
					return ARG_TYPE::OBJECT;
				case 'p':
					return ARG_TYPE::POINTER;
				default:
					return ARG_TYPE::CHARACTER;
				}
			case PROTOCOL_TYPE::XML:

				break;
			case PROTOCOL_TYPE::JSON:

				break;
			case PROTOCOL_TYPE::FIPA_ACL:

				break;
			}
		}
		std::string ProtocolParser::GetArg(PROTOCOL_TYPE input, std::string cmd_template, int id)
		{
			int start;
			int end;
			switch (input) {
			case PROTOCOL_TYPE::SIMPLE_MRS:
				start = find_Nth_of(cmd_template, '=', id);
				end = cmd_template.find(' ', start + 1);
				return cmd_template.substr(start, end - start - 1);
			case PROTOCOL_TYPE::XML:

				break;
			case PROTOCOL_TYPE::JSON:

				break;
			case PROTOCOL_TYPE::FIPA_ACL:

				break;
			}
		}
		std::string ProtocolParser::GetSeperatorString(PROTOCOL_TYPE output)
		{
			switch (output) {
			case PROTOCOL_TYPE::SIMPLE_MRS:
				return " ";
			case PROTOCOL_TYPE::XML:

				break;
			case PROTOCOL_TYPE::JSON:

				break;
			case PROTOCOL_TYPE::FIPA_ACL:

				break;
			}
		}

		std::string ProtocolParser::ArgTypeString(PROTOCOL_TYPE output, ARG_TYPE arg)
		{
			switch (output) {
			case PROTOCOL_TYPE::SIMPLE_MRS:
				// get type char or substring
				switch (arg) {
				case ARG_TYPE::INTEGER:
					return "i";
				case ARG_TYPE::DOUBLE:
					return "d";
				case ARG_TYPE::BOOLEAN:
					return "b";
				case ARG_TYPE::CHARACTER:
					return "c";
				case ARG_TYPE::C_STRING:
					return "s";
				case ARG_TYPE::OBJECT:
					return "o";
				case ARG_TYPE::POINTER:
					return "p";
				default:
					return "c";
				}
			case PROTOCOL_TYPE::XML:

				break;
			case PROTOCOL_TYPE::JSON:

				break;
			case PROTOCOL_TYPE::Cpp_FORMAT:
				switch (arg) {
				case ARG_TYPE::INTEGER:
					return "%d";
				case ARG_TYPE::DOUBLE:
					return "%f";
				case ARG_TYPE::BOOLEAN:
					return "BOOL";
				case ARG_TYPE::CHARACTER:
					return "%c";
				case ARG_TYPE::C_STRING:
					return "%s";
				case ARG_TYPE::OBJECT:
					return "OBJ";
				case ARG_TYPE::POINTER:
					return "%p";
				default:
					return "%p";
				}
				break;
			}
		}

		std::string ProtocolParser::ParseArgsIntoString(PROTOCOL_TYPE protocol, std::string cmd_template, std::string args)
		{
			switch (protocol) {
			case PROTOCOL_TYPE::SIMPLE_MRS:

				return SIMPLE_MRS_PARSER::ParseArgsIntoString(&cmd_template, &args);
			case PROTOCOL_TYPE::XML:

				break;
			case PROTOCOL_TYPE::JSON:

				break;
			case PROTOCOL_TYPE::FIPA_ACL:

				break;
			}
		}

		int ProtocolParser::find_Nth_of(std::string search_str, char a, int n)
		{
			int last_spot = search_str.find(a);
			if (n > 1) {
				for (int i = 0; i < n - 1; i++) {
					last_spot = search_str.find(a, last_spot + 1);
				}
			}
			return last_spot;
		}
	}
}