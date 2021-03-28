#include "../Include/SIMPLE_MRS_PARSER.hpp"
#include <iostream>
namespace MRS {
	namespace Device {
        SIMPLE_MRS_PARSER::SIMPLE_MRS_PARSER()
        {
        }
        SIMPLE_MRS_PARSER::~SIMPLE_MRS_PARSER()
        {
        }
        std::string SIMPLE_MRS_PARSER::ParseArgsIntoString(std::string * templ, std::string* arg)
		{
            // count necessary args 
            int ARG_COUNT_TEMPLATE = 0;
            int last_ArgInd = 0;
            while (templ->find("Arg_", last_ArgInd) != std::string::npos) {
                last_ArgInd = templ->find("Arg_", last_ArgInd);
                if ((*templ)[last_ArgInd - 1] != '[') ARG_COUNT_TEMPLATE++;
                last_ArgInd += std::strlen("Arg_");

            }
            // count given values
            int ARG_COUNT_VALUES = 0;
            last_ArgInd = 0;
            while (arg->find("=", last_ArgInd) != std::string::npos) {
                last_ArgInd = arg->find("=", last_ArgInd);
                last_ArgInd++;
                ARG_COUNT_VALUES++;
            }

            if (ARG_COUNT_TEMPLATE > ARG_COUNT_VALUES) return "";
            std::string result = "" + *templ;
            if (ARG_COUNT_TEMPLATE == 0 && ARG_COUNT_VALUES == 0) return result;
            std::string substr = "";
            int arg_num = 0;
            int replaced = 0;
            last_ArgInd = arg->find('=');
            int last_ArgEndInd = arg->find(' ', last_ArgInd + 1);
            while (result.find("Arg_") != std::string::npos) {
                substr = arg->substr(last_ArgInd + 1, last_ArgEndInd - last_ArgInd - 1);
                last_ArgInd = arg->find('=', last_ArgEndInd + 1);
                last_ArgEndInd = arg->find_first_of(" \r\n", last_ArgInd + 1);
                if (replaced < ARG_COUNT_VALUES) {
                    result.replace(result.find("Arg_"), result.find_first_of(" )", result.find("Arg_") + 1) - result.find("Arg_"), substr);
                    replaced++;
                }
                else {
                    // replace [] with empty
                    result.replace(result.find("[Arg_") - 1, result.find_first_of(" ])", result.find("[Arg_") + 1) - result.find("[Arg_") + 1, "");
                }
                std::cout << "Result: " << result << "\n";
            }
            while (result.find_first_of("[]") != std::string::npos) {
                result.replace(result.find_first_of("[]"), 1, "");
            }
            while (result.find_first_of("\"") != std::string::npos) {
                result.replace(result.find_first_of("\""), 1, "");
            }
            return result;
		}
	}
}