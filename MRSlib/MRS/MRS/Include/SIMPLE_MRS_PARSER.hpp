#pragma once
#include <string>
#include <cstring>
namespace MRS {
	namespace Device {
		class SIMPLE_MRS_PARSER //: Protocol 
		{
		public:
			SIMPLE_MRS_PARSER ();
			~SIMPLE_MRS_PARSER ();
			static std::string ParseArgsIntoString(std::string* templ, std::string* arg);
		private:
			
		};
	}
}