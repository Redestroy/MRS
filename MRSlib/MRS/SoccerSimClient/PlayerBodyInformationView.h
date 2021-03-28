#pragma once
#include <MRS/Include/View.hpp>
#include <map>
namespace MRS {
	namespace Device {
		class PlayerBodyInformationView :
			public View
		{
		private:
			std::map<std::string, std::pair<std::string, std::string>> attributes; // parameter name, type, value

		public:
			int GetValueAsInt(std::string);
			double GetValueAsDouble(std::string);

		};
	}
}

