#include "View.hpp"
#include <string>
#include <map>

namespace MRS {
	namespace Device {
		class InformationView : public View
		{
		private:
			// contains a map that could be later converted into a struct (through c++ magic if possible) - although it is a good question for what the struct could be used
			std::map<std::string, std::string> parameters;
			//struct info;
		public:
			InformationView();
			~InformationView();

			void AddParameter(std::string identifier, std::string value);

			std::string GetValue(std::string identifier, std::string default_value = "NO_VAL");

		};
	}
}