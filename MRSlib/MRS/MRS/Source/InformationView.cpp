#include "../Include/InformationView.hpp"

namespace MRS {
	namespace Device {
		InformationView::InformationView() : View(MRS::Device::ViewType::VIEW_TEXT), parameters()
		{
			
		}

		InformationView::~InformationView()
		{
		}

		void InformationView::AddParameter(std::string identifier, std::string value)
		{
			parameters.emplace(identifier, value);
		}

		std::string InformationView::GetValue(std::string identifier, std::string default_value)
		{
			std::string temp = "";
			try {
				temp += parameters.at(identifier);
			}
			catch (...) {
				temp += default_value;
			}
			return temp;
		}
	}
}