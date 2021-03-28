#include "../Include/InformationSensor.hpp"

namespace MRS {
	namespace Device {
		InformationSensor::InformationSensor()
		{
		}
		InformationSensor::~InformationSensor()
		{
		}
		void InformationSensor::Init()
		{
		}
		void InformationSensor::DeInit()
		{
		}

		void InformationSensor::OnUpdate(long delta_time)
		{
			//TODO: add a way to cycle through parameters in information view
			//For efficiency don't update all values each cycle
			if (update_all) {
				for (auto param : *information) {
					param.value = UpdateParameter(param.key);
				}
				update_all = false;
			}
			else {
				//TODO: keep a list of parameters that need updating
				//TODO: update changed parameters only
			}
		}

		void InformationSensor::SetUpdateAll()
		{
			update_all = true;
		}
	}
}