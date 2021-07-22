
#include "Task.hpp"
#include "ParametricAction.hpp"

namespace MRS {
	namespace Task {
		class ParametricATask : public Task {
		private:
			// TODO: Evaluate use of vectors
			std::vector<Device::ParametricAction> actions;
			std::vector<Device::ParametricAction>::iterator taskSize;
		public:
			ParametricATask(unsigned char id = 0, float priority = 0.0f);
			ParametricATask(unsigned char id, float priority, Condition* condition);
			ParametricATask(unsigned char id, float priority, Condition* condition, Condition* end_condition);
			ParametricATask(std::string description);
			~ParametricATask();

			void Add(Device::ParametricAction);
			virtual Device::Action GetNextCommand();
			virtual std::string* toString();
		};
	}
}