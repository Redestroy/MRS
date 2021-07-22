#include "../Include/AbsoluteTimeCondition.hpp"
namespace MRS {
	namespace Task {
		AbsoluteTimeCondition::AbsoluteTimeCondition() : AbsoluteTimeCondition(0, true)
		{
		}

		AbsoluteTimeCondition::AbsoluteTimeCondition(std::string description)
		{
		}

		AbsoluteTimeCondition::AbsoluteTimeCondition(boost::timer::nanosecond_type time, bool absolute_mode) : Condition(ConditionType::PARAMETER)
		{
			this->time = time;
			this->absolute_mode = absolute_mode;
		}

		AbsoluteTimeCondition::~AbsoluteTimeCondition()
		{
		}

		bool AbsoluteTimeCondition::isMet(Environment::Worldview* world)
		{
			return world->GetTime() >= time; //TODO add start time for relative mode
		}

		std::string* AbsoluteTimeCondition::toString()
		{
			condition_string = "TODO: create to string method for condition";
			return &condition_string;
		}
	}
}