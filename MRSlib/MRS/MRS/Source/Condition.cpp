#include "../Include/Condition.hpp"
namespace MRS {
	namespace Task {

		void Condition::SetConditionType(ConditionType type)
		{
			conditionType = type;
		}

		Condition::Condition() : Condition(ConditionType::VIEW_MATCHING)
		{
		}

		Condition::Condition(ConditionType type)
		{
			conditionType = type;
		}


		Condition::~Condition()
		{
		}

	}
}