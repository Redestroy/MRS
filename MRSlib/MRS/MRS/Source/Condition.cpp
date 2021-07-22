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

		ConditionType Condition::CharToCondition(char t)
		{
			switch (t) {
			case (char)ConditionType::COMPLEX_CONDITION: return ConditionType::COMPLEX_CONDITION;
			case (char)ConditionType::SYMBOLIC_LOGIC: return ConditionType::VIEW_MATCHING;
			case (char)ConditionType::CONDITION_POSITION: return ConditionType::CONDITION_POSITION;
			case (char)ConditionType::CONDITION_INPUT_TEXT_EQUALS: return ConditionType::CONDITION_INPUT_TEXT_EQUALS;
			case (char)ConditionType::CONDITION_NULL: return ConditionType::CONDITION_NULL;
			case (char)ConditionType::CONDITION_SUBTASK: return ConditionType::CONDITION_SUBTASK;
			case (char)ConditionType::CONDITION_UNDEFINED: return ConditionType::CONDITION_UNDEFINED;
			case (char)ConditionType::CONDITION_WAIT: return ConditionType::CONDITION_WAIT;
			default: return ConditionType::CONDITION_UNDEFINED;
			}
		}

	}
}