#pragma once
#include "Worldview.hpp"
namespace MRS {
	namespace Task {
		enum class ConditionType
		{
			VIEW_MATCHING = 'V',
			PREDICATE = 'p',
			PARAMETER = 'm',
			SYMBOLIC_LOGIC = 'L',
			COMPLEX_CONDITION = 'C',
			//Legacy types
			CONDITION_POSITION = 'P',
			CONDITION_IN_RANGE = 'R',
			CONDITION_WAIT = 'W',
			CONDITION_INPUT_TEXT_EQUALS = 'I',
			CONDITION_UNDEFINED = 'U',
			CONDITION_SUBTASK = 'S',
			CONDITION_NULL = 'N'
		};

		class Condition
		{
			ConditionType conditionType;
		protected:
			void SetConditionType(ConditionType);
		public:
			Condition();
			Condition(ConditionType);
			~Condition();
			ConditionType GetType() {
				return conditionType;
			};

			virtual bool isMet(Environment::Worldview*) = 0;
			virtual std::string* toString() = 0;

			static ConditionType CharToCondition(char);
		};
	}
}