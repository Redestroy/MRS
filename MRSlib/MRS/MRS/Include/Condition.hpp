#pragma once
#include "Worldview.hpp"
namespace MRS {
	namespace Task {
		enum class ConditionType
		{
			VIEW_MATCHING,
			SYMBOLIC_LOGIC,
			COMPLEX_CONDITION
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
		};
	}
}