#include "../Include/ConditionNull.hpp"

namespace MRS {
	namespace Task {
		ConditionNull::ConditionNull() : Condition(ConditionType::CONDITION_NULL)
		{

		}

		ConditionNull::ConditionNull(std::string*) : ConditionNull()
		{
		}


		ConditionNull::~ConditionNull()
		{
		}

		bool ConditionNull::isMet(Environment::Worldview* world)
		{
			return true;
		}

		std::string* ConditionNull::toString()
		{
			char temp[4];//, temp2[4];
			//TypeDefinitions::TypeToString(temp, TypeDefinitions::Condition, (int)this->GetType());
			//sprintf(temp, "%s %.2f %.2f %.2f %.2f", temp2);
			return new std::string(temp);
		}
	}
}