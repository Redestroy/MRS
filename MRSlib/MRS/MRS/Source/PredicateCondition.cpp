#include "../Include/PredicateCondition.hpp"

namespace MRS {
	namespace Task {
		PredicateCondition::PredicateCondition() : PredicateCondition("NAN", false)
		{

		}

		PredicateCondition::PredicateCondition(std::string description) : Condition(ConditionType::PREDICATE)
		{
			//Parse from string

		}

		PredicateCondition::PredicateCondition(std::string name, bool value, bool invert) : Condition(ConditionType::PREDICATE)
		{
			this->name = name;
			this->last_value = value;
		}

		PredicateCondition::~PredicateCondition()
		{
		}

		bool PredicateCondition::isMet(Environment::Worldview* world)
		{
			return world->GetPredicateValue(name) ^ invert;
		}

		std::string* PredicateCondition::toString()
		{
			predicate_string = "TODO: Predicate to string method";
			return &predicate_string;
		}

		void PredicateCondition::SetPredicateName(std::string name)
		{
			this->name = name;
		}
	}
}