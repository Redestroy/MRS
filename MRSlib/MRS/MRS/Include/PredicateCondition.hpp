#pragma once
#include "Condition.hpp"
namespace MRS {
	namespace Task {
		class PredicateCondition : public Condition {
		private:
			std::string name;
			bool last_value = false;
			bool invert = false;
			std::string predicate_string = "";
		public:
			PredicateCondition();
			PredicateCondition(std::string);
			PredicateCondition(std::string, bool value, bool invert = false);
			~PredicateCondition();

			virtual bool isMet(Environment::Worldview*);
			virtual std::string* toString(); //TODO fix/optimise string methods

			void SetPredicateName(std::string name);
		};
	}
}