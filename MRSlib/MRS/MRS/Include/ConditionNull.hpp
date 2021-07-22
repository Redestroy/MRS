#pragma once
#ifndef CONDITIONNULL
#define CONDITIONNULL
#include "Condition.hpp"
#include <string>
namespace MRS {
	namespace Task {
		class ConditionNull :
			public Condition
		{
		public:
			ConditionNull();
			ConditionNull(std::string*);
			~ConditionNull();
			bool isMet(Environment::Worldview*);
			std::string* toString();
		};
	}
}
#endif // !CONDITIONNULL