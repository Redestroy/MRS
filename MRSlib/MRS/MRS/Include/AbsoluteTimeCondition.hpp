#pragma once
#include "Condition.hpp"
#include <boost/timer/timer.hpp>
//#include "C:/Users/JB00636/AppData/Local/Microsoft/Linux/HeaderCache/1.0/15928398/usr/local/include/boost/timer/timer.hpp"

namespace MRS {
	namespace Task {
		class AbsoluteTimeCondition : public Condition {
		private:
			boost::timer::nanosecond_type time;
			bool absolute_mode;
			std::string condition_string;
		public:
			AbsoluteTimeCondition();
			AbsoluteTimeCondition(std::string);
			AbsoluteTimeCondition(boost::timer::nanosecond_type time, bool absolute_mode);
			~AbsoluteTimeCondition();

			virtual bool isMet(Environment::Worldview*);
			virtual std::string* toString(); //TODO fix/optimise string methods
		};
	}
}