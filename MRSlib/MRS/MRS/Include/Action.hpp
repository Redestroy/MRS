#pragma once
#ifndef ACTION
#define ACTION
#include <string>
namespace MRS {
	namespace Device {
		enum class ActionType {
			ACTION_VIRTUAL_DIGITAL = 'L',
			ACTION_VIRTUAL_PWM,
			ACTION_TEXT_OUTPUT,
			ACTION_FORWARD,
			ACTION_BACKWARD,
			ACTION_CLOCKWISE,
			ACTION_COUNTER_CLOCKWISE,
			ACTION_ARC_DRIVE,
			ACTION_RAW_DRIVE,
			ACTION_NULL = 'N',
			ACTION_DELAY = 'W',
			ACTION_IMPOSSIBLE,
			DASH,
			MOVE
		};

		class Action
		{
		private:
			ActionType actionType;
			long long int actionParameter;
			char data[13];
		public:
			Action();
			Action(Action*);
			Action(ActionType, long long int);
			Action(std::string*);
			~Action();

			void SetActionType(ActionType);
			void SetActionParameter(long long int);

			ActionType GetActionType();
			long long int GetActionParameter();

			std::string* toString();
			static ActionType CharToAction(char a);
		};
	}
}
#endif
