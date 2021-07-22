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
			MOVE,
			TURN,
			KICK,
			TACKLE,
			CATCH,
			TURN_VIEWPORT
		};

		class Action
		{
		private:
			ActionType actionType;
			char data[13];
		protected:
			long long int actionParameter;
			double actionParameter2;
		public:
			Action();
			Action(Action*);
			Action(ActionType, long long int);
			Action(std::string*);
			~Action();

			void SetActionType(ActionType);
			void SetActionParameter(long long int);

			ActionType GetActionType();
			virtual long long int GetActionParameter();

			std::string* toString();
			static ActionType CharToAction(char a);
			static double FirstHalfAsDouble(long long int p);
			static double SecondHalfAsDouble(long long int p);
			static int FirstHalfAsInteger(long long int p);
			static int SecondHalfAsInteger(long long int p);
		};
	}
}
#endif
