#pragma once
#include "State.hpp"
#include "../../MRS/Include/Robot.hpp"
#include <random>
namespace MRS{
	namespace Algorithms {
		template <std::size_t state_size>
		class StatePredictor {
		private:
		public:
			StatePredictor() {}
			~StatePredictor() {}
			virtual State<state_size>* PredictInternalState(Task::Robot* me, Task::RobotInfo<Task::Robot*> * other) = 0; // Hidden variable passed - world
			virtual State<state_size>* GetInteractionState(Task::Robot* me, Task::RobotInfo<Task::Robot*> * other) = 0; // Hidden variable passed - world
		};

		class BinaryStateRandomPredictor : public StatePredictor<1> {
		private:
			BinaryState state;
		public:
			State<1>* PredictInternalState(Task::Robot* me, Task::RobotInfo<Task::Robot*>* other) {
				state.SetState((bool)(rand()%2));
				return &state;
			}
			State<1>* GetInteractionState(Task::Robot* me, Task::RobotInfo<Task::Robot*>* other) {
				state.SetState(!state.GetBinaryState());
				return &state;
			}
		};
	}
}