#pragma once
#include "Value.hpp"
#include "State.hpp"
#include "../../MRS/Include/Task.hpp"
namespace MRS {
	namespace Algorithms {
		class ValueCalculator {
		public:
			//TODO: Add boost variant as argument for functions
			virtual MRS::Algorithms::Value Value(Task::Task*) = 0;
			virtual MRS::Algorithms::Value Value(MRS::Algorithms::State<1>*) = 0;
			virtual MRS::Algorithms::Value Sum(MRS::Algorithms::Value, MRS::Algorithms::Value) = 0;
			virtual MRS::Algorithms::Value Sum(MRS::Algorithms::Value*, int size, int i) = 0;
			virtual MRS::Algorithms::Value Sum(Task::Task** t_array, int size) {
				MRS::Algorithms::Value values[size];
				for (int i = 0; i < size; i++) {
					values[i] = Value(t_array[i]);
				}
				return Sum(values, size, 1);
			}

			virtual MRS::Algorithms::Value Sum2(MRS::Algorithms::State<1>** t_array, int size, int i) {
				MRS::Algorithms::Value values[size];
				for (int i = 0; i < size; i++) {
					values[i] = Value(t_array[i]);
				}
				return Sum(values, size, 1);
			}
		};

		class ArithmeticCalculator : public ValueCalculator {
		private:
			Task::Task* currentTask;
			//ValueTable behaviour_values;
		public:
			virtual MRS::Algorithms::Value Value(Task::Task* t) {
				//Value of task is obtained from default priority, task type, task state and current task condition
				// Values should be obtained with clasical reinforcement techniques based on previous games
				// Initial values can be given via priority and calculator adjusts them with the Value table
				// For example: 
				//switch(t->GetType()){
				std::complex<double> result;
				int NullFactor = 1; //TODO: add check for legality
				MRS::Algorithms::Value value((double)(t->GetPriority()), 0.0);
				result = NullFactor* (double)(t->GetPriority());
				//TODO:: add imaginary part
				value = result;
				return value; //Imaginary = WeightFromState(t->TaskState)*WeightFromParent(currentTask)
			};

			virtual MRS::Algorithms::Value Value(MRS::Algorithms::State<1>* s) {
				//TODO: Seperate football code from MRS::lib
				//   Currently use football code for refinement
				//Value of state is obtained from the probability and type of interference generated
				// Interference is proportional to the player distance from trajectory
				// Interferance is negative for opposition players and zero for far away players.
				// For same team players, the interferance is behaviour dependent
				// Also ball control changes the behaviour as well
				// For example: 
				//switch(t->GetType()){
				//if (s.is_me()) {
				//	if (s.controls_ball()) {
						// zero no ball behaviours
						// ball behaviours get assigned values based on player position
						// further from goal - better dribble behaviours
						// closer to goal - better kick behaviours
						// pass is dependent on the number of nearby players
				//	}
				//	else {
						// zero ball behaviours 
				//	}
				//}

				return MRS::Algorithms::Value(s->GetBinaryState() ? 1 : -1, (double)s->GetStateAsInt()); //Imaginary = WeightFromState(t->TaskState)*WeightFromParent(currentTask)
			};

			virtual MRS::Algorithms::Value Sum(MRS::Algorithms::Value v1, MRS::Algorithms::Value v2) {
				return MRS::Algorithms::Value(v1.Scalar() * (1 + v2.Imaginary()) + v2.Scalar() * (1 + v1.Imaginary()), v1.Imaginary() * (1 + v2.Scalar()) + v2.Imaginary() * (1 + v1.Scalar()));
			}

			virtual MRS::Algorithms::Value Sum(MRS::Algorithms::Value* values, int size, int i) {
				MRS::Algorithms::Value value = values[0];
				for(int i = 1; i < size; i++) value = value + values[i];
				return value;
			}
		};
	}
}