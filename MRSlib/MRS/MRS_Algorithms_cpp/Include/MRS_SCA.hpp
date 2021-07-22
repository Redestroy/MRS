// SCA algorithm
// Use a static method
#pragma once
#include "StatePredictor.hpp"
#include "ValueCalculator.hpp"

//#include "../../MRS/Include/Robot.hpp"
#include "../../MRS/Include/BehaviourLibrary.hpp"
//#include "../../MRS/Include/TaskPriorityList.hpp"
#include <queue>

namespace MRS {
	namespace Algorithms{
		typedef std::priority_queue<std::pair<Value, Task::Behaviour*>, std::deque<std::pair<Value, Task::Behaviour*>>, ValueOperations::CompareValues> PriorityList;
#define state_size 1
		class SCA {
		private:
			StatePredictor<state_size>* predictor; // TODO: hide template from view - make interface independent of state size
			ArithmeticCalculator* calculator;

			Task::Robot* robot;
			Task::RobotInfo<Task::Robot*> * all_robots_in_system[22]; //TODO: remove magic constant
			Environment::Worldview* view;

			MRS::Algorithms::State<1>* states[22]; //TODO: remove magic constant
			PriorityList ranked_behaviours;

			void PredictStates();
			Value GetValue(Task::Behaviour*);
			void RankBehaviours();
			Task::Task* GetBestBehaviour();

		public:
			SCA();
			~SCA();
			
			void Init();
			void SetPredictor(StatePredictor<state_size>*);
			void SetCalculator(ArithmeticCalculator*);
			void SetRobot(Task::Robot*);
			void SetWorldview(Environment::Worldview*);
			Task::Task* GenerateBestBehaviour();
			void ExecuteBestBehaviour();
		};

		/*
		static Task::Task* MRS_SCA(Robot*, TaskLibrary*, ...) {
			// Do steps 1. and 2. but don't execute
			return nullptr;
		}
		*/
	}
}

