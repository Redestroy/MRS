#include "../Include/MRS_SCA.hpp"
namespace MRS {
	namespace Algorithms {
		
		void SCA::PredictStates()
		{
			for (std::size_t i = 0; i < sizeof(all_robots_in_system); i++) {
				states[i] = predictor->GetInteractionState(robot, all_robots_in_system[i]);
			}
		}

		Value SCA::GetValue(Task::Behaviour* b)
		{
			Value v = calculator->Sum(calculator->Value((Task::Task*)b), calculator->Sum2(states, sizeof(states), 1));
			return v;
		}

		void SCA::RankBehaviours()
		{
			//for all behaviours in the behaviour library, update their value, then sort
			Task::BehaviourLibrary* library = robot->GetBehaviourLibrary();
			for (auto b : *library) {
				ranked_behaviours.emplace(GetValue(b.second), b.second);
			}
			//sort
			//ranked_behaviours;
		}

		Task::Task* SCA::GetBestBehaviour()
		{
			Task::Behaviour * behaviour = ranked_behaviours.top().second;
			return (Task::Task*)behaviour;
		}

		SCA::SCA() {
			
		}

		SCA::~SCA()
		{
		}

		void SCA::Init()
		{
		}

		void SCA::SetPredictor(StatePredictor<state_size>* p)
		{
			predictor = p;
		}

		void SCA::SetCalculator(ArithmeticCalculator* c)
		{
			calculator = c;
		}

		void SCA::SetRobot(Task::Robot* r)
		{
			robot = r;
		}

		void SCA::SetWorldview(Environment::Worldview* w)
		{
			view = w;
		}

		Task::Task* SCA::GenerateBestBehaviour()
		{
			PredictStates();
			RankBehaviours();
			Task::Task* behaviour = GetBestBehaviour();
			return behaviour;
		}

		void SCA::ExecuteBestBehaviour()
		{
			//PredictStates();
			Task::Task* behaviour = GenerateBestBehaviour();
			robot->execute(behaviour);
		}

	}
}