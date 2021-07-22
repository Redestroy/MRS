#pragma once
#include <MRS/Include/Task.hpp>
#include <MRS/Include/Behaviour.hpp>
#include <rcsc/player/player_agent.h>

namespace MRS {
	class BehaviourRCSC :
		public Task::Behaviour
	{
	private:
		//RCSCTask<>* rcscTask;
	public:
		BehaviourRCSC();
		//BehaviourRCSC(RCSCTask*);
		virtual ~BehaviourRCSC();
		
	};

	template <class T>
	class RCSCTask : public Task::Task {
	private:
		T* behaviour;
	public:
		RCSCTask();
		RCSCTask(T* behaviour);
		~RCSCTask();

	};
}

