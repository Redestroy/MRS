#include "../Include/TaskParser.hpp"

namespace MRS {
	namespace Task {
		void TaskParser::SetAttributesFromString(std::string description, std::size_t length, Task* task)
		{
			task->SetString(description.copy());
			task->SetTimestamp(ProtocolParser::ParseTaskStringForTimestamp());
			task->SetState(ProtocolParser::ParseTaskStringForState());
			//Conditions
			task->SetStartCondition(ProtocolParser::ParseTaskStringForType());
			task->SetExecutedCondition(ProtocolParser::ParseTaskStringForType());
			//Task type
			task->SetTaskType(ProtocolParser::ParseTaskStringForType());
			task->SetID(ProtocolParser::ParseTaskStringForID());
			task->SetPriority(ProtocolParser::ParseTaskStringForPriority());
			std::string substring = "";
			switch (task->GetType()){
			case TaskType::ATOMIC:
				substring = ProtocolParser::ParseAtomicSubstringChain();
				SetAttributesFromString(substring, substring.size(), (ATask*) task);
			case TaskType::BEHAVIOUR:
				substring = ProtocolParser::ParseBehaviourSubstring();
				SetAttributesFromString(substring, substring.size(), (Behaviour*)task);
			case TaskType::PARAMETRIC:
				substring = ProtocolParser::ParseParametricSubstringChain();
				SetAttributesFromString(substring, substring.size(), (Behaviour*)task);
			}
		}

		void TaskParser::SetAttributesFromString(std::string description, std::size_t length, ATask* task)
		{
			//TODO: Store string?
			std::string token;
			while (token = ProtocolParser::GetNextActionToken() != "") {
				task->Add(ProtocolParser::ParseAction(token));
			}
			//if(task->GetLastAction().GetActionType()!= Device::ActionType::NULL)
			task->Add(Device::Action());
		}

		void TaskParser::SetAttributesFromString(std::string description, std::size_t length, Behaviour* behaviour)
		{
			//Store behaviour string
			behaviour->SetBehaviourString(description.copy());
			//Parse end condition
			behaviour->SetEndCondition();
			//Parse base task
			std::string substring = ProtocolParser::ParseTaskSubstringForBehaviour(description, length);
			behaviour->SetBaseTask(CreateTask(substring)); //TODO:: figure out factory method
		}

		void TaskParser::SetAttributesFromString(std::string, std::size_t, ParametricATask* task)
		{
			//Parametric task consists of additional parts when compared to ATask
			// if ATask is just concerned with action count A
			// Parametric ATask is also concerned with
			// parameter count P (parameters by default are double valued)
			// parameter list P_L  example position.x position.y sensor1_val neural_list
			// koeficient functions K : function type, function: F()
			// Function types F, supported: CONSTANT, Linear, Binomial, Polynomial, Fourier, External Network (works for most functions to complex for string representations), 
			// Example array	A = 5
			//	A1: F-Constant P-0
			// Protocol:
			// T_PA Action_count Parameter_count /P_L . . ./A_L 
			// /A A_type F_C(5)/
			// /A A_type F_L(5Arg1 [+ 5])/
			// /A A_type F_B(5Arg2^2 [5*Arg3 +5])/
			// /A A_type F_P(n 5Arg4^n [...] [+5])/
			// /A A_type F_F(n m )/
			// /A A_type F_N(n(inputs) Arg1,Arg2,...,Arg_n external_network_reference:<FILENAME,FACTORY>)/
			// Store task string

			//Parameter count
			task->SetParameterCount(ProtocolParser::ParseParameterCount);
			//Parametric action creation
			while (token = ProtocolParser::GetNextActionToken() != "") {
				task->Add(ProtocolParser::ParseParametricAction(token));
			}
			//if(task->GetLastAction().GetActionType()!= Device::ActionType::NULL)
			task->Add(Device::Action());
		}
	}
}