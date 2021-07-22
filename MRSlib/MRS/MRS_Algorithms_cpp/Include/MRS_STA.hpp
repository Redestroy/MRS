#pragma once
#include "../Include/MRS_Layer.hpp"
#include "../../MRS/Include/Robot.hpp";
#include "../../MRS/Include/TaskPriorityList.hpp";
namespace MRS {
	namespace Algorithms {
		namespace TA {
			class MRS_STA : SimpleLayer {
			private:
				//TODO: USE smart pointers
				Task::TaskPriorityList* task_list;
				Task::Robot* robot;
			public:
				MRS_STA();
				MRS_STA(bool);
				~MRS_STA();
			};

		}
	}
}
