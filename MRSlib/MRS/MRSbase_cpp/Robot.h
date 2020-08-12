#ifndef CLASS_ROBOT
#define CLASS_ROBOT
#include "pch.h"
#include <string.h>
namespace MRSbase {
	class Robot
	{
	private:
		char* name;
	public:
		Robot();
		Robot(char*);
		~Robot();
		void SetName(char*);
		char* GetName() const;
	};
}
#endif // !CLASS_ROBOT
