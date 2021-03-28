#include "../Include/View.hpp"

namespace MRS {
	namespace Device {
		void View::SetType(ViewType type)
		{
			this->type = type;
		}

		unsigned char* View::GetDataPtr()
		{
			return data;
		}

		View::View() : View(ViewType::VIEW_NULL)
		{
			
		}

		View::View(ViewType type)
		{
			this->type = type;
			timeCreated = -1;
			data[0] = '\0';
		}


		View::~View()
		{
		}

		void View::SetData(unsigned char pos, unsigned char data)
		{
			if (pos < DATA_SIZE) this->data[pos] = data;
		}

		char View::GetData(unsigned char pos)
		{
			if (pos < DATA_SIZE) return data[pos];
			return -1;
		}

		void View::onCreate(long time)
		{
			timeCreated = time;
			//timeCreated = millis() - zeroTime;
		}
	}
}