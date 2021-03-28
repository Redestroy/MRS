#include "../Include/ObjectViewList.hpp"

namespace MRS {
	namespace Device {
		ObjectViewList::ObjectViewList() : View(), object_list()
		{

		}

		ObjectViewList::~ObjectViewList()
		{
		}

		void ObjectViewList::Add(std::unique_ptr<ObjectView> ptr)
		{
			object_list.emplace_back();
			object_list.back() = move(ptr);
		}

		void ObjectViewList::Clear()
		{
			object_list.clear();
		}

		void ObjectViewList::Remove(int index) //TODO: replace with iterator
		{
			// need to get iterator
			
			return;
		}
	}
}
