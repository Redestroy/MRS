#pragma once
#include "ObjectView.hpp"
#include <deque>
#include <string>
#include <memory>
namespace MRS {
	namespace Device {
		class ObjectViewList : public View {
		private:
			std::deque<std::unique_ptr<ObjectView>> object_list;
		public:
			ObjectViewList();
			~ObjectViewList();
			void Add(std::unique_ptr<ObjectView> ptr);
			void Remove(int index);
			void Clear();
		};
	}
}