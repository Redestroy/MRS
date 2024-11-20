#pragma once
#ifndef CLASS_TREE
#define CLASS_TREE
#include <deque>
namespace MRSbase {
	template <typename T>
	class tree
	{
	private:
		T value;
		tree<T>* parent;
		std::deque<tree<T>*> children;
		size_t size;
		size_t ID;
		tree();
		~tree();

		tree<T>* GetParent();
		tree<T>* GetRoot();
		tree<T>* GetRootRecursive(size_t i);
		tree<T>* GetRootIterative(size_t i);
		std::deque<tree<T>*> * GetChildren();
		tree* GetChild(size_t);
		T GetValue();
		size_t GetID();
		void SetID(size_t);

		void AddChild(tree<T>* child);		
		void RemoveChild(tree<T>* child);	//Not implamented
		void DeleteChild(tree<T>* child);	//Not implamented
		void DeleteChildren();				//Not implamented
		void emplace(T);					//Not implamented
		bool operator==(tree<T>*);			//Not implamented

		// add all search algortihms
		// maybe in an algorihm static class
		static tree<T>* DepthFirstSearch(tree<T>* root, size_t id);		//Not implamented
		static tree<T>* BreadthFirstSearch(tree<T>* root, size_t id);		//Not implamented
		static tree<T>* InOrderSearch(tree<T>* root, size_t id);			//Not implamented
		static tree<T>* GreadyInOrderSearch(tree<T>* root, size_t id);		//Not implamented

	};

}
#endif // !1