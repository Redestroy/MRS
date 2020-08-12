#include "pch.h"
#include "tree.h"
namespace MRSbase {
	template <typename T>
	tree<T>::tree() : children()
	{
		size = 0;
		parent = nullptr;
		ID = 0;
	}

	template <typename T>
	tree<T>::~tree() {
	
	}

	template<typename T>
	tree<T>* tree<T>::GetParent()
	{
		return parent;
	}

	template <typename T>
	tree<T>* tree<T>::GetRoot() {
		if (DEFAULT_MODE) return GetRootRecursive(DEFAULT_MAX_DEPTH);
		else return GetRootIterative(DEFAULT_MAX_DEPTH);
	}
	template<typename T>
	tree<T>* tree<T>::GetRootRecursive(size_t i)
	{
		if (i <= 0) {
			printf("Max recursion reached\n");
			return nullptr;
		}
		if (this->GetParent() == nullptr)return this;
		else return this->GetParent()->GetRootRecursive(--i);
	}
	template<typename T>
	tree<T>* tree<T>::GetRootIterative(size_t i)
	{
		tree<T>* current = this;
		for (int j = 0; j < i; j++) {
			if (current->GetParent() == nullptr) return current;
			else current = current->GetParent();
		}
		return nullptr;
	}
	template<typename T>
	std::deque<tree<T>*>* tree<T>::GetChildren()
	{
		return &children;
	}
	template<typename T>
	tree<T>* tree<T>::GetChild(size_t id)
	{
		if (children == nullptr) return nullptr;
		if (children.empty()) return nullptr;
		for (auto child : children) {
			if (child == nullptr) continue;
			if (child.GetID() == id) return child;
		}
		return nullptr;
	}
	template<typename T>
	T tree<T>::GetValue()
	{
		return value;
	}
	template<typename T>
	size_t tree<T>::GetID()
	{
		return ID;
	}
	template<typename T>
	void tree<T>::SetID(size_t id)
	{
		ID = id;
	}
	template<typename T>
	void tree<T>::AddChild(tree<T>* child)
	{
		children.emplace_back(child);
		this->UpdateIDs(this->GetRoot());
	}
	template<typename T>
	void tree<T>::RemoveChild(tree<T>* child)
	{
		if (children == nullptr) return nullptr;
		if (children.empty()) return nullptr;
		for (auto ch : children) {
			if (ch == child) {
				children.erase(ch);
			}
		}
	}
	template<typename T>
	void tree<T>::DeleteChild(tree<T>* child)
	{
		for (auto ch : children) {
			if (ch == child) {
				ch.DeleteChildren();
				children.erase(ch);
			}
		}
	}
	template<typename T>
	void tree<T>::DeleteChildren()
	{
		if (children == nullptr) return;
		if (children.empty()) {
			delete child;
		}
		for (auto ch = children.begin(); ch != children.end(); ch++) {
			(*ch)->DeleteChildren();
		}
		children.clear();
	}
	template<typename T>
	void tree<T>::emplace(T value)
	{
		children.emplace_back(tree<T>(value));
	}
	template<typename T>
	bool tree<T>::operator==(tree<T>*)
	{
		return false;
	}
	template<typename T>
	tree<T>* tree<T>::DepthFirstSearch(tree<T>* root, size_t id)
	{
		return nullptr;
	}
	template<typename T>
	tree<T>* tree<T>::BreadthFirstSearch(tree<T>* root, size_t id)
	{
		return nullptr;
	}
	template<typename T>
	tree<T>* tree<T>::InOrderSearch(tree<T>* root, size_t id)
	{
		return nullptr;
	}
	template<typename T>
	tree<T>* tree<T>::GreadyInOrderSearch(tree<T>* root, size_t id)
	{
		return nullptr;
	}
}