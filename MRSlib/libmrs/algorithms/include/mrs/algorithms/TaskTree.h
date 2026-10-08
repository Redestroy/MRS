#pragma once
// Tree tasks split between robots (plan §8.7, spec 03 §6.2, spec 11 §2): the decomposer that
// turns a T_S, T_L or T_O tree into leaf tasks, and the derived state of the tree's nodes.
#include <functional>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "mrs/algorithms/TaskPool.h"
#include "mrs/protocol/Record.h"

namespace MRS {
	namespace Algorithms {
		struct DecomposeError : std::runtime_error {
			using std::runtime_error::runtime_error;
		};

		// A tree has a complex root: T_S, T_L or T_O.
		bool IsComplexTask(const Protocol::Record& task);

		struct TreeNode {
			std::string id;                     // p.i ids (spec 03 §6.2 rule 1)
			std::string code;                   // T_A, T_P, T_B, T_S, T_L, T_O
			bool leaf = false;
			int k = 0;                          // T_L: children needed, 0 = all
			std::string parent;                 // empty for the root
			std::vector<std::string> children;  // in listed order
			Protocol::Record start, end;        // the node's own start and end conditions
			Protocol::Record task;              // leaves: the leaf task, with its id and effective priority
			// Leaves: nodes that must be DONE first (precedence of a T_S, rule 3), and start
			// conditions of complex ancestors that must hold first (rule 6). Both gate the leaf:
			// it is BLOCKED until they hold.
			std::vector<std::string> after;
			std::vector<Protocol::Record> gates;
			std::string affinity;               // leaves: the R_K partner, or empty (rule 8)
		};

		struct TaskTree {
			std::string root;
			std::map<std::string, TreeNode> nodes;
			std::vector<std::string> leaves;  // depth-first, in listed order
			const TreeNode& Node(const std::string& id) const { return nodes.at(id); }
			bool Has(const std::string& id) const { return nodes.count(id) > 0; }
		};

		// Decomposes a tree (spec 03 §6.2). The result depends only on the record, so every robot
		// and the issuer get the same leaves. Throws DecomposeError for a tree without leaves, a
		// reserved code (T_D, T_G, T_U) or a T_L whose k exceeds its children.
		TaskTree Decompose(const Protocol::Record& root);

		// Derived node states (spec 11 §2.3). Leaves read `leaf`; complex nodes follow their link:
		// T_S is DONE when every child is, FAILED when one fails; T_L is DONE at k children DONE,
		// FAILED when fewer than k can still be; T_O is DONE at the first child DONE, FAILED when
		// every child failed. FAILED, IMPOSSIBLE and CANCELLED children count as failed. A
		// complex node with an override (its end condition was FALSE) takes the override.
		// Anything not yet decided is AVAILABLE.
		class TreeState {
		public:
			explicit TreeState(const TaskTree& tree) : tree_(&tree) {}
			using LeafState = std::function<PoolState(const std::string& leaf)>;
			PoolState Of(const std::string& id, const LeafState& leaf) const;
			void Override(const std::string& id, PoolState s) { overrides_[id] = s; }
			const std::map<std::string, PoolState>& Overrides() const { return overrides_; }

		private:
			const TaskTree* tree_;
			std::map<std::string, PoolState> overrides_;
		};

		// The leaves under a node whose ancestors below it are all finished first: the leaves a
		// T_S child's precedence applies to.
		std::vector<std::string> FirstLeaves(const TaskTree& tree, const std::string& id);
		// Every leaf under a node.
		std::vector<std::string> LeavesUnder(const TaskTree& tree, const std::string& id);
	}
}
