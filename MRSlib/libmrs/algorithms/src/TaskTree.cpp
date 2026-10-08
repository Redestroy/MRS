#include "mrs/algorithms/TaskTree.h"

#include <algorithm>

namespace MRS {
	namespace Algorithms {
		using Protocol::Field;
		using Protocol::FieldType;
		using Protocol::Record;

		bool IsComplexTask(const Record& task) { return task.code == "T_S" || task.code == "T_L" || task.code == "T_O"; }

		namespace {
			bool IsLeafCode(const std::string& c) { return c == "T_A" || c == "T_P" || c == "T_B"; }

			double NumOf(const Field& f) { return f.type == FieldType::Int ? static_cast<double>(f.i) : f.n; }

			// The children of a record with a given kind letter, in slot order.
			std::vector<const Record*> RefsOfKind(const Record& r, char kind) {
				std::vector<const Record*> out;
				for (const auto& f : r.fields)
					if (f.type == FieldType::Ref && r.children.at(f.ref).kind == kind) out.push_back(&r.children.at(f.ref));
				return out;
			}

			// The first R_K in a requirement tree.
			std::string AffinityOf(const Record& r) {
				if (r.code == "R_K" && !r.fields.empty()) return r.fields[0].s;
				for (const auto& c : r.children) {
					auto a = AffinityOf(c);
					if (!a.empty()) return a;
				}
				return {};
			}

			class Builder {
			public:
				explicit Builder(TaskTree& tree) : tree_(tree) {}

				void Add(const Record& r, const std::string& id, const std::string& parent, double priority) {
					if (r.code == "T_D" || r.code == "T_G" || r.code == "T_U") throw DecomposeError(r.code + " is reserved (spec 03 §2)");
					if (r.fields.size() < 3) throw DecomposeError("task " + id + " has no common slots");
					TreeNode n;
					n.id = id;
					n.code = r.code;
					n.parent = parent;
					n.leaf = IsLeafCode(r.code);
					const double p = priority * NumOf(r.fields[1]);
					const auto conds = RefsOfKind(r, 'C');
					if (conds.size() >= 2) {
						n.start = *conds[0];
						n.end = *conds[1];
					}
					if (n.leaf) {
						n.task = r;
						n.task.fields[0] = Field::MakeText(FieldType::TaskId, id);
						n.task.fields[1] = Field::MakeNum(p);
						for (const Record* req : RefsOfKind(r, 'R')) {
							n.affinity = AffinityOf(*req);
							if (!n.affinity.empty()) break;
						}
						tree_.leaves.push_back(id);
						tree_.nodes[id] = std::move(n);
						return;
					}
					if (!IsComplexTask(r)) throw DecomposeError("task " + id + " has an unknown code " + r.code);
					if (r.code == "T_L") n.k = static_cast<int>(r.fields.at(3).i);
					const auto kids = RefsOfKind(r, 'T');
					if (kids.empty()) throw DecomposeError("complex task " + id + " has no children");
					if (n.k < 0 || n.k > static_cast<int>(kids.size())) throw DecomposeError("T_L " + id + ": k is more than its children");
					for (std::size_t i = 0; i < kids.size(); ++i) n.children.push_back(id + "." + std::to_string(i + 1));
					const auto children = n.children;
					tree_.nodes[id] = std::move(n);
					for (std::size_t i = 0; i < kids.size(); ++i) Add(*kids[i], children[i], id, p);
				}

			private:
				TaskTree& tree_;
			};

			bool IsNull(const Record& c) { return c.code.empty() || c.code == "C_N"; }
		}

		std::vector<std::string> LeavesUnder(const TaskTree& tree, const std::string& id) {
			const TreeNode& n = tree.Node(id);
			if (n.leaf) return {id};
			std::vector<std::string> out;
			for (const auto& c : n.children) {
				auto sub = LeavesUnder(tree, c);
				out.insert(out.end(), sub.begin(), sub.end());
			}
			return out;
		}

		std::vector<std::string> FirstLeaves(const TaskTree& tree, const std::string& id) {
			const TreeNode& n = tree.Node(id);
			if (n.leaf) return {id};
			if (n.code == "T_S") return FirstLeaves(tree, n.children.front());
			std::vector<std::string> out;  // T_L, T_O: every child can start at once
			for (const auto& c : n.children) {
				auto sub = FirstLeaves(tree, c);
				out.insert(out.end(), sub.begin(), sub.end());
			}
			return out;
		}

		TaskTree Decompose(const Record& root) {
			if (!IsComplexTask(root)) throw DecomposeError("not a tree task: " + root.code);
			if (root.fields.empty() || root.fields[0].type != FieldType::TaskId || root.fields[0].s == "0")
				throw DecomposeError("a tree's root needs an id (spec 03 §6.2)");
			TaskTree tree;
			tree.root = root.fields[0].s;
			Builder(tree).Add(root, tree.root, {}, 1.0);
			// Precedence (rule 3) and the start conditions of complex nodes (rule 6).
			for (auto& [id, n] : tree.nodes) {
				if (n.leaf) continue;
				if (!IsNull(n.start))
					for (const auto& leaf : FirstLeaves(tree, id)) tree.nodes.at(leaf).gates.push_back(n.start);
				if (n.code != "T_S") continue;
				for (std::size_t i = 1; i < n.children.size(); ++i)
					for (const auto& leaf : FirstLeaves(tree, n.children[i])) tree.nodes.at(leaf).after.push_back(n.children[i - 1]);
			}
			return tree;
		}

		PoolState TreeState::Of(const std::string& id, const LeafState& leaf) const {
			if (auto it = overrides_.find(id); it != overrides_.end()) return it->second;
			const TreeNode& n = tree_->Node(id);
			if (n.leaf) return leaf(id);
			int done = 0, failed = 0;
			const int total = static_cast<int>(n.children.size());
			for (const auto& c : n.children) {
				const PoolState s = Of(c, leaf);
				if (s == PoolState::DONE) ++done;
				else if (Finished(s)) ++failed;
			}
			if (n.code == "T_S") {
				if (failed > 0) return PoolState::FAILED;
				return done == total ? PoolState::DONE : PoolState::AVAILABLE;
			}
			if (n.code == "T_L") {
				const int need = n.k == 0 ? total : n.k;
				if (done >= need) return PoolState::DONE;
				return total - failed < need ? PoolState::FAILED : PoolState::AVAILABLE;
			}
			// T_O
			if (done > 0) return PoolState::DONE;
			return failed == total ? PoolState::FAILED : PoolState::AVAILABLE;
		}
	}
}
