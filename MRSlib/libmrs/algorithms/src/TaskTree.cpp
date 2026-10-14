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
					n.task = r;
					n.task.fields[0] = Field::MakeText(FieldType::TaskId, id);
					if (n.leaf) {
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

			// Writes the decomposer's ids into a subtree record: child i of `id` is `id.i`.
			void WriteIds(Record& r, const std::string& id) {
				r.fields[0] = Field::MakeText(FieldType::TaskId, id);
				if (!IsComplexTask(r)) return;
				std::size_t i = 0;
				for (const auto& f : r.fields)
					if (f.type == FieldType::Ref && r.children.at(f.ref).kind == 'T') WriteIds(r.children[f.ref], id + "." + std::to_string(++i));
			}

			bool Under(const std::string& id, const std::string& node) { return id == node || id.rfind(node + ".", 0) == 0; }

			// Marks the units below `id` (spec 12 §2.1).
			void MarkUnits(TaskTree& tree, const std::string& id) {
				TreeNode& n = tree.nodes.at(id);
				if (n.leaf || !Splittable(tree, id)) {
					n.unit = true;
					tree.units.push_back(id);
					for (const auto& leaf : LeavesUnder(tree, id)) tree.unit_of[leaf] = id;
					return;
				}
				for (const auto& c : n.children) MarkUnits(tree, c);
			}
		}

		bool Splittable(const TaskTree& tree, const std::string& id) {
			const TreeNode& n = tree.Node(id);
			if (n.leaf) return false;
			if (n.code != "T_S") return true;
			for (std::size_t i = 0; i < n.children.size(); ++i) {
				if (Splittable(tree, n.children[i])) return true;
				if (i == 0) continue;
				for (const auto& first : FirstLeaves(tree, n.children[i])) {
					const std::string& partner = tree.Node(first).affinity;
					if (partner.empty() || !tree.Has(partner) || !Under(partner, n.children[i - 1])) return true;
				}
			}
			return false;
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
			MarkUnits(tree, tree.root);
			// A complex unit runs on one robot as a whole: its record carries the subtree with ids,
			// and its constraints are those of its first leaf that come from outside it.
			for (const auto& u : tree.units) {
				TreeNode& n = tree.nodes.at(u);
				if (n.leaf) continue;
				WriteIds(n.task, u);
				double p = 1.0;
				for (std::string a = u; !a.empty(); a = tree.Node(a).parent) p *= NumOf(tree.Node(a).task.fields[1]);
				n.task.fields[1] = Field::MakeNum(p);
				const TreeNode& first = tree.Node(FirstLeaves(tree, u).front());
				n.after = first.after;
				for (std::string a = n.parent; !a.empty(); a = tree.Node(a).parent) {
					const auto fl = FirstLeaves(tree, a);
					if (!IsNull(tree.Node(a).start) && std::find(fl.begin(), fl.end(), first.id) != fl.end()) n.gates.push_back(tree.Node(a).start);
				}
				if (!first.affinity.empty() && !Under(first.affinity, u)) n.affinity = first.affinity;
			}
			return tree;
		}

		PoolState TreeState::Of(const std::string& id, const LeafState& leaf) const {
			if (auto it = overrides_.find(id); it != overrides_.end()) return it->second;
			const TreeNode& n = tree_->Node(id);
			if (n.unit || n.leaf) return leaf(id);  // a leaf inside a complex unit asks the caller too
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
