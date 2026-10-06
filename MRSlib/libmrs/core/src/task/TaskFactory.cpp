#include "mrs/task/TaskFactory.h"

#include "mrs/protocol/Parser.h"

namespace MRS {
	namespace Task {
		namespace {
			using Protocol::Field;
			using Protocol::FieldType;
			using Protocol::Record;

			const Record& Ref(const Record& r, const Field& f) { return r.children.at(f.ref); }

			// The children referenced by fields[from..], in order.
			std::vector<const Record*> RefsFrom(const Record& r, std::size_t from) {
				std::vector<const Record*> out;
				for (std::size_t i = from; i < r.fields.size(); ++i)
					if (r.fields[i].type == FieldType::Ref) out.push_back(&Ref(r, r.fields[i]));
				return out;
			}

			TaskType TypeOf(const std::string& code) {
				if (code == "T_A") return TaskType::ATOMIC;
				if (code == "T_P") return TaskType::PARAMETRIC;
				if (code == "T_B") return TaskType::BEHAVIOUR;
				if (code == "T_S") return TaskType::SEQUENTIAL;
				if (code == "T_L") return TaskType::PARALLEL;
				if (code == "T_O") return TaskType::CHOICE;
				throw TaskLoadError("not a task code: " + code);
			}

			void Visit(Task& t, double parent_priority) {
				t.SetEffectivePriority(parent_priority * t.Priority());
				if (auto* c = dynamic_cast<ComplexTask*>(&t)) {
					const auto& children = c->Children();
					for (std::size_t i = 0; i < children.size(); ++i) {
						Task& child = *children[i];
						if (child.Id() == "0" && t.Id() != "0") child.SetId(t.Id() + "." + std::to_string(i + 1));
						Visit(child, t.EffectivePriority());
					}
				}
			}
		}

		void AssignTreeIds(Task& root) { Visit(root, 1.0); }

		std::unique_ptr<Condition> TaskFactory::BuildCondition(const Record& c) const {
			const auto& f = c.fields;
			const std::string& code = c.code;
			if (code == "C_N") return std::make_unique<NullCondition>();
			if (code == "C_F") return std::make_unique<FalseCondition>();
			if (code == "C_?") {
				const Record& lit = Ref(c, f[0]);
				return std::make_unique<PredicateCondition>(lit.fields[0].s, lit.fields[1].b);
			}
			if (code == "C_m") return std::make_unique<ParameterCondition>(f[0].s, f[1].s, f[2].n, f[3].n);
			if (code == "C_T") return std::make_unique<AbsoluteTimeCondition>(f[0].i);
			if (code == "C_W") return std::make_unique<ElapsedTimeCondition>(f[0].n);
			if (code == "C_L") {
				std::vector<ConditionPtr> children;
				for (const Record* child : RefsFrom(c, 1)) children.push_back(BuildCondition(*child));
				return std::make_unique<LogicCondition>(static_cast<LogicalOperation>(f[0].i), std::move(children));
			}
			if (code == "C_V") {
				const Record& v = Ref(c, f[0]);
				if (v.code == "V_PEER") return std::make_unique<ViewMatchCondition>(ViewMatchCondition::Peer(v.fields[1].i));
				return std::make_unique<ViewMatchCondition>(
				    ViewMatchCondition::Detection(v.fields[1].s, v.fields[2].n, v.fields[3].n, v.fields[4].n));
			}
			if (code == "C_S") return std::make_unique<TaskStateCondition>(f[0].s, f[1].s);
			if (code == "C_P") return std::make_unique<PositionCondition>(f[0].n, f[1].n, f[2].n, f[3].n, f[4].n);
			if (code == "C_P3") return std::make_unique<PositionCondition3D>(f[0].n, f[1].n, f[2].n, f[3].n, f[4].n, f[5].n, f[6].n);
			if (code == "C_G") return std::make_unique<GeoPositionCondition>(f[0].n, f[1].n, f[2].n, f[3].n, f[4].n);
			if (code == "C_H") return std::make_unique<AltitudeCondition>(f[0].n, f[1].n);
			throw TaskLoadError("unsupported condition " + code);
		}

		std::unique_ptr<Function> TaskFactory::BuildFunction(const Record& r) const {
			const auto& f = r.fields;
			if (r.code == "F_K") return std::make_unique<ConstantFunction>(f[0].n);
			if (r.code == "F_L") {
				const auto n = static_cast<std::size_t>(f[0].i);
				std::vector<double> k;
				std::vector<std::string> fields;
				for (std::size_t i = 0; i < n; ++i) k.push_back(f[1 + i].n);
				for (std::size_t i = 0; i < n; ++i) fields.push_back(f[2 + n + i].s);
				return std::make_unique<LinearFunction>(std::move(k), f[1 + n].n, std::move(fields));
			}
			if (r.code == "F_X") {
				const std::string& key = f[0].s;
				if (!registry_.Has(key)) throw TaskLoadError("unknown registry function " + key);
				const auto n = static_cast<std::size_t>(f[1].i);
				if (n != registry_.CoefficientCount(key))
					throw TaskLoadError("registry function " + key + " takes " + std::to_string(registry_.CoefficientCount(key)) +
					                    " coefficients, not " + std::to_string(n));
				std::vector<double> k;
				for (std::size_t i = 0; i < n; ++i) k.push_back(f[2 + i].n);
				return std::make_unique<RegistryCall>(key, std::move(k), registry_);
			}
			if (r.code == "F_S" || r.code == "F_P") {
				std::vector<FunctionPtr> children;
				for (const Record* child : RefsFrom(r, 0)) children.push_back(BuildFunction(*child));
				return std::make_unique<CombinedFunction>(r.code, std::move(children));
			}
			if (r.code == "F_C") return std::make_unique<ClampFunction>(f[0].n, f[1].n, BuildFunction(Ref(r, f[2])));
			throw TaskLoadError("unsupported function " + r.code);
		}

		TaskAction TaskFactory::BuildAction(const Record& a) const {
			const auto& f = a.fields;
			if (a.code == "A_MAP") {
				std::vector<std::pair<std::string, TaskAction>> entries;
				for (std::size_t i = 0; i + 1 < f.size(); i += 2) entries.emplace_back(f[i].s, BuildAction(Ref(a, f[i + 1])));
				return TaskAction::Map(std::move(entries));
			}
			if (a.code == "A_FN") {
				std::vector<std::shared_ptr<const Function>> functions;
				for (const Record* child : RefsFrom(a, 1)) functions.push_back(BuildFunction(*child));
				return TaskAction::Parametric(f[0].s, std::move(functions));
			}
			const auto info = Device::FindAction(a.code);
			if (!info) throw TaskLoadError("unsupported action " + a.code);
			std::optional<std::uint64_t> arg;
			if (Device::IsIntegerLayout(info->layout)) {
				std::vector<std::int64_t> v;
				for (const auto& x : f) v.push_back(x.i);
				arg = Device::PackArgument(info->layout, v);
			} else {
				std::vector<double> v;
				for (const auto& x : f) v.push_back(x.n);
				arg = Device::PackArgument(info->layout, v);
			}
			if (!arg) throw TaskLoadError("action values outside the layout range: " + a.code);
			return TaskAction::Leaf(Device::Action{a.code, *arg});
		}

		Requirement TaskFactory::BuildRequirement(const Record& r) const {
			Requirement q;
			q.code = r.code;
			const auto& f = r.fields;
			if (r.code == "R_S") {
				for (const Record* child : RefsFrom(r, 0)) q.children.push_back(BuildRequirement(*child));
			} else if (r.code == "R_F") {
				q.max_age = f[0].n;
				for (std::size_t i = 1; i < f.size(); ++i) q.items.push_back(f[i].s);
			} else if (r.code == "R_A") {
				for (const auto& x : f) q.items.push_back(x.s);
			} else if (r.code == "R_R" || r.code == "R_K") {
				q.name = f[0].s;
			} else if (r.code == "R_E") {
				q.name = f[0].s;
				q.amount = f[1].n;
			} else if (r.code == "R_C") {
				q.condition = BuildCondition(Ref(r, f[0]));
			}
			return q;
		}

		std::unique_ptr<Task> TaskFactory::BuildTask(const Record& t) const {
			const auto& f = t.fields;
			const TaskType type = TypeOf(t.code);
			std::size_t i = 3;
			long long k = 0;
			if (type == TaskType::PARALLEL) k = f[i++].i;
			std::shared_ptr<const Condition> start = BuildCondition(Ref(t, f[i++]));
			std::shared_ptr<const Condition> end = BuildCondition(Ref(t, f[i++]));
			std::shared_ptr<const Condition> until;
			if (type == TaskType::BEHAVIOUR) until = BuildCondition(Ref(t, f[i++]));
			std::optional<Requirement> req;
			if (i < f.size() && Ref(t, f[i]).kind == 'R') req = BuildRequirement(Ref(t, f[i++]));
			const std::vector<const Record*> rest = RefsFrom(t, i);

			std::unique_ptr<Task> task;
			switch (type) {
			case TaskType::ATOMIC:
			case TaskType::PARAMETRIC: {
				auto a = type == TaskType::ATOMIC ? std::make_unique<ATask>() : std::make_unique<ParametricATask>();
				std::vector<TaskAction> actions;
				for (const Record* r : rest) actions.push_back(BuildAction(*r));
				a->SetActions(std::move(actions));
				task = std::move(a);
				break;
			}
			case TaskType::BEHAVIOUR: {
				auto b = std::make_unique<Behaviour>();
				b->SetUntil(until);
				auto base = BuildTask(*rest.at(0));
				auto* base_a = dynamic_cast<ATask*>(base.get());
				if (!base_a) throw TaskLoadError("version 0.1 runs only a T_A or T_P as the base of a T_B");
				base.release();
				b->SetBase(std::unique_ptr<ATask>(base_a));
				task = std::move(b);
				break;
			}
			default: {
				auto c = std::make_unique<ComplexTask>();
				c->SetRequiredSuccesses(k);
				for (const Record* r : rest) c->AddChild(BuildTask(*r));
				task = std::move(c);
				break;
			}
			}
			task->SetCommon(t.code, type, f[0].s, f[1].n, static_cast<TaskState>(f[2].i), start, end, req, t);
			return task;
		}

		std::vector<std::unique_ptr<Task>> TaskFactory::BuildTasks(const std::string& text) const {
			auto parsed = Protocol::Parse(text);
			if (!parsed.Ok())
				throw TaskLoadError(std::string(Protocol::ErrorClassName(parsed.error->error_class)) + " at byte " +
				                    std::to_string(parsed.error->offset) + ": " + parsed.error->message);
			std::vector<std::unique_ptr<Task>> out;
			for (const auto& r : parsed.document.records) {
				if (r.kind != 'T') continue;
				auto task = BuildTask(r);
				AssignTreeIds(*task);
				out.push_back(std::move(task));
			}
			return out;
		}
	}
}
