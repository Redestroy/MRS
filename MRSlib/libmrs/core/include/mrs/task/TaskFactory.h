#pragma once
// Builds task objects from protocol records (spec 03). The records are expected to
// have passed the parser; the factory adds the checks that need a function registry.
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "mrs/protocol/Record.h"
#include "mrs/task/Condition.h"
#include "mrs/task/Function.h"
#include "mrs/task/Task.h"

namespace MRS {
	namespace Task {
		// A record that parses but cannot be loaded: an unknown registry key, a wrong
		// coefficient count (spec 02 §8), or a construct version 0.1 does not run.
		class TaskLoadError : public std::runtime_error {
		public:
			using std::runtime_error::runtime_error;
		};

		class TaskFactory {
		public:
			explicit TaskFactory(const FunctionRegistry& registry) : registry_(registry) {}

			std::unique_ptr<Task> BuildTask(const Protocol::Record& t) const;
			std::unique_ptr<Condition> BuildCondition(const Protocol::Record& c) const;
			std::unique_ptr<Function> BuildFunction(const Protocol::Record& f) const;
			TaskAction BuildAction(const Protocol::Record& a) const;
			Requirement BuildRequirement(const Protocol::Record& r) const;

			// Parses text and builds every top-level T record (others are ignored).
			std::vector<std::unique_ptr<Task>> BuildTasks(const std::string& text) const;

		private:
			const FunctionRegistry& registry_;
		};

		// Child i of task p gets the id p.i when its id is unassigned ("0"), and every task
		// gets its effective priority (spec 03 §2, §6.2).
		void AssignTreeIds(Task& root);
	}
}
