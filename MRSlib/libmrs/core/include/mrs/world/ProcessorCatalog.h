#pragma once
// Descriptions of the worldview processors (spec 05 §5.1), used by the self model to
// work out which fields a robot can provide without instantiating any processor.
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace MRS {
	namespace Environment {
		struct ProcessorDescriptor {
			std::string name;
			std::vector<std::string> subscriptions;  // view codes; any one is enough. Empty: none needed
			std::vector<std::string> needs;          // fields read
			std::vector<std::string> provides;       // fields written
			std::vector<std::pair<std::string, std::string>> optional;  // (view code, field) written when the view exists
		};

		struct CatalogResolution {
			std::set<std::string> fields;
			std::vector<std::string> active;  // processor names, in activation order
		};

		class ProcessorCatalog {
		public:
			ProcessorCatalog() = default;
			explicit ProcessorCatalog(std::vector<ProcessorDescriptor> entries) : entries_(std::move(entries)) {}

			// The version 0.1 catalog of spec 05 §5.1.
			static const ProcessorCatalog& Default();

			void Add(ProcessorDescriptor d) { entries_.push_back(std::move(d)); }
			const std::vector<ProcessorDescriptor>& Entries() const { return entries_; }

			// Fixed point (spec 04 §7): a processor is active when one of its views is produced (or it
			// subscribes to none) and all its needs are provided. A processor that would provide a field
			// already provided is skipped; catalog order decides.
			CatalogResolution Resolve(const std::set<std::string>& views) const;

		private:
			std::vector<ProcessorDescriptor> entries_;
		};
	}
}
