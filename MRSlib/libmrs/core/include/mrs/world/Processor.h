#pragma once
// View processors and their chain (spec 05 §5).
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "mrs/world/View.h"
#include "mrs/world/Worldview.h"

namespace MRS {
	namespace Environment {
		// A field a processor offers as one named source (spec 05 §5.1).
		struct Offer {
			std::string field;
			std::string source;
		};

		class IViewProcessor {
		public:
			virtual ~IViewProcessor() = default;
			virtual std::string Name() const = 0;
			virtual std::vector<std::string> Subscriptions() const = 0;  // view codes
			virtual std::vector<std::string> Provides() const { return {}; }
			virtual std::vector<Offer> Offers() const { return {}; }
			virtual std::vector<std::string> Needs() const { return {}; }
			virtual std::vector<std::pair<std::string, std::string>> Optional() const { return {}; }  // (view, field)
			virtual void Process(const View& v, Worldview& w, double t) {
				(void)v;
				(void)w;
				(void)t;
			}
			virtual void Tick(Worldview& w, double t) {
				(void)w;
				(void)t;
			}
		};

		// Forwards each view to the processors subscribed to its code.
		class ViewRouter {
		public:
			void Clear() { routes_.clear(); }
			void Subscribe(IViewProcessor* p);
			// Returns the number of processors the view reached.
			std::size_t Route(const View& v, Worldview& w, double t) const;

		private:
			std::vector<std::pair<std::string, IViewProcessor*>> routes_;
		};

		class ProcessorChain {
		public:
			ProcessorChain() = default;
			ProcessorChain(ProcessorChain&&) = default;
			ProcessorChain& operator=(ProcessorChain&&) = default;

			// Adding or removing a processor re-orders the chain on the next Build or Update.
			void Add(std::unique_ptr<IViewProcessor> processor);
			bool Remove(const std::string& name);
			IViewProcessor* Find(const std::string& name) const;

			// Checks the rules of spec 05 §5 (no field provided twice, or both provided and offered;
			// no cycle) and orders the processors so that each runs after those that provide or offer
			// its needs; ties keep the order of Add. Throws BuildError.
			void Build();
			const std::vector<IViewProcessor*>& Order();

			// Keeps the processors that are active for a robot producing these views (the fixed point of
			// spec 04 §7). Returns the names removed.
			std::vector<std::string> Prune(const std::set<std::string>& views);

			// One worldview update (spec 05 §5): route views, Tick in order, re-select offered fields.
			void Update(const std::vector<View>& views, Worldview& w, double t);

			std::vector<const IViewProcessor*> Processors() const;

		private:
			std::vector<std::unique_ptr<IViewProcessor>> processors_;
			std::vector<IViewProcessor*> order_;
			ViewRouter router_;
			bool built_ = false;
		};
	}
}
