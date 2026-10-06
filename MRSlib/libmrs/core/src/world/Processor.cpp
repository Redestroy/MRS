#include "mrs/world/Processor.h"

#include <algorithm>
#include <map>

#include "mrs/BuildError.h"
#include "mrs/world/ProcessorCatalog.h"

namespace MRS {
	namespace Environment {
		void ViewRouter::Subscribe(IViewProcessor* p) {
			for (const auto& code : p->Subscriptions()) routes_.emplace_back(code, p);
		}

		std::size_t ViewRouter::Route(const View& v, Worldview& w, double t) const {
			std::size_t n = 0;
			for (const auto& r : routes_)
				if (r.first == v.code) {
					r.second->Process(v, w, t);
					++n;
				}
			return n;
		}

		void ProcessorChain::Add(std::unique_ptr<IViewProcessor> processor) {
			processors_.push_back(std::move(processor));
			built_ = false;
		}

		bool ProcessorChain::Remove(const std::string& name) {
			auto it = std::find_if(processors_.begin(), processors_.end(), [&](const auto& p) { return p->Name() == name; });
			if (it == processors_.end()) return false;
			processors_.erase(it);
			built_ = false;
			return true;
		}

		IViewProcessor* ProcessorChain::Find(const std::string& name) const {
			for (const auto& p : processors_)
				if (p->Name() == name) return p.get();
			return nullptr;
		}

		std::vector<const IViewProcessor*> ProcessorChain::Processors() const {
			std::vector<const IViewProcessor*> out;
			for (const auto& p : processors_) out.push_back(p.get());
			return out;
		}

		void ProcessorChain::Build() {
			std::map<std::string, std::string> provided;  // field -> processor
			std::map<std::string, std::vector<std::size_t>> writers;  // field -> processors providing or offering it
			for (std::size_t k = 0; k < processors_.size(); ++k) {
				const auto& p = processors_[k];
				for (const auto& f : p->Provides()) {
					auto [it, fresh] = provided.emplace(f, p->Name());
					if (!fresh) throw BuildError("field " + f + " is provided by both " + it->second + " and " + p->Name());
					writers[f].push_back(k);
				}
				for (const auto& o : p->Offers()) writers[o.field].push_back(k);
			}
			for (const auto& p : processors_)
				for (const auto& o : p->Offers())
					if (provided.count(o.field))
						throw BuildError("field " + o.field + " is provided by " + provided[o.field] + " and offered by " + p->Name());

			// Kahn's algorithm, picking the earliest-added ready processor each time.
			const std::size_t n = processors_.size();
			std::vector<std::vector<std::size_t>> after(n);
			std::vector<std::size_t> pending(n, 0);
			for (std::size_t k = 0; k < n; ++k)
				for (const auto& need : processors_[k]->Needs()) {
					auto it = writers.find(need);
					if (it == writers.end()) continue;
					for (std::size_t w : it->second) {
						if (w == k) continue;
						after[w].push_back(k);
						++pending[k];
					}
				}
			order_.clear();
			std::vector<bool> placed(n, false);
			for (std::size_t round = 0; round < n; ++round) {
				std::size_t next = n;
				for (std::size_t k = 0; k < n; ++k)
					if (!placed[k] && pending[k] == 0) {
						next = k;
						break;
					}
				if (next == n) {
					std::string names;
					for (std::size_t k = 0; k < n; ++k)
						if (!placed[k]) names += (names.empty() ? "" : ", ") + processors_[k]->Name();
					throw BuildError("processor needs form a cycle: " + names);
				}
				placed[next] = true;
				order_.push_back(processors_[next].get());
				for (std::size_t k : after[next]) --pending[k];
			}
			router_.Clear();
			for (IViewProcessor* p : order_) router_.Subscribe(p);
			built_ = true;
		}

		const std::vector<IViewProcessor*>& ProcessorChain::Order() {
			if (!built_) Build();
			return order_;
		}

		std::vector<std::string> ProcessorChain::Prune(const std::set<std::string>& views) {
			const auto resolution = ProcessorCatalog::FromProcessors(Processors()).Resolve(views);
			std::vector<std::string> removed;
			for (const auto& p : Processors())
				if (std::find(resolution.active.begin(), resolution.active.end(), p->Name()) == resolution.active.end())
					removed.push_back(p->Name());
			for (const auto& name : removed) Remove(name);
			return removed;
		}

		void ProcessorChain::Update(const std::vector<View>& views, Worldview& w, double t) {
			if (!built_) Build();
			for (const auto& v : views) router_.Route(v, w, t);
			for (IViewProcessor* p : order_) p->Tick(w, t);
			w.RefreshSources(t);
		}
	}
}
