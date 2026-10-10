// Worldview sync of the MRS layer (spec 15 §2): world fields and detections a robot measured go
// out in M_SYNC, a new peer is asked for what it knows (M_INFOREQ / M_INFO), and what peers
// share comes back in as views with source peer:<sender>.
#include <algorithm>
#include <cmath>

#include "mrs/algorithms/MrsLayer.h"

namespace MRS {
	namespace Algorithms {
		using Protocol::Field;
		using Protocol::FieldType;

		namespace {
			bool IsDetections(const std::string& prefix) { return prefix.rfind("det", 0) == 0; }

			// "det.person.3" -> "person".
			std::string DetectionClass(const std::string& id) {
				const auto first = id.find('.'), last = id.rfind('.');
				return first == std::string::npos || last <= first ? std::string() : id.substr(first + 1, last - first - 1);
			}
		}

		std::vector<Environment::View> MrsLayer::SharedViews(const std::string& prefix, bool own_only, bool changed_only, double t,
		                                                     std::vector<std::string>* keys) {
			std::vector<Environment::View> out;
			const auto& w = robot_.World();
			auto changed = [&](const std::string& key, double stamp) {
				if (!changed_only) return true;
				auto it = sync_sent_.find(key);
				return it == sync_sent_.end() || stamp > it->second;
			};
			if (IsDetections(prefix)) {
				const auto x = w.Scalar("pose.enu.x", t), y = w.Scalar("pose.enu.y", t), z = w.Scalar("pose.enu.z", t);
				for (const auto& [id, o] : w.Objects()) {
					if (o.cls != "detection" || id.rfind(prefix, 0) != 0) continue;
					if (own_only && Environment::IsPeerSource(o.source)) continue;
					if (c_.sync_range > 0.0 && own_only) {
						if (!x || !y || !z) continue;
						const double dx = o.position.x - *x, dy = o.position.y - *y, dz = o.position.z - *z;
						if (std::sqrt(dx * dx + dy * dy + dz * dz) > c_.sync_range) continue;
					}
					if (!changed(id, o.stamp)) continue;
					const std::string cls = DetectionClass(id);
					if (cls.empty()) continue;
					out.push_back({"V_DET", o.stamp, {o.position.x, o.position.y, o.position.z, o.confidence}, cls, {}});
					if (keys) keys->push_back(id);
				}
				return out;
			}
			if (Environment::IsSelfStateField(prefix)) return out;
			for (const auto& path : w.PathsWithPrefix(prefix)) {
				if (Environment::IsSelfStateField(path)) continue;
				const auto e = w.Raw(path);
				if (!e || !e->valid || !std::holds_alternative<double>(e->value)) continue;
				if (t - e->stamp > w.MaxAge(path)) continue;
				if (own_only && Environment::IsPeerSource(e->source)) continue;
				if (!changed(path, e->stamp)) continue;
				out.push_back({"V_FLD", e->stamp, {std::get<double>(e->value)}, path, {}});
				if (keys) keys->push_back(path);
			}
			return out;
		}

		void MrsLayer::PostViews(const std::string& code, const std::string& recipient, const std::vector<Environment::View>& views,
		                         double t) {
			std::size_t k = 0;
			while (k < views.size()) {
				auto r = messenger_.Begin(code, recipient, t);
				std::size_t n = 0;
				for (; k < views.size(); ++k, ++n) {
					r.fields.push_back(Field::MakeRef(r.children.size()));
					r.children.push_back(Environment::ToRecord(views[k]));
					if (max_payload_ > 0 && n > 0 && Comm::Encode(r).size() > max_payload_) {
						r.fields.pop_back();
						r.children.pop_back();
						break;
					}
				}
				messenger_.Post(r);
				++stats_.sync_sent;
				stats_.sync_views_sent += static_cast<long>(n);
			}
		}

		void MrsLayer::SendSync(double t) {
			for (const auto& entry : c_.sync) {
				double& next = sync_next_[entry.prefix];
				if (t < next) continue;
				next = t + entry.period;
				std::vector<std::string> keys;
				auto views = SharedViews(entry.prefix, true, true, t, &keys);
				if (views.empty()) continue;
				for (std::size_t k = 0; k < views.size(); ++k) sync_sent_[keys[k]] = views[k].stamp;
				PostViews("M_SYNC", "all", views, t);
				// On a slow link, sync takes at most sync_share of this robot's share (spec 15 §2.3).
				if (const double b = Budget(t); b > 0.0) {
					double bytes = 0.0;
					for (const auto& v : views) bytes += 40.0 + 8.0 * static_cast<double>(v.values.size()) + static_cast<double>(v.text.size());
					next = std::max(next, t + bytes * 8.0 / (c_.sync_share * b));
				}
			}
		}

		void MrsLayer::AskPeer(const std::string& peer, double t) {
			asked_.insert(peer);
			if (c_.sync.empty() || peer.empty() || peer[0] != 'r') return;  // robots only; the issuer keeps no worldview
			auto r = messenger_.Begin("M_INFOREQ", peer, t);
			r.fields.push_back(Field::MakeInt(static_cast<std::int64_t>(c_.sync.size())));
			for (const auto& entry : c_.sync) r.fields.push_back(Field::MakeText(FieldType::Id, entry.prefix));
			messenger_.Post(r);
			++stats_.info_requests;
		}

		int MrsLayer::AskPeers(const std::vector<std::string>& topics, double t) {
			int asked = 0;
			for (const Peer* p : peers_.Alive(t, c_.peer_timeout)) {
				if (p->name == self_ || p->name.empty() || p->name[0] != 'r') continue;  // robots only
				auto r = messenger_.Begin("M_INFOREQ", p->name, t);
				r.fields.push_back(Field::MakeInt(static_cast<std::int64_t>(topics.size())));
				for (const auto& topic : topics) r.fields.push_back(Field::MakeText(FieldType::Id, topic));
				messenger_.Post(r);
				++stats_.info_requests;
				++asked;
			}
			return asked;
		}

		void MrsLayer::OnInfoRequest(const Comm::Message& m, double t) {
			if (m.Broadcast() || m.SlotCount() < 1) return;
			std::vector<Environment::View> views;
			for (std::size_t k = 1; k < m.SlotCount(); ++k) {
				const std::string& topic = m.Slot(k).s;
				if (topic == "mission" || topic == "tasks" || topic == "profile") continue;  // not answered by robots in 0.1
				auto v = SharedViews(topic, false, false, t);
				views.insert(views.end(), v.begin(), v.end());
			}
			if (!views.empty()) PostViews("M_INFO", m.sender, views, t);
		}

		void MrsLayer::OnShared(const Comm::Message& m) {
			std::vector<Environment::View> views;
			for (const auto& child : m.record.children) {
				if (child.code != "V_FLD" && child.code != "V_DET") continue;
				auto v = Environment::ViewFromRecord(child);
				if (!v) continue;
				v->source = "peer:" + m.sender;
				views.push_back(std::move(*v));
			}
			stats_.sync_views_received += static_cast<long>(views.size());
			if (!views.empty()) robot_.InjectViews(std::move(views));
		}
	}
}
