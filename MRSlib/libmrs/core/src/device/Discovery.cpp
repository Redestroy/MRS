#include "mrs/device/Discovery.h"

#include <memory>
#include <set>

#include "mrs/BuildError.h"
#include "mrs/device/DeviceNode.h"

namespace MRS {
	namespace Device {
		using Protocol::Field;
		using Protocol::FieldType;
		using Protocol::Record;

		namespace {
			Protocol::Document Versioned(double version) {
				Protocol::Document doc;
				Record rec;
				rec.kind = '@';
				rec.fields = {Field::MakeText(FieldType::Id, "MRS"), Field::MakeNum(version)};
				doc.records.push_back(rec);
				return doc;
			}

			bool ValidName(const std::string& s) {
				if (s.empty() || !((s[0] >= 'a' && s[0] <= 'z') || s[0] == '_')) return false;
				for (char c : s)
					if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_')) return false;
				return s != "nan" && s != "inf" && s != "infinity";
			}

			std::string FromAddress(const std::string& address) {
				std::string s;
				for (char c : address) {
					if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
					s += ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9')) ? c : '_';
				}
				return s;
			}

			void AddChild(Record& parent, Record child) {
				parent.fields.push_back(Field::MakeRef(parent.children.size()));
				parent.children.push_back(std::move(child));
			}

			class Discoverer {
			public:
				Discoverer(const DeviceRegistry& registry, const DiscoveryOptions& options)
				    : registry_(registry), options_(options), resolver_(registry, options.robot_map, LibraryMaps(options)),
				      report_{{}, Versioned(0.2), Versioned(0.2)} {}

				DiscoveryReport Run(const Record& head, Port::IPlatform& platform) {
					Record top;
					top.kind = 'D';
					top.code = "D_H";
					for (const auto& f : head.fields)
						if (f.type != FieldType::Ref) top.fields.push_back(f);
					if (top.fields.empty()) throw BuildError("discovery needs a D_H record");
					names_.insert(top.fields[0].s);
					names_.insert("base");
					Record base = Joint("base", {});
					Process(Port::kPlatformProvider, platform, base);
					AddChild(top, std::move(base));
					report_.mrsd = Versioned(0.2);
					report_.mrsd.records.push_back(std::move(top));
					return std::move(report_);
				}

			private:
				static std::vector<const IdentityMap*> LibraryMaps(const DiscoveryOptions& o) {
					return o.library ? o.library->Maps() : std::vector<const IdentityMap*>{};
				}

				static Record Joint(const std::string& name, const Params& params) {
					Record rec;
					rec.kind = 'D';
					rec.code = "D_J";
					rec.fields = {Field::MakeText(FieldType::Id, name), Field::MakeText(FieldType::Id, "mech_bus")};
					for (auto& f : params.ToFields()) rec.fields.push_back(std::move(f));
					return rec;
				}

				void Unknown(const std::string& provider, const Port::PortInfo& p) {
					Record rec;
					rec.kind = 'I';
					rec.code = "I_U";
					rec.fields = {Field::MakeText(FieldType::Id, provider), Field::MakeText(FieldType::Str, p.address),
					              Field::MakeText(FieldType::Id, Port::PortTypeName(p.type)), Field::MakeText(FieldType::Str, p.identity)};
					if (!p.descriptor.empty()) rec.fields.push_back(Field::MakeText(FieldType::Str, p.descriptor));
					report_.unknown.records.push_back(std::move(rec));
				}

				// §7.2 step 3.
				std::string Name(const Params& match, const Port::PortInfo& p, const std::string& type) {
					for (const std::string& candidate : {match.Text("name"), FromAddress(p.address)})
						if (ValidName(candidate) && !names_.count(candidate)) return *names_.insert(candidate).first;
					const auto id = Identity::Parse(p.identity);
					std::string model = FromAddress(id ? id->model : type.substr(0, type.find('.')));
					if (!ValidName(model)) model = "device";
					for (int k = 1;; ++k) {
						const std::string candidate = model + "_" + std::to_string(k);
						if (!names_.count(candidate)) return *names_.insert(candidate).first;
					}
				}

				void Process(const std::string& provider_name, Port::IPortProvider& provider, Record& parent) {
					for (const auto& p : provider.Scan()) {
						std::optional<IdentityMatch> match;
						try {
							match = resolver_.Resolve(p.identity, p.descriptor);
						} catch (const BuildError&) {
							match.reset();
						}
						if (!match) {
							Unknown(provider_name, p);
							continue;
						}
						if (match->Ignored()) continue;
						Params params;
						for (const auto& item : match->params.Items())
							if (item.key != "name" && item.key != "req") params.Add(item.key, item.value);

						// A part: an include, whose devices declare their own ports (§6.2).
						if (match->type.rfind(kPartPrefix, 0) == 0) {
							if (!options_.library || !options_.library->Part(match->type.substr(std::string(kPartPrefix).size()))) {
								Unknown(provider_name, p);
								continue;
							}
							Record rec;
							rec.kind = 'D';
							rec.code = "D_X";
							rec.fields = {Field::MakeText(FieldType::Id, Name(match->params, p, match->type)),
							              Field::MakeText(FieldType::Id, match->type)};
							for (auto& f : params.ToFields()) rec.fields.push_back(std::move(f));
							AddChild(parent, std::move(rec));
							continue;
						}

						// The class tells the node's kind and the requirement the port fills (§7.2 step 2).
						std::unique_ptr<DeviceNode> node;
						try {
							node = registry_.Create(match->type, "discovered", params);
						} catch (const BuildError&) {
							params.Set("device", StrValue(p.address));
							try {
								node = registry_.Create(match->type, "discovered", params);
							} catch (const BuildError&) {
								node.reset();
							}
						}
						const Port::PortRequirement* req = nullptr;
						if (node) {
							const std::string wanted = match->params.Text("req");
							int count = 0;
							for (const auto& r : node->PortRequirements()) {
								if (r.type != p.type || (!wanted.empty() && r.name != wanted)) continue;
								req = &r;
								++count;
							}
							if (count != 1) req = nullptr;
						}
						if (!req) {
							Unknown(provider_name, p);
							continue;
						}

						const std::string name = Name(match->params, p, match->type);
						auto* joint = dynamic_cast<ProviderJoint*>(node.get());
						Record rec;
						if (joint) {
							Params joint_params;
							joint_params.Add("provider", IdValue(match->type));
							for (const auto& item : params.Items()) joint_params.Add(item.key, item.value);
							rec = Joint(name, joint_params);
						} else {
							rec.kind = 'D';
							rec.code = NodeCode(node->Kind());
							rec.fields = {Field::MakeText(FieldType::Id, name), Field::MakeText(FieldType::Id, match->type)};
							for (auto& f : params.ToFields()) rec.fields.push_back(std::move(f));
						}
						if (match->descriptor)
							for (const auto& sub : match->descriptor->children)
								if (sub.kind != 'D') AddChild(rec, sub);

						// §7.2 step 6.
						Record pa;
						pa.kind = 'P';
						pa.code = "P_A";
						pa.fields = {Field::MakeText(FieldType::Id, name), Field::MakeText(FieldType::Id, req->name),
						             Field::MakeText(FieldType::Id, Port::PortTypeName(p.type)), Field::MakeText(FieldType::Str, p.address)};
						Params pa_params;
						if (!p.identity.empty()) pa_params.Add("identity", StrValue(p.identity));
						if (provider_name != Port::kPlatformProvider) pa_params.Add("via", IdValue(provider_name));
						for (auto& f : pa_params.ToFields()) pa.fields.push_back(std::move(f));
						report_.mrsp.records.push_back(std::move(pa));

						// A provider joint's own devices are its children (§7.2 step 4).
						if (joint) {
							Port::PortAssignment a;
							a.node = name;
							a.requirement = req->name;
							a.type = p.type;
							a.address = p.address;
							a.exclusive = req->exclusive;
							a.params = req->params;
							a.provider = provider_name;
							a.identity = p.identity;
							if (auto port = provider.Open(a)) {
								joint->AttachPort(req->name, std::move(port));
								if (joint->Start()) Process(name, *joint, rec);
							}
							keep_.push_back(std::move(node));
						}
						AddChild(parent, std::move(rec));
					}
				}

				const DeviceRegistry& registry_;
				const DiscoveryOptions& options_;
				IdentityResolver resolver_;
				DiscoveryReport report_;
				std::set<std::string> names_;
				std::vector<std::unique_ptr<DeviceNode>> keep_;  // started provider joints, until the end
			};
		}

		DiscoveryReport Discover(const Protocol::Record& head, const DeviceRegistry& registry, Port::IPlatform& platform,
		                         const DiscoveryOptions& options) {
			return Discoverer(registry, options).Run(head, platform);
		}
	}
}
