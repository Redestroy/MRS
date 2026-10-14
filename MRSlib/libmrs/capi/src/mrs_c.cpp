// libmrs C API, bench slice (spec 16 §9).
#include "mrs/mrs.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "mrs/BuildError.h"
#include "mrs/device/Bench.h"
#include "mrs/device/uav/UavDevices.h"
#include "mrs/protocol/Parser.h"
#include "mrs/protocol/Writer.h"
#include "mrs/world/View.h"

using namespace MRS;

struct mrs_registry {
	Device::DeviceRegistry registry;
};

namespace {
	thread_local std::string last_error;

	struct ParseFailure {
		std::string message;
	};
	struct NotFound {
		std::string message;
	};

	mrs_status Fail(mrs_status status, std::string message) {
		last_error = std::move(message);
		return status;
	}

	// Runs `body`, turning exceptions into a status and the thread's last error.
	template <typename F>
	mrs_status Guard(F&& body) {
		last_error.clear();
		try {
			return body();
		} catch (const ParseFailure& e) {
			return Fail(MRS_ERR_PARSE, e.message);
		} catch (const NotFound& e) {
			return Fail(MRS_ERR_NOT_FOUND, e.message);
		} catch (const BuildError& e) {
			return Fail(MRS_ERR_BUILD, e.what());
		} catch (const std::exception& e) {
			return Fail(MRS_ERR_INTERNAL, e.what());
		} catch (...) {
			return Fail(MRS_ERR_INTERNAL, "unknown error");
		}
	}

	char* Copy(const std::string& s) {
		char* out = static_cast<char*>(std::malloc(s.size() + 1));
		if (!out) throw std::bad_alloc();
		std::memcpy(out, s.c_str(), s.size() + 1);
		return out;
	}

	// Protocol text; a version line is added when the text has none.
	Protocol::Document ParseText(const char* text, const char* what) {
		std::string s = text;
		std::size_t i = s.find_first_not_of(" \t\r\n");
		if (i == std::string::npos || s[i] != '@') s = "@: MRS 0.2/\n" + s;
		auto result = Protocol::Parse(s);
		if (!result.Ok())
			throw ParseFailure{std::string(what) + ": " + Protocol::ErrorClassName(result.error->error_class) + " at byte " +
			                   std::to_string(result.error->offset) + ": " + result.error->message};
		return result.document;
	}

	std::vector<std::string> Split(const std::string& s, char sep) {
		std::vector<std::string> out;
		std::string item;
		std::istringstream in(s);
		while (std::getline(in, item, sep)) out.push_back(item);
		return out;
	}

	std::vector<std::string> Words(const char* text) {
		std::vector<std::string> out;
		if (!text) return out;
		std::istringstream in(text);
		std::string w;
		while (in >> w) out.push_back(w);
		return out;
	}

	Port::PortType TypeFromName(const std::string& name) {
		auto type = Port::PortType::FromName(name);
		if (!type) throw BuildError("\"" + name + "\" is not a port type name");
		return *type;
	}

	// A port of a callback platform.
	class CallbackPort : public ::MRS::Port::Port {
	public:
		CallbackPort(::MRS::Port::PortAssignment a, const mrs_platform_callbacks& cb, int id)
		    : ::MRS::Port::Port(std::move(a)), cb_(cb), id_(id) {}
		~CallbackPort() override {
			if (cb_.close) cb_.close(cb_.user, id_);
		}
		bool Read(std::vector<double>& values) override {
			if (!cb_.read) return false;
			double buffer[64];
			const int n = cb_.read(cb_.user, id_, buffer, 64);
			if (n < 0) return false;
			values.assign(buffer, buffer + std::min(n, 64));
			return true;
		}
		bool Write(const std::vector<double>& values) override {
			return cb_.write && cb_.write(cb_.user, id_, values.data(), static_cast<int>(values.size())) != 0;
		}
		bool Send(const std::string& data) override { return cb_.send && cb_.send(cb_.user, id_, data.c_str()) != 0; }
		std::optional<std::string> Receive() override {
			if (!cb_.receive) return std::nullopt;
			const char* m = cb_.receive(cb_.user, id_);
			if (!m) return std::nullopt;
			return std::string(m);
		}

	private:
		mrs_platform_callbacks cb_;
		int id_;
	};
}

struct mrs_platform : Port::IPlatform {
	explicit mrs_platform(const mrs_platform_callbacks& cb) : cb(cb) {}

	std::vector<Port::PortInfo> Scan() override {
		std::vector<Port::PortInfo> out;
		const char* text = cb.scan ? cb.scan(cb.user) : nullptr;
		if (!text) return out;
		for (const auto& line : Split(text, '\n')) {
			if (line.empty() || line == "\r") continue;
			auto fields = Split(line, '\t');
			while (fields.size() < 4) fields.emplace_back();
			if (!fields[3].empty() && fields[3].back() == '\r') fields[3].pop_back();
			out.push_back({TypeFromName(fields[0]), fields[1], fields[2], fields[3]});
		}
		return out;
	}

	std::unique_ptr<Port::Port> Open(const Port::PortAssignment& a) override {
		if (!cb.open) return nullptr;
		const int id = cb.open(cb.user, Port::PortTypeName(a.type).c_str(), a.address.c_str());
		if (id < 0) return nullptr;
		return std::make_unique<CallbackPort>(a, cb, id);
	}

	std::vector<Port::PortType> PortTypes() const override {
		std::vector<Port::PortType> out;
		if (cb.port_types)
			for (const auto& name : Words(cb.port_types(cb.user))) out.push_back(TypeFromName(name));
		return out;
	}

	double Time() const override { return cb.time ? cb.time(cb.user) : 0.0; }
	bool Step() override { return cb.step ? cb.step(cb.user) != 0 : false; }

	mrs_platform_callbacks cb;
};

struct mrs_bench {
	Device::Bench bench;
};

// Outside the extern "C" block: it throws, and MSVC assumes extern "C" functions do not (/EHsc).
namespace {
	Port::Port& RawPort(mrs_bench* bench, int handle) {
		Port::Port* p = bench->bench.Raw(handle);
		if (!p) throw NotFound{"no raw port " + std::to_string(handle)};
		return *p;
	}
}

extern "C" {

const char* mrs_version(void) { return "0.2.0"; }
const char* mrs_last_error(void) { return last_error.c_str(); }
void mrs_free(char* text) { std::free(text); }

mrs_status mrs_registry_create(const char* sets, mrs_registry** out) {
	return Guard([&] {
		if (!out) return Fail(MRS_ERR_ARGUMENT, "out is null");
		*out = nullptr;
		auto r = std::make_unique<mrs_registry>();
		auto words = Words(sets);
		if (words.empty()) words.push_back("base");
		bool base = false, uav = false;
		for (const auto& w : words) {
			if (w == "base") base = true;
			else if (w == "uav") uav = true;
			else return Fail(MRS_ERR_ARGUMENT, "unknown device set " + w + " (base, uav)");
		}
		if (uav) Device::Uav::RegisterUavDevices(r->registry);
		else if (base) Device::RegisterBaseDevices(r->registry);
		*out = r.release();
		return MRS_OK;
	});
}

void mrs_registry_destroy(mrs_registry* registry) { delete registry; }

mrs_status mrs_platform_create(const mrs_platform_callbacks* callbacks, mrs_platform** out) {
	return Guard([&] {
		if (!out) return Fail(MRS_ERR_ARGUMENT, "out is null");
		*out = nullptr;
		if (!callbacks || !callbacks->scan || !callbacks->open || !callbacks->time || !callbacks->step)
			return Fail(MRS_ERR_ARGUMENT, "scan, open, time and step are required");
		*out = new mrs_platform(*callbacks);
		return MRS_OK;
	});
}

void mrs_platform_destroy(mrs_platform* platform) { delete platform; }

mrs_status mrs_bench_build(const char* mrsd, const char* mrsp, const char* mrsi, const mrs_bench_settings* settings,
                           const mrs_registry* registry, mrs_platform* platform, mrs_bench** out) {
	return Guard([&] {
		if (!out) return Fail(MRS_ERR_ARGUMENT, "out is null");
		*out = nullptr;
		if (!mrsd || !registry || !platform) return Fail(MRS_ERR_ARGUMENT, "mrsd, registry and platform are required");
		const auto definition = ParseText(mrsd, "definition");
		std::optional<Port::PortMap> port_map;
		std::optional<Device::IdentityMap> identity_map;
		Device::BuildOptions options;
		if (mrsp) {
			port_map = Port::PortMap::FromDocument(ParseText(mrsp, "port map"));
			options.port_map = &*port_map;
		}
		if (mrsi) {
			identity_map = Device::IdentityMap::FromDocument(ParseText(mrsi, "identity map"));
			options.identity_map = &*identity_map;
		}
		options.bench = true;
		Device::BenchSettings s;
		if (settings) {
			if (settings->timeout >= 0) s.timeout = settings->timeout;
			if (settings->on_timeout) {
				const auto mode = Device::TimeoutModeFromName(settings->on_timeout);
				if (!mode) return Fail(MRS_ERR_ARGUMENT, "on_timeout must be default, safe or continue");
				s.on_timeout = *mode;
			}
			for (const auto& code : Words(settings->allow)) s.allow.insert(code);
		}
		auto robot = Device::BuildRobot(definition, registry->registry, *platform, options);
		*out = new mrs_bench{Device::Bench(std::move(robot), *platform, std::move(s))};
		return MRS_OK;
	});
}

void mrs_bench_destroy(mrs_bench* bench) { delete bench; }

mrs_status mrs_bench_apply(mrs_bench* bench, const char* node, const char* action, double timeout, const char* on_timeout,
                           mrs_action_status* status) {
	return Guard([&] {
		if (!bench || !node || !action || !status) return Fail(MRS_ERR_ARGUMENT, "bench, node, action and status are required");
		const auto doc = ParseText(action, "action");
		const Protocol::Record* rec = nullptr;
		for (const auto& r : doc.records)
			if (r.kind != '@') {
				if (rec) return Fail(MRS_ERR_ARGUMENT, "one action at a time");
				rec = &r;
			}
		if (!rec || rec->kind != 'A') return Fail(MRS_ERR_ARGUMENT, "the action must be one A record");
		std::optional<Device::TimeoutMode> mode;
		if (on_timeout) {
			mode = Device::TimeoutModeFromName(on_timeout);
			if (!mode) return Fail(MRS_ERR_ARGUMENT, "on_timeout must be default, safe or continue");
		}
		const auto result = bench->bench.Apply(node, *rec, timeout >= 0 ? std::optional<double>(timeout) : std::nullopt, mode);
		*status = static_cast<mrs_action_status>(static_cast<int>(result));
		return MRS_OK;
	});
}

mrs_status mrs_bench_sample(mrs_bench* bench, const char* node, char** views) {
	return Guard([&] {
		if (!bench || !node || !views) return Fail(MRS_ERR_ARGUMENT, "bench, node and views are required");
		*views = nullptr;
		Protocol::Document doc;
		try {
			for (const auto& v : bench->bench.Sample(node)) doc.records.push_back(Environment::ToRecord(v));
		} catch (const BuildError& e) {
			throw NotFound{e.what()};
		}
		*views = Copy(doc.records.empty() ? std::string() : Protocol::Write(doc));
		return MRS_OK;
	});
}

mrs_status mrs_bench_nodes(const mrs_bench* bench, char** text) {
	return Guard([&] {
		if (!bench || !text) return Fail(MRS_ERR_ARGUMENT, "bench and text are required");
		std::string s;
		for (const auto& n : bench->bench.Nodes()) s += n.name + "\t" + n.kind + "\t" + n.type + "\t" + (n.available ? "1" : "0") + "\n";
		*text = Copy(s);
		return MRS_OK;
	});
}

mrs_status mrs_bench_faults(const mrs_bench* bench, char** text) {
	return Guard([&] {
		if (!bench || !text) return Fail(MRS_ERR_ARGUMENT, "bench and text are required");
		std::string s;
		for (const auto& f : bench->bench.Faults()) s += f.node + "\t" + f.reason + "\n";
		*text = Copy(s);
		return MRS_OK;
	});
}

mrs_status mrs_bench_check(const mrs_bench* bench, char** records) {
	return Guard([&] {
		if (!bench || !records) return Fail(MRS_ERR_ARGUMENT, "bench and records are required");
		Protocol::Document doc;
		for (const auto& f : bench->bench.Check()) doc.records.push_back(f.ToRecord());
		*records = Copy(doc.records.empty() ? std::string() : Protocol::Write(doc));
		return MRS_OK;
	});
}

mrs_status mrs_bench_self(const mrs_bench* bench, char** records) {
	return Guard([&] {
		if (!bench || !records) return Fail(MRS_ERR_ARGUMENT, "bench and records are required");
		Protocol::Document doc;
		for (const auto& c : bench->bench.Self().capabilities) doc.records.push_back(c.ToRecord());
		*records = Copy(doc.records.empty() ? std::string() : Protocol::Write(doc));
		return MRS_OK;
	});
}

mrs_status mrs_bench_stop(mrs_bench* bench) {
	return Guard([&] {
		if (!bench) return Fail(MRS_ERR_ARGUMENT, "bench is null");
		bench->bench.Stop();
		return MRS_OK;
	});
}

mrs_status mrs_bench_step(mrs_bench* bench, int* running) {
	return Guard([&] {
		if (!bench) return Fail(MRS_ERR_ARGUMENT, "bench is null");
		const bool r = bench->bench.Step();
		if (running) *running = r ? 1 : 0;
		return MRS_OK;
	});
}

mrs_status mrs_bench_time(const mrs_bench* bench, double* t) {
	return Guard([&] {
		if (!bench || !t) return Fail(MRS_ERR_ARGUMENT, "bench and t are required");
		*t = bench->bench.Time();
		return MRS_OK;
	});
}

mrs_status mrs_bench_open_raw(mrs_bench* bench, const char* provider, const char* type, const char* address, int* handle) {
	return Guard([&] {
		if (!bench || !provider || !type || !address || !handle) return Fail(MRS_ERR_ARGUMENT, "all arguments are required");
		*handle = -1;
		const int h = bench->bench.OpenRaw(provider, TypeFromName(type), address);
		if (h < 0) return Fail(MRS_ERR_NOT_FOUND, std::string("the provider did not open ") + type + " \"" + address + "\"");
		*handle = h;
		return MRS_OK;
	});
}

mrs_status mrs_raw_read(mrs_bench* bench, int handle, double* values, int capacity, int* count) {
	return Guard([&] {
		if (!bench || !count || capacity < 0 || (capacity > 0 && !values)) return Fail(MRS_ERR_ARGUMENT, "bad arguments");
		std::vector<double> v;
		if (!RawPort(bench, handle).Read(v)) {
			*count = -1;
			return MRS_OK;
		}
		const int n = std::min(capacity, static_cast<int>(v.size()));
		for (int k = 0; k < n; ++k) values[k] = v[k];
		*count = n;
		return MRS_OK;
	});
}

mrs_status mrs_raw_write(mrs_bench* bench, int handle, const double* values, int count, int* written) {
	return Guard([&] {
		if (!bench || count < 0 || (count > 0 && !values)) return Fail(MRS_ERR_ARGUMENT, "bad arguments");
		const bool ok = RawPort(bench, handle).Write(std::vector<double>(values, values + count));
		if (written) *written = ok ? 1 : 0;
		return MRS_OK;
	});
}

mrs_status mrs_raw_send(mrs_bench* bench, int handle, const char* data, int* sent) {
	return Guard([&] {
		if (!bench || !data) return Fail(MRS_ERR_ARGUMENT, "bench and data are required");
		const bool ok = RawPort(bench, handle).Send(data);
		if (sent) *sent = ok ? 1 : 0;
		return MRS_OK;
	});
}

mrs_status mrs_raw_receive(mrs_bench* bench, int handle, char** message) {
	return Guard([&] {
		if (!bench || !message) return Fail(MRS_ERR_ARGUMENT, "bench and message are required");
		*message = nullptr;
		if (auto m = RawPort(bench, handle).Receive()) *message = Copy(*m);
		return MRS_OK;
	});
}

mrs_status mrs_raw_close(mrs_bench* bench, int handle) {
	return Guard([&] {
		if (!bench) return Fail(MRS_ERR_ARGUMENT, "bench is null");
		RawPort(bench, handle);
		bench->bench.CloseRaw(handle);
		return MRS_OK;
	});
}

}  // extern "C"
