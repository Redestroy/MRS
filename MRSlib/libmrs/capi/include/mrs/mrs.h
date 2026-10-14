/* libmrs C API, bench slice (spec 16 §8-§9).
 *
 * Everything crosses the boundary as protocol text (.mrsd, .mrsp, .mrsi, A, V and I records) or as plain
 * numbers. Objects are opaque handles. Every call returns an mrs_status; on an error, mrs_last_error()
 * gives the message for the calling thread. Strings the library returns are freed with mrs_free().
 */
#ifndef MRS_MRS_H
#define MRS_MRS_H

#include <stddef.h>

#if defined(MRS_CAPI_SHARED)
#if defined(_WIN32)
#if defined(MRS_CAPI_BUILDING)
#define MRS_API __declspec(dllexport)
#else
#define MRS_API __declspec(dllimport)
#endif
#else
#define MRS_API __attribute__((visibility("default")))
#endif
#else
#define MRS_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum mrs_status {
	MRS_OK = 0,
	MRS_ERR_ARGUMENT = 1, /* a null handle or pointer, or a value out of range */
	MRS_ERR_PARSE = 2,    /* protocol text that does not parse */
	MRS_ERR_BUILD = 3,    /* a definition, map or setting the library refuses (BuildError) */
	MRS_ERR_NOT_FOUND = 4, /* no such node, provider or raw port */
	MRS_ERR_INTERNAL = 99
} mrs_status;

/* Action status of mrs_bench_apply (spec 03 §8.2). */
typedef enum mrs_action_status { MRS_ACTION_DONE = 0, MRS_ACTION_RUNNING = 1, MRS_ACTION_REJECTED = 2, MRS_ACTION_FAILED = 3 } mrs_action_status;

typedef struct mrs_registry mrs_registry;
typedef struct mrs_platform mrs_platform;
typedef struct mrs_bench mrs_bench;

MRS_API const char* mrs_version(void);
/* The message of the last error on this thread; empty after a successful call. */
MRS_API const char* mrs_last_error(void);
MRS_API void mrs_free(char* text);

/* --- Registry ------------------------------------------------------------------------------------- */

/* `sets` is a space-separated list of device sets: "base" (head.default, group.default) and "uav"
 * (spec 04 §3, includes base). */
MRS_API mrs_status mrs_registry_create(const char* sets, mrs_registry** out);
MRS_API void mrs_registry_destroy(mrs_registry* registry);

/* --- Platform from callbacks (spec 16 §9, user-written providers) -------------------------------- */

/* Strings returned by callbacks stay owned by the caller and must stay valid until the next callback. */
typedef struct mrs_platform_callbacks {
	void* user;
	/* The ports present: one per line, fields separated by tabs: type, address, identity, descriptor
	 * (the last two may be empty or left out). */
	const char* (*scan)(void* user);
	/* Opens a port; returns a port id >= 0, or -1 if it cannot be opened. */
	int (*open)(void* user, const char* type, const char* address);
	/* Fills up to `capacity` values and returns how many, or -1 when there is no new sample. May be null. */
	int (*read)(void* user, int port, double* values, int capacity);
	/* Returns 1 when written. May be null. */
	int (*write)(void* user, int port, const double* values, int count);
	/* Returns 1 when sent. May be null. */
	int (*send)(void* user, int port, const char* data);
	/* The next received message, or null. May be null. */
	const char* (*receive)(void* user, int port);
	/* Called once when a port is closed. May be null. */
	void (*close)(void* user, int port);
	/* Mission time, s. */
	double (*time)(void* user);
	/* Advances one control period; returns 0 when the platform stops. */
	int (*step)(void* user);
	/* Space-separated port type names it offers (spec 16 §4.5), or null for the built-in types. May be null. */
	const char* (*port_types)(void* user);
} mrs_platform_callbacks;

MRS_API mrs_status mrs_platform_create(const mrs_platform_callbacks* callbacks, mrs_platform** out);
/* Destroy the benches that use a platform first. */
MRS_API void mrs_platform_destroy(mrs_platform* platform);

/* --- Bench (spec 16 §8) ---------------------------------------------------------------------------- */

typedef struct mrs_bench_settings {
	double timeout;         /* s; 0: none; negative: the default, 5 s */
	const char* on_timeout; /* "default", "safe" or "continue"; null: "default" */
	const char* allow;      /* space-separated motion codes the bench accepts; may be null */
} mrs_bench_settings;

/* Builds a bench robot. `mrsp` and `mrsi` may be null; `settings` may be null for the defaults. */
MRS_API mrs_status mrs_bench_build(const char* mrsd, const char* mrsp, const char* mrsi, const mrs_bench_settings* settings,
                                   const mrs_registry* registry, mrs_platform* platform, mrs_bench** out);
MRS_API void mrs_bench_destroy(mrs_bench* bench);

/* One leaf A record, such as "A: A_MOT 30/", to the actuator `node` (or "any"). `timeout` < 0 and a
 * null `on_timeout` use the bench settings. A rejected action is MRS_OK with MRS_ACTION_REJECTED. */
MRS_API mrs_status mrs_bench_apply(mrs_bench* bench, const char* node, const char* action, double timeout, const char* on_timeout,
                                   mrs_action_status* status);
/* The node's new views, as V records. */
MRS_API mrs_status mrs_bench_sample(mrs_bench* bench, const char* node, char** views);
/* One line per node: full name, kind (D_H, D_J, ...), type key or interface, 1 if available, separated by tabs. */
MRS_API mrs_status mrs_bench_nodes(const mrs_bench* bench, char** text);
/* One line per fault: node, tab, reason. */
MRS_API mrs_status mrs_bench_faults(const mrs_bench* bench, char** text);
/* The boot check findings as I_R records. */
MRS_API mrs_status mrs_bench_check(const mrs_bench* bench, char** records);
/* The self model's capabilities as K records. */
MRS_API mrs_status mrs_bench_self(const mrs_bench* bench, char** records);
/* Every actuator to its safe state. */
MRS_API mrs_status mrs_bench_stop(mrs_bench* bench);
/* One step; `running` (may be null) is 0 when the platform stopped. */
MRS_API mrs_status mrs_bench_step(mrs_bench* bench, int* running);
MRS_API mrs_status mrs_bench_time(const mrs_bench* bench, double* t);

/* Raw ports (spec 16 §8.2): a port no node holds, on "platform" or a provider joint. */
MRS_API mrs_status mrs_bench_open_raw(mrs_bench* bench, const char* provider, const char* type, const char* address, int* handle);
/* `count` gets the number of values, or -1 when there is no new sample. */
MRS_API mrs_status mrs_raw_read(mrs_bench* bench, int handle, double* values, int capacity, int* count);
MRS_API mrs_status mrs_raw_write(mrs_bench* bench, int handle, const double* values, int count, int* written);
MRS_API mrs_status mrs_raw_send(mrs_bench* bench, int handle, const char* data, int* sent);
/* `message` is null when nothing was received. */
MRS_API mrs_status mrs_raw_receive(mrs_bench* bench, int handle, char** message);
MRS_API mrs_status mrs_raw_close(mrs_bench* bench, int handle);

#ifdef __cplusplus
}
#endif

#endif
