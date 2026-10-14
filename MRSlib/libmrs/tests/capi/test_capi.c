/* The C API bench slice from plain C (spec 16 §9): a callback platform with a motor, a compass and a lamp. */
#include <stdio.h>
#include <string.h>

#include "mrs/mrs.h"

static int failures = 0;
#define CHECK(cond)                                                     \
	do {                                                                \
		if (!(cond)) {                                                  \
			fprintf(stderr, "%s:%d: CHECK(%s) failed: %s\n", __FILE__, __LINE__, #cond, mrs_last_error()); \
			++failures;                                                 \
		}                                                               \
	} while (0)

/* Ports 0 motor, 1 compass, 2 lamp. */
typedef struct Rig {
	double time;
	double motor;
	int motor_writes;
	int compass_fresh;
	double lamp;
	int open[3];
	int closed;
} Rig;

static const char* Scan(void* user) {
	(void)user;
	return "SIM\tm1\tacme:motor:1\nSIM\tcompass\nSIM\tlamp\t\t\n";
}
static int Open(void* user, const char* type, const char* address) {
	Rig* rig = (Rig*)user;
	int id = -1;
	if (strcmp(type, "SIM") != 0) return -1;
	if (strcmp(address, "m1") == 0) id = 0;
	if (strcmp(address, "compass") == 0) id = 1;
	if (strcmp(address, "lamp") == 0) id = 2;
	if (id >= 0) ++rig->open[id];
	return id;
}
static int Read(void* user, int port, double* values, int capacity) {
	Rig* rig = (Rig*)user;
	if (port != 1 || !rig->compass_fresh || capacity < 3) return -1;
	rig->compass_fresh = 0;
	values[0] = 1; /* north along body x: heading 0 */
	values[1] = 0;
	values[2] = 0;
	return 3;
}
static int Write(void* user, int port, const double* values, int count) {
	Rig* rig = (Rig*)user;
	if (count < 1) return 0;
	if (port == 0) {
		rig->motor = values[0];
		++rig->motor_writes;
	}
	if (port == 2) rig->lamp = values[0];
	return 1;
}
static void Close(void* user, int port) {
	(void)port;
	++((Rig*)user)->closed;
}
static double Time(void* user) { return ((Rig*)user)->time; }
static int Step(void* user) {
	((Rig*)user)->time += 0.01;
	return 1;
}

static const char* kRig = "@: MRS 0.2/\n"
                          "D: D_H rig head.default 2 robot_type rig id 1 D_1/\n"
                          "D_1: D_J base mech_bus 0 D_1..2/\n"
                          "D_1: D_A motor motor.webots 2 device \"m1\" max_speed 50/\n"
                          "D_2: D_S compass compass.webots 1 device \"compass\"/\n";

int main(void) {
	Rig rig;
	mrs_platform_callbacks cb;
	mrs_registry* registry = NULL;
	mrs_platform* platform = NULL;
	mrs_bench* bench = NULL;
	mrs_bench_settings settings;
	mrs_action_status status = MRS_ACTION_FAILED;
	char* text = NULL;
	int k, handle = -1, written = 0, running = 0;
	double t = -1;

	memset(&rig, 0, sizeof rig);
	memset(&cb, 0, sizeof cb);
	cb.user = &rig;
	cb.scan = Scan;
	cb.open = Open;
	cb.read = Read;
	cb.write = Write;
	cb.close = Close;
	cb.time = Time;
	cb.step = Step;

	CHECK(strlen(mrs_version()) > 0);
	CHECK(mrs_registry_create("uav", &registry) == MRS_OK);
	CHECK(mrs_registry_create("cars", &registry) == MRS_ERR_ARGUMENT);
	CHECK(strstr(mrs_last_error(), "cars") != NULL);
	CHECK(mrs_registry_create("uav", &registry) == MRS_OK);
	CHECK(mrs_platform_create(&cb, &platform) == MRS_OK);

	/* Errors come back as codes with a message. */
	CHECK(mrs_bench_build("D: D_H rig", NULL, NULL, NULL, registry, platform, &bench) == MRS_ERR_PARSE);
	CHECK(bench == NULL);
	CHECK(mrs_bench_build("D: D_H rig head.default 0 D_1/\nD_1: D_A x nothing.webots 0/", NULL, NULL, NULL, registry, platform,
	                      &bench) == MRS_ERR_BUILD);

	settings.timeout = 0.05;
	settings.on_timeout = NULL;
	settings.allow = "A_MOT";
	CHECK(mrs_bench_build(kRig, NULL, NULL, &settings, registry, platform, &bench) == MRS_OK);
	if (!bench) return 1;

	CHECK(mrs_bench_nodes(bench, &text) == MRS_OK);
	CHECK(strstr(text, "motor\tD_A\tmotor.webots\t1\n") != NULL);
	mrs_free(text);
	CHECK(mrs_bench_faults(bench, &text) == MRS_OK);
	CHECK(text[0] == '\0');
	mrs_free(text);
	CHECK(mrs_bench_self(bench, &text) == MRS_OK);
	CHECK(strstr(text, "A_MOT") != NULL);
	mrs_free(text);

	/* Apply, limits and the timeout. */
	CHECK(mrs_bench_apply(bench, "motor", "A: A_MOT 30/", -1, NULL, &status) == MRS_OK);
	CHECK(status == MRS_ACTION_DONE);
	CHECK(rig.motor == 30);
	CHECK(mrs_bench_apply(bench, "motor", "A: A_MOT 80/", -1, NULL, &status) == MRS_OK);
	CHECK(status == MRS_ACTION_REJECTED);
	CHECK(mrs_bench_apply(bench, "motor", "A: A_MOT 30/", -1, "later", &status) == MRS_ERR_ARGUMENT);
	CHECK(mrs_bench_apply(bench, "motor", "V: V_MAG 0 1/", -1, NULL, &status) != MRS_OK);
	for (k = 0; k < 10; ++k) CHECK(mrs_bench_step(bench, &running) == MRS_OK);
	CHECK(running == 1);
	CHECK(rig.motor == 0); /* the rotor's default on a timeout is speed zero */
	CHECK(mrs_bench_time(bench, &t) == MRS_OK);
	CHECK(t > 0.09);

	/* Sampling. */
	rig.compass_fresh = 1;
	CHECK(mrs_bench_sample(bench, "compass", &text) == MRS_OK);
	CHECK(strstr(text, "V_MAG") != NULL);
	mrs_free(text);
	CHECK(mrs_bench_sample(bench, "nowhere", &text) == MRS_ERR_NOT_FOUND);

	/* A raw port. */
	CHECK(mrs_bench_open_raw(bench, "platform", "SIM", "lamp", &handle) == MRS_OK);
	{
		double on = 1;
		CHECK(mrs_raw_write(bench, handle, &on, 1, &written) == MRS_OK);
	}
	CHECK(written == 1);
	CHECK(rig.lamp == 1);
	CHECK(mrs_bench_open_raw(bench, "platform", "SIM", "m1", &handle) == MRS_ERR_BUILD); /* held by the motor */
	CHECK(mrs_raw_close(bench, 99) == MRS_ERR_NOT_FOUND);

	CHECK(mrs_bench_stop(bench) == MRS_OK);
	mrs_bench_destroy(bench);
	CHECK(rig.closed == 3);
	mrs_platform_destroy(platform);
	mrs_registry_destroy(registry);
	if (failures) fprintf(stderr, "%d checks failed\n", failures);
	else printf("C API: all checks passed\n");
	return failures ? 1 : 0;
}
