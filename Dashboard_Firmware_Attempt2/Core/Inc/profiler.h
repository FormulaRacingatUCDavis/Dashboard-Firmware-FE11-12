/*
 * profiler.h
 *
 *  Created on: Jan 14, 2025
 *      Author: IanKM
 */

#ifndef INC_PROFILER_H_
#define INC_PROFILER_H_

#include <stdint.h>

// Switch this to 1 to enable the profiler
#define FRUCD_PROFILER_ENABLED 1

typedef struct {
	// id: u32
	uint32_t magic;
	uint32_t thread_id;
	uint64_t start;
	uint64_t end;
	char scope_name[32];
} profiler_data_t;

extern uint64_t profiler_perf_timer;

void profiler_record_marker(profiler_data_t marker);

#if FRUCD_PROFILER_ENABLED
	#define PROFILER_SCOPE_AUTO(x) \
		profiler_data_t marker __attribute__((cleanup(profiler_auto_ended))); \
		marker.magic = 0xFA57FA57; \
		marker.scope_name = x; \
		marker.start = profiler_perf_timer; \
		marker.thread_id = 0;

	#define PROFILER_FUNC_AUTO() PROFILER_SCOPE_AUTO(__func__)

#else
	#define PROFILER_SCOPE_AUTO(x)
	#define PROFILER_FUNC_AUTO()
#endif

#endif /* INC_PROFILER_H_ */
