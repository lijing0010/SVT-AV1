/*
* Copyright(c) 2019 Intel Corporation
*
* This source code is subject to the terms of the BSD 2 Clause License and
* the Alliance for Open Media Patent License 1.0. If the BSD 2 Clause License
* was not distributed with this source code in the LICENSE file, you can
* obtain it at https://www.aomedia.org/license/software-license. If the Alliance for Open
* Media Patent License 1.0 was not distributed with this source code in the
* PATENTS file, you can obtain it at https://www.aomedia.org/license/patent-license.
*/

#ifndef EbTime_h
#define EbTime_h

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#define NANOSECS_PER_SEC ((uint32_t)(1000000000L))

void eb_start_time(uint64_t *start_seconds, uint64_t *start_u_seconds);
void eb_finish_time(uint64_t *finish_seconds, uint64_t *finish_u_seconds);
void eb_compute_overall_elapsed_time(uint64_t start_seconds, uint64_t start_u_seconds,
                                     uint64_t finish_seconds, uint64_t finish_u_seconds,
                                     double *duration);
void eb_compute_overall_elapsed_time_ms(uint64_t start_seconds, uint64_t start_u_seconds,
                                        uint64_t finish_seconds, uint64_t finish_u_seconds,
                                        double *duration);
void eb_sleep_ms(uint64_t milli_seconds);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // EbTime_h
/* File EOF */
