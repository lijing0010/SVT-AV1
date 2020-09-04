/*
* Copyright(c) 2019 Intel Corporation
* SPDX - License - Identifier: BSD - 2 - Clause - Patent
*/

#ifndef EbRestProcess_h
#define EbRestProcess_h

#include "EbDefinitions.h"

/**************************************
 * Extern Function Declarations
 **************************************/
#if !RE_ENCODE_SUPPORT
extern EbErrorType rest_context_ctor(EbThreadContext *  thread_context_ptr,
                                     const EbEncHandle *enc_handle_ptr, int index, int demux_index);
#else
extern EbErrorType rest_context_ctor(EbThreadContext *  thread_context_ptr,
                                     const EbEncHandle *enc_handle_ptr, int index);
#endif

extern void *rest_kernel(void *input_ptr);

#endif
