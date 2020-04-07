/*
* Copyright(c) 2019 Intel Corporation
* SPDX - License - Identifier: BSD - 2 - Clause - Patent
*/

/*
* Copyright (c) 2016, Alliance for Open Media. All rights reserved
*
* This source code is subject to the terms of the BSD 2 Clause License and
* the Alliance for Open Media Patent License 1.0. If the BSD 2 Clause License
* was not distributed with this source code in the LICENSE file, you can
* obtain it at www.aomedia.org/license/software. If the Alliance for Open
* Media Patent License 1.0 was not distributed with this source code in the
* PATENTS file, you can obtain it at www.aomedia.org/license/patent.
*/
#include <stdlib.h>

#include "EbDefinitions.h"
#include "EbEncHandle.h"
#include "EbRateControlProcess.h"
#include "EbSequenceControlSet.h"
#include "EbPictureControlSet.h"
#include "EbUtility.h"
#include "EbSvtAv1ErrorCodes.h"
#include "EbEntropyCoding.h"

#include "EbRateControlResults.h"
#include "EbRateControlTasks.h"
#if CUTREE_LA
#include "EbRateDistortionCost.h"
#endif

#include "EbSegmentation.h"
#include "EbLog.h"

static const uint32_t rate_percentage_layer_array[EB_MAX_TEMPORAL_LAYERS][EB_MAX_TEMPORAL_LAYERS] =
    {{100, 0, 0, 0, 0, 0},
     {70, 30, 0, 0, 0, 0},
     {70, 15, 15, 0, 0, 0},
     {55, 15, 15, 15, 0, 0},
     {40, 15, 15, 15, 15, 0},
     {30, 10, 15, 15, 15, 15}};

// range from 0 to 51
// precision is 16 bits
static const uint64_t two_to_power_qp_over_three[] = {
    0x10000,     0x1428A,     0x19660,    0x20000,    0x28514,    0x32CC0,    0x40000,
    0x50A29,     0x65980,     0x80000,    0xA1451,    0xCB2FF,    0x100000,   0x1428A3,
    0x1965FF,    0x200000,    0x285146,   0x32CBFD,   0x400000,   0x50A28C,   0x6597FB,
    0x800000,    0xA14518,    0xCB2FF5,   0x1000000,  0x1428A30,  0x1965FEA,  0x2000000,
    0x285145F,   0x32CBFD5,   0x4000000,  0x50A28BE,  0x6597FA9,  0x8000000,  0xA14517D,
    0xCB2FF53,   0x10000000,  0x1428A2FA, 0x1965FEA5, 0x20000000, 0x285145F3, 0x32CBFD4A,
    0x40000000,  0x50A28BE6,  0x6597FA95, 0x80000000, 0xA14517CC, 0xCB2FF52A, 0x100000000,
    0x1428A2F99, 0x1965FEA54, 0x200000000};

/**************************************
 * Coded Frames Stats
 **************************************/
typedef struct CodedFramesStatsEntry {
    EbDctor  dctor;
    uint64_t picture_number;
    int64_t  frame_total_bit_actual;
    EbBool   end_of_sequence_flag;
} CodedFramesStatsEntry;

typedef struct RateControlIntervalParamContext {
    EbDctor                   dctor;
    uint64_t                  first_poc;
    uint64_t                  last_poc;
    EbBool                    in_use;
    EbBool                    was_used;
    uint64_t                  processed_frames_number;
    EbBool                    last_gop;
    RateControlLayerContext **rate_control_layer_array;

    int64_t  virtual_buffer_level;
    int64_t  previous_virtual_buffer_level;
    uint32_t intra_frames_qp;
    uint8_t  intra_frames_qp_bef_scal;

    uint32_t next_gop_intra_frame_qp;
    uint64_t first_pic_pred_bits;
    uint64_t first_pic_actual_bits;
    uint16_t first_pic_pred_qp;
    uint16_t first_pic_actual_qp;
    EbBool   first_pic_actual_qp_assigned;
    EbBool   scene_change_in_gop;
    int64_t  extra_ap_bit_ratio_i;
} RateControlIntervalParamContext;

typedef struct HighLevelRateControlContext {
    EbDctor  dctor;
    uint64_t target_bit_rate;
    uint64_t frame_rate;
    uint64_t channel_bit_rate_per_frame;
    uint64_t channel_bit_rate_per_sw;
    uint64_t bit_constraint_per_sw;
    uint64_t pred_bits_ref_qp_per_sw[MAX_REF_QP_NUM];
    uint32_t prev_intra_selected_ref_qp;
    uint32_t prev_intra_org_selected_ref_qp;
    uint64_t previous_updated_bit_constraint_per_sw;
} HighLevelRateControlContext;

typedef struct RateControlContext {
    EbFifo *rate_control_input_tasks_fifo_ptr;
    EbFifo *rate_control_output_results_fifo_ptr;

    HighLevelRateControlContext *high_level_rate_control_ptr;

    RateControlIntervalParamContext **rate_control_param_queue;
    uint64_t                          rate_control_param_queue_head_index;

    uint64_t frame_rate;

    uint64_t virtual_buffer_size;

    int64_t virtual_buffer_level_initial_value;
    int64_t previous_virtual_buffer_level;

    int64_t virtual_buffer_level;

    //Virtual Buffer Thresholds
    int64_t vb_fill_threshold1;
    int64_t vb_fill_threshold2;

    // Rate Control Previous Bits Queue
#if OVERSHOOT_STAT_PRINT
    CodedFramesStatsEntry **coded_frames_stat_queue;
    uint32_t                coded_frames_stat_queue_head_index;
    uint32_t                coded_frames_stat_queue_tail_index;

    uint64_t total_bit_actual_per_sw;
    uint64_t max_bit_actual_per_sw;
    uint64_t max_bit_actual_per_gop;
    uint64_t min_bit_actual_per_gop;
    uint64_t avg_bit_actual_per_gop;

#endif

    uint64_t rate_average_periodin_frames;
    uint32_t base_layer_frames_avg_qp;
    uint32_t base_layer_intra_frames_avg_qp;

    EbBool end_of_sequence_region;

    uint32_t intra_coef_rate;

    uint64_t frames_in_interval[EB_MAX_TEMPORAL_LAYERS];
    int64_t  extra_bits;
    int64_t  extra_bits_gen;
    int16_t  max_rate_adjust_delta_qp;

    uint32_t qp_scaling_map[EB_MAX_TEMPORAL_LAYERS][MAX_REF_QP_NUM];
    uint32_t qp_scaling_map_i_slice[MAX_REF_QP_NUM];
} RateControlContext;

// calculate the QP based on the QP scaling
uint32_t qp_scaling_calc(SequenceControlSet *scs_ptr, EB_SLICE slice_type,
                         uint32_t temporal_layer_index, uint32_t base_qp);

/*****************************
* Internal Typedefs
*****************************/
void rate_control_layer_reset(RateControlLayerContext *rate_control_layer_ptr,
                              PictureControlSet *      pcs_ptr,
                              RateControlContext *     rate_control_context_ptr,
                              uint32_t picture_area_in_pixel, EbBool was_used) {
    SequenceControlSet *scs_ptr = (SequenceControlSet *)pcs_ptr->scs_wrapper_ptr->object_ptr;
    uint32_t            slice_num;
    uint32_t            temporal_layer_index;
    uint64_t            total_frame_in_interval;
    uint64_t            sum_bits_per_sw = 0;

    rate_control_layer_ptr->target_bit_rate =
        pcs_ptr->parent_pcs_ptr->target_bit_rate *
        (uint64_t)rate_percentage_layer_array[scs_ptr->static_config.hierarchical_levels]
                                             [rate_control_layer_ptr->temporal_index] /
        100;
    // update this based on temporal layers
    rate_control_layer_ptr->frame_rate = scs_ptr->frame_rate;

    total_frame_in_interval = scs_ptr->static_config.intra_period_length + 1;

    if (scs_ptr->static_config.look_ahead_distance != 0 && scs_ptr->intra_period_length != -1) {
        if (pcs_ptr->picture_number % ((scs_ptr->intra_period_length + 1)) == 0) {
            total_frame_in_interval = 0;
            for (temporal_layer_index = 0; temporal_layer_index < EB_MAX_TEMPORAL_LAYERS;
                 temporal_layer_index++) {
                rate_control_context_ptr->frames_in_interval[temporal_layer_index] =
                    pcs_ptr->parent_pcs_ptr->frames_in_interval[temporal_layer_index];
                total_frame_in_interval +=
                    pcs_ptr->parent_pcs_ptr->frames_in_interval[temporal_layer_index];
                sum_bits_per_sw +=
                    pcs_ptr->parent_pcs_ptr->bits_per_sw_per_layer[temporal_layer_index];
            }
#if ADAPTIVE_PERCENTAGE
            rate_control_layer_ptr->target_bit_rate =
                pcs_ptr->parent_pcs_ptr->target_bit_rate *
                pcs_ptr->parent_pcs_ptr
                    ->bits_per_sw_per_layer[rate_control_layer_ptr->temporal_index] /
                sum_bits_per_sw;
#endif
        }
    }

    if (scs_ptr->static_config.intra_period_length != -1)
        rate_control_layer_ptr->frame_rate =
            scs_ptr->frame_rate *
            rate_control_context_ptr->frames_in_interval[rate_control_layer_ptr->temporal_index] /
            total_frame_in_interval;
    else {
        switch (pcs_ptr->parent_pcs_ptr->hierarchical_levels) {
        case 0: break;
        case 1:
            if (scs_ptr->static_config.intra_period_length == -1)
                rate_control_layer_ptr->frame_rate = rate_control_layer_ptr->frame_rate >> 1;
            break;
        case 2:
            if (rate_control_layer_ptr->temporal_index == 0)
                rate_control_layer_ptr->frame_rate = rate_control_layer_ptr->frame_rate >> 2;
            else
                rate_control_layer_ptr->frame_rate = rate_control_layer_ptr->frame_rate >>
                                                     (3 - rate_control_layer_ptr->temporal_index);
            break;
        case 3:
            if (rate_control_layer_ptr->temporal_index == 0)
                rate_control_layer_ptr->frame_rate = rate_control_layer_ptr->frame_rate >> 3;
            else
                rate_control_layer_ptr->frame_rate = rate_control_layer_ptr->frame_rate >>
                                                     (4 - rate_control_layer_ptr->temporal_index);
            break;
        case 4:
            if (rate_control_layer_ptr->temporal_index == 0)
                rate_control_layer_ptr->frame_rate = rate_control_layer_ptr->frame_rate >> 4;
            else
                rate_control_layer_ptr->frame_rate = rate_control_layer_ptr->frame_rate >>
                                                     (5 - rate_control_layer_ptr->temporal_index);
            break;
        case 5:
            if (rate_control_layer_ptr->temporal_index == 0)
                rate_control_layer_ptr->frame_rate = rate_control_layer_ptr->frame_rate >> 5;
            else
                rate_control_layer_ptr->frame_rate = rate_control_layer_ptr->frame_rate >>
                                                     (6 - rate_control_layer_ptr->temporal_index);
            break;

        default: break;
        }
    }

    rate_control_layer_ptr->coeff_averaging_weight1 = 5;

    rate_control_layer_ptr->coeff_averaging_weight2 =
        16 - rate_control_layer_ptr->coeff_averaging_weight1;
    if (rate_control_layer_ptr->frame_rate == 0) { // no frame in that layer
        rate_control_layer_ptr->frame_rate = 1 << RC_PRECISION;
    }
    rate_control_layer_ptr->channel_bit_rate =
        (((rate_control_layer_ptr->target_bit_rate << (2 * RC_PRECISION)) /
          rate_control_layer_ptr->frame_rate) +
         RC_PRECISION_OFFSET) >>
        RC_PRECISION;
    rate_control_layer_ptr->channel_bit_rate =
        (uint64_t)MAX((int64_t)1, (int64_t)rate_control_layer_ptr->channel_bit_rate);
    rate_control_layer_ptr->ec_bit_constraint = rate_control_layer_ptr->channel_bit_rate;

    // This is only for the initial frame, because the feedback is from packetization now and all of these are considered
    // considering the bits for slice header
    // *Note - only one-slice-per picture is supported for UHD
    slice_num = 1;

    rate_control_layer_ptr->ec_bit_constraint -= SLICE_HEADER_BITS_NUM * slice_num;

    rate_control_layer_ptr->ec_bit_constraint = MAX(1, rate_control_layer_ptr->ec_bit_constraint);

    rate_control_layer_ptr->previous_bit_constraint = rate_control_layer_ptr->channel_bit_rate;
    rate_control_layer_ptr->bit_constraint          = rate_control_layer_ptr->channel_bit_rate;
    rate_control_layer_ptr->dif_total_and_ec_bits   = 0;

    rate_control_layer_ptr->frame_same_distortion_min_qp_count = 0;
    rate_control_layer_ptr->max_qp                             = pcs_ptr->picture_qp;

    rate_control_layer_ptr->alpha = 1 << (RC_PRECISION - 1);
    {
        if (!was_used) {
            rate_control_layer_ptr->same_distortion_count = 0;
            rate_control_layer_ptr->k_coeff               = 3 << RC_PRECISION;
            rate_control_layer_ptr->previous_k_coeff      = 3 << RC_PRECISION;
            rate_control_layer_ptr->c_coeff =
                (rate_control_layer_ptr->channel_bit_rate << (2 * RC_PRECISION)) /
                picture_area_in_pixel / CCOEFF_INIT_FACT;
            rate_control_layer_ptr->previous_c_coeff =
                (rate_control_layer_ptr->channel_bit_rate << (2 * RC_PRECISION)) /
                picture_area_in_pixel / CCOEFF_INIT_FACT;
            // These are for handling Pred structure 2, when for higher temporal layer, frames can arrive in different orders
            // They should be modifed in a way that gets these from previous layers
            rate_control_layer_ptr->previous_frame_qp                        = 32;
            rate_control_layer_ptr->previous_frame_bit_actual                = 1200;
            rate_control_layer_ptr->previous_framequantized_coeff_bit_actual = 1000;
            rate_control_layer_ptr->previous_frame_distortion_me             = 10000000;
            rate_control_layer_ptr->previous_frame_qp                        = pcs_ptr->picture_qp;
            rate_control_layer_ptr->delta_qp_fraction                        = 0;
            rate_control_layer_ptr->previous_frame_average_qp                = pcs_ptr->picture_qp;
            rate_control_layer_ptr->previous_calculated_frame_qp             = pcs_ptr->picture_qp;
            rate_control_layer_ptr->calculated_frame_qp                      = pcs_ptr->picture_qp;
            rate_control_layer_ptr->critical_states                          = 0;
        } else {
            rate_control_layer_ptr->same_distortion_count = 0;
            rate_control_layer_ptr->critical_states       = 0;
        }
    }
}

void rate_control_layer_reset_part2(RateControlContext *     context_ptr,
                                    RateControlLayerContext *rate_control_layer_ptr,
                                    PictureControlSet *      pcs_ptr) {
    // update this based on temporal layers
    rate_control_layer_ptr->max_qp = (uint32_t)CLIP3(
        0,
        63,
        (int32_t)context_ptr
            ->qp_scaling_map[rate_control_layer_ptr->temporal_index][pcs_ptr->picture_qp]);
    // These are for handling Pred structure 2, when for higher temporal layer, frames can arrive in different orders
    // They should be modifed in a way that gets these from previous layers
    rate_control_layer_ptr->previous_frame_qp            = rate_control_layer_ptr->max_qp;
    rate_control_layer_ptr->previous_frame_average_qp    = rate_control_layer_ptr->max_qp;
    rate_control_layer_ptr->previous_calculated_frame_qp = rate_control_layer_ptr->max_qp;
    rate_control_layer_ptr->calculated_frame_qp          = rate_control_layer_ptr->max_qp;
}

EbErrorType high_level_rate_control_context_ctor(HighLevelRateControlContext *entry_ptr) {
    (void)entry_ptr;

    return EB_ErrorNone;
}

EbErrorType rate_control_layer_context_ctor(RateControlLayerContext *entry_ptr) {
    entry_ptr->first_frame           = 1;
    entry_ptr->first_non_intra_frame = 1;

    return EB_ErrorNone;
}

static void rate_control_interval_param_context_dctor(EbPtr p) {
    RateControlIntervalParamContext *obj = (RateControlIntervalParamContext *)p;
    EB_DELETE_PTR_ARRAY(obj->rate_control_layer_array, EB_MAX_TEMPORAL_LAYERS);
}

EbErrorType rate_control_interval_param_context_ctor(RateControlIntervalParamContext *entry_ptr) {
    uint32_t temporal_index;

    entry_ptr->dctor = rate_control_interval_param_context_dctor;

    EB_ALLOC_PTR_ARRAY(entry_ptr->rate_control_layer_array, EB_MAX_TEMPORAL_LAYERS);

    for (temporal_index = 0; temporal_index < EB_MAX_TEMPORAL_LAYERS; temporal_index++) {
        EB_NEW(entry_ptr->rate_control_layer_array[temporal_index],
               rate_control_layer_context_ctor);
        entry_ptr->rate_control_layer_array[temporal_index]->temporal_index = temporal_index;
        entry_ptr->rate_control_layer_array[temporal_index]->frame_rate     = 1 << RC_PRECISION;
    }

    return EB_ErrorNone;
}

EbErrorType rate_control_coded_frames_stats_context_ctor(CodedFramesStatsEntry *entry_ptr,
                                                         uint64_t               picture_number) {
    entry_ptr->picture_number         = picture_number;
    entry_ptr->frame_total_bit_actual = -1;

    return EB_ErrorNone;
}

static void rate_control_context_dctor(EbPtr p) {
    EbThreadContext *   thread_context_ptr = (EbThreadContext *)p;
    RateControlContext *obj                = (RateControlContext *)thread_context_ptr->priv;
#if OVERSHOOT_STAT_PRINT
    EB_DELETE_PTR_ARRAY(obj->coded_frames_stat_queue, CODED_FRAMES_STAT_QUEUE_MAX_DEPTH);
#endif
    EB_DELETE_PTR_ARRAY(obj->rate_control_param_queue, PARALLEL_GOP_MAX_NUMBER);
    EB_DELETE(obj->high_level_rate_control_ptr);
    EB_FREE_ARRAY(obj);
}

EbErrorType rate_control_context_ctor(EbThreadContext *  thread_context_ptr,
                                      const EbEncHandle *enc_handle_ptr) {
    uint32_t interval_index;

#if OVERSHOOT_STAT_PRINT
    uint32_t picture_index;
#endif
    int32_t intra_period = enc_handle_ptr->scs_instance_array[0]->scs_ptr->intra_period_length;

    RateControlContext *context_ptr;
    EB_CALLOC_ARRAY(context_ptr, 1);
    thread_context_ptr->priv  = context_ptr;
    thread_context_ptr->dctor = rate_control_context_dctor;

    context_ptr->rate_control_input_tasks_fifo_ptr =
        eb_system_resource_get_consumer_fifo(enc_handle_ptr->rate_control_tasks_resource_ptr, 0);
    context_ptr->rate_control_output_results_fifo_ptr =
        eb_system_resource_get_producer_fifo(enc_handle_ptr->rate_control_results_resource_ptr, 0);

    // High level RC
    EB_NEW(context_ptr->high_level_rate_control_ptr, high_level_rate_control_context_ctor);

    EB_ALLOC_PTR_ARRAY(context_ptr->rate_control_param_queue, PARALLEL_GOP_MAX_NUMBER);

    for (interval_index = 0; interval_index < PARALLEL_GOP_MAX_NUMBER; interval_index++) {
        EB_NEW(context_ptr->rate_control_param_queue[interval_index],
               rate_control_interval_param_context_ctor);
        context_ptr->rate_control_param_queue[interval_index]->first_poc =
            (interval_index * (uint32_t)(intra_period + 1));
        context_ptr->rate_control_param_queue[interval_index]->last_poc =
            ((interval_index + 1) * (uint32_t)(intra_period + 1)) - 1;
    }

#if OVERSHOOT_STAT_PRINT
    EB_ALLOC_PTR_ARRAY(context_ptr->coded_frames_stat_queue, CODED_FRAMES_STAT_QUEUE_MAX_DEPTH);

    for (picture_index = 0; picture_index < CODED_FRAMES_STAT_QUEUE_MAX_DEPTH; ++picture_index) {
        EB_NEW(context_ptr->coded_frames_stat_queue[picture_index],
               rate_control_coded_frames_stats_context_ctor,
               picture_index);
    }
    context_ptr->min_bit_actual_per_gop = 0xfffffffffffff;
#endif
    context_ptr->intra_coef_rate = 4;

    return EB_ErrorNone;
}
uint64_t predict_bits(EncodeContext *              encode_context_ptr,
                      HlRateControlHistogramEntry *hl_rate_control_histogram_ptr_temp, uint32_t qp,
                      uint32_t area_in_pixel) {
    uint64_t total_bits = 0;

    if (hl_rate_control_histogram_ptr_temp->is_coded) {
        // If the frame is already coded, use the actual number of bits
        total_bits = hl_rate_control_histogram_ptr_temp->total_num_bits_coded;
    } else {
        RateControlTables *rate_control_tables_ptr =
            &encode_context_ptr->rate_control_tables_array[qp];
        EbBitNumber *sad_bits_array_ptr =
            rate_control_tables_ptr
                ->sad_bits_array[hl_rate_control_histogram_ptr_temp->temporal_layer_index];
        EbBitNumber *intra_sad_bits_array_ptr = rate_control_tables_ptr->intra_sad_bits_array[0];
        uint32_t     pred_bits_ref_qp         = 0;

        if (hl_rate_control_histogram_ptr_temp->slice_type == I_SLICE) {
            // Loop over block in the frame and calculated the predicted bits at reg QP
            unsigned i;
            uint32_t accum = 0;
            for (i = 0; i < NUMBER_OF_INTRA_SAD_INTERVALS; ++i)
                accum += (uint32_t)(
                    (uint32_t)hl_rate_control_histogram_ptr_temp->ois_distortion_histogram[i] *
                    (uint32_t)intra_sad_bits_array_ptr[i]);
            pred_bits_ref_qp = accum;
            total_bits += pred_bits_ref_qp;
        } else {
            unsigned i;
            uint32_t accum       = 0;
            uint32_t accum_intra = 0;
            for (i = 0; i < NUMBER_OF_SAD_INTERVALS; ++i) {
                accum += (uint32_t)(
                    (uint32_t)hl_rate_control_histogram_ptr_temp->me_distortion_histogram[i] *
                    (uint32_t)sad_bits_array_ptr[i]);
                accum_intra += (uint32_t)(
                    (uint32_t)hl_rate_control_histogram_ptr_temp->ois_distortion_histogram[i] *
                    (uint32_t)intra_sad_bits_array_ptr[i]);
            }
            if (accum > accum_intra * 3)
                pred_bits_ref_qp = accum_intra;
            else
                pred_bits_ref_qp = accum;
            total_bits += pred_bits_ref_qp;
        }

        // Scale for in complete LCSs
        //  total_bits is normalized based on the area because of the sbs at the picture boundries
        total_bits = total_bits * (uint64_t)area_in_pixel /
                     (hl_rate_control_histogram_ptr_temp->full_sb_count << 12);
    }
    return total_bits;
}

void high_level_rc_input_picture_vbr(PictureParentControlSet *pcs_ptr, SequenceControlSet *scs_ptr,
                                     EncodeContext *              encode_context_ptr,
                                     RateControlContext *         context_ptr,
                                     HighLevelRateControlContext *high_level_rate_control_ptr) {
    EbBool end_of_sequence_flag = EB_TRUE;

    HlRateControlHistogramEntry *hl_rate_control_histogram_ptr_temp;
    // Queue variables
    uint32_t queue_entry_index_temp;
    uint32_t queue_entry_index_temp2;
    int64_t  queue_entry_index_head_temp;

    uint64_t min_la_bit_distance;
    uint32_t selected_ref_qp_table_index;
    uint32_t selected_ref_qp;
    uint32_t selected_org_ref_qp;
    uint32_t previous_selected_ref_qp      = encode_context_ptr->previous_selected_ref_qp;
    uint64_t max_coded_poc                 = encode_context_ptr->max_coded_poc;
    uint32_t max_coded_poc_selected_ref_qp = encode_context_ptr->max_coded_poc_selected_ref_qp;

    uint32_t ref_qp_index;
    uint32_t ref_qp_index_temp;
    uint32_t ref_qp_table_index;

    uint32_t area_in_pixel;
    uint32_t num_of_full_sbs;
    uint32_t qp_search_min;
    uint32_t qp_search_max;
    int32_t  qp_step = 1;
    EbBool   best_qp_found;
    uint32_t temporal_layer_index;
    EbBool   tables_updated;

    uint64_t bit_constraint_per_sw = 0;

    RateControlTables *rate_control_tables_ptr;
    EbBitNumber *      sad_bits_array_ptr;
    EbBitNumber *      intra_sad_bits_array_ptr;
    uint32_t           pred_bits_ref_qp;

    for (temporal_layer_index = 0; temporal_layer_index < EB_MAX_TEMPORAL_LAYERS;
         temporal_layer_index++)
        pcs_ptr->bits_per_sw_per_layer[temporal_layer_index] = 0;
    pcs_ptr->total_bits_per_gop = 0;

    area_in_pixel = pcs_ptr->aligned_width * pcs_ptr->aligned_height;
    ;

    eb_block_on_mutex(scs_ptr->encode_context_ptr->rate_table_update_mutex);

    tables_updated              = scs_ptr->encode_context_ptr->rate_control_tables_array_updated;
    pcs_ptr->percentage_updated = EB_FALSE;
    if (scs_ptr->static_config.look_ahead_distance != 0) {
        // Increamenting the head of the hl_rate_control_historgram_queue and clean up the entores
        hl_rate_control_histogram_ptr_temp =
            (encode_context_ptr->hl_rate_control_historgram_queue
                 [encode_context_ptr->hl_rate_control_historgram_queue_head_index]);
        while ((hl_rate_control_histogram_ptr_temp->life_count == 0) &&
               hl_rate_control_histogram_ptr_temp->passed_to_hlrc) {
            eb_block_on_mutex(scs_ptr->encode_context_ptr->hl_rate_control_historgram_queue_mutex);
            // Reset the Reorder Queue Entry
            hl_rate_control_histogram_ptr_temp->picture_number +=
                INITIAL_RATE_CONTROL_REORDER_QUEUE_MAX_DEPTH;
            hl_rate_control_histogram_ptr_temp->life_count           = -1;
            hl_rate_control_histogram_ptr_temp->passed_to_hlrc       = EB_FALSE;
            hl_rate_control_histogram_ptr_temp->is_coded             = EB_FALSE;
            hl_rate_control_histogram_ptr_temp->total_num_bits_coded = 0;

            // Increment the Reorder Queue head Ptr
            encode_context_ptr->hl_rate_control_historgram_queue_head_index =
                (encode_context_ptr->hl_rate_control_historgram_queue_head_index ==
                 HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH - 1)
                    ? 0
                    : encode_context_ptr->hl_rate_control_historgram_queue_head_index + 1;
            eb_release_mutex(scs_ptr->encode_context_ptr->hl_rate_control_historgram_queue_mutex);
            hl_rate_control_histogram_ptr_temp =
                encode_context_ptr->hl_rate_control_historgram_queue
                    [encode_context_ptr->hl_rate_control_historgram_queue_head_index];
        }
        // For the case that number of frames in the sliding window is less than size of the look ahead or intra Refresh. i.e. end of sequence
        if ((pcs_ptr->frames_in_sw < MIN(scs_ptr->static_config.look_ahead_distance + 1,
                                         (uint32_t)scs_ptr->intra_period_length + 1))) {
            selected_ref_qp = max_coded_poc_selected_ref_qp;

            // Update the QP for the sliding window based on the status of RC
            if ((context_ptr->extra_bits_gen > (int64_t)(context_ptr->virtual_buffer_size << 3)))
                selected_ref_qp = (uint32_t)MAX((int32_t)selected_ref_qp - 2, 0);
            else if ((context_ptr->extra_bits_gen >
                      (int64_t)(context_ptr->virtual_buffer_size << 2)))
                selected_ref_qp = (uint32_t)MAX((int32_t)selected_ref_qp - 1, 0);
            if ((context_ptr->extra_bits_gen < -(int64_t)(context_ptr->virtual_buffer_size << 2)))
                selected_ref_qp += 2;
            else if ((context_ptr->extra_bits_gen <
                      -(int64_t)(context_ptr->virtual_buffer_size << 1)))
                selected_ref_qp += 1;
            if ((pcs_ptr->frames_in_sw < (uint32_t)(scs_ptr->intra_period_length + 1)) &&
                (pcs_ptr->picture_number % ((scs_ptr->intra_period_length + 1)) == 0)) {
                selected_ref_qp++;
            }

            selected_ref_qp = (uint32_t)CLIP3(scs_ptr->static_config.min_qp_allowed,
                                              scs_ptr->static_config.max_qp_allowed,
                                              selected_ref_qp);

            queue_entry_index_head_temp =
                (int32_t)(pcs_ptr->picture_number -
                          encode_context_ptr
                              ->hl_rate_control_historgram_queue
                                  [encode_context_ptr->hl_rate_control_historgram_queue_head_index]
                              ->picture_number);
            queue_entry_index_head_temp +=
                encode_context_ptr->hl_rate_control_historgram_queue_head_index;
            queue_entry_index_head_temp =
                (queue_entry_index_head_temp >
                 HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH - 1)
                    ? queue_entry_index_head_temp -
                          HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH
                    : (queue_entry_index_head_temp < 0)
                          ? queue_entry_index_head_temp +
                                HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH
                          : queue_entry_index_head_temp;

            queue_entry_index_temp = (uint32_t)queue_entry_index_head_temp;
            {
                hl_rate_control_histogram_ptr_temp =
                    (encode_context_ptr->hl_rate_control_historgram_queue[queue_entry_index_temp]);

                if (hl_rate_control_histogram_ptr_temp->slice_type == I_SLICE)
                    ref_qp_index_temp = context_ptr->qp_scaling_map_i_slice[selected_ref_qp];
                else
                    ref_qp_index_temp =
                        context_ptr->qp_scaling_map[hl_rate_control_histogram_ptr_temp
                                                        ->temporal_layer_index][selected_ref_qp];

                ref_qp_index_temp = (uint32_t)CLIP3(scs_ptr->static_config.min_qp_allowed,
                                                    scs_ptr->static_config.max_qp_allowed,
                                                    ref_qp_index_temp);

                hl_rate_control_histogram_ptr_temp->pred_bits_ref_qp[ref_qp_index_temp] = 0;
                rate_control_tables_ptr =
                    &encode_context_ptr->rate_control_tables_array[ref_qp_index_temp];
                sad_bits_array_ptr =
                    rate_control_tables_ptr
                        ->sad_bits_array[hl_rate_control_histogram_ptr_temp->temporal_layer_index];
                intra_sad_bits_array_ptr =
                    rate_control_tables_ptr->intra_sad_bits_array[hl_rate_control_histogram_ptr_temp
                                                                      ->temporal_layer_index];
                pred_bits_ref_qp = 0;
                num_of_full_sbs  = 0;

                if (hl_rate_control_histogram_ptr_temp->slice_type == I_SLICE) {
                    // Loop over block in the frame and calculated the predicted bits at reg QP
                    {
                        unsigned i;
                        uint32_t accum = 0;
                        for (i = 0; i < NUMBER_OF_INTRA_SAD_INTERVALS; ++i)
                            accum += (uint32_t)(
                                hl_rate_control_histogram_ptr_temp->ois_distortion_histogram[i] *
                                intra_sad_bits_array_ptr[i]);
                        pred_bits_ref_qp = accum;
                        num_of_full_sbs  = hl_rate_control_histogram_ptr_temp->full_sb_count;
                    }
                    hl_rate_control_histogram_ptr_temp->pred_bits_ref_qp[ref_qp_index_temp] +=
                        pred_bits_ref_qp;
                }

                else {
                    {
                        unsigned i;
                        uint32_t accum = 0;
                        for (i = 0; i < NUMBER_OF_SAD_INTERVALS; ++i)
                            accum += (uint32_t)(
                                hl_rate_control_histogram_ptr_temp->me_distortion_histogram[i] *
                                sad_bits_array_ptr[i]);
                        pred_bits_ref_qp = accum;
                        num_of_full_sbs  = hl_rate_control_histogram_ptr_temp->full_sb_count;
                    }
                    hl_rate_control_histogram_ptr_temp->pred_bits_ref_qp[ref_qp_index_temp] +=
                        pred_bits_ref_qp;
                }

                // Scale for in complete
                //  pred_bits_ref_qp is normalized based on the area because of the SBs at the picture boundries
                hl_rate_control_histogram_ptr_temp->pred_bits_ref_qp[ref_qp_index_temp] =
                    hl_rate_control_histogram_ptr_temp->pred_bits_ref_qp[ref_qp_index_temp] *
                    (uint64_t)area_in_pixel / (num_of_full_sbs << 12);

                // Store the pred_bits_ref_qp for the first frame in the window to PCS
                pcs_ptr->pred_bits_ref_qp[ref_qp_index_temp] =
                    hl_rate_control_histogram_ptr_temp->pred_bits_ref_qp[ref_qp_index_temp];
            }
        } else {
            // Loop over the QPs and find the best QP
            min_la_bit_distance = MAX_UNSIGNED_VALUE;
            qp_search_min =
                (uint8_t)CLIP3(scs_ptr->static_config.min_qp_allowed,
                               MAX_REF_QP_NUM, //scs_ptr->static_config.max_qp_allowed,
                               (uint32_t)MAX((int32_t)scs_ptr->static_config.qp - 40, 0));

            qp_search_max = (uint8_t)CLIP3(scs_ptr->static_config.min_qp_allowed,
                                           MAX_REF_QP_NUM, //scs_ptr->static_config.max_qp_allowed,
                                           scs_ptr->static_config.qp + 40);

            for (ref_qp_table_index = qp_search_min; ref_qp_table_index < qp_search_max;
                 ref_qp_table_index++)
                high_level_rate_control_ptr->pred_bits_ref_qp_per_sw[ref_qp_table_index] = 0;
            bit_constraint_per_sw = high_level_rate_control_ptr->bit_constraint_per_sw *
                                    pcs_ptr->frames_in_sw /
                                    (scs_ptr->static_config.look_ahead_distance + 1);

            // Update the target rate for the sliding window based on the status of RC
            if ((context_ptr->extra_bits_gen > (int64_t)(context_ptr->virtual_buffer_size * 10)))
                bit_constraint_per_sw = bit_constraint_per_sw * 130 / 100;
            else if ((context_ptr->extra_bits_gen >
                      (int64_t)(context_ptr->virtual_buffer_size << 3)))
                bit_constraint_per_sw = bit_constraint_per_sw * 120 / 100;
            else if ((context_ptr->extra_bits_gen >
                      (int64_t)(context_ptr->virtual_buffer_size << 2)))
                bit_constraint_per_sw = bit_constraint_per_sw * 110 / 100;
            if ((context_ptr->extra_bits_gen < -(int64_t)(context_ptr->virtual_buffer_size << 3)))
                bit_constraint_per_sw = bit_constraint_per_sw * 80 / 100;
            else if ((context_ptr->extra_bits_gen <
                      -(int64_t)(context_ptr->virtual_buffer_size << 2)))
                bit_constraint_per_sw = bit_constraint_per_sw * 90 / 100;
            // Loop over proper QPs and find the Predicted bits for that QP. Find the QP with the closest total predicted rate to target bits for the sliding window.
            previous_selected_ref_qp =
                CLIP3(qp_search_min, qp_search_max, previous_selected_ref_qp);
            ref_qp_table_index          = previous_selected_ref_qp;
            selected_ref_qp_table_index = ref_qp_table_index;
            selected_ref_qp             = selected_ref_qp_table_index;
            best_qp_found               = EB_FALSE;
            while (ref_qp_table_index >= qp_search_min && ref_qp_table_index <= qp_search_max &&
                   !best_qp_found) {
                ref_qp_index = CLIP3(scs_ptr->static_config.min_qp_allowed,
                                     MAX_REF_QP_NUM, //scs_ptr->static_config.max_qp_allowed,
                                     ref_qp_table_index);
                high_level_rate_control_ptr->pred_bits_ref_qp_per_sw[ref_qp_index] = 0;

                // Finding the predicted bits for each frame in the sliding window at the reference Qp(s)
                queue_entry_index_head_temp = (int32_t)(
                    pcs_ptr->picture_number -
                    encode_context_ptr
                        ->hl_rate_control_historgram_queue
                            [encode_context_ptr->hl_rate_control_historgram_queue_head_index]
                        ->picture_number);
                queue_entry_index_head_temp +=
                    encode_context_ptr->hl_rate_control_historgram_queue_head_index;
                queue_entry_index_head_temp =
                    (queue_entry_index_head_temp >
                     HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH - 1)
                        ? queue_entry_index_head_temp -
                              HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH
                        : (queue_entry_index_head_temp < 0)
                              ? queue_entry_index_head_temp +
                                    HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH
                              : queue_entry_index_head_temp;

                queue_entry_index_temp = (uint32_t)queue_entry_index_head_temp;
                // This is set to false, so the last frame would go inside the loop
                end_of_sequence_flag = EB_FALSE;

                while (!end_of_sequence_flag &&
                       queue_entry_index_temp <= queue_entry_index_head_temp +
                                                     scs_ptr->static_config.look_ahead_distance) {
                    queue_entry_index_temp2 =
                        (queue_entry_index_temp >
                         HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH - 1)
                            ? queue_entry_index_temp -
                                  HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH
                            : queue_entry_index_temp;
                    hl_rate_control_histogram_ptr_temp =
                        (encode_context_ptr
                             ->hl_rate_control_historgram_queue[queue_entry_index_temp2]);

                    if (hl_rate_control_histogram_ptr_temp->slice_type == I_SLICE)
                        ref_qp_index_temp = context_ptr->qp_scaling_map_i_slice[ref_qp_index];
                    else
                        ref_qp_index_temp =
                            context_ptr->qp_scaling_map[hl_rate_control_histogram_ptr_temp
                                                            ->temporal_layer_index][ref_qp_index];

                    ref_qp_index_temp = (uint32_t)CLIP3(scs_ptr->static_config.min_qp_allowed,
                                                        scs_ptr->static_config.max_qp_allowed,
                                                        ref_qp_index_temp);

                    hl_rate_control_histogram_ptr_temp->pred_bits_ref_qp[ref_qp_index_temp] = 0;

                    if (ref_qp_table_index == previous_selected_ref_qp) {
                        eb_block_on_mutex(
                            scs_ptr->encode_context_ptr->hl_rate_control_historgram_queue_mutex);
                        hl_rate_control_histogram_ptr_temp->life_count--;
                        eb_release_mutex(
                            scs_ptr->encode_context_ptr->hl_rate_control_historgram_queue_mutex);
                    }
                    hl_rate_control_histogram_ptr_temp->pred_bits_ref_qp[ref_qp_index_temp] =
                        predict_bits(encode_context_ptr,
                                     hl_rate_control_histogram_ptr_temp,
                                     ref_qp_index_temp,
                                     area_in_pixel);

                    high_level_rate_control_ptr->pred_bits_ref_qp_per_sw[ref_qp_index] +=
                        hl_rate_control_histogram_ptr_temp->pred_bits_ref_qp[ref_qp_index_temp];

                    // Store the pred_bits_ref_qp for the first frame in the window to PCS
                    if (queue_entry_index_head_temp == queue_entry_index_temp2)
                        pcs_ptr->pred_bits_ref_qp[ref_qp_index_temp] =
                            hl_rate_control_histogram_ptr_temp->pred_bits_ref_qp[ref_qp_index_temp];

                    end_of_sequence_flag = hl_rate_control_histogram_ptr_temp->end_of_sequence_flag;
                    queue_entry_index_temp++;
                }

                if (min_la_bit_distance >=
                    (uint64_t)ABS((int64_t)high_level_rate_control_ptr
                                      ->pred_bits_ref_qp_per_sw[ref_qp_index] -
                                  (int64_t)bit_constraint_per_sw)) {
                    min_la_bit_distance = (uint64_t)ABS(
                        (int64_t)
                            high_level_rate_control_ptr->pred_bits_ref_qp_per_sw[ref_qp_index] -
                        (int64_t)bit_constraint_per_sw);
                    selected_ref_qp_table_index = ref_qp_table_index;
                    selected_ref_qp             = ref_qp_index;
                } else
                    best_qp_found = EB_TRUE;
                if (ref_qp_table_index == previous_selected_ref_qp) {
                    if (high_level_rate_control_ptr->pred_bits_ref_qp_per_sw[ref_qp_index] >
                        bit_constraint_per_sw)
                        qp_step = +1;
                    else
                        qp_step = -1;
                }
                ref_qp_table_index = (uint32_t)(ref_qp_table_index + qp_step);
            }
        }

        selected_org_ref_qp = selected_ref_qp;
        if (scs_ptr->intra_period_length != -1 &&
            pcs_ptr->picture_number % ((scs_ptr->intra_period_length + 1)) == 0 &&
            (int32_t)pcs_ptr->frames_in_sw > scs_ptr->intra_period_length) {
            if (pcs_ptr->picture_number > 0)
                pcs_ptr->intra_selected_org_qp = (uint8_t)selected_ref_qp;
            ref_qp_index                                                       = selected_ref_qp;
            high_level_rate_control_ptr->pred_bits_ref_qp_per_sw[ref_qp_index] = 0;

            if (high_level_rate_control_ptr->pred_bits_ref_qp_per_sw[ref_qp_index] == 0) {
                // Finding the predicted bits for each frame in the sliding window at the reference Qp(s)
                //queue_entry_index_temp = encode_context_ptr->hl_rate_control_historgram_queue_head_index;
                queue_entry_index_head_temp = (int32_t)(
                    pcs_ptr->picture_number -
                    encode_context_ptr
                        ->hl_rate_control_historgram_queue
                            [encode_context_ptr->hl_rate_control_historgram_queue_head_index]
                        ->picture_number);
                queue_entry_index_head_temp +=
                    encode_context_ptr->hl_rate_control_historgram_queue_head_index;
                queue_entry_index_head_temp =
                    (queue_entry_index_head_temp >
                     HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH - 1)
                        ? queue_entry_index_head_temp -
                              HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH
                        : (queue_entry_index_head_temp < 0)
                              ? queue_entry_index_head_temp +
                                    HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH
                              : queue_entry_index_head_temp;

                queue_entry_index_temp = (uint32_t)queue_entry_index_head_temp;

                // This is set to false, so the last frame would go inside the loop
                end_of_sequence_flag = EB_FALSE;

                while (
                    !end_of_sequence_flag &&
                    //queue_entry_index_temp <= encode_context_ptr->hl_rate_control_historgram_queue_head_index+scs_ptr->static_config.look_ahead_distance){
                    queue_entry_index_temp <=
                        queue_entry_index_head_temp + scs_ptr->static_config.look_ahead_distance) {
                    queue_entry_index_temp2 =
                        (queue_entry_index_temp >
                         HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH - 1)
                            ? queue_entry_index_temp -
                                  HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH
                            : queue_entry_index_temp;
                    hl_rate_control_histogram_ptr_temp =
                        (encode_context_ptr
                             ->hl_rate_control_historgram_queue[queue_entry_index_temp2]);

                    if (hl_rate_control_histogram_ptr_temp->slice_type == I_SLICE)
                        ref_qp_index_temp = context_ptr->qp_scaling_map_i_slice[ref_qp_index];
                    else
                        ref_qp_index_temp =
                            context_ptr->qp_scaling_map[hl_rate_control_histogram_ptr_temp
                                                            ->temporal_layer_index][ref_qp_index];

                    ref_qp_index_temp = (uint32_t)CLIP3(scs_ptr->static_config.min_qp_allowed,
                                                        scs_ptr->static_config.max_qp_allowed,
                                                        ref_qp_index_temp);

                    hl_rate_control_histogram_ptr_temp->pred_bits_ref_qp[ref_qp_index_temp] =
                        predict_bits(encode_context_ptr,
                                     hl_rate_control_histogram_ptr_temp,
                                     ref_qp_index_temp,
                                     area_in_pixel);

                    high_level_rate_control_ptr->pred_bits_ref_qp_per_sw[ref_qp_index] +=
                        hl_rate_control_histogram_ptr_temp->pred_bits_ref_qp[ref_qp_index_temp];
                    // Store the pred_bits_ref_qp for the first frame in the window to PCS
                    //  if(encode_context_ptr->hl_rate_control_historgram_queue_head_index == queue_entry_index_temp2)
                    if (queue_entry_index_head_temp == queue_entry_index_temp2)
                        pcs_ptr->pred_bits_ref_qp[ref_qp_index_temp] =
                            hl_rate_control_histogram_ptr_temp->pred_bits_ref_qp[ref_qp_index_temp];

                    end_of_sequence_flag = hl_rate_control_histogram_ptr_temp->end_of_sequence_flag;
                    queue_entry_index_temp++;
                }
            }
        }
        pcs_ptr->tables_updated  = tables_updated;
        EbBool expensive_i_slice = EB_FALSE;
        // Looping over the window to find the percentage of bit allocation in each layer
        if ((scs_ptr->intra_period_length != -1) &&
            ((int32_t)pcs_ptr->frames_in_sw > scs_ptr->intra_period_length) &&
            ((int32_t)pcs_ptr->frames_in_sw > scs_ptr->intra_period_length)) {
            uint64_t i_slice_bits = 0;

            if (pcs_ptr->picture_number % ((scs_ptr->intra_period_length + 1)) == 0) {
                queue_entry_index_head_temp = (int32_t)(
                    pcs_ptr->picture_number -
                    encode_context_ptr
                        ->hl_rate_control_historgram_queue
                            [encode_context_ptr->hl_rate_control_historgram_queue_head_index]
                        ->picture_number);
                queue_entry_index_head_temp +=
                    encode_context_ptr->hl_rate_control_historgram_queue_head_index;
                queue_entry_index_head_temp =
                    (queue_entry_index_head_temp >
                     HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH - 1)
                        ? queue_entry_index_head_temp -
                              HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH
                        : (queue_entry_index_head_temp < 0)
                              ? queue_entry_index_head_temp +
                                    HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH
                              : queue_entry_index_head_temp;

                queue_entry_index_temp = (uint32_t)queue_entry_index_head_temp;

                // This is set to false, so the last frame would go inside the loop
                end_of_sequence_flag = EB_FALSE;

                while (!end_of_sequence_flag &&
                       queue_entry_index_temp <=
                           queue_entry_index_head_temp + scs_ptr->intra_period_length) {
                    queue_entry_index_temp2 =
                        (queue_entry_index_temp >
                         HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH - 1)
                            ? queue_entry_index_temp -
                                  HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH
                            : queue_entry_index_temp;
                    hl_rate_control_histogram_ptr_temp =
                        (encode_context_ptr
                             ->hl_rate_control_historgram_queue[queue_entry_index_temp2]);

                    if (hl_rate_control_histogram_ptr_temp->slice_type == I_SLICE)
                        ref_qp_index_temp = context_ptr->qp_scaling_map_i_slice[selected_ref_qp];
                    else
                        ref_qp_index_temp =
                            context_ptr
                                ->qp_scaling_map[hl_rate_control_histogram_ptr_temp
                                                     ->temporal_layer_index][selected_ref_qp];

                    ref_qp_index_temp = (uint32_t)CLIP3(scs_ptr->static_config.min_qp_allowed,
                                                        scs_ptr->static_config.max_qp_allowed,
                                                        ref_qp_index_temp);

                    if (queue_entry_index_temp == queue_entry_index_head_temp)
                        i_slice_bits =
                            hl_rate_control_histogram_ptr_temp->pred_bits_ref_qp[ref_qp_index_temp];
                    pcs_ptr->total_bits_per_gop +=
                        hl_rate_control_histogram_ptr_temp->pred_bits_ref_qp[ref_qp_index_temp];
                    pcs_ptr->bits_per_sw_per_layer[hl_rate_control_histogram_ptr_temp
                                                       ->temporal_layer_index] +=
                        hl_rate_control_histogram_ptr_temp->pred_bits_ref_qp[ref_qp_index_temp];
                    pcs_ptr->percentage_updated = EB_TRUE;

                    end_of_sequence_flag = hl_rate_control_histogram_ptr_temp->end_of_sequence_flag;
                    queue_entry_index_temp++;
                }
                if (i_slice_bits * 100 > 85 * pcs_ptr->total_bits_per_gop)
                    expensive_i_slice = EB_TRUE;
                if (pcs_ptr->total_bits_per_gop == 0) {
                    for (temporal_layer_index = 0; temporal_layer_index < EB_MAX_TEMPORAL_LAYERS;
                         temporal_layer_index++)
                        pcs_ptr->bits_per_sw_per_layer[temporal_layer_index] =
                            rate_percentage_layer_array[scs_ptr->static_config.hierarchical_levels]
                                                       [temporal_layer_index];
                }
            }
        } else {
            for (temporal_layer_index = 0; temporal_layer_index < EB_MAX_TEMPORAL_LAYERS;
                 temporal_layer_index++)
                pcs_ptr->bits_per_sw_per_layer[temporal_layer_index] =
                    rate_percentage_layer_array[scs_ptr->static_config.hierarchical_levels]
                                               [temporal_layer_index];
        }
        if (expensive_i_slice) {
            if (tables_updated)
                selected_ref_qp = (uint32_t)MAX((int32_t)selected_ref_qp - 1, 0);
            else
                selected_ref_qp = (uint32_t)MAX((int32_t)selected_ref_qp - 3, 0);
            selected_ref_qp = (uint32_t)CLIP3(scs_ptr->static_config.min_qp_allowed,
                                              scs_ptr->static_config.max_qp_allowed,
                                              selected_ref_qp);
        }
        // Set the QP
        previous_selected_ref_qp = selected_ref_qp;
        if (pcs_ptr->picture_number > max_coded_poc && pcs_ptr->temporal_layer_index < 2 &&
            !pcs_ptr->end_of_sequence_region) {
            max_coded_poc                                     = pcs_ptr->picture_number;
            max_coded_poc_selected_ref_qp                     = previous_selected_ref_qp;
            encode_context_ptr->previous_selected_ref_qp      = previous_selected_ref_qp;
            encode_context_ptr->max_coded_poc                 = max_coded_poc;
            encode_context_ptr->max_coded_poc_selected_ref_qp = max_coded_poc_selected_ref_qp;
        }

        if (pcs_ptr->slice_type == I_SLICE)
            pcs_ptr->best_pred_qp = (uint8_t)context_ptr->qp_scaling_map_i_slice[selected_ref_qp];
        else
            pcs_ptr->best_pred_qp =
                (uint8_t)
                    context_ptr->qp_scaling_map[pcs_ptr->temporal_layer_index][selected_ref_qp];

        pcs_ptr->best_pred_qp = (uint8_t)CLIP3(scs_ptr->static_config.min_qp_allowed,
                                               scs_ptr->static_config.max_qp_allowed,
                                               pcs_ptr->best_pred_qp);

        if (pcs_ptr->picture_number == 0) {
            high_level_rate_control_ptr->prev_intra_selected_ref_qp     = selected_ref_qp;
            high_level_rate_control_ptr->prev_intra_org_selected_ref_qp = selected_ref_qp;
        }
        if (scs_ptr->intra_period_length != -1) {
            if (pcs_ptr->picture_number % ((scs_ptr->intra_period_length + 1)) == 0) {
                high_level_rate_control_ptr->prev_intra_selected_ref_qp     = selected_ref_qp;
                high_level_rate_control_ptr->prev_intra_org_selected_ref_qp = selected_org_ref_qp;
            }
        }
        pcs_ptr->target_bits_best_pred_qp = pcs_ptr->pred_bits_ref_qp[pcs_ptr->best_pred_qp];
#if RC_PRINTS
        if (pcs_ptr->slice_type == 2) {
            SVT_LOG("\nTID: %d\t", pcs_ptr->temporal_layer_index);
            SVT_LOG("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t\n",
                pcs_ptr->picture_number,
                pcs_ptr->best_pred_qp,
                selected_ref_qp,
                (int)pcs_ptr->target_bits_best_pred_qp,
                (int)high_level_rate_control_ptr->pred_bits_ref_qp_per_sw[selected_ref_qp - 1],
                (int)high_level_rate_control_ptr->pred_bits_ref_qp_per_sw[selected_ref_qp],
                (int)high_level_rate_control_ptr->pred_bits_ref_qp_per_sw[selected_ref_qp + 1],
                (int)high_level_rate_control_ptr->bit_constraint_per_sw,
                (int)bit_constraint_per_sw/*,
                (int)high_level_rate_control_ptr->virtual_buffer_level*/);
        }
#endif
    }
    eb_release_mutex(scs_ptr->encode_context_ptr->rate_table_update_mutex);
}
void frame_level_rc_input_picture_vbr(PictureControlSet *pcs_ptr, SequenceControlSet *scs_ptr,
                                      RateControlContext *             context_ptr,
                                      RateControlLayerContext *        rate_control_layer_ptr,
                                      RateControlIntervalParamContext *rate_control_param_ptr) {
    RateControlLayerContext *rate_control_layer_temp_ptr;

    // Tiles
    uint32_t picture_area_in_pixel;
    uint32_t area_in_pixel;

    // SB Loop variables
    SbParams *sb_params_ptr;
    uint32_t  sb_index;
    uint64_t  temp_qp;
    uint32_t  area_in_sbs;

    picture_area_in_pixel =
            pcs_ptr->parent_pcs_ptr->aligned_height * pcs_ptr->parent_pcs_ptr->aligned_width;

    if (rate_control_layer_ptr->first_frame == 1) {
        rate_control_layer_ptr->first_frame                    = 0;
        pcs_ptr->parent_pcs_ptr->first_frame_in_temporal_layer = 1;
    } else
        pcs_ptr->parent_pcs_ptr->first_frame_in_temporal_layer = 0;
    if (pcs_ptr->slice_type != I_SLICE) {
        if (rate_control_layer_ptr->first_non_intra_frame == 1) {
            rate_control_layer_ptr->first_non_intra_frame                    = 0;
            pcs_ptr->parent_pcs_ptr->first_non_intra_frame_in_temporal_layer = 1;
        } else
            pcs_ptr->parent_pcs_ptr->first_non_intra_frame_in_temporal_layer = 0;
    } else
        pcs_ptr->parent_pcs_ptr->first_non_intra_frame_in_temporal_layer = 0;

    pcs_ptr->parent_pcs_ptr->target_bits_rc = 0;

    // ***Rate Control***
    area_in_sbs   = 0;
    area_in_pixel = 0;

    for (sb_index = 0; sb_index < pcs_ptr->sb_total_count; ++sb_index) {
        sb_params_ptr = &pcs_ptr->parent_pcs_ptr->sb_params_array[sb_index];

        if (sb_params_ptr->is_complete_sb) {
            // add the area of one SB (64x64=4096) to the area of the tile
            area_in_pixel += 4096;
            area_in_sbs++;
        } else {
            // add the area of the SB to the area of the tile
            area_in_pixel += sb_params_ptr->width * sb_params_ptr->height;
        }
    }
    rate_control_layer_ptr->area_in_pixel = area_in_pixel;

    if (pcs_ptr->parent_pcs_ptr->first_frame_in_temporal_layer ||
        (pcs_ptr->picture_number == rate_control_param_ptr->first_poc)) {
        if (scs_ptr->static_config.enable_qp_scaling_flag &&
            (pcs_ptr->picture_number != rate_control_param_ptr->first_poc)) {
            pcs_ptr->picture_qp = (uint8_t)CLIP3(
                (int32_t)scs_ptr->static_config.min_qp_allowed,
                (int32_t)scs_ptr->static_config.max_qp_allowed,
                (int32_t)(
                    rate_control_param_ptr->intra_frames_qp +
                    context_ptr->qp_scaling_map[pcs_ptr->temporal_layer_index]
                                               [rate_control_param_ptr->intra_frames_qp_bef_scal] -
                    context_ptr->qp_scaling_map_i_slice[rate_control_param_ptr
                                                            ->intra_frames_qp_bef_scal]));
        }

        if (pcs_ptr->picture_number == 0) {
            rate_control_param_ptr->intra_frames_qp          = scs_ptr->static_config.qp;
            rate_control_param_ptr->intra_frames_qp_bef_scal = (uint8_t)scs_ptr->static_config.qp;
        }

        if (pcs_ptr->picture_number == rate_control_param_ptr->first_poc) {
            uint32_t temporal_layer_idex;
            rate_control_param_ptr->previous_virtual_buffer_level =
                context_ptr->virtual_buffer_level_initial_value;
            rate_control_param_ptr->virtual_buffer_level =
                context_ptr->virtual_buffer_level_initial_value;
            rate_control_param_ptr->extra_ap_bit_ratio_i = 0;
            if (pcs_ptr->parent_pcs_ptr->end_of_sequence_region) {
                rate_control_param_ptr->last_poc = MAX(
                    rate_control_param_ptr->first_poc + pcs_ptr->parent_pcs_ptr->frames_in_sw - 1,
                    rate_control_param_ptr->first_poc);
                rate_control_param_ptr->last_gop = EB_TRUE;
            }

            if ((context_ptr->extra_bits > (int64_t)(context_ptr->virtual_buffer_size >> 8)) ||
                (context_ptr->extra_bits < -(int64_t)(context_ptr->virtual_buffer_size >> 8))) {
                int64_t extra_bits_per_gop = 0;

                if (pcs_ptr->parent_pcs_ptr->end_of_sequence_region) {
                    if ((context_ptr->extra_bits >
                         (int64_t)(context_ptr->virtual_buffer_size << 4)) ||
                        (context_ptr->extra_bits <
                         -(int64_t)(context_ptr->virtual_buffer_size << 4))) {
                        extra_bits_per_gop = context_ptr->extra_bits;
                        extra_bits_per_gop = CLIP3(-(int64_t)(context_ptr->vb_fill_threshold2 << 3),
                                                   (int64_t)(context_ptr->vb_fill_threshold2 << 3),
                                                   extra_bits_per_gop);
                    } else if ((context_ptr->extra_bits >
                                (int64_t)(context_ptr->virtual_buffer_size << 3)) ||
                               (context_ptr->extra_bits <
                                -(int64_t)(context_ptr->virtual_buffer_size << 3))) {
                        extra_bits_per_gop = context_ptr->extra_bits;
                        extra_bits_per_gop = CLIP3(-(int64_t)(context_ptr->vb_fill_threshold2 << 2),
                                                   (int64_t)(context_ptr->vb_fill_threshold2 << 2),
                                                   extra_bits_per_gop);
                    } else if ((context_ptr->extra_bits >
                                (int64_t)(context_ptr->virtual_buffer_size << 2)) ||
                               (context_ptr->extra_bits <
                                -(int64_t)(context_ptr->virtual_buffer_size << 2))) {
                        extra_bits_per_gop = CLIP3(-(int64_t)context_ptr->vb_fill_threshold2 << 1,
                                                   (int64_t)context_ptr->vb_fill_threshold2 << 1,
                                                   extra_bits_per_gop);
                    } else {
                        extra_bits_per_gop = CLIP3(-(int64_t)context_ptr->vb_fill_threshold1,
                                                   (int64_t)context_ptr->vb_fill_threshold1,
                                                   extra_bits_per_gop);
                    }
                } else {
                    if ((context_ptr->extra_bits >
                         (int64_t)(context_ptr->virtual_buffer_size << 3)) ||
                        (context_ptr->extra_bits <
                         -(int64_t)(context_ptr->virtual_buffer_size << 3))) {
                        extra_bits_per_gop = context_ptr->extra_bits;
                        extra_bits_per_gop = CLIP3(-(int64_t)(context_ptr->vb_fill_threshold2 << 2),
                                                   (int64_t)(context_ptr->vb_fill_threshold2 << 2),
                                                   extra_bits_per_gop);
                    } else if ((context_ptr->extra_bits >
                                (int64_t)(context_ptr->virtual_buffer_size << 2)) ||
                               (context_ptr->extra_bits <
                                -(int64_t)(context_ptr->virtual_buffer_size << 2))) {
                        extra_bits_per_gop = CLIP3(-(int64_t)context_ptr->vb_fill_threshold2 << 1,
                                                   (int64_t)context_ptr->vb_fill_threshold2 << 1,
                                                   extra_bits_per_gop);
                    }
                }

                rate_control_param_ptr->virtual_buffer_level -= extra_bits_per_gop;
                rate_control_param_ptr->previous_virtual_buffer_level -= extra_bits_per_gop;
                context_ptr->extra_bits -= extra_bits_per_gop;
            }

            for (temporal_layer_idex = 0; temporal_layer_idex < EB_MAX_TEMPORAL_LAYERS;
                 temporal_layer_idex++) {
                rate_control_layer_temp_ptr =
                    rate_control_param_ptr->rate_control_layer_array[temporal_layer_idex];
                rate_control_layer_reset(rate_control_layer_temp_ptr,
                                         pcs_ptr,
                                         context_ptr,
                                         picture_area_in_pixel,
                                         rate_control_param_ptr->was_used);
            }
        }

        pcs_ptr->parent_pcs_ptr->sad_me = 0;
        // Finding the QP of the Intra frame by using variance tables
        if (pcs_ptr->slice_type == I_SLICE) {
            uint32_t selected_ref_qp;

            if (scs_ptr->static_config.look_ahead_distance == 0)
                SVT_LOG("ERROR: LAD=0 is not supported\n");
            else {
                selected_ref_qp                        = pcs_ptr->parent_pcs_ptr->best_pred_qp;
                pcs_ptr->picture_qp                    = (uint8_t)selected_ref_qp;
                pcs_ptr->parent_pcs_ptr->calculated_qp = pcs_ptr->picture_qp;
            }

            // Update the QP based on the VB
            if (pcs_ptr->parent_pcs_ptr->end_of_sequence_region) {
                if (rate_control_param_ptr->virtual_buffer_level >= context_ptr->vb_fill_threshold2
                                                                        << 1)
                    pcs_ptr->picture_qp = pcs_ptr->picture_qp + (uint8_t)THRESHOLD2QPINCREASE + 2;
                else if (rate_control_param_ptr->virtual_buffer_level >=
                         context_ptr->vb_fill_threshold2)
                    pcs_ptr->picture_qp = pcs_ptr->picture_qp + (uint8_t)THRESHOLD2QPINCREASE;
                else if (rate_control_param_ptr->virtual_buffer_level >=
                             context_ptr->vb_fill_threshold1 &&
                         rate_control_param_ptr->virtual_buffer_level <
                             context_ptr->vb_fill_threshold2) {
                    pcs_ptr->picture_qp = pcs_ptr->picture_qp + (uint8_t)THRESHOLD1QPINCREASE;
                }
                if (rate_control_param_ptr->virtual_buffer_level <=
                    -(context_ptr->vb_fill_threshold2 << 2))
                    pcs_ptr->picture_qp = (uint8_t)MAX(
                        (int32_t)pcs_ptr->picture_qp - (int32_t)THRESHOLD2QPINCREASE - (int32_t)2,
                        0);
                else if (rate_control_param_ptr->virtual_buffer_level <=
                         -(context_ptr->vb_fill_threshold2 << 1))
                    pcs_ptr->picture_qp = (uint8_t)MAX(
                        (int32_t)pcs_ptr->picture_qp - (int32_t)THRESHOLD2QPINCREASE - (int32_t)1,
                        0);
                else if (rate_control_param_ptr->virtual_buffer_level <= 0)
                    pcs_ptr->picture_qp = (uint8_t)MAX(
                        (int32_t)pcs_ptr->picture_qp - (int32_t)THRESHOLD2QPINCREASE, 0);
            } else {
                if (rate_control_param_ptr->virtual_buffer_level >= context_ptr->vb_fill_threshold2)
                    pcs_ptr->picture_qp = pcs_ptr->picture_qp + (uint8_t)THRESHOLD2QPINCREASE;
                if (rate_control_param_ptr->virtual_buffer_level <=
                    -(context_ptr->vb_fill_threshold2 << 2))
                    pcs_ptr->picture_qp =
                        pcs_ptr->picture_qp - (uint8_t)THRESHOLD2QPINCREASE - (int32_t)2;
                else if (rate_control_param_ptr->virtual_buffer_level <=
                         -(context_ptr->vb_fill_threshold2 << 1))
                    pcs_ptr->picture_qp =
                        pcs_ptr->picture_qp - (uint8_t)THRESHOLD2QPINCREASE - (int32_t)1;
                else if (rate_control_param_ptr->virtual_buffer_level <= 0)
                    pcs_ptr->picture_qp = (uint8_t)MAX(
                        (int32_t)pcs_ptr->picture_qp - (int32_t)THRESHOLD2QPINCREASE, 0);
            }
            pcs_ptr->picture_qp = (uint8_t)CLIP3(scs_ptr->static_config.min_qp_allowed,
                                                 scs_ptr->static_config.max_qp_allowed,
                                                 pcs_ptr->picture_qp);
        } else {
            // SB Loop
            for (sb_index = 0; sb_index < pcs_ptr->sb_total_count; ++sb_index) {
                sb_params_ptr = &pcs_ptr->parent_pcs_ptr->sb_params_array[sb_index];

                if (sb_params_ptr->is_complete_sb)
                    pcs_ptr->parent_pcs_ptr->sad_me +=
                        pcs_ptr->parent_pcs_ptr->rc_me_distortion[sb_index];
            }

            //  tilesad_Me is normalized based on the area because of the SBs at the tile boundries
            pcs_ptr->parent_pcs_ptr->sad_me =
                MAX((pcs_ptr->parent_pcs_ptr->sad_me * rate_control_layer_ptr->area_in_pixel /
                     (area_in_sbs << 12)),
                    1);

            // totalSquareMad has RC_PRECISION precision
            pcs_ptr->parent_pcs_ptr->sad_me <<= RC_PRECISION;
        }

        temp_qp = pcs_ptr->picture_qp;

        if (pcs_ptr->picture_number == rate_control_param_ptr->first_poc) {
            uint32_t temporal_layer_idex;
            for (temporal_layer_idex = 0; temporal_layer_idex < EB_MAX_TEMPORAL_LAYERS;
                 temporal_layer_idex++) {
                rate_control_layer_temp_ptr =
                    rate_control_param_ptr->rate_control_layer_array[temporal_layer_idex];
                rate_control_layer_reset_part2(context_ptr, rate_control_layer_temp_ptr, pcs_ptr);
            }
        }

        if (pcs_ptr->picture_number == 0) {
            context_ptr->base_layer_frames_avg_qp       = pcs_ptr->picture_qp + 1;
            context_ptr->base_layer_intra_frames_avg_qp = pcs_ptr->picture_qp;
        }
    } else {
        pcs_ptr->parent_pcs_ptr->sad_me = 0;

        // if the pixture is an I slice, for now we set the QP as the QP of the previous frame
        if (pcs_ptr->slice_type == I_SLICE) {
            uint32_t selected_ref_qp;

            if (scs_ptr->static_config.look_ahead_distance == 0)
                SVT_LOG("ERROR: LAD=0 is not supported\n");
            else {
                selected_ref_qp                        = pcs_ptr->parent_pcs_ptr->best_pred_qp;
                pcs_ptr->picture_qp                    = (uint8_t)selected_ref_qp;
                pcs_ptr->parent_pcs_ptr->calculated_qp = pcs_ptr->picture_qp;
            }

            pcs_ptr->picture_qp = (uint8_t)CLIP3(scs_ptr->static_config.min_qp_allowed,
                                                 scs_ptr->static_config.max_qp_allowed,
                                                 pcs_ptr->picture_qp);

            temp_qp = pcs_ptr->picture_qp;
        }

        else { // Not an I slice
            // combining the target rate from initial RC and frame level RC
            if (scs_ptr->static_config.look_ahead_distance != 0) {
                pcs_ptr->parent_pcs_ptr->target_bits_rc = rate_control_layer_ptr->bit_constraint;
                rate_control_layer_ptr->ec_bit_constraint =
                    (rate_control_layer_ptr->alpha *
                         pcs_ptr->parent_pcs_ptr->target_bits_best_pred_qp +
                     ((1 << RC_PRECISION) - rate_control_layer_ptr->alpha) *
                         pcs_ptr->parent_pcs_ptr->target_bits_rc +
                     RC_PRECISION_OFFSET) >>
                    RC_PRECISION;

                rate_control_layer_ptr->ec_bit_constraint =
                    (uint64_t)MAX((int64_t)rate_control_layer_ptr->ec_bit_constraint -
                                      (int64_t)rate_control_layer_ptr->dif_total_and_ec_bits,
                                  1);

                pcs_ptr->parent_pcs_ptr->target_bits_rc = rate_control_layer_ptr->ec_bit_constraint;
            }

            // SB Loop
            for (sb_index = 0; sb_index < pcs_ptr->sb_total_count; ++sb_index) {
                sb_params_ptr = &pcs_ptr->parent_pcs_ptr->sb_params_array[sb_index];

                if (sb_params_ptr->is_complete_sb)
                    pcs_ptr->parent_pcs_ptr->sad_me +=
                        pcs_ptr->parent_pcs_ptr->rc_me_distortion[sb_index];
            }

            //  tilesad_Me is normalized based on the area because of the SBs at the tile boundries
            pcs_ptr->parent_pcs_ptr->sad_me =
                MAX((pcs_ptr->parent_pcs_ptr->sad_me * rate_control_layer_ptr->area_in_pixel /
                     (area_in_sbs << 12)),
                    1);
            pcs_ptr->parent_pcs_ptr->sad_me <<= RC_PRECISION;
            if (rate_control_layer_ptr->area_in_pixel > 0)
                rate_control_layer_ptr->total_mad = MAX(
                    (pcs_ptr->parent_pcs_ptr->sad_me / rate_control_layer_ptr->area_in_pixel), 1);
            if (!rate_control_layer_ptr->feedback_arrived)
                rate_control_layer_ptr->previous_frame_distortion_me =
                    pcs_ptr->parent_pcs_ptr->sad_me;
            {
                uint64_t qp_calc_temp1, qp_calc_temp2, qp_calc_temp3;

                qp_calc_temp1 = pcs_ptr->parent_pcs_ptr->sad_me * rate_control_layer_ptr->total_mad;
                qp_calc_temp2 = MAX(
                    (int64_t)(rate_control_layer_ptr->ec_bit_constraint << (2 * RC_PRECISION)) -
                        (int64_t)rate_control_layer_ptr->c_coeff *
                            (int64_t)rate_control_layer_ptr->area_in_pixel,
                    (int64_t)(rate_control_layer_ptr->ec_bit_constraint << (2 * RC_PRECISION - 2)));

                // This is a more complex but with higher precision implementation
                if (qp_calc_temp1 > qp_calc_temp2)
                    qp_calc_temp3 = (uint64_t)((qp_calc_temp1 / qp_calc_temp2) *
                                               rate_control_layer_ptr->k_coeff);
                else
                    qp_calc_temp3 =
                        (uint64_t)(qp_calc_temp1 * rate_control_layer_ptr->k_coeff / qp_calc_temp2);
                temp_qp = (uint64_t)(log2f_high_precision(
                    MAX(((qp_calc_temp3 + RC_PRECISION_OFFSET) >> RC_PRECISION) *
                            ((qp_calc_temp3 + RC_PRECISION_OFFSET) >> RC_PRECISION) *
                            ((qp_calc_temp3 + RC_PRECISION_OFFSET) >> RC_PRECISION),
                        1),
                    RC_PRECISION));

                rate_control_layer_ptr->calculated_frame_qp = (uint8_t)(
                    CLIP3(1, 63, (uint32_t)(temp_qp + RC_PRECISION_OFFSET) >> RC_PRECISION));
                pcs_ptr->parent_pcs_ptr->calculated_qp = (uint8_t)(
                    CLIP3(1, 63, (uint32_t)(temp_qp + RC_PRECISION_OFFSET) >> RC_PRECISION));
            }

            temp_qp += rate_control_layer_ptr->delta_qp_fraction;
            pcs_ptr->picture_qp = (uint8_t)((temp_qp + RC_PRECISION_OFFSET) >> RC_PRECISION);
            // Use the QP of HLRC instead of calculated one in FLRC
            if (pcs_ptr->parent_pcs_ptr->hierarchical_levels > 1) {
                pcs_ptr->picture_qp                    = pcs_ptr->parent_pcs_ptr->best_pred_qp;
                pcs_ptr->parent_pcs_ptr->calculated_qp = pcs_ptr->parent_pcs_ptr->best_pred_qp;
            }
        }
        if (pcs_ptr->parent_pcs_ptr->first_non_intra_frame_in_temporal_layer &&
            pcs_ptr->temporal_layer_index == 0 && pcs_ptr->slice_type != I_SLICE)
            pcs_ptr->picture_qp = (uint8_t)(
                rate_control_param_ptr->intra_frames_qp +
                context_ptr->qp_scaling_map[pcs_ptr->temporal_layer_index]
                                           [rate_control_param_ptr->intra_frames_qp_bef_scal] -
                context_ptr
                    ->qp_scaling_map_i_slice[rate_control_param_ptr->intra_frames_qp_bef_scal]);
        if (!rate_control_layer_ptr->feedback_arrived && pcs_ptr->slice_type != I_SLICE) {
            pcs_ptr->picture_qp = (uint8_t)CLIP3(
                (int32_t)scs_ptr->static_config.min_qp_allowed,
                (int32_t)scs_ptr->static_config.max_qp_allowed,
                (int32_t)(
                    rate_control_param_ptr->intra_frames_qp +
                    context_ptr->qp_scaling_map[pcs_ptr->temporal_layer_index]
                                               [rate_control_param_ptr->intra_frames_qp_bef_scal] -
                    context_ptr->qp_scaling_map_i_slice[rate_control_param_ptr
                                                            ->intra_frames_qp_bef_scal]));
        }

        if (pcs_ptr->parent_pcs_ptr->end_of_sequence_region) {
            if (rate_control_param_ptr->virtual_buffer_level > context_ptr->vb_fill_threshold2 << 2)
                pcs_ptr->picture_qp = pcs_ptr->picture_qp + (uint8_t)THRESHOLD2QPINCREASE + 4;
            else if (rate_control_param_ptr->virtual_buffer_level > context_ptr->vb_fill_threshold2
                                                                        << 1)
                pcs_ptr->picture_qp = pcs_ptr->picture_qp + (uint8_t)THRESHOLD2QPINCREASE + 3;
            else if (rate_control_param_ptr->virtual_buffer_level > context_ptr->vb_fill_threshold2)
                pcs_ptr->picture_qp = pcs_ptr->picture_qp + (uint8_t)THRESHOLD2QPINCREASE + 2;
            else if (rate_control_param_ptr->virtual_buffer_level >
                         context_ptr->vb_fill_threshold1 &&
                     rate_control_param_ptr->virtual_buffer_level <
                         context_ptr->vb_fill_threshold2) {
                pcs_ptr->picture_qp = pcs_ptr->picture_qp + (uint8_t)THRESHOLD1QPINCREASE + 2;
            }
        } else {
            if (rate_control_param_ptr->virtual_buffer_level > context_ptr->vb_fill_threshold2 << 2)
                pcs_ptr->picture_qp = pcs_ptr->picture_qp + (uint8_t)THRESHOLD2QPINCREASE + 2;
            else if (rate_control_param_ptr->virtual_buffer_level > context_ptr->vb_fill_threshold2
                                                                        << 1)
                pcs_ptr->picture_qp = pcs_ptr->picture_qp + (uint8_t)THRESHOLD2QPINCREASE + 1;
            else if (rate_control_param_ptr->virtual_buffer_level > context_ptr->vb_fill_threshold2)
                pcs_ptr->picture_qp = pcs_ptr->picture_qp + (uint8_t)THRESHOLD2QPINCREASE + 1;
            else if (rate_control_param_ptr->virtual_buffer_level >
                         context_ptr->vb_fill_threshold1 &&
                     rate_control_param_ptr->virtual_buffer_level <
                         context_ptr->vb_fill_threshold2) {
                pcs_ptr->picture_qp = pcs_ptr->picture_qp + (uint8_t)THRESHOLD1QPINCREASE;
            }
        }
        if (pcs_ptr->parent_pcs_ptr->end_of_sequence_region) {
            if (rate_control_param_ptr->virtual_buffer_level <
                -(context_ptr->vb_fill_threshold2 << 2))
                pcs_ptr->picture_qp = (uint8_t)MAX(
                    (int32_t)pcs_ptr->picture_qp - (int32_t)THRESHOLD2QPINCREASE - 2, 0);
            else if (rate_control_param_ptr->virtual_buffer_level <
                     -(context_ptr->vb_fill_threshold2 << 1))
                pcs_ptr->picture_qp = (uint8_t)MAX(
                    (int32_t)pcs_ptr->picture_qp - (int32_t)THRESHOLD2QPINCREASE - 1, 0);
            else if (rate_control_param_ptr->virtual_buffer_level < 0)
                pcs_ptr->picture_qp =
                    (uint8_t)MAX((int32_t)pcs_ptr->picture_qp - (int32_t)THRESHOLD2QPINCREASE, 0);
        } else {
            if (rate_control_param_ptr->virtual_buffer_level <
                -(context_ptr->vb_fill_threshold2 << 2))
                pcs_ptr->picture_qp = (uint8_t)MAX(
                    (int32_t)pcs_ptr->picture_qp - (int32_t)THRESHOLD2QPINCREASE - 1, 0);
            else if (rate_control_param_ptr->virtual_buffer_level <
                     -context_ptr->vb_fill_threshold2)
                pcs_ptr->picture_qp =
                    (uint8_t)MAX((int32_t)pcs_ptr->picture_qp - (int32_t)THRESHOLD2QPINCREASE, 0);
        }

        // limiting the QP based on the predicted QP
        if (scs_ptr->static_config.look_ahead_distance != 0) {
            if (pcs_ptr->parent_pcs_ptr->end_of_sequence_region) {
                pcs_ptr->picture_qp = (uint8_t)CLIP3(
                    (uint32_t)MAX((int32_t)pcs_ptr->parent_pcs_ptr->best_pred_qp - 8, 0),
                    (uint32_t)pcs_ptr->parent_pcs_ptr->best_pred_qp + 8,
                    (uint32_t)pcs_ptr->picture_qp);
            } else {
                pcs_ptr->picture_qp = (uint8_t)CLIP3(
                    (uint32_t)MAX((int32_t)pcs_ptr->parent_pcs_ptr->best_pred_qp - 8, 0),
                    (uint32_t)pcs_ptr->parent_pcs_ptr->best_pred_qp + 8,
                    (uint32_t)pcs_ptr->picture_qp);
            }
        }
        if (pcs_ptr->picture_number != rate_control_param_ptr->first_poc &&
            pcs_ptr->picture_qp == pcs_ptr->parent_pcs_ptr->best_pred_qp &&
            rate_control_param_ptr->virtual_buffer_level > context_ptr->vb_fill_threshold1) {
            if (rate_control_param_ptr->extra_ap_bit_ratio_i > 200)
                pcs_ptr->picture_qp = pcs_ptr->picture_qp + 3;
            else if (rate_control_param_ptr->extra_ap_bit_ratio_i > 100)
                pcs_ptr->picture_qp = pcs_ptr->picture_qp + 2;
            else if (rate_control_param_ptr->extra_ap_bit_ratio_i > 50)
                pcs_ptr->picture_qp++;
        }
        //Limiting the QP based on the QP of the Reference frame

        uint32_t ref_qp;
        if ((int32_t)pcs_ptr->temporal_layer_index == 0 && pcs_ptr->slice_type != I_SLICE) {
            if (pcs_ptr->ref_slice_type_array[0][0] == I_SLICE) {
                pcs_ptr->picture_qp = (uint8_t)CLIP3((uint32_t)pcs_ptr->ref_pic_qp_array[0][0],
                                                     (uint32_t)pcs_ptr->picture_qp,
                                                     pcs_ptr->picture_qp);
            } else {
                pcs_ptr->picture_qp =
                    (uint8_t)CLIP3((uint32_t)MAX((int32_t)pcs_ptr->ref_pic_qp_array[0][0] - 1, 0),
                                   (uint32_t)pcs_ptr->picture_qp,
                                   pcs_ptr->picture_qp);
            }
        } else {
            ref_qp = 0;
            if (pcs_ptr->ref_slice_type_array[0][0] != I_SLICE)
                ref_qp = MAX(ref_qp, pcs_ptr->ref_pic_qp_array[0][0]);
            if ((pcs_ptr->slice_type == B_SLICE) &&
                (pcs_ptr->ref_slice_type_array[1][0] != I_SLICE))
                ref_qp = MAX(ref_qp, pcs_ptr->ref_pic_qp_array[1][0]);
            if (ref_qp > 0) {
                pcs_ptr->picture_qp =
                    (uint8_t)CLIP3((uint32_t)ref_qp - 1, pcs_ptr->picture_qp, pcs_ptr->picture_qp);
            }
        }
        // limiting the QP between min Qp allowed and max Qp allowed
        pcs_ptr->picture_qp = (uint8_t)CLIP3(scs_ptr->static_config.min_qp_allowed,
                                             scs_ptr->static_config.max_qp_allowed,
                                             pcs_ptr->picture_qp);

        rate_control_layer_ptr->delta_qp_fraction =
            CLIP3(-RC_PRECISION_OFFSET,
                  RC_PRECISION_OFFSET,
                  -((int64_t)temp_qp - (int64_t)(pcs_ptr->picture_qp << RC_PRECISION)));

        if (pcs_ptr->parent_pcs_ptr->sad_me ==
                rate_control_layer_ptr->previous_frame_distortion_me &&
            (rate_control_layer_ptr->previous_frame_distortion_me != 0))
            rate_control_layer_ptr->same_distortion_count++;
        else
            rate_control_layer_ptr->same_distortion_count = 0;
    }

    rate_control_layer_ptr->previous_c_coeff = rate_control_layer_ptr->c_coeff;
    rate_control_layer_ptr->previous_k_coeff = rate_control_layer_ptr->k_coeff;
    rate_control_layer_ptr->previous_calculated_frame_qp =
        rate_control_layer_ptr->calculated_frame_qp;
}

void frame_level_rc_feedback_picture_vbr(PictureParentControlSet *parentpicture_control_set_ptr,
                                         SequenceControlSet *     scs_ptr,
                                         RateControlContext *     context_ptr) {
    RateControlLayerContext *        rate_control_layer_temp_ptr;
    RateControlIntervalParamContext *rate_control_param_ptr;
    RateControlLayerContext *        rate_control_layer_ptr;
    // SB Loop variables
    uint32_t slice_num;
    uint64_t previous_frame_bit_actual;

    if (scs_ptr->intra_period_length == -1)
        rate_control_param_ptr = context_ptr->rate_control_param_queue[0];
    else {
        uint32_t interval_index_temp = 0;
        while ((!(parentpicture_control_set_ptr->picture_number >=
                      context_ptr->rate_control_param_queue[interval_index_temp]->first_poc &&
                  parentpicture_control_set_ptr->picture_number <=
                      context_ptr->rate_control_param_queue[interval_index_temp]->last_poc)) &&
               (interval_index_temp < PARALLEL_GOP_MAX_NUMBER)) {
            interval_index_temp++;
        }
        CHECK_REPORT_ERROR(interval_index_temp != PARALLEL_GOP_MAX_NUMBER,
                           scs_ptr->encode_context_ptr->app_callback_ptr,
                           EB_ENC_RC_ERROR2);
        rate_control_param_ptr = context_ptr->rate_control_param_queue[interval_index_temp];
    }

    rate_control_layer_ptr =
        rate_control_param_ptr
            ->rate_control_layer_array[parentpicture_control_set_ptr->temporal_layer_index];

    rate_control_layer_ptr->max_qp = 0;

    rate_control_layer_ptr->feedback_arrived = EB_TRUE;
    rate_control_layer_ptr->max_qp =
        MAX(rate_control_layer_ptr->max_qp, parentpicture_control_set_ptr->picture_qp);

    rate_control_layer_ptr->previous_frame_qp = parentpicture_control_set_ptr->picture_qp;
    rate_control_layer_ptr->previous_frame_bit_actual =
        parentpicture_control_set_ptr->total_num_bits;
    if (parentpicture_control_set_ptr->quantized_coeff_num_bits == 0)
        parentpicture_control_set_ptr->quantized_coeff_num_bits = 1;
    rate_control_layer_ptr->previous_framequantized_coeff_bit_actual =
        parentpicture_control_set_ptr->quantized_coeff_num_bits;

    // Setting Critical states for adjusting the averaging weights on C and K
    if ((parentpicture_control_set_ptr->sad_me >
         (3 * rate_control_layer_ptr->previous_frame_distortion_me) >> 1) &&
        (rate_control_layer_ptr->previous_frame_distortion_me != 0)) {
        rate_control_layer_ptr->critical_states = 3;
    } else if (rate_control_layer_ptr->critical_states)
        rate_control_layer_ptr->critical_states--;
    else
        rate_control_layer_ptr->critical_states = 0;
    if (parentpicture_control_set_ptr->slice_type != I_SLICE) {
        // Updating c_coeff
        rate_control_layer_ptr->c_coeff =
            (((int64_t)rate_control_layer_ptr->previous_frame_bit_actual -
              (int64_t)rate_control_layer_ptr->previous_framequantized_coeff_bit_actual)
             << (2 * RC_PRECISION)) /
            rate_control_layer_ptr->area_in_pixel;
        rate_control_layer_ptr->c_coeff = MAX(rate_control_layer_ptr->c_coeff, 1);

        // Updating k_coeff
        if ((parentpicture_control_set_ptr->sad_me + RC_PRECISION_OFFSET) >> RC_PRECISION > 5) {
            {
                uint64_t test1, test2, test3;
                test1 = rate_control_layer_ptr->previous_framequantized_coeff_bit_actual *
                        (two_to_power_qp_over_three[parentpicture_control_set_ptr->picture_qp]);
                test2 = MAX(
                    parentpicture_control_set_ptr->sad_me / rate_control_layer_ptr->area_in_pixel,
                    1);
                test3 = test1 * 65536 / test2 * 65536 / parentpicture_control_set_ptr->sad_me;

                rate_control_layer_ptr->k_coeff = test3;
            }
        }

        if (rate_control_layer_ptr->critical_states) {
            rate_control_layer_ptr->k_coeff = (8 * rate_control_layer_ptr->k_coeff +
                                               8 * rate_control_layer_ptr->previous_k_coeff + 8) >>
                                              4;
            rate_control_layer_ptr->c_coeff = (8 * rate_control_layer_ptr->c_coeff +
                                               8 * rate_control_layer_ptr->previous_c_coeff + 8) >>
                                              4;
        } else {
            rate_control_layer_ptr->k_coeff =
                (rate_control_layer_ptr->coeff_averaging_weight1 * rate_control_layer_ptr->k_coeff +
                 rate_control_layer_ptr->coeff_averaging_weight2 *
                     rate_control_layer_ptr->previous_k_coeff +
                 8) >>
                4;
            rate_control_layer_ptr->c_coeff =
                (rate_control_layer_ptr->coeff_averaging_weight1 * rate_control_layer_ptr->c_coeff +
                 rate_control_layer_ptr->coeff_averaging_weight2 *
                     rate_control_layer_ptr->previous_c_coeff +
                 8) >>
                4;
        }
        rate_control_layer_ptr->k_coeff =
            MIN(rate_control_layer_ptr->k_coeff, rate_control_layer_ptr->previous_k_coeff * 4);
        rate_control_layer_ptr->c_coeff =
            MIN(rate_control_layer_ptr->c_coeff, rate_control_layer_ptr->previous_c_coeff * 4);
        if (parentpicture_control_set_ptr->slice_type != I_SLICE)
            rate_control_layer_ptr->previous_frame_distortion_me =
                parentpicture_control_set_ptr->sad_me;
        else
            rate_control_layer_ptr->previous_frame_distortion_me = 0;
    }

    if (scs_ptr->static_config.look_ahead_distance != 0) {
        if (parentpicture_control_set_ptr->slice_type == I_SLICE) {
            if (parentpicture_control_set_ptr->total_num_bits <
                parentpicture_control_set_ptr->target_bits_best_pred_qp << 1)
                context_ptr->base_layer_intra_frames_avg_qp =
                    (3 * context_ptr->base_layer_intra_frames_avg_qp +
                     parentpicture_control_set_ptr->picture_qp + 2) >>
                    2;
            else if (parentpicture_control_set_ptr->total_num_bits >
                     parentpicture_control_set_ptr->target_bits_best_pred_qp << 2)
                context_ptr->base_layer_intra_frames_avg_qp =
                    (3 * context_ptr->base_layer_intra_frames_avg_qp +
                     parentpicture_control_set_ptr->picture_qp + 4 + 2) >>
                    2;
            else if (parentpicture_control_set_ptr->total_num_bits >
                     parentpicture_control_set_ptr->target_bits_best_pred_qp << 1)
                context_ptr->base_layer_intra_frames_avg_qp =
                    (3 * context_ptr->base_layer_intra_frames_avg_qp +
                     parentpicture_control_set_ptr->picture_qp + 2 + 2) >>
                    2;
        }
    }

    {
        uint64_t previous_frame_ec_bits                   = 0;
        EbBool   picture_min_qp_allowed                   = EB_TRUE;
        rate_control_layer_ptr->previous_frame_average_qp = 0;
        rate_control_layer_ptr->previous_frame_average_qp +=
            rate_control_layer_ptr->previous_frame_qp;
        previous_frame_ec_bits += rate_control_layer_ptr->previous_frame_bit_actual;
        if (rate_control_layer_ptr->same_distortion_count == 0 ||
            parentpicture_control_set_ptr->picture_qp != scs_ptr->static_config.min_qp_allowed) {
            picture_min_qp_allowed = EB_FALSE;
        }
        if (picture_min_qp_allowed)
            rate_control_layer_ptr->frame_same_distortion_min_qp_count++;
        else
            rate_control_layer_ptr->frame_same_distortion_min_qp_count = 0;

        rate_control_layer_ptr->previous_ec_bits = previous_frame_ec_bits;
        previous_frame_bit_actual                = parentpicture_control_set_ptr->total_num_bits;
        if (parentpicture_control_set_ptr->first_frame_in_temporal_layer)
            rate_control_layer_ptr->dif_total_and_ec_bits =
                (previous_frame_bit_actual - previous_frame_ec_bits);
        else
            rate_control_layer_ptr->dif_total_and_ec_bits =
                ((previous_frame_bit_actual - previous_frame_ec_bits) +
                 rate_control_layer_ptr->dif_total_and_ec_bits) >>
                1;
        // update bitrate of different layers in the interval based on the rate of the I frame
        if (parentpicture_control_set_ptr->picture_number == rate_control_param_ptr->first_poc &&
            (parentpicture_control_set_ptr->slice_type == I_SLICE) &&
            scs_ptr->static_config.intra_period_length != -1) {
            uint32_t temporal_layer_idex;
            uint64_t target_bit_rate;
            uint64_t channel_bit_rate;
            uint64_t sum_bits_per_sw = 0;
#if ADAPTIVE_PERCENTAGE
            if (scs_ptr->static_config.look_ahead_distance != 0) {
                if (parentpicture_control_set_ptr->tables_updated &&
                    parentpicture_control_set_ptr->percentage_updated) {
                    parentpicture_control_set_ptr->bits_per_sw_per_layer[0] = (uint64_t)MAX(
                        (int64_t)parentpicture_control_set_ptr->bits_per_sw_per_layer[0] +
                            (int64_t)parentpicture_control_set_ptr->total_num_bits -
                            (int64_t)parentpicture_control_set_ptr->target_bits_best_pred_qp,
                        1);
                }
            }
#endif

            if (scs_ptr->static_config.look_ahead_distance != 0 &&
                scs_ptr->intra_period_length != -1) {
                for (temporal_layer_idex = 0; temporal_layer_idex < EB_MAX_TEMPORAL_LAYERS;
                     temporal_layer_idex++)
                    sum_bits_per_sw +=
                        parentpicture_control_set_ptr->bits_per_sw_per_layer[temporal_layer_idex];
            }

            for (temporal_layer_idex = 0; temporal_layer_idex < EB_MAX_TEMPORAL_LAYERS;
                 temporal_layer_idex++) {
                rate_control_layer_temp_ptr =
                    rate_control_param_ptr->rate_control_layer_array[temporal_layer_idex];

                target_bit_rate =
                    (uint64_t)((int64_t)parentpicture_control_set_ptr->target_bit_rate -
                               MIN((int64_t)parentpicture_control_set_ptr->target_bit_rate * 3 / 4,
                                   (int64_t)(parentpicture_control_set_ptr->total_num_bits *
                                             context_ptr->frame_rate /
                                             (scs_ptr->static_config.intra_period_length + 1)) >>
                                       RC_PRECISION)) *
                    rate_percentage_layer_array[scs_ptr->static_config.hierarchical_levels]
                                               [temporal_layer_idex] /
                    100;

#if ADAPTIVE_PERCENTAGE
                if (scs_ptr->static_config.look_ahead_distance != 0 &&
                    scs_ptr->intra_period_length != -1) {
                    target_bit_rate =
                        (uint64_t)(
                            (int64_t)parentpicture_control_set_ptr->target_bit_rate -
                            MIN((int64_t)parentpicture_control_set_ptr->target_bit_rate * 3 / 4,
                                (int64_t)(parentpicture_control_set_ptr->total_num_bits *
                                          context_ptr->frame_rate /
                                          (scs_ptr->static_config.intra_period_length + 1)) >>
                                    RC_PRECISION)) *
                        parentpicture_control_set_ptr->bits_per_sw_per_layer[temporal_layer_idex] /
                        sum_bits_per_sw;
                }
#endif
                // update this based on temporal layers
                if (temporal_layer_idex == 0)
                    channel_bit_rate =
                        (((target_bit_rate << (2 * RC_PRECISION)) /
                          MAX(1,
                              rate_control_layer_temp_ptr->frame_rate -
                                  (1 * context_ptr->frame_rate /
                                   (scs_ptr->static_config.intra_period_length + 1)))) +
                         RC_PRECISION_OFFSET) >>
                        RC_PRECISION;
                else
                    channel_bit_rate = (((target_bit_rate << (2 * RC_PRECISION)) /
                                         rate_control_layer_temp_ptr->frame_rate) +
                                        RC_PRECISION_OFFSET) >>
                                       RC_PRECISION;
                channel_bit_rate = (uint64_t)MAX((int64_t)1, (int64_t)channel_bit_rate);
                rate_control_layer_temp_ptr->ec_bit_constraint = channel_bit_rate;

                slice_num = 1;
                rate_control_layer_temp_ptr->ec_bit_constraint -= SLICE_HEADER_BITS_NUM * slice_num;

                rate_control_layer_temp_ptr->previous_bit_constraint = channel_bit_rate;
                rate_control_layer_temp_ptr->bit_constraint          = channel_bit_rate;
                rate_control_layer_temp_ptr->channel_bit_rate        = channel_bit_rate;
            }
            if ((int64_t)parentpicture_control_set_ptr->target_bit_rate * 3 / 4 <
                (int64_t)(parentpicture_control_set_ptr->total_num_bits * context_ptr->frame_rate /
                          (scs_ptr->static_config.intra_period_length + 1)) >>
                RC_PRECISION) {
                rate_control_param_ptr->previous_virtual_buffer_level +=
                    (int64_t)(
                        (parentpicture_control_set_ptr->total_num_bits * context_ptr->frame_rate /
                         (scs_ptr->static_config.intra_period_length + 1)) >>
                        RC_PRECISION) -
                    (int64_t)parentpicture_control_set_ptr->target_bit_rate * 3 / 4;
                context_ptr->extra_bits_gen -=
                    (int64_t)(
                        (parentpicture_control_set_ptr->total_num_bits * context_ptr->frame_rate /
                         (scs_ptr->static_config.intra_period_length + 1)) >>
                        RC_PRECISION) -
                    (int64_t)parentpicture_control_set_ptr->target_bit_rate * 3 / 4;
            }
        }

        if (previous_frame_bit_actual) {
            uint64_t bit_changes_rate;
            // Updating virtual buffer level and it can be negative
            if ((parentpicture_control_set_ptr->picture_number ==
                 rate_control_param_ptr->first_poc) &&
                (parentpicture_control_set_ptr->slice_type == I_SLICE) &&
                (rate_control_param_ptr->last_gop == EB_FALSE) &&
                scs_ptr->static_config.intra_period_length != -1) {
                rate_control_param_ptr->virtual_buffer_level =
                    (int64_t)rate_control_param_ptr->previous_virtual_buffer_level;
            } else {
                rate_control_param_ptr->virtual_buffer_level =
                    (int64_t)rate_control_param_ptr->previous_virtual_buffer_level +
                    (int64_t)previous_frame_bit_actual -
                    (int64_t)rate_control_layer_ptr->channel_bit_rate;
                context_ptr->extra_bits_gen -= (int64_t)previous_frame_bit_actual -
                                               (int64_t)rate_control_layer_ptr->channel_bit_rate;
            }
            if (parentpicture_control_set_ptr->hierarchical_levels > 1 &&
                rate_control_layer_ptr->frame_same_distortion_min_qp_count > 10) {
                rate_control_layer_ptr->previous_bit_constraint =
                    (int64_t)rate_control_layer_ptr->channel_bit_rate;
                rate_control_param_ptr->virtual_buffer_level =
                    ((int64_t)context_ptr->virtual_buffer_size >> 1);
            }
            // Updating bit difference
            rate_control_layer_ptr->bit_diff =
                (int64_t)rate_control_param_ptr->virtual_buffer_level
                //- ((int64_t)context_ptr->virtual_buffer_size>>1);
                - ((int64_t)rate_control_layer_ptr->channel_bit_rate >> 1);

            // Limit the bit difference
            rate_control_layer_ptr->bit_diff =
                CLIP3(-(int64_t)(rate_control_layer_ptr->channel_bit_rate),
                      (int64_t)(rate_control_layer_ptr->channel_bit_rate >> 1),
                      rate_control_layer_ptr->bit_diff);
            bit_changes_rate = rate_control_layer_ptr->frame_rate;

            // Updating bit Constraint
            rate_control_layer_ptr->bit_constraint =
                MAX((int64_t)rate_control_layer_ptr->previous_bit_constraint -
                        ((rate_control_layer_ptr->bit_diff << RC_PRECISION) /
                         ((int64_t)bit_changes_rate)),
                    1);

            // Limiting the bit_constraint
            if (parentpicture_control_set_ptr->temporal_layer_index == 0) {
                rate_control_layer_ptr->bit_constraint =
                    CLIP3(rate_control_layer_ptr->channel_bit_rate >> 2,
                          rate_control_layer_ptr->channel_bit_rate * 200 / 100,
                          rate_control_layer_ptr->bit_constraint);
            } else {
                rate_control_layer_ptr->bit_constraint =
                    CLIP3(rate_control_layer_ptr->channel_bit_rate >> 1,
                          rate_control_layer_ptr->channel_bit_rate * 200 / 100,
                          rate_control_layer_ptr->bit_constraint);
            }
            rate_control_layer_ptr->ec_bit_constraint =
                (uint64_t)MAX((int64_t)rate_control_layer_ptr->bit_constraint -
                                  (int64_t)rate_control_layer_ptr->dif_total_and_ec_bits,
                              1);
            rate_control_param_ptr->previous_virtual_buffer_level =
                rate_control_param_ptr->virtual_buffer_level;
            rate_control_layer_ptr->previous_bit_constraint =
                rate_control_layer_ptr->bit_constraint;
        }

        rate_control_param_ptr->processed_frames_number++;
        rate_control_param_ptr->in_use = EB_TRUE;
        // check if all the frames in the interval have arrived
        if (rate_control_param_ptr->processed_frames_number ==
                (rate_control_param_ptr->last_poc - rate_control_param_ptr->first_poc + 1) &&
            scs_ptr->intra_period_length != -1) {
            uint32_t temporal_index;
            int64_t  extra_bits;
            rate_control_param_ptr->first_poc +=
                PARALLEL_GOP_MAX_NUMBER * (uint32_t)(scs_ptr->intra_period_length + 1);
            rate_control_param_ptr->last_poc +=
                PARALLEL_GOP_MAX_NUMBER * (uint32_t)(scs_ptr->intra_period_length + 1);
            rate_control_param_ptr->processed_frames_number      = 0;
            rate_control_param_ptr->extra_ap_bit_ratio_i         = 0;
            rate_control_param_ptr->in_use                       = EB_FALSE;
            rate_control_param_ptr->was_used                     = EB_TRUE;
            rate_control_param_ptr->last_gop                     = EB_FALSE;
            rate_control_param_ptr->first_pic_actual_qp_assigned = EB_FALSE;
            for (temporal_index = 0; temporal_index < EB_MAX_TEMPORAL_LAYERS; temporal_index++) {
                rate_control_param_ptr->rate_control_layer_array[temporal_index]->first_frame = 1;
                rate_control_param_ptr->rate_control_layer_array[temporal_index]
                    ->first_non_intra_frame = 1;
                rate_control_param_ptr->rate_control_layer_array[temporal_index]->feedback_arrived =
                    EB_FALSE;
            }
            extra_bits = ((int64_t)context_ptr->virtual_buffer_size >> 1) -
                         (int64_t)rate_control_param_ptr->virtual_buffer_level;

            rate_control_param_ptr->virtual_buffer_level = context_ptr->virtual_buffer_size >> 1;
            context_ptr->extra_bits += extra_bits;
        }
        // Allocate the extra_bits among other GOPs
        if ((parentpicture_control_set_ptr->temporal_layer_index <= 2) &&
            ((context_ptr->extra_bits > (int64_t)(context_ptr->virtual_buffer_size >> 8)) ||
             (context_ptr->extra_bits < -(int64_t)(context_ptr->virtual_buffer_size >> 8)))) {
            uint32_t interval_index_temp, interval_in_use_count;
            int64_t  extra_bits_per_gop;
            int64_t  extra_bits = context_ptr->extra_bits;
            int32_t  clip_coef1, clip_coef2;
            if (parentpicture_control_set_ptr->end_of_sequence_region) {
                clip_coef1 = -1;
                clip_coef2 = -1;
            } else {
                if (context_ptr->extra_bits > (int64_t)(context_ptr->virtual_buffer_size << 3) ||
                    context_ptr->extra_bits < -(int64_t)(context_ptr->virtual_buffer_size << 3)) {
                    clip_coef1 = 0;
                    clip_coef2 = 0;
                } else {
                    clip_coef1 = 2;
                    clip_coef2 = 4;
                }
            }

            interval_in_use_count = 0;

            if (extra_bits > 0) {
                // Extra bits to be distributed
                // Distribute it among those that are consuming more
                for (interval_index_temp = 0; interval_index_temp < PARALLEL_GOP_MAX_NUMBER;
                     interval_index_temp++) {
                    if (context_ptr->rate_control_param_queue[interval_index_temp]->in_use &&
                        context_ptr->rate_control_param_queue[interval_index_temp]
                                ->virtual_buffer_level >
                            ((int64_t)context_ptr->virtual_buffer_size >> 1)) {
                        interval_in_use_count++;
                    }
                }
                // Distribute the rate among them
                if (interval_in_use_count) {
                    extra_bits_per_gop = extra_bits / interval_in_use_count;
                    if (clip_coef1 > 0)
                        extra_bits_per_gop =
                            CLIP3(-(int64_t)context_ptr->virtual_buffer_size >> clip_coef1,
                                  (int64_t)context_ptr->virtual_buffer_size >> clip_coef1,
                                  extra_bits_per_gop);
                    else
                        extra_bits_per_gop =
                            CLIP3(-(int64_t)context_ptr->virtual_buffer_size << (-clip_coef1),
                                  (int64_t)context_ptr->virtual_buffer_size << (-clip_coef1),
                                  extra_bits_per_gop);

                    for (interval_index_temp = 0; interval_index_temp < PARALLEL_GOP_MAX_NUMBER;
                         interval_index_temp++) {
                        if (context_ptr->rate_control_param_queue[interval_index_temp]->in_use &&
                            context_ptr->rate_control_param_queue[interval_index_temp]
                                    ->virtual_buffer_level >
                                ((int64_t)context_ptr->virtual_buffer_size >> 1)) {
                            context_ptr->rate_control_param_queue[interval_index_temp]
                                ->virtual_buffer_level -= extra_bits_per_gop;
                            context_ptr->rate_control_param_queue[interval_index_temp]
                                ->previous_virtual_buffer_level -= extra_bits_per_gop;
                            context_ptr->extra_bits -= extra_bits_per_gop;
                        }
                    }
                }
                // if no interval with more consuming was found, allocate it to ones with consuming less
                else {
                    interval_in_use_count = 0;
                    // Distribute it among those that are consuming less
                    for (interval_index_temp = 0; interval_index_temp < PARALLEL_GOP_MAX_NUMBER;
                         interval_index_temp++) {
                        if (context_ptr->rate_control_param_queue[interval_index_temp]->in_use &&
                            context_ptr->rate_control_param_queue[interval_index_temp]
                                    ->virtual_buffer_level <=
                                ((int64_t)context_ptr->virtual_buffer_size >> 1)) {
                            interval_in_use_count++;
                        }
                    }
                    if (interval_in_use_count) {
                        extra_bits_per_gop = extra_bits / interval_in_use_count;
                        if (clip_coef2 > 0)
                            extra_bits_per_gop =
                                CLIP3(-(int64_t)context_ptr->virtual_buffer_size >> clip_coef2,
                                      (int64_t)context_ptr->virtual_buffer_size >> clip_coef2,
                                      extra_bits_per_gop);
                        else
                            extra_bits_per_gop =
                                CLIP3(-(int64_t)context_ptr->virtual_buffer_size << (-clip_coef2),
                                      (int64_t)context_ptr->virtual_buffer_size << (-clip_coef2),
                                      extra_bits_per_gop);
                        // Distribute the rate among them
                        for (interval_index_temp = 0; interval_index_temp < PARALLEL_GOP_MAX_NUMBER;
                             interval_index_temp++) {
                            if (context_ptr->rate_control_param_queue[interval_index_temp]
                                    ->in_use &&
                                context_ptr->rate_control_param_queue[interval_index_temp]
                                        ->virtual_buffer_level <=
                                    ((int64_t)context_ptr->virtual_buffer_size >> 1)) {
                                context_ptr->rate_control_param_queue[interval_index_temp]
                                    ->virtual_buffer_level -= extra_bits_per_gop;
                                context_ptr->rate_control_param_queue[interval_index_temp]
                                    ->previous_virtual_buffer_level -= extra_bits_per_gop;
                                context_ptr->extra_bits -= extra_bits_per_gop;
                            }
                        }
                    }
                }
            } else {
                // Distribute it among those that are consuming less
                for (interval_index_temp = 0; interval_index_temp < PARALLEL_GOP_MAX_NUMBER;
                     interval_index_temp++) {
                    if (context_ptr->rate_control_param_queue[interval_index_temp]->in_use &&
                        context_ptr->rate_control_param_queue[interval_index_temp]
                                ->virtual_buffer_level <
                            ((int64_t)context_ptr->virtual_buffer_size >> 1)) {
                        interval_in_use_count++;
                    }
                }
                if (interval_in_use_count) {
                    extra_bits_per_gop = extra_bits / interval_in_use_count;
                    if (clip_coef1 > 0)
                        extra_bits_per_gop =
                            CLIP3(-(int64_t)context_ptr->virtual_buffer_size >> clip_coef1,
                                  (int64_t)context_ptr->virtual_buffer_size >> clip_coef1,
                                  extra_bits_per_gop);
                    else
                        extra_bits_per_gop =
                            CLIP3(-(int64_t)context_ptr->virtual_buffer_size << (-clip_coef1),
                                  (int64_t)context_ptr->virtual_buffer_size << (-clip_coef1),
                                  extra_bits_per_gop);
                    // Distribute the rate among them
                    for (interval_index_temp = 0; interval_index_temp < PARALLEL_GOP_MAX_NUMBER;
                         interval_index_temp++) {
                        if (context_ptr->rate_control_param_queue[interval_index_temp]->in_use &&
                            context_ptr->rate_control_param_queue[interval_index_temp]
                                    ->virtual_buffer_level <
                                ((int64_t)context_ptr->virtual_buffer_size >> 1)) {
                            context_ptr->rate_control_param_queue[interval_index_temp]
                                ->virtual_buffer_level -= extra_bits_per_gop;
                            context_ptr->rate_control_param_queue[interval_index_temp]
                                ->previous_virtual_buffer_level -= extra_bits_per_gop;
                            context_ptr->extra_bits -= extra_bits_per_gop;
                        }
                    }
                }
                // if no interval with less consuming was found, allocate it to ones with consuming more
                else {
                    interval_in_use_count = 0;
                    for (interval_index_temp = 0; interval_index_temp < PARALLEL_GOP_MAX_NUMBER;
                         interval_index_temp++) {
                        if (context_ptr->rate_control_param_queue[interval_index_temp]->in_use &&
                            context_ptr->rate_control_param_queue[interval_index_temp]
                                    ->virtual_buffer_level <
                                (int64_t)(context_ptr->virtual_buffer_size)) {
                            interval_in_use_count++;
                        }
                    }
                    if (interval_in_use_count) {
                        extra_bits_per_gop = extra_bits / interval_in_use_count;
                        if (clip_coef2 > 0)
                            extra_bits_per_gop =
                                CLIP3(-(int64_t)context_ptr->virtual_buffer_size >> clip_coef2,
                                      (int64_t)context_ptr->virtual_buffer_size >> clip_coef2,
                                      extra_bits_per_gop);
                        else
                            extra_bits_per_gop =
                                CLIP3(-(int64_t)context_ptr->virtual_buffer_size << (-clip_coef2),
                                      (int64_t)context_ptr->virtual_buffer_size << (-clip_coef2),
                                      extra_bits_per_gop);
                        // Distribute the rate among them
                        for (interval_index_temp = 0; interval_index_temp < PARALLEL_GOP_MAX_NUMBER;
                             interval_index_temp++) {
                            if (context_ptr->rate_control_param_queue[interval_index_temp]
                                    ->in_use &&
                                context_ptr->rate_control_param_queue[interval_index_temp]
                                        ->virtual_buffer_level <
                                    (int64_t)(context_ptr->virtual_buffer_size)) {
                                context_ptr->rate_control_param_queue[interval_index_temp]
                                    ->virtual_buffer_level -= extra_bits_per_gop;
                                context_ptr->rate_control_param_queue[interval_index_temp]
                                    ->previous_virtual_buffer_level -= extra_bits_per_gop;
                                context_ptr->extra_bits -= extra_bits_per_gop;
                            }
                        }
                    }
                }
            }
        }
    }

#if RC_PRINTS
    if (parentpicture_control_set_ptr->temporal_layer_index == 0) {
        SVT_LOG("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%.0f\t%.0f\t%.0f\t%.0f\t%d\t%d\n",
                (int)parentpicture_control_set_ptr->slice_type,
                (int)parentpicture_control_set_ptr->picture_number,
                (int)parentpicture_control_set_ptr->temporal_layer_index,
                (int)parentpicture_control_set_ptr->picture_qp,
                (int)parentpicture_control_set_ptr->calculated_qp,
                (int)parentpicture_control_set_ptr->best_pred_qp,
                (int)previous_frame_bit_actual,
                (int)parentpicture_control_set_ptr->target_bits_best_pred_qp,
                (int)parentpicture_control_set_ptr->target_bits_rc,
                (int)rate_control_layer_ptr->channel_bit_rate,
                (int)rate_control_layer_ptr->bit_constraint,
                (double)rate_control_layer_ptr->c_coeff,
                (double)rate_control_layer_ptr->k_coeff,
                (double)parentpicture_control_set_ptr->sad_me,
                (double)context_ptr->extra_bits_gen,
                (int)rate_control_param_ptr->virtual_buffer_level,
                (int)context_ptr->extra_bits);
    }
#endif
}
void high_level_rc_input_picture_cvbr(PictureParentControlSet *pcs_ptr, SequenceControlSet *scs_ptr,
                                      EncodeContext *              encode_context_ptr,
                                      RateControlContext *         context_ptr,
                                      HighLevelRateControlContext *high_level_rate_control_ptr) {
    EbBool end_of_sequence_flag = EB_TRUE;

    HlRateControlHistogramEntry *hl_rate_control_histogram_ptr_temp;
    // Queue variables
    uint32_t queue_entry_index_temp;
    uint32_t queue_entry_index_temp2;
    int64_t  queue_entry_index_head_temp;

    uint64_t min_la_bit_distance;
    uint32_t selected_ref_qp_table_index;
    uint32_t selected_ref_qp;
    uint32_t selected_org_ref_qp;
    uint32_t previous_selected_ref_qp      = encode_context_ptr->previous_selected_ref_qp;
    uint64_t max_coded_poc                 = encode_context_ptr->max_coded_poc;
    uint32_t max_coded_poc_selected_ref_qp = encode_context_ptr->max_coded_poc_selected_ref_qp;

    uint32_t ref_qp_index;
    uint32_t ref_qp_index_temp;
    uint32_t ref_qp_table_index;

    uint32_t area_in_pixel;
    uint32_t num_of_full_sbs;
    uint32_t qp_search_min;
    uint32_t qp_search_max;
    int32_t  qp_step = 1;
    EbBool   best_qp_found;
    uint32_t temporal_layer_index;
    EbBool   tables_updated;

    uint64_t bit_constraint_per_sw = 0;

    RateControlTables *rate_control_tables_ptr;
    EbBitNumber *      sad_bits_array_ptr;
    EbBitNumber *      intra_sad_bits_array_ptr;
    uint32_t           pred_bits_ref_qp;
    int                delta_qp = 0;

    for (temporal_layer_index = 0; temporal_layer_index < EB_MAX_TEMPORAL_LAYERS;
         temporal_layer_index++)
        pcs_ptr->bits_per_sw_per_layer[temporal_layer_index] = 0;
    pcs_ptr->total_bits_per_gop = 0;

    area_in_pixel = pcs_ptr->aligned_width * pcs_ptr->aligned_height;
    ;

    eb_block_on_mutex(scs_ptr->encode_context_ptr->rate_table_update_mutex);

    tables_updated              = scs_ptr->encode_context_ptr->rate_control_tables_array_updated;
    pcs_ptr->percentage_updated = EB_FALSE;
    if (scs_ptr->static_config.look_ahead_distance != 0) {
        // Increamenting the head of the hl_rate_control_historgram_queue and clean up the entores
        hl_rate_control_histogram_ptr_temp =
            (encode_context_ptr->hl_rate_control_historgram_queue
                 [encode_context_ptr->hl_rate_control_historgram_queue_head_index]);

        while ((hl_rate_control_histogram_ptr_temp->life_count == 0) &&
               hl_rate_control_histogram_ptr_temp->passed_to_hlrc) {
            eb_block_on_mutex(scs_ptr->encode_context_ptr->hl_rate_control_historgram_queue_mutex);
            // Reset the Reorder Queue Entry
            hl_rate_control_histogram_ptr_temp->picture_number +=
                INITIAL_RATE_CONTROL_REORDER_QUEUE_MAX_DEPTH;
            hl_rate_control_histogram_ptr_temp->life_count           = -1;
            hl_rate_control_histogram_ptr_temp->passed_to_hlrc       = EB_FALSE;
            hl_rate_control_histogram_ptr_temp->is_coded             = EB_FALSE;
            hl_rate_control_histogram_ptr_temp->total_num_bits_coded = 0;

            // Increment the Reorder Queue head Ptr
            encode_context_ptr->hl_rate_control_historgram_queue_head_index =
                (encode_context_ptr->hl_rate_control_historgram_queue_head_index ==
                 HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH - 1)
                    ? 0
                    : encode_context_ptr->hl_rate_control_historgram_queue_head_index + 1;
            eb_release_mutex(scs_ptr->encode_context_ptr->hl_rate_control_historgram_queue_mutex);
            hl_rate_control_histogram_ptr_temp =
                encode_context_ptr->hl_rate_control_historgram_queue
                    [encode_context_ptr->hl_rate_control_historgram_queue_head_index];
        }
        // For the case that number of frames in the sliding window is less than size of the look ahead or intra Refresh. i.e. end of sequence
        if ((pcs_ptr->frames_in_sw < MIN(scs_ptr->static_config.look_ahead_distance + 1,
                                         (uint32_t)scs_ptr->intra_period_length + 1))) {
            selected_ref_qp = max_coded_poc_selected_ref_qp;

            // Update the QP for the sliding window based on the status of RC
            if ((context_ptr->extra_bits_gen > (int64_t)(context_ptr->virtual_buffer_size << 3)))
                selected_ref_qp = (uint32_t)MAX((int32_t)selected_ref_qp - 2, 0);
            else if ((context_ptr->extra_bits_gen >
                      (int64_t)(context_ptr->virtual_buffer_size << 2)))
                selected_ref_qp = (uint32_t)MAX((int32_t)selected_ref_qp - 1, 0);
            if ((context_ptr->extra_bits_gen < -(int64_t)(context_ptr->virtual_buffer_size << 2)))
                selected_ref_qp += 2;
            else if ((context_ptr->extra_bits_gen <
                      -(int64_t)(context_ptr->virtual_buffer_size << 1)))
                selected_ref_qp += 1;
            if ((pcs_ptr->frames_in_sw < (uint32_t)(scs_ptr->intra_period_length + 1)) &&
                (pcs_ptr->picture_number % ((scs_ptr->intra_period_length + 1)) == 0)) {
                selected_ref_qp++;
            }

            selected_ref_qp = (uint32_t)CLIP3(scs_ptr->static_config.min_qp_allowed,
                                              scs_ptr->static_config.max_qp_allowed,
                                              selected_ref_qp);

            queue_entry_index_head_temp =
                (int32_t)(pcs_ptr->picture_number -
                          encode_context_ptr
                              ->hl_rate_control_historgram_queue
                                  [encode_context_ptr->hl_rate_control_historgram_queue_head_index]
                              ->picture_number);
            queue_entry_index_head_temp +=
                encode_context_ptr->hl_rate_control_historgram_queue_head_index;
            queue_entry_index_head_temp =
                (queue_entry_index_head_temp >
                 HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH - 1)
                    ? queue_entry_index_head_temp -
                          HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH
                    : (queue_entry_index_head_temp < 0)
                          ? queue_entry_index_head_temp +
                                HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH
                          : queue_entry_index_head_temp;

            queue_entry_index_temp = (uint32_t)queue_entry_index_head_temp;
            {
                hl_rate_control_histogram_ptr_temp =
                    (encode_context_ptr->hl_rate_control_historgram_queue[queue_entry_index_temp]);

                if (hl_rate_control_histogram_ptr_temp->slice_type == I_SLICE)
                    ref_qp_index_temp = context_ptr->qp_scaling_map_i_slice[selected_ref_qp];
                else
                    ref_qp_index_temp =
                        context_ptr->qp_scaling_map[hl_rate_control_histogram_ptr_temp
                                                        ->temporal_layer_index][selected_ref_qp];

                ref_qp_index_temp = (uint32_t)CLIP3(scs_ptr->static_config.min_qp_allowed,
                                                    scs_ptr->static_config.max_qp_allowed,
                                                    ref_qp_index_temp);

                hl_rate_control_histogram_ptr_temp->pred_bits_ref_qp[ref_qp_index_temp] = 0;
                rate_control_tables_ptr =
                    &encode_context_ptr->rate_control_tables_array[ref_qp_index_temp];
                sad_bits_array_ptr =
                    rate_control_tables_ptr
                        ->sad_bits_array[hl_rate_control_histogram_ptr_temp->temporal_layer_index];
                intra_sad_bits_array_ptr =
                    rate_control_tables_ptr->intra_sad_bits_array[hl_rate_control_histogram_ptr_temp
                                                                      ->temporal_layer_index];
                pred_bits_ref_qp = 0;
                num_of_full_sbs  = 0;

                if (hl_rate_control_histogram_ptr_temp->slice_type == I_SLICE) {
                    // Loop over block in the frame and calculated the predicted bits at reg QP
                    {
                        unsigned i;
                        uint32_t accum = 0;
                        for (i = 0; i < NUMBER_OF_INTRA_SAD_INTERVALS; ++i)
                            accum += (uint32_t)(
                                hl_rate_control_histogram_ptr_temp->ois_distortion_histogram[i] *
                                intra_sad_bits_array_ptr[i]);
                        pred_bits_ref_qp = accum;
                        num_of_full_sbs  = hl_rate_control_histogram_ptr_temp->full_sb_count;
                    }
                    hl_rate_control_histogram_ptr_temp->pred_bits_ref_qp[ref_qp_index_temp] +=
                        pred_bits_ref_qp;
                }

                else {
                    {
                        unsigned i;
                        uint32_t accum = 0;
                        for (i = 0; i < NUMBER_OF_SAD_INTERVALS; ++i)
                            accum += (uint32_t)(
                                hl_rate_control_histogram_ptr_temp->me_distortion_histogram[i] *
                                sad_bits_array_ptr[i]);
                        pred_bits_ref_qp = accum;
                        num_of_full_sbs  = hl_rate_control_histogram_ptr_temp->full_sb_count;
                    }
                    hl_rate_control_histogram_ptr_temp->pred_bits_ref_qp[ref_qp_index_temp] +=
                        pred_bits_ref_qp;
                }

                // Scale for in complete
                //  pred_bits_ref_qp is normalized based on the area because of the SBs at the picture boundries
                hl_rate_control_histogram_ptr_temp->pred_bits_ref_qp[ref_qp_index_temp] =
                    hl_rate_control_histogram_ptr_temp->pred_bits_ref_qp[ref_qp_index_temp] *
                    (uint64_t)area_in_pixel / (num_of_full_sbs << 12);

                // Store the pred_bits_ref_qp for the first frame in the window to PCS
                pcs_ptr->pred_bits_ref_qp[ref_qp_index_temp] =
                    hl_rate_control_histogram_ptr_temp->pred_bits_ref_qp[ref_qp_index_temp];
            }
        } else {
            // Loop over the QPs and find the best QP
            min_la_bit_distance = MAX_UNSIGNED_VALUE;
            qp_search_min =
                (uint8_t)CLIP3(scs_ptr->static_config.min_qp_allowed,
                               MAX_REF_QP_NUM, //scs_ptr->static_config.max_qp_allowed,
                               (uint32_t)MAX((int32_t)scs_ptr->static_config.qp - 40, 0));

            qp_search_max = (uint8_t)CLIP3(scs_ptr->static_config.min_qp_allowed,
                                           MAX_REF_QP_NUM, //scs_ptr->static_config.max_qp_allowed,
                                           scs_ptr->static_config.qp + 40);

            for (ref_qp_table_index = qp_search_min; ref_qp_table_index < qp_search_max;
                 ref_qp_table_index++)
                high_level_rate_control_ptr->pred_bits_ref_qp_per_sw[ref_qp_table_index] = 0;
            bit_constraint_per_sw = high_level_rate_control_ptr->bit_constraint_per_sw *
                                    pcs_ptr->frames_in_sw /
                                    (scs_ptr->static_config.look_ahead_distance + 1);

            // Update the target rate for the sliding window based on the status of RC
            if ((context_ptr->extra_bits_gen > (int64_t)(context_ptr->virtual_buffer_size * 10)))
                bit_constraint_per_sw = bit_constraint_per_sw * 130 / 100;
            else if ((context_ptr->extra_bits_gen >
                      (int64_t)(context_ptr->virtual_buffer_size << 3)))
                bit_constraint_per_sw = bit_constraint_per_sw * 120 / 100;
            else if ((context_ptr->extra_bits_gen >
                      (int64_t)(context_ptr->virtual_buffer_size << 2)))
                bit_constraint_per_sw = bit_constraint_per_sw * 110 / 100;
            if ((context_ptr->extra_bits_gen < -(int64_t)(context_ptr->virtual_buffer_size << 3)))
                bit_constraint_per_sw = bit_constraint_per_sw * 80 / 100;
            else if ((context_ptr->extra_bits_gen <
                      -(int64_t)(context_ptr->virtual_buffer_size << 2)))
                bit_constraint_per_sw = bit_constraint_per_sw * 90 / 100;
            // Loop over proper QPs and find the Predicted bits for that QP. Find the QP with the closest total predicted rate to target bits for the sliding window.
            previous_selected_ref_qp =
                CLIP3(qp_search_min + 1, qp_search_max - 1, previous_selected_ref_qp);
            ref_qp_table_index          = previous_selected_ref_qp;
            ref_qp_index                = ref_qp_table_index;
            selected_ref_qp_table_index = ref_qp_table_index;
            selected_ref_qp             = selected_ref_qp_table_index;
            if (scs_ptr->intra_period_length != -1 &&
                pcs_ptr->picture_number % ((scs_ptr->intra_period_length + 1)) == 0 &&
                (int32_t)pcs_ptr->frames_in_sw > scs_ptr->intra_period_length) {
                best_qp_found = EB_FALSE;
                while (ref_qp_table_index >= qp_search_min && ref_qp_table_index <= qp_search_max &&
                       !best_qp_found) {
                    ref_qp_index = CLIP3(scs_ptr->static_config.min_qp_allowed,
                                         MAX_REF_QP_NUM, //scs_ptr->static_config.max_qp_allowed,
                                         ref_qp_table_index);
                    high_level_rate_control_ptr->pred_bits_ref_qp_per_sw[ref_qp_index] = 0;

                    // Finding the predicted bits for each frame in the sliding window at the reference Qp(s)
                    queue_entry_index_head_temp = (int32_t)(
                        pcs_ptr->picture_number -
                        encode_context_ptr
                            ->hl_rate_control_historgram_queue
                                [encode_context_ptr->hl_rate_control_historgram_queue_head_index]
                            ->picture_number);
                    queue_entry_index_head_temp +=
                        encode_context_ptr->hl_rate_control_historgram_queue_head_index;
                    queue_entry_index_head_temp =
                        (queue_entry_index_head_temp >
                         HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH - 1)
                            ? queue_entry_index_head_temp -
                                  HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH
                            : (queue_entry_index_head_temp < 0)
                                  ? queue_entry_index_head_temp +
                                        HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH
                                  : queue_entry_index_head_temp;

                    queue_entry_index_temp = (uint32_t)queue_entry_index_head_temp;
                    // This is set to false, so the last frame would go inside the loop
                    end_of_sequence_flag = EB_FALSE;

                    while (!end_of_sequence_flag &&
                           queue_entry_index_temp <=
                               queue_entry_index_head_temp +
                                   scs_ptr->static_config.look_ahead_distance) {
                        queue_entry_index_temp2 =
                            (queue_entry_index_temp >
                             HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH - 1)
                                ? queue_entry_index_temp -
                                      HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH
                                : queue_entry_index_temp;
                        hl_rate_control_histogram_ptr_temp =
                            (encode_context_ptr
                                 ->hl_rate_control_historgram_queue[queue_entry_index_temp2]);

                        if (hl_rate_control_histogram_ptr_temp->slice_type == I_SLICE)
                            ref_qp_index_temp = context_ptr->qp_scaling_map_i_slice[ref_qp_index];
                        else
                            ref_qp_index_temp =
                                context_ptr
                                    ->qp_scaling_map[hl_rate_control_histogram_ptr_temp
                                                         ->temporal_layer_index][ref_qp_index];

                        ref_qp_index_temp = (uint32_t)CLIP3(scs_ptr->static_config.min_qp_allowed,
                                                            scs_ptr->static_config.max_qp_allowed,
                                                            ref_qp_index_temp);

                        hl_rate_control_histogram_ptr_temp->pred_bits_ref_qp[ref_qp_index_temp] = 0;

                        if (ref_qp_table_index == previous_selected_ref_qp) {
                            eb_block_on_mutex(scs_ptr->encode_context_ptr
                                                  ->hl_rate_control_historgram_queue_mutex);
                            hl_rate_control_histogram_ptr_temp->life_count--;
                            eb_release_mutex(scs_ptr->encode_context_ptr
                                                 ->hl_rate_control_historgram_queue_mutex);
                        }
                        hl_rate_control_histogram_ptr_temp->pred_bits_ref_qp[ref_qp_index_temp] =
                            predict_bits(encode_context_ptr,
                                         hl_rate_control_histogram_ptr_temp,
                                         ref_qp_index_temp,
                                         area_in_pixel);

                        high_level_rate_control_ptr->pred_bits_ref_qp_per_sw[ref_qp_index] +=
                            hl_rate_control_histogram_ptr_temp->pred_bits_ref_qp[ref_qp_index_temp];
                        // Store the pred_bits_ref_qp for the first frame in the window to PCS
                        if (queue_entry_index_head_temp == queue_entry_index_temp2)
                            pcs_ptr->pred_bits_ref_qp[ref_qp_index_temp] =
                                hl_rate_control_histogram_ptr_temp
                                    ->pred_bits_ref_qp[ref_qp_index_temp];

                        end_of_sequence_flag =
                            hl_rate_control_histogram_ptr_temp->end_of_sequence_flag;
                        queue_entry_index_temp++;
                    }

                    if (min_la_bit_distance >=
                        (uint64_t)ABS((int64_t)high_level_rate_control_ptr
                                          ->pred_bits_ref_qp_per_sw[ref_qp_index] -
                                      (int64_t)bit_constraint_per_sw)) {
                        min_la_bit_distance = (uint64_t)ABS(
                            (int64_t)
                                high_level_rate_control_ptr->pred_bits_ref_qp_per_sw[ref_qp_index] -
                            (int64_t)bit_constraint_per_sw);
                        selected_ref_qp_table_index = ref_qp_table_index;
                        selected_ref_qp             = ref_qp_index;
                    } else
                        best_qp_found = EB_TRUE;
                    if (ref_qp_table_index == previous_selected_ref_qp) {
                        if (high_level_rate_control_ptr->pred_bits_ref_qp_per_sw[ref_qp_index] >
                            bit_constraint_per_sw)
                            qp_step = +1;
                        else
                            qp_step = -1;
                    }
                    ref_qp_table_index = (uint32_t)(ref_qp_table_index + qp_step);
                }

                if (ref_qp_index == scs_ptr->static_config.max_qp_allowed &&
                    high_level_rate_control_ptr->pred_bits_ref_qp_per_sw[ref_qp_index] >
                        bit_constraint_per_sw) {
                    delta_qp =
                        (int)((high_level_rate_control_ptr->pred_bits_ref_qp_per_sw[ref_qp_index] -
                               bit_constraint_per_sw) *
                              100 /
                              (high_level_rate_control_ptr
                                   ->pred_bits_ref_qp_per_sw[ref_qp_index - 1] -
                               high_level_rate_control_ptr->pred_bits_ref_qp_per_sw[ref_qp_index]));
                    delta_qp = (delta_qp + 50) / 100;
                }
            }
        }
        selected_org_ref_qp = selected_ref_qp;
        if (scs_ptr->intra_period_length != -1 &&
            pcs_ptr->picture_number % ((scs_ptr->intra_period_length + 1)) == 0 &&
            (int32_t)pcs_ptr->frames_in_sw > scs_ptr->intra_period_length) {
            if (pcs_ptr->picture_number > 0)
                pcs_ptr->intra_selected_org_qp = (uint8_t)selected_ref_qp;
            ref_qp_index                                                       = selected_ref_qp;
            high_level_rate_control_ptr->pred_bits_ref_qp_per_sw[ref_qp_index] = 0;

            if (high_level_rate_control_ptr->pred_bits_ref_qp_per_sw[ref_qp_index] == 0) {
                // Finding the predicted bits for each frame in the sliding window at the reference Qp(s)
                //queue_entry_index_temp = encode_context_ptr->hl_rate_control_historgram_queue_head_index;
                queue_entry_index_head_temp = (int32_t)(
                    pcs_ptr->picture_number -
                    encode_context_ptr
                        ->hl_rate_control_historgram_queue
                            [encode_context_ptr->hl_rate_control_historgram_queue_head_index]
                        ->picture_number);
                queue_entry_index_head_temp +=
                    encode_context_ptr->hl_rate_control_historgram_queue_head_index;
                queue_entry_index_head_temp =
                    (queue_entry_index_head_temp >
                     HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH - 1)
                        ? queue_entry_index_head_temp -
                              HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH
                        : (queue_entry_index_head_temp < 0)
                              ? queue_entry_index_head_temp +
                                    HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH
                              : queue_entry_index_head_temp;

                queue_entry_index_temp = (uint32_t)queue_entry_index_head_temp;

                // This is set to false, so the last frame would go inside the loop
                end_of_sequence_flag = EB_FALSE;

                while (
                    !end_of_sequence_flag &&
                    //queue_entry_index_temp <= encode_context_ptr->hl_rate_control_historgram_queue_head_index+scs_ptr->static_config.look_ahead_distance){
                    queue_entry_index_temp <=
                        queue_entry_index_head_temp + scs_ptr->static_config.look_ahead_distance) {
                    queue_entry_index_temp2 =
                        (queue_entry_index_temp >
                         HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH - 1)
                            ? queue_entry_index_temp -
                                  HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH
                            : queue_entry_index_temp;
                    hl_rate_control_histogram_ptr_temp =
                        (encode_context_ptr
                             ->hl_rate_control_historgram_queue[queue_entry_index_temp2]);

                    if (hl_rate_control_histogram_ptr_temp->slice_type == I_SLICE)
                        ref_qp_index_temp = context_ptr->qp_scaling_map_i_slice[ref_qp_index];
                    else
                        ref_qp_index_temp =
                            context_ptr->qp_scaling_map[hl_rate_control_histogram_ptr_temp
                                                            ->temporal_layer_index][ref_qp_index];

                    ref_qp_index_temp = (uint32_t)CLIP3(scs_ptr->static_config.min_qp_allowed,
                                                        scs_ptr->static_config.max_qp_allowed,
                                                        ref_qp_index_temp);

                    hl_rate_control_histogram_ptr_temp->pred_bits_ref_qp[ref_qp_index_temp] =
                        predict_bits(encode_context_ptr,
                                     hl_rate_control_histogram_ptr_temp,
                                     ref_qp_index_temp,
                                     area_in_pixel);

                    high_level_rate_control_ptr->pred_bits_ref_qp_per_sw[ref_qp_index] +=
                        hl_rate_control_histogram_ptr_temp->pred_bits_ref_qp[ref_qp_index_temp];
                    // Store the pred_bits_ref_qp for the first frame in the window to PCS
                    //  if(encode_context_ptr->hl_rate_control_historgram_queue_head_index == queue_entry_index_temp2)
                    if (queue_entry_index_head_temp == queue_entry_index_temp2)
                        pcs_ptr->pred_bits_ref_qp[ref_qp_index_temp] =
                            hl_rate_control_histogram_ptr_temp->pred_bits_ref_qp[ref_qp_index_temp];

                    end_of_sequence_flag = hl_rate_control_histogram_ptr_temp->end_of_sequence_flag;
                    queue_entry_index_temp++;
                }
            }
        }
        pcs_ptr->tables_updated = tables_updated;

        // Looping over the window to find the percentage of bit allocation in each layer
        if ((scs_ptr->intra_period_length != -1) &&
            ((int32_t)pcs_ptr->frames_in_sw > scs_ptr->intra_period_length) &&
            ((int32_t)pcs_ptr->frames_in_sw > scs_ptr->intra_period_length)) {
            if (pcs_ptr->picture_number % ((scs_ptr->intra_period_length + 1)) == 0) {
                queue_entry_index_head_temp = (int32_t)(
                    pcs_ptr->picture_number -
                    encode_context_ptr
                        ->hl_rate_control_historgram_queue
                            [encode_context_ptr->hl_rate_control_historgram_queue_head_index]
                        ->picture_number);
                queue_entry_index_head_temp +=
                    encode_context_ptr->hl_rate_control_historgram_queue_head_index;
                queue_entry_index_head_temp =
                    (queue_entry_index_head_temp >
                     HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH - 1)
                        ? queue_entry_index_head_temp -
                              HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH
                        : (queue_entry_index_head_temp < 0)
                              ? queue_entry_index_head_temp +
                                    HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH
                              : queue_entry_index_head_temp;

                queue_entry_index_temp = (uint32_t)queue_entry_index_head_temp;

                // This is set to false, so the last frame would go inside the loop
                end_of_sequence_flag = EB_FALSE;

                while (!end_of_sequence_flag &&
                       queue_entry_index_temp <=
                           queue_entry_index_head_temp + scs_ptr->intra_period_length) {
                    queue_entry_index_temp2 =
                        (queue_entry_index_temp >
                         HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH - 1)
                            ? queue_entry_index_temp -
                                  HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH
                            : queue_entry_index_temp;
                    hl_rate_control_histogram_ptr_temp =
                        (encode_context_ptr
                             ->hl_rate_control_historgram_queue[queue_entry_index_temp2]);

                    if (hl_rate_control_histogram_ptr_temp->slice_type == I_SLICE)
                        ref_qp_index_temp = context_ptr->qp_scaling_map_i_slice[selected_ref_qp];
                    else
                        ref_qp_index_temp =
                            context_ptr
                                ->qp_scaling_map[hl_rate_control_histogram_ptr_temp
                                                     ->temporal_layer_index][selected_ref_qp];

                    ref_qp_index_temp = (uint32_t)CLIP3(scs_ptr->static_config.min_qp_allowed,
                                                        scs_ptr->static_config.max_qp_allowed,
                                                        ref_qp_index_temp);

                    pcs_ptr->total_bits_per_gop +=
                        hl_rate_control_histogram_ptr_temp->pred_bits_ref_qp[ref_qp_index_temp];
                    pcs_ptr->bits_per_sw_per_layer[hl_rate_control_histogram_ptr_temp
                                                       ->temporal_layer_index] +=
                        hl_rate_control_histogram_ptr_temp->pred_bits_ref_qp[ref_qp_index_temp];
                    pcs_ptr->percentage_updated = EB_TRUE;

                    end_of_sequence_flag = hl_rate_control_histogram_ptr_temp->end_of_sequence_flag;
                    queue_entry_index_temp++;
                }
                if (pcs_ptr->total_bits_per_gop == 0) {
                    for (temporal_layer_index = 0; temporal_layer_index < EB_MAX_TEMPORAL_LAYERS;
                         temporal_layer_index++)
                        pcs_ptr->bits_per_sw_per_layer[temporal_layer_index] =
                            rate_percentage_layer_array[scs_ptr->static_config.hierarchical_levels]
                                                       [temporal_layer_index];
                }
            }
        } else {
            for (temporal_layer_index = 0; temporal_layer_index < EB_MAX_TEMPORAL_LAYERS;
                 temporal_layer_index++)
                pcs_ptr->bits_per_sw_per_layer[temporal_layer_index] =
                    rate_percentage_layer_array[scs_ptr->static_config.hierarchical_levels]
                                               [temporal_layer_index];
        }

        // Set the QP
        previous_selected_ref_qp = selected_ref_qp;
        if (pcs_ptr->picture_number > max_coded_poc && pcs_ptr->temporal_layer_index < 2 &&
            !pcs_ptr->end_of_sequence_region) {
            max_coded_poc                                     = pcs_ptr->picture_number;
            max_coded_poc_selected_ref_qp                     = previous_selected_ref_qp;
            encode_context_ptr->previous_selected_ref_qp      = previous_selected_ref_qp;
            encode_context_ptr->max_coded_poc                 = max_coded_poc;
            encode_context_ptr->max_coded_poc_selected_ref_qp = max_coded_poc_selected_ref_qp;
        }

        if (pcs_ptr->slice_type == I_SLICE)
            pcs_ptr->best_pred_qp = (uint8_t)context_ptr->qp_scaling_map_i_slice[selected_ref_qp];
        else
            pcs_ptr->best_pred_qp =
                (uint8_t)
                    context_ptr->qp_scaling_map[pcs_ptr->temporal_layer_index][selected_ref_qp];

        pcs_ptr->target_bits_best_pred_qp = pcs_ptr->pred_bits_ref_qp[pcs_ptr->best_pred_qp];
        pcs_ptr->best_pred_qp             = (uint8_t)CLIP3(scs_ptr->static_config.min_qp_allowed,
                                               scs_ptr->static_config.max_qp_allowed,
                                               (uint8_t)((int)pcs_ptr->best_pred_qp + delta_qp));

        if (pcs_ptr->picture_number == 0) {
            high_level_rate_control_ptr->prev_intra_selected_ref_qp     = selected_ref_qp;
            high_level_rate_control_ptr->prev_intra_org_selected_ref_qp = selected_ref_qp;
        }
        if (scs_ptr->intra_period_length != -1) {
            if (pcs_ptr->picture_number % ((scs_ptr->intra_period_length + 1)) == 0) {
                high_level_rate_control_ptr->prev_intra_selected_ref_qp     = selected_ref_qp;
                high_level_rate_control_ptr->prev_intra_org_selected_ref_qp = selected_org_ref_qp;
            }
        }
#if RC_PRINTS
        ////if (pcs_ptr->slice_type == 2)
        {
            SVT_LOG("\nTID: %d\t", pcs_ptr->temporal_layer_index);
            SVT_LOG("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t\n",
                pcs_ptr->picture_number,
                pcs_ptr->best_pred_qp,
                selected_ref_qp,
                (int)pcs_ptr->target_bits_best_pred_qp,
                (int)high_level_rate_control_ptr->pred_bits_ref_qp_per_sw[selected_ref_qp - 1],
                (int)high_level_rate_control_ptr->pred_bits_ref_qp_per_sw[selected_ref_qp],
                (int)high_level_rate_control_ptr->pred_bits_ref_qp_per_sw[selected_ref_qp + 1],
                (int)high_level_rate_control_ptr->bit_constraint_per_sw,
                (int)bit_constraint_per_sw/*,
                (int)high_level_rate_control_ptr->virtual_buffer_level*/);
        }
#endif
    }
    eb_release_mutex(scs_ptr->encode_context_ptr->rate_table_update_mutex);
}
void frame_level_rc_input_picture_cvbr(PictureControlSet *pcs_ptr, SequenceControlSet *scs_ptr,
                                       RateControlContext *             context_ptr,
                                       RateControlLayerContext *        rate_control_layer_ptr,
                                       RateControlIntervalParamContext *rate_control_param_ptr) {
    RateControlLayerContext *rate_control_layer_temp_ptr;

    // Tiles
    uint32_t picture_area_in_pixel;
    uint32_t area_in_pixel;

    // SB Loop variables
    SbParams *sb_params_ptr;
    uint32_t  sb_index;
    uint64_t  temp_qp;
    uint32_t  area_in_sbs;

    picture_area_in_pixel =
            pcs_ptr->parent_pcs_ptr->aligned_height * pcs_ptr->parent_pcs_ptr->aligned_width;

    if (rate_control_layer_ptr->first_frame == 1) {
        rate_control_layer_ptr->first_frame                    = 0;
        pcs_ptr->parent_pcs_ptr->first_frame_in_temporal_layer = 1;
    } else
        pcs_ptr->parent_pcs_ptr->first_frame_in_temporal_layer = 0;
    if (pcs_ptr->slice_type != I_SLICE) {
        if (rate_control_layer_ptr->first_non_intra_frame == 1) {
            rate_control_layer_ptr->first_non_intra_frame                    = 0;
            pcs_ptr->parent_pcs_ptr->first_non_intra_frame_in_temporal_layer = 1;
        } else
            pcs_ptr->parent_pcs_ptr->first_non_intra_frame_in_temporal_layer = 0;
    } else
        pcs_ptr->parent_pcs_ptr->first_non_intra_frame_in_temporal_layer = 0;

    pcs_ptr->parent_pcs_ptr->target_bits_rc = 0;

    // ***Rate Control***
    area_in_sbs   = 0;
    area_in_pixel = 0;

    for (sb_index = 0; sb_index < pcs_ptr->sb_total_count; ++sb_index) {
        sb_params_ptr = &pcs_ptr->parent_pcs_ptr->sb_params_array[sb_index];

        if (sb_params_ptr->is_complete_sb) {
            // add the area of one SB (64x64=4096) to the area of the tile
            area_in_pixel += 4096;
            area_in_sbs++;
        } else {
            // add the area of the SB to the area of the tile
            area_in_pixel += sb_params_ptr->width * sb_params_ptr->height;
        }
    }
    rate_control_layer_ptr->area_in_pixel = area_in_pixel;

    if (pcs_ptr->parent_pcs_ptr->first_frame_in_temporal_layer ||
        (pcs_ptr->picture_number == rate_control_param_ptr->first_poc)) {
        if (scs_ptr->static_config.enable_qp_scaling_flag &&
            (pcs_ptr->picture_number != rate_control_param_ptr->first_poc)) {
            pcs_ptr->picture_qp = (uint8_t)CLIP3(
                (int32_t)scs_ptr->static_config.min_qp_allowed,
                (int32_t)scs_ptr->static_config.max_qp_allowed,
                (int32_t)(
                    rate_control_param_ptr->intra_frames_qp +
                    context_ptr->qp_scaling_map[pcs_ptr->temporal_layer_index]
                                               [rate_control_param_ptr->intra_frames_qp_bef_scal] -
                    context_ptr->qp_scaling_map_i_slice[rate_control_param_ptr
                                                            ->intra_frames_qp_bef_scal]));
        }

        if (pcs_ptr->picture_number == 0) {
            rate_control_param_ptr->intra_frames_qp          = scs_ptr->static_config.qp;
            rate_control_param_ptr->intra_frames_qp_bef_scal = (uint8_t)scs_ptr->static_config.qp;
        }

        if (pcs_ptr->picture_number == rate_control_param_ptr->first_poc) {
            uint32_t temporal_layer_idex;
            rate_control_param_ptr->previous_virtual_buffer_level =
                context_ptr->virtual_buffer_level_initial_value;
            rate_control_param_ptr->virtual_buffer_level =
                context_ptr->virtual_buffer_level_initial_value;
            rate_control_param_ptr->extra_ap_bit_ratio_i = 0;
            if (pcs_ptr->parent_pcs_ptr->end_of_sequence_region) {
                rate_control_param_ptr->last_poc = MAX(
                    rate_control_param_ptr->first_poc + pcs_ptr->parent_pcs_ptr->frames_in_sw - 1,
                    rate_control_param_ptr->first_poc);
                rate_control_param_ptr->last_gop = EB_TRUE;
            }

            for (temporal_layer_idex = 0; temporal_layer_idex < EB_MAX_TEMPORAL_LAYERS;
                 temporal_layer_idex++) {
                rate_control_layer_temp_ptr =
                    rate_control_param_ptr->rate_control_layer_array[temporal_layer_idex];
                rate_control_layer_reset(rate_control_layer_temp_ptr,
                                         pcs_ptr,
                                         context_ptr,
                                         picture_area_in_pixel,
                                         rate_control_param_ptr->was_used);
            }
        }

        pcs_ptr->parent_pcs_ptr->sad_me = 0;
        // Finding the QP of the Intra frame by using variance tables
        if (pcs_ptr->slice_type == I_SLICE) {
            uint32_t selected_ref_qp;

            if (scs_ptr->static_config.look_ahead_distance == 0)
                SVT_LOG("ERROR: LAD=0 is not supported\n");
            else {
                selected_ref_qp                        = pcs_ptr->parent_pcs_ptr->best_pred_qp;
                pcs_ptr->picture_qp                    = (uint8_t)selected_ref_qp;
                pcs_ptr->parent_pcs_ptr->calculated_qp = pcs_ptr->picture_qp;
            }

            pcs_ptr->picture_qp = (uint8_t)CLIP3(scs_ptr->static_config.min_qp_allowed,
                                                 scs_ptr->static_config.max_qp_allowed,
                                                 pcs_ptr->picture_qp);
        } else {
            // SB Loop
            for (sb_index = 0; sb_index < pcs_ptr->sb_total_count; ++sb_index) {
                sb_params_ptr = &pcs_ptr->parent_pcs_ptr->sb_params_array[sb_index];

                if (sb_params_ptr->is_complete_sb)
                    pcs_ptr->parent_pcs_ptr->sad_me +=
                        pcs_ptr->parent_pcs_ptr->rc_me_distortion[sb_index];
            }

            //  tilesad_Me is normalized based on the area because of the SBs at the tile boundries
            pcs_ptr->parent_pcs_ptr->sad_me =
                MAX((pcs_ptr->parent_pcs_ptr->sad_me * rate_control_layer_ptr->area_in_pixel /
                     (area_in_sbs << 12)),
                    1);

            // totalSquareMad has RC_PRECISION precision
            pcs_ptr->parent_pcs_ptr->sad_me <<= RC_PRECISION;
        }

        temp_qp = pcs_ptr->picture_qp;

        if (pcs_ptr->picture_number == rate_control_param_ptr->first_poc) {
            uint32_t temporal_layer_idex;
            for (temporal_layer_idex = 0; temporal_layer_idex < EB_MAX_TEMPORAL_LAYERS;
                 temporal_layer_idex++) {
                rate_control_layer_temp_ptr =
                    rate_control_param_ptr->rate_control_layer_array[temporal_layer_idex];
                rate_control_layer_reset_part2(context_ptr, rate_control_layer_temp_ptr, pcs_ptr);
            }
        }

        if (pcs_ptr->picture_number == 0) {
            context_ptr->base_layer_frames_avg_qp       = pcs_ptr->picture_qp + 1;
            context_ptr->base_layer_intra_frames_avg_qp = pcs_ptr->picture_qp;
        }
    } else {
        pcs_ptr->parent_pcs_ptr->sad_me = 0;

        HighLevelRateControlContext *high_level_rate_control_ptr =
            context_ptr->high_level_rate_control_ptr;
        EncodeContext *              encode_context_ptr = scs_ptr->encode_context_ptr;
        HlRateControlHistogramEntry *hl_rate_control_histogram_ptr_temp;
        // Queue variables
        uint32_t queue_entry_index_temp;
        uint32_t queue_entry_index_temp2;
        int64_t  queue_entry_index_head_temp;

        uint64_t min_la_bit_distance;
        uint32_t selected_ref_qp_table_index;
        uint32_t selected_ref_qp;
        uint32_t previous_selected_ref_qp = encode_context_ptr->previous_selected_ref_qp;

        uint32_t ref_qp_index;
        uint32_t ref_qp_index_temp;
        uint32_t ref_qp_table_index;

        uint32_t qp_search_min;
        uint32_t qp_search_max;
        int32_t  qp_step = 1;
        EbBool   best_qp_found;

        uint64_t bit_constraint_per_sw = 0;
        EbBool   end_of_sequence_flag  = EB_TRUE;

        // Loop over the QPs and find the best QP
        min_la_bit_distance = MAX_UNSIGNED_VALUE;
        qp_search_min       = (uint8_t)CLIP3(scs_ptr->static_config.min_qp_allowed,
                                       MAX_REF_QP_NUM, //scs_ptr->static_config.max_qp_allowed,
                                       (uint32_t)MAX((int32_t)scs_ptr->static_config.qp - 40, 0));

        qp_search_max = (uint8_t)CLIP3(
            scs_ptr->static_config.min_qp_allowed, MAX_REF_QP_NUM, scs_ptr->static_config.qp + 40);

        for (ref_qp_table_index = qp_search_min; ref_qp_table_index < qp_search_max;
             ref_qp_table_index++)
            high_level_rate_control_ptr->pred_bits_ref_qp_per_sw[ref_qp_table_index] = 0;
        // Finding the predicted bits for each frame in the sliding window at the reference Qp(s)
        ///queue_entry_index_head_temp = (int32_t)(pcs_ptr->picture_number - encode_context_ptr->hl_rate_control_historgram_queue[encode_context_ptr->hl_rate_control_historgram_queue_head_index]->picture_number);
        queue_entry_index_head_temp =
            (int32_t)(rate_control_param_ptr->first_poc -
                      encode_context_ptr
                          ->hl_rate_control_historgram_queue
                              [encode_context_ptr->hl_rate_control_historgram_queue_head_index]
                          ->picture_number);
        queue_entry_index_head_temp +=
            encode_context_ptr->hl_rate_control_historgram_queue_head_index;
        queue_entry_index_head_temp =
            (queue_entry_index_head_temp > HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH - 1)
                ? queue_entry_index_head_temp - HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH
                : (queue_entry_index_head_temp < 0)
                      ? queue_entry_index_head_temp +
                            HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH
                      : queue_entry_index_head_temp;

        if (pcs_ptr->parent_pcs_ptr->picture_number + pcs_ptr->parent_pcs_ptr->frames_in_sw >
            rate_control_param_ptr->first_poc + scs_ptr->static_config.intra_period_length + 1)
            bit_constraint_per_sw = high_level_rate_control_ptr->bit_constraint_per_sw;
        else
            bit_constraint_per_sw =
                high_level_rate_control_ptr->bit_constraint_per_sw *
                encode_context_ptr->hl_rate_control_historgram_queue[queue_entry_index_head_temp]
                    ->frames_in_sw /
                (scs_ptr->static_config.look_ahead_distance + 1);

        // Loop over proper QPs and find the Predicted bits for that QP. Find the QP with the closest total predicted rate to target bits for the sliding window.
        previous_selected_ref_qp =
            CLIP3(qp_search_min + 1, qp_search_max - 1, previous_selected_ref_qp);
        ref_qp_table_index          = previous_selected_ref_qp;
        ref_qp_index                = ref_qp_table_index;
        selected_ref_qp_table_index = ref_qp_table_index;
        selected_ref_qp             = selected_ref_qp_table_index;
        best_qp_found               = EB_FALSE;
        while (ref_qp_table_index >= qp_search_min && ref_qp_table_index <= qp_search_max &&
               !best_qp_found) {
            ref_qp_index =
                CLIP3(scs_ptr->static_config.min_qp_allowed, MAX_REF_QP_NUM, ref_qp_table_index);
            high_level_rate_control_ptr->pred_bits_ref_qp_per_sw[ref_qp_index] = 0;

            queue_entry_index_temp = (uint32_t)queue_entry_index_head_temp;
            // This is set to false, so the last frame would go inside the loop
            end_of_sequence_flag = EB_FALSE;

            while (
                !end_of_sequence_flag &&
                //queue_entry_index_temp <= queue_entry_index_head_temp + scs_ptr->static_config.look_ahead_distance) {
                queue_entry_index_temp <=
                    queue_entry_index_head_temp +
                        encode_context_ptr
                            ->hl_rate_control_historgram_queue[queue_entry_index_head_temp]
                            ->frames_in_sw -
                        1) {
                queue_entry_index_temp2 =
                    (queue_entry_index_temp > HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH - 1)
                        ? queue_entry_index_temp - HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH
                        : queue_entry_index_temp;
                hl_rate_control_histogram_ptr_temp =
                    (encode_context_ptr->hl_rate_control_historgram_queue[queue_entry_index_temp2]);

                if (hl_rate_control_histogram_ptr_temp->slice_type == I_SLICE)
                    ref_qp_index_temp = context_ptr->qp_scaling_map_i_slice[ref_qp_index];
                else
                    ref_qp_index_temp =
                        context_ptr->qp_scaling_map[hl_rate_control_histogram_ptr_temp
                                                        ->temporal_layer_index][ref_qp_index];

                ref_qp_index_temp = (uint32_t)CLIP3(scs_ptr->static_config.min_qp_allowed,
                                                    scs_ptr->static_config.max_qp_allowed,
                                                    ref_qp_index_temp);

                hl_rate_control_histogram_ptr_temp->pred_bits_ref_qp[ref_qp_index_temp] = 0;

                if (ref_qp_table_index == previous_selected_ref_qp) {
                    eb_block_on_mutex(
                        scs_ptr->encode_context_ptr->hl_rate_control_historgram_queue_mutex);
                    hl_rate_control_histogram_ptr_temp->life_count--;
                    eb_release_mutex(
                        scs_ptr->encode_context_ptr->hl_rate_control_historgram_queue_mutex);
                }

                hl_rate_control_histogram_ptr_temp->pred_bits_ref_qp[ref_qp_index_temp] =
                    predict_bits(encode_context_ptr,
                                 hl_rate_control_histogram_ptr_temp,
                                 ref_qp_index_temp,
                                 area_in_pixel);

                high_level_rate_control_ptr->pred_bits_ref_qp_per_sw[ref_qp_index] +=
                    hl_rate_control_histogram_ptr_temp->pred_bits_ref_qp[ref_qp_index_temp];
                // Store the pred_bits_ref_qp for the first frame in the window to PCS
                if (queue_entry_index_head_temp == queue_entry_index_temp2)
                    pcs_ptr->parent_pcs_ptr->pred_bits_ref_qp[ref_qp_index_temp] =
                        hl_rate_control_histogram_ptr_temp->pred_bits_ref_qp[ref_qp_index_temp];

                end_of_sequence_flag = hl_rate_control_histogram_ptr_temp->end_of_sequence_flag;
                queue_entry_index_temp++;
            }

            if (min_la_bit_distance >=
                (uint64_t)ABS(
                    (int64_t)high_level_rate_control_ptr->pred_bits_ref_qp_per_sw[ref_qp_index] -
                    (int64_t)bit_constraint_per_sw)) {
                min_la_bit_distance = (uint64_t)ABS(
                    (int64_t)high_level_rate_control_ptr->pred_bits_ref_qp_per_sw[ref_qp_index] -
                    (int64_t)bit_constraint_per_sw);
                selected_ref_qp_table_index = ref_qp_table_index;
                selected_ref_qp             = ref_qp_index;
            } else
                best_qp_found = EB_TRUE;
            if (ref_qp_table_index == previous_selected_ref_qp) {
                if (high_level_rate_control_ptr->pred_bits_ref_qp_per_sw[ref_qp_index] >
                    bit_constraint_per_sw)
                    qp_step = +1;
                else
                    qp_step = -1;
            }
            ref_qp_table_index = (uint32_t)(ref_qp_table_index + qp_step);
        }

        int delta_qp = 0;
        if (ref_qp_index == scs_ptr->static_config.max_qp_allowed &&
            high_level_rate_control_ptr->pred_bits_ref_qp_per_sw[ref_qp_index] >
                bit_constraint_per_sw) {
            delta_qp =
                (int)((high_level_rate_control_ptr->pred_bits_ref_qp_per_sw[ref_qp_index] -
                       bit_constraint_per_sw) *
                      100 /
                      (high_level_rate_control_ptr->pred_bits_ref_qp_per_sw[ref_qp_index - 1] -
                       high_level_rate_control_ptr->pred_bits_ref_qp_per_sw[ref_qp_index]));
            delta_qp = (delta_qp + 50) / 100;
        }

        if (pcs_ptr->slice_type == I_SLICE)
            pcs_ptr->parent_pcs_ptr->best_pred_qp =
                (uint8_t)context_ptr->qp_scaling_map_i_slice[selected_ref_qp];
        else
            pcs_ptr->parent_pcs_ptr->best_pred_qp =
                (uint8_t)
                    context_ptr->qp_scaling_map[pcs_ptr->temporal_layer_index][selected_ref_qp];

        pcs_ptr->parent_pcs_ptr->best_pred_qp =
            (uint8_t)CLIP3(scs_ptr->static_config.min_qp_allowed,
                           scs_ptr->static_config.max_qp_allowed,
                           (uint8_t)((int)pcs_ptr->parent_pcs_ptr->best_pred_qp + delta_qp));

        // if the pixture is an I slice, for now we set the QP as the QP of the previous frame
        if (pcs_ptr->slice_type == I_SLICE) {
            uint32_t selected_ref_qp;

            if (scs_ptr->static_config.look_ahead_distance == 0)
                SVT_LOG("ERROR: LAD=0 is not supported\n");
            else {
                selected_ref_qp                        = pcs_ptr->parent_pcs_ptr->best_pred_qp;
                pcs_ptr->picture_qp                    = (uint8_t)selected_ref_qp;
                pcs_ptr->parent_pcs_ptr->calculated_qp = pcs_ptr->picture_qp;
            }

            pcs_ptr->picture_qp = (uint8_t)CLIP3(scs_ptr->static_config.min_qp_allowed,
                                                 scs_ptr->static_config.max_qp_allowed,
                                                 pcs_ptr->picture_qp);

            temp_qp = pcs_ptr->picture_qp;
        }

        else { // Not an I slice
            // combining the target rate from initial RC and frame level RC
            if (scs_ptr->static_config.look_ahead_distance != 0) {
                pcs_ptr->parent_pcs_ptr->target_bits_rc = rate_control_layer_ptr->bit_constraint;
                rate_control_layer_ptr->ec_bit_constraint =
                    (rate_control_layer_ptr->alpha *
                         pcs_ptr->parent_pcs_ptr->target_bits_best_pred_qp +
                     ((1 << RC_PRECISION) - rate_control_layer_ptr->alpha) *
                         pcs_ptr->parent_pcs_ptr->target_bits_rc +
                     RC_PRECISION_OFFSET) >>
                    RC_PRECISION;

                rate_control_layer_ptr->ec_bit_constraint =
                    (uint64_t)MAX((int64_t)rate_control_layer_ptr->ec_bit_constraint -
                                      (int64_t)rate_control_layer_ptr->dif_total_and_ec_bits,
                                  1);

                pcs_ptr->parent_pcs_ptr->target_bits_rc = rate_control_layer_ptr->ec_bit_constraint;
            }

            // SB Loop
            for (sb_index = 0; sb_index < pcs_ptr->sb_total_count; ++sb_index) {
                sb_params_ptr = &pcs_ptr->parent_pcs_ptr->sb_params_array[sb_index];

                if (sb_params_ptr->is_complete_sb)
                    pcs_ptr->parent_pcs_ptr->sad_me +=
                        pcs_ptr->parent_pcs_ptr->rc_me_distortion[sb_index];
            }

            //  tilesad_Me is normalized based on the area because of the SBs at the tile boundries
            pcs_ptr->parent_pcs_ptr->sad_me =
                MAX((pcs_ptr->parent_pcs_ptr->sad_me * rate_control_layer_ptr->area_in_pixel /
                     (area_in_sbs << 12)),
                    1);
            pcs_ptr->parent_pcs_ptr->sad_me <<= RC_PRECISION;
            if (rate_control_layer_ptr->area_in_pixel > 0)
                rate_control_layer_ptr->total_mad = MAX(
                    (pcs_ptr->parent_pcs_ptr->sad_me / rate_control_layer_ptr->area_in_pixel), 1);
            if (!rate_control_layer_ptr->feedback_arrived)
                rate_control_layer_ptr->previous_frame_distortion_me =
                    pcs_ptr->parent_pcs_ptr->sad_me;
            {
                uint64_t qp_calc_temp1, qp_calc_temp2, qp_calc_temp3;

                qp_calc_temp1 = pcs_ptr->parent_pcs_ptr->sad_me * rate_control_layer_ptr->total_mad;
                qp_calc_temp2 = MAX(
                    (int64_t)(rate_control_layer_ptr->ec_bit_constraint << (2 * RC_PRECISION)) -
                        (int64_t)rate_control_layer_ptr->c_coeff *
                            (int64_t)rate_control_layer_ptr->area_in_pixel,
                    (int64_t)(rate_control_layer_ptr->ec_bit_constraint << (2 * RC_PRECISION - 2)));

                // This is a more complex but with higher precision implementation
                if (qp_calc_temp1 > qp_calc_temp2)
                    qp_calc_temp3 = (uint64_t)((qp_calc_temp1 / qp_calc_temp2) *
                                               rate_control_layer_ptr->k_coeff);
                else
                    qp_calc_temp3 =
                        (uint64_t)(qp_calc_temp1 * rate_control_layer_ptr->k_coeff / qp_calc_temp2);
                temp_qp = (uint64_t)(log2f_high_precision(
                    MAX(((qp_calc_temp3 + RC_PRECISION_OFFSET) >> RC_PRECISION) *
                            ((qp_calc_temp3 + RC_PRECISION_OFFSET) >> RC_PRECISION) *
                            ((qp_calc_temp3 + RC_PRECISION_OFFSET) >> RC_PRECISION),
                        1),
                    RC_PRECISION));

                rate_control_layer_ptr->calculated_frame_qp = (uint8_t)(
                    CLIP3(1, 63, (uint32_t)(temp_qp + RC_PRECISION_OFFSET) >> RC_PRECISION));
                pcs_ptr->parent_pcs_ptr->calculated_qp = (uint8_t)(
                    CLIP3(1, 63, (uint32_t)(temp_qp + RC_PRECISION_OFFSET) >> RC_PRECISION));
            }

            temp_qp += rate_control_layer_ptr->delta_qp_fraction;
            pcs_ptr->picture_qp = (uint8_t)((temp_qp + RC_PRECISION_OFFSET) >> RC_PRECISION);
            // Use the QP of HLRC instead of calculated one in FLRC
            if (pcs_ptr->parent_pcs_ptr->hierarchical_levels > 1) {
                pcs_ptr->picture_qp                    = pcs_ptr->parent_pcs_ptr->best_pred_qp;
                pcs_ptr->parent_pcs_ptr->calculated_qp = pcs_ptr->parent_pcs_ptr->best_pred_qp;
            }
        }
        if (pcs_ptr->parent_pcs_ptr->first_non_intra_frame_in_temporal_layer &&
            pcs_ptr->temporal_layer_index == 0 && pcs_ptr->slice_type != I_SLICE)
            pcs_ptr->picture_qp = (uint8_t)(
                rate_control_param_ptr->intra_frames_qp +
                context_ptr->qp_scaling_map[pcs_ptr->temporal_layer_index]
                                           [rate_control_param_ptr->intra_frames_qp_bef_scal] -
                context_ptr
                    ->qp_scaling_map_i_slice[rate_control_param_ptr->intra_frames_qp_bef_scal]);
        if (!rate_control_layer_ptr->feedback_arrived && pcs_ptr->slice_type != I_SLICE) {
            pcs_ptr->picture_qp = (uint8_t)CLIP3(
                (int32_t)scs_ptr->static_config.min_qp_allowed,
                (int32_t)scs_ptr->static_config.max_qp_allowed,
                (int32_t)(
                    rate_control_param_ptr->intra_frames_qp +
                    context_ptr->qp_scaling_map[pcs_ptr->temporal_layer_index]
                                               [rate_control_param_ptr->intra_frames_qp_bef_scal] -
                    context_ptr->qp_scaling_map_i_slice[rate_control_param_ptr
                                                            ->intra_frames_qp_bef_scal]));
        }

        if (pcs_ptr->parent_pcs_ptr->end_of_sequence_region) {
            if (rate_control_param_ptr->virtual_buffer_level > context_ptr->vb_fill_threshold2 << 2)
                pcs_ptr->picture_qp = pcs_ptr->picture_qp + (uint8_t)THRESHOLD2QPINCREASE + 4;
            else if (rate_control_param_ptr->virtual_buffer_level > context_ptr->vb_fill_threshold2
                                                                        << 1)
                pcs_ptr->picture_qp = pcs_ptr->picture_qp + (uint8_t)THRESHOLD2QPINCREASE + 3;
            else if (rate_control_param_ptr->virtual_buffer_level > context_ptr->vb_fill_threshold2)
                pcs_ptr->picture_qp = pcs_ptr->picture_qp + (uint8_t)THRESHOLD2QPINCREASE + 2;
            else if (rate_control_param_ptr->virtual_buffer_level >
                         context_ptr->vb_fill_threshold1 &&
                     rate_control_param_ptr->virtual_buffer_level <
                         context_ptr->vb_fill_threshold2) {
                pcs_ptr->picture_qp = pcs_ptr->picture_qp + (uint8_t)THRESHOLD1QPINCREASE + 2;
            }
        } else {
            //if (rate_control_param_ptr->virtual_buffer_level > context_ptr->vb_fill_threshold2 << 2){
            if (rate_control_param_ptr->virtual_buffer_level >
                context_ptr->vb_fill_threshold2 +
                    (int64_t)(context_ptr->virtual_buffer_size * 2 / 3))
                pcs_ptr->picture_qp = pcs_ptr->picture_qp + (uint8_t)THRESHOLD2QPINCREASE + 2;
            //else if (rate_control_param_ptr->virtual_buffer_level > context_ptr->vb_fill_threshold2 << 1){
            else if (rate_control_param_ptr->virtual_buffer_level >
                     context_ptr->vb_fill_threshold2 +
                         (int64_t)(context_ptr->virtual_buffer_size / 3))
                pcs_ptr->picture_qp = pcs_ptr->picture_qp + (uint8_t)THRESHOLD2QPINCREASE + 1;
            else if (rate_control_param_ptr->virtual_buffer_level > context_ptr->vb_fill_threshold2)
                pcs_ptr->picture_qp = pcs_ptr->picture_qp + (uint8_t)THRESHOLD2QPINCREASE + 1;
            else if (rate_control_param_ptr->virtual_buffer_level >
                         context_ptr->vb_fill_threshold1 &&
                     rate_control_param_ptr->virtual_buffer_level <
                         context_ptr->vb_fill_threshold2) {
                pcs_ptr->picture_qp = pcs_ptr->picture_qp + (uint8_t)THRESHOLD1QPINCREASE;
            }
        }
        if (pcs_ptr->parent_pcs_ptr->end_of_sequence_region) {
            if (rate_control_param_ptr->virtual_buffer_level <
                -(context_ptr->vb_fill_threshold2 << 2))
                pcs_ptr->picture_qp = (uint8_t)MAX(
                    (int32_t)pcs_ptr->picture_qp - (int32_t)THRESHOLD2QPINCREASE - 2, 0);
            else if (rate_control_param_ptr->virtual_buffer_level <
                     -(context_ptr->vb_fill_threshold2 << 1))
                pcs_ptr->picture_qp = (uint8_t)MAX(
                    (int32_t)pcs_ptr->picture_qp - (int32_t)THRESHOLD2QPINCREASE - 1, 0);
            else if (rate_control_param_ptr->virtual_buffer_level < 0)
                pcs_ptr->picture_qp =
                    (uint8_t)MAX((int32_t)pcs_ptr->picture_qp - (int32_t)THRESHOLD2QPINCREASE, 0);
        } else {
            if (rate_control_param_ptr->virtual_buffer_level <
                -(context_ptr->vb_fill_threshold2 << 2))
                pcs_ptr->picture_qp = (uint8_t)MAX(
                    (int32_t)pcs_ptr->picture_qp - (int32_t)THRESHOLD2QPINCREASE - 1, 0);
            else if (rate_control_param_ptr->virtual_buffer_level <
                     -context_ptr->vb_fill_threshold2)
                pcs_ptr->picture_qp =
                    (uint8_t)MAX((int32_t)pcs_ptr->picture_qp - (int32_t)THRESHOLD2QPINCREASE, 0);
        }

        uint32_t ref_qp;
        if ((int32_t)pcs_ptr->temporal_layer_index == 0 && pcs_ptr->slice_type != I_SLICE) {
            if (pcs_ptr->ref_slice_type_array[0][0] == I_SLICE) {
                /*    pcs_ptr->picture_qp = (uint8_t)CLIP3(
                        (uint32_t)pcs_ptr->ref_pic_qp_array[0],
                        (uint32_t)pcs_ptr->picture_qp,
                        pcs_ptr->picture_qp);*/
            } else {
                pcs_ptr->picture_qp =
                    (uint8_t)CLIP3((uint32_t)MAX((int32_t)pcs_ptr->ref_pic_qp_array[0][0] - 1, 0),
                                   (uint32_t)pcs_ptr->ref_pic_qp_array[0][0] + 3,
                                   pcs_ptr->picture_qp);
            }
        } else {
            ref_qp = 0;
            if (pcs_ptr->ref_slice_type_array[0][0] != I_SLICE)
                ref_qp = MAX(ref_qp, pcs_ptr->ref_pic_qp_array[0][0]);
            if ((pcs_ptr->slice_type == B_SLICE) &&
                (pcs_ptr->ref_slice_type_array[1][0] != I_SLICE))
                ref_qp = MAX(ref_qp, pcs_ptr->ref_pic_qp_array[1][0]);
            if (ref_qp > 0) {
                pcs_ptr->picture_qp =
                    (uint8_t)CLIP3((uint32_t)ref_qp - 1, pcs_ptr->picture_qp, pcs_ptr->picture_qp);
            }
        }
        // limiting the QP between min Qp allowed and max Qp allowed
        pcs_ptr->picture_qp = (uint8_t)CLIP3(scs_ptr->static_config.min_qp_allowed,
                                             scs_ptr->static_config.max_qp_allowed,
                                             pcs_ptr->picture_qp);

        rate_control_layer_ptr->delta_qp_fraction =
            CLIP3(-RC_PRECISION_OFFSET,
                  RC_PRECISION_OFFSET,
                  -((int64_t)temp_qp - (int64_t)(pcs_ptr->picture_qp << RC_PRECISION)));

        if (pcs_ptr->parent_pcs_ptr->sad_me ==
                rate_control_layer_ptr->previous_frame_distortion_me &&
            (rate_control_layer_ptr->previous_frame_distortion_me != 0))
            rate_control_layer_ptr->same_distortion_count++;
        else
            rate_control_layer_ptr->same_distortion_count = 0;
    }

    rate_control_layer_ptr->previous_c_coeff = rate_control_layer_ptr->c_coeff;
    rate_control_layer_ptr->previous_k_coeff = rate_control_layer_ptr->k_coeff;
    rate_control_layer_ptr->previous_calculated_frame_qp =
        rate_control_layer_ptr->calculated_frame_qp;
}

void frame_level_rc_feedback_picture_cvbr(PictureParentControlSet *parentpicture_control_set_ptr,
                                          SequenceControlSet *     scs_ptr,
                                          RateControlContext *     context_ptr) {
    RateControlLayerContext *        rate_control_layer_temp_ptr;
    RateControlIntervalParamContext *rate_control_param_ptr;
    RateControlLayerContext *        rate_control_layer_ptr;
    // SB Loop variables
    uint32_t slice_num;
    uint64_t previous_frame_bit_actual;

    if (scs_ptr->intra_period_length == -1)
        rate_control_param_ptr = context_ptr->rate_control_param_queue[0];
    else {
        uint32_t interval_index_temp = 0;
        while ((!(parentpicture_control_set_ptr->picture_number >=
                      context_ptr->rate_control_param_queue[interval_index_temp]->first_poc &&
                  parentpicture_control_set_ptr->picture_number <=
                      context_ptr->rate_control_param_queue[interval_index_temp]->last_poc)) &&
               (interval_index_temp < PARALLEL_GOP_MAX_NUMBER)) {
            interval_index_temp++;
        }
        CHECK_REPORT_ERROR(interval_index_temp != PARALLEL_GOP_MAX_NUMBER,
                           scs_ptr->encode_context_ptr->app_callback_ptr,
                           EB_ENC_RC_ERROR2);
        rate_control_param_ptr = context_ptr->rate_control_param_queue[interval_index_temp];
    }

    rate_control_layer_ptr =
        rate_control_param_ptr
            ->rate_control_layer_array[parentpicture_control_set_ptr->temporal_layer_index];

    rate_control_layer_ptr->max_qp = 0;

    rate_control_layer_ptr->feedback_arrived = EB_TRUE;
    rate_control_layer_ptr->max_qp =
        MAX(rate_control_layer_ptr->max_qp, parentpicture_control_set_ptr->picture_qp);

    rate_control_layer_ptr->previous_frame_qp = parentpicture_control_set_ptr->picture_qp;
    rate_control_layer_ptr->previous_frame_bit_actual =
        parentpicture_control_set_ptr->total_num_bits;
    if (parentpicture_control_set_ptr->quantized_coeff_num_bits == 0)
        parentpicture_control_set_ptr->quantized_coeff_num_bits = 1;
    rate_control_layer_ptr->previous_framequantized_coeff_bit_actual =
        parentpicture_control_set_ptr->quantized_coeff_num_bits;

    // Setting Critical states for adjusting the averaging weights on C and K
    if ((parentpicture_control_set_ptr->sad_me >
         (3 * rate_control_layer_ptr->previous_frame_distortion_me) >> 1) &&
        (rate_control_layer_ptr->previous_frame_distortion_me != 0)) {
        rate_control_layer_ptr->critical_states = 3;
    } else if (rate_control_layer_ptr->critical_states)
        rate_control_layer_ptr->critical_states--;
    else
        rate_control_layer_ptr->critical_states = 0;
    if (parentpicture_control_set_ptr->slice_type != I_SLICE) {
        // Updating c_coeff
        rate_control_layer_ptr->c_coeff =
            (((int64_t)rate_control_layer_ptr->previous_frame_bit_actual -
              (int64_t)rate_control_layer_ptr->previous_framequantized_coeff_bit_actual)
             << (2 * RC_PRECISION)) /
            rate_control_layer_ptr->area_in_pixel;
        rate_control_layer_ptr->c_coeff = MAX(rate_control_layer_ptr->c_coeff, 1);

        // Updating k_coeff
        if ((parentpicture_control_set_ptr->sad_me + RC_PRECISION_OFFSET) >> RC_PRECISION > 5) {
            {
                uint64_t test1, test2, test3;
                test1 = rate_control_layer_ptr->previous_framequantized_coeff_bit_actual *
                        (two_to_power_qp_over_three[parentpicture_control_set_ptr->picture_qp]);
                test2 = MAX(
                    parentpicture_control_set_ptr->sad_me / rate_control_layer_ptr->area_in_pixel,
                    1);
                test3 = test1 * 65536 / test2 * 65536 / parentpicture_control_set_ptr->sad_me;

                rate_control_layer_ptr->k_coeff = test3;
            }
        }

        if (rate_control_layer_ptr->critical_states) {
            rate_control_layer_ptr->k_coeff = (8 * rate_control_layer_ptr->k_coeff +
                                               8 * rate_control_layer_ptr->previous_k_coeff + 8) >>
                                              4;
            rate_control_layer_ptr->c_coeff = (8 * rate_control_layer_ptr->c_coeff +
                                               8 * rate_control_layer_ptr->previous_c_coeff + 8) >>
                                              4;
        } else {
            rate_control_layer_ptr->k_coeff =
                (rate_control_layer_ptr->coeff_averaging_weight1 * rate_control_layer_ptr->k_coeff +
                 rate_control_layer_ptr->coeff_averaging_weight2 *
                     rate_control_layer_ptr->previous_k_coeff +
                 8) >>
                4;
            rate_control_layer_ptr->c_coeff =
                (rate_control_layer_ptr->coeff_averaging_weight1 * rate_control_layer_ptr->c_coeff +
                 rate_control_layer_ptr->coeff_averaging_weight2 *
                     rate_control_layer_ptr->previous_c_coeff +
                 8) >>
                4;
        }
        rate_control_layer_ptr->k_coeff =
            MIN(rate_control_layer_ptr->k_coeff, rate_control_layer_ptr->previous_k_coeff * 4);
        rate_control_layer_ptr->c_coeff =
            MIN(rate_control_layer_ptr->c_coeff, rate_control_layer_ptr->previous_c_coeff * 4);
        if (parentpicture_control_set_ptr->slice_type != I_SLICE)
            rate_control_layer_ptr->previous_frame_distortion_me =
                parentpicture_control_set_ptr->sad_me;
        else
            rate_control_layer_ptr->previous_frame_distortion_me = 0;
    }

    if (scs_ptr->static_config.look_ahead_distance != 0) {
        if (parentpicture_control_set_ptr->slice_type == I_SLICE) {
            if (parentpicture_control_set_ptr->total_num_bits <
                parentpicture_control_set_ptr->target_bits_best_pred_qp << 1)
                context_ptr->base_layer_intra_frames_avg_qp =
                    (3 * context_ptr->base_layer_intra_frames_avg_qp +
                     parentpicture_control_set_ptr->picture_qp + 2) >>
                    2;
            else if (parentpicture_control_set_ptr->total_num_bits >
                     parentpicture_control_set_ptr->target_bits_best_pred_qp << 2)
                context_ptr->base_layer_intra_frames_avg_qp =
                    (3 * context_ptr->base_layer_intra_frames_avg_qp +
                     parentpicture_control_set_ptr->picture_qp + 4 + 2) >>
                    2;
            else if (parentpicture_control_set_ptr->total_num_bits >
                     parentpicture_control_set_ptr->target_bits_best_pred_qp << 1)
                context_ptr->base_layer_intra_frames_avg_qp =
                    (3 * context_ptr->base_layer_intra_frames_avg_qp +
                     parentpicture_control_set_ptr->picture_qp + 2 + 2) >>
                    2;
        }
    }

    {
        uint64_t previous_frame_ec_bits                   = 0;
        EbBool   picture_min_qp_allowed                   = EB_TRUE;
        rate_control_layer_ptr->previous_frame_average_qp = 0;
        rate_control_layer_ptr->previous_frame_average_qp +=
            rate_control_layer_ptr->previous_frame_qp;
        previous_frame_ec_bits += rate_control_layer_ptr->previous_frame_bit_actual;
        if (rate_control_layer_ptr->same_distortion_count == 0 ||
            parentpicture_control_set_ptr->picture_qp != scs_ptr->static_config.min_qp_allowed) {
            picture_min_qp_allowed = EB_FALSE;
        }
        if (picture_min_qp_allowed)
            rate_control_layer_ptr->frame_same_distortion_min_qp_count++;
        else
            rate_control_layer_ptr->frame_same_distortion_min_qp_count = 0;

        rate_control_layer_ptr->previous_ec_bits = previous_frame_ec_bits;
        previous_frame_bit_actual                = parentpicture_control_set_ptr->total_num_bits;
        if (parentpicture_control_set_ptr->first_frame_in_temporal_layer)
            rate_control_layer_ptr->dif_total_and_ec_bits =
                (previous_frame_bit_actual - previous_frame_ec_bits);
        else
            rate_control_layer_ptr->dif_total_and_ec_bits =
                ((previous_frame_bit_actual - previous_frame_ec_bits) +
                 rate_control_layer_ptr->dif_total_and_ec_bits) >>
                1;
        // update bitrate of different layers in the interval based on the rate of the I frame
        if (parentpicture_control_set_ptr->picture_number == rate_control_param_ptr->first_poc &&
            (parentpicture_control_set_ptr->slice_type == I_SLICE) &&
            scs_ptr->static_config.intra_period_length != -1) {
            uint32_t temporal_layer_idex;
            uint64_t target_bit_rate;
            uint64_t channel_bit_rate;
            uint64_t sum_bits_per_sw = 0;
#if ADAPTIVE_PERCENTAGE
            if (scs_ptr->static_config.look_ahead_distance != 0) {
                if (parentpicture_control_set_ptr->tables_updated &&
                    parentpicture_control_set_ptr->percentage_updated) {
                    parentpicture_control_set_ptr->bits_per_sw_per_layer[0] = (uint64_t)MAX(
                        (int64_t)parentpicture_control_set_ptr->bits_per_sw_per_layer[0] +
                            (int64_t)parentpicture_control_set_ptr->total_num_bits -
                            (int64_t)parentpicture_control_set_ptr->target_bits_best_pred_qp,
                        1);
                }
            }
#endif

            if (scs_ptr->static_config.look_ahead_distance != 0 &&
                scs_ptr->intra_period_length != -1) {
                for (temporal_layer_idex = 0; temporal_layer_idex < EB_MAX_TEMPORAL_LAYERS;
                     temporal_layer_idex++)
                    sum_bits_per_sw +=
                        parentpicture_control_set_ptr->bits_per_sw_per_layer[temporal_layer_idex];
            }

            for (temporal_layer_idex = 0; temporal_layer_idex < EB_MAX_TEMPORAL_LAYERS;
                 temporal_layer_idex++) {
                rate_control_layer_temp_ptr =
                    rate_control_param_ptr->rate_control_layer_array[temporal_layer_idex];

                target_bit_rate =
                    (uint64_t)((int64_t)parentpicture_control_set_ptr->target_bit_rate -
                               MIN((int64_t)parentpicture_control_set_ptr->target_bit_rate * 3 / 4,
                                   (int64_t)(parentpicture_control_set_ptr->total_num_bits *
                                             context_ptr->frame_rate /
                                             (scs_ptr->static_config.intra_period_length + 1)) >>
                                       RC_PRECISION)) *
                    rate_percentage_layer_array[scs_ptr->static_config.hierarchical_levels]
                                               [temporal_layer_idex] /
                    100;

#if ADAPTIVE_PERCENTAGE
                if (scs_ptr->static_config.look_ahead_distance != 0 &&
                    scs_ptr->intra_period_length != -1) {
                    target_bit_rate =
                        (uint64_t)(
                            (int64_t)parentpicture_control_set_ptr->target_bit_rate -
                            MIN((int64_t)parentpicture_control_set_ptr->target_bit_rate * 3 / 4,
                                (int64_t)(parentpicture_control_set_ptr->total_num_bits *
                                          context_ptr->frame_rate /
                                          (scs_ptr->static_config.intra_period_length + 1)) >>
                                    RC_PRECISION)) *
                        parentpicture_control_set_ptr->bits_per_sw_per_layer[temporal_layer_idex] /
                        sum_bits_per_sw;
                }
#endif
                // update this based on temporal layers
                if (temporal_layer_idex == 0)
                    channel_bit_rate =
                        (((target_bit_rate << (2 * RC_PRECISION)) /
                          MAX(1,
                              rate_control_layer_temp_ptr->frame_rate -
                                  (1 * context_ptr->frame_rate /
                                   (scs_ptr->static_config.intra_period_length + 1)))) +
                         RC_PRECISION_OFFSET) >>
                        RC_PRECISION;
                else
                    channel_bit_rate = (((target_bit_rate << (2 * RC_PRECISION)) /
                                         rate_control_layer_temp_ptr->frame_rate) +
                                        RC_PRECISION_OFFSET) >>
                                       RC_PRECISION;
                channel_bit_rate = (uint64_t)MAX((int64_t)1, (int64_t)channel_bit_rate);
                rate_control_layer_temp_ptr->ec_bit_constraint = channel_bit_rate;

                slice_num = 1;
                rate_control_layer_temp_ptr->ec_bit_constraint -= SLICE_HEADER_BITS_NUM * slice_num;

                rate_control_layer_temp_ptr->previous_bit_constraint = channel_bit_rate;
                rate_control_layer_temp_ptr->bit_constraint          = channel_bit_rate;
                rate_control_layer_temp_ptr->channel_bit_rate        = channel_bit_rate;
            }
            if ((int64_t)parentpicture_control_set_ptr->target_bit_rate * 3 / 4 <
                (int64_t)(parentpicture_control_set_ptr->total_num_bits * context_ptr->frame_rate /
                          (scs_ptr->static_config.intra_period_length + 1)) >>
                RC_PRECISION) {
                rate_control_param_ptr->previous_virtual_buffer_level +=
                    (int64_t)(
                        (parentpicture_control_set_ptr->total_num_bits * context_ptr->frame_rate /
                         (scs_ptr->static_config.intra_period_length + 1)) >>
                        RC_PRECISION) -
                    (int64_t)parentpicture_control_set_ptr->target_bit_rate * 3 / 4;
                context_ptr->extra_bits_gen = 0;
            }
        }

        if (previous_frame_bit_actual) {
            uint64_t bit_changes_rate;
            // Updating virtual buffer level and it can be negative
            context_ptr->extra_bits_gen = 0;
            rate_control_param_ptr->virtual_buffer_level =
                (int64_t)rate_control_param_ptr->previous_virtual_buffer_level +
                (int64_t)previous_frame_bit_actual -
                (int64_t)context_ptr->high_level_rate_control_ptr->channel_bit_rate_per_frame;
            if (parentpicture_control_set_ptr->hierarchical_levels > 1 &&
                rate_control_layer_ptr->frame_same_distortion_min_qp_count > 10) {
                rate_control_layer_ptr->previous_bit_constraint =
                    (int64_t)rate_control_layer_ptr->channel_bit_rate;
                rate_control_param_ptr->virtual_buffer_level =
                    ((int64_t)context_ptr->virtual_buffer_size >> 1);
            }
            // Updating bit difference
            rate_control_layer_ptr->bit_diff =
                (int64_t)rate_control_param_ptr->virtual_buffer_level
                //- ((int64_t)context_ptr->virtual_buffer_size>>1);
                - ((int64_t)rate_control_layer_ptr->channel_bit_rate >> 1);

            // Limit the bit difference
            rate_control_layer_ptr->bit_diff =
                CLIP3(-(int64_t)(rate_control_layer_ptr->channel_bit_rate),
                      (int64_t)(rate_control_layer_ptr->channel_bit_rate >> 1),
                      rate_control_layer_ptr->bit_diff);
            bit_changes_rate = rate_control_layer_ptr->frame_rate;

            // Updating bit Constraint
            rate_control_layer_ptr->bit_constraint =
                MAX((int64_t)rate_control_layer_ptr->previous_bit_constraint -
                        ((rate_control_layer_ptr->bit_diff << RC_PRECISION) /
                         ((int64_t)bit_changes_rate)),
                    1);

            // Limiting the bit_constraint
            if (parentpicture_control_set_ptr->temporal_layer_index == 0) {
                rate_control_layer_ptr->bit_constraint =
                    CLIP3(rate_control_layer_ptr->channel_bit_rate >> 2,
                          rate_control_layer_ptr->channel_bit_rate * 200 / 100,
                          rate_control_layer_ptr->bit_constraint);
            } else {
                rate_control_layer_ptr->bit_constraint =
                    CLIP3(rate_control_layer_ptr->channel_bit_rate >> 1,
                          rate_control_layer_ptr->channel_bit_rate * 200 / 100,
                          rate_control_layer_ptr->bit_constraint);
            }
            rate_control_layer_ptr->ec_bit_constraint =
                (uint64_t)MAX((int64_t)rate_control_layer_ptr->bit_constraint -
                                  (int64_t)rate_control_layer_ptr->dif_total_and_ec_bits,
                              1);
            rate_control_param_ptr->previous_virtual_buffer_level =
                rate_control_param_ptr->virtual_buffer_level;
            rate_control_layer_ptr->previous_bit_constraint =
                rate_control_layer_ptr->bit_constraint;
        }

        rate_control_param_ptr->processed_frames_number++;
        rate_control_param_ptr->in_use = EB_TRUE;
        // check if all the frames in the interval have arrived
        if (rate_control_param_ptr->processed_frames_number ==
                (rate_control_param_ptr->last_poc - rate_control_param_ptr->first_poc + 1) &&
            scs_ptr->intra_period_length != -1) {
            uint32_t temporal_index;
            int64_t  extra_bits;
            rate_control_param_ptr->first_poc +=
                PARALLEL_GOP_MAX_NUMBER * (uint32_t)(scs_ptr->intra_period_length + 1);
            rate_control_param_ptr->last_poc +=
                PARALLEL_GOP_MAX_NUMBER * (uint32_t)(scs_ptr->intra_period_length + 1);
            rate_control_param_ptr->processed_frames_number      = 0;
            rate_control_param_ptr->extra_ap_bit_ratio_i         = 0;
            rate_control_param_ptr->in_use                       = EB_FALSE;
            rate_control_param_ptr->was_used                     = EB_TRUE;
            rate_control_param_ptr->last_gop                     = EB_FALSE;
            rate_control_param_ptr->first_pic_actual_qp_assigned = EB_FALSE;
            for (temporal_index = 0; temporal_index < EB_MAX_TEMPORAL_LAYERS; temporal_index++) {
                rate_control_param_ptr->rate_control_layer_array[temporal_index]->first_frame = 1;
                rate_control_param_ptr->rate_control_layer_array[temporal_index]
                    ->first_non_intra_frame = 1;
                rate_control_param_ptr->rate_control_layer_array[temporal_index]->feedback_arrived =
                    EB_FALSE;
            }
            extra_bits = ((int64_t)context_ptr->virtual_buffer_size >> 1) -
                         (int64_t)rate_control_param_ptr->virtual_buffer_level;

            rate_control_param_ptr->virtual_buffer_level = context_ptr->virtual_buffer_size >> 1;
            context_ptr->extra_bits += extra_bits;
        }
        context_ptr->extra_bits = 0;
    }

#if RC_PRINTS
    ///if (parentpicture_control_set_ptr->temporal_layer_index == 0)
    {
        SVT_LOG("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%.0f\t%.0f\t%.0f\t%.0f\t%d\t%d\n",
                (int)parentpicture_control_set_ptr->slice_type,
                (int)parentpicture_control_set_ptr->picture_number,
                (int)parentpicture_control_set_ptr->temporal_layer_index,
                (int)parentpicture_control_set_ptr->picture_qp,
                (int)parentpicture_control_set_ptr->calculated_qp,
                (int)parentpicture_control_set_ptr->best_pred_qp,
                (int)previous_frame_bit_actual,
                (int)parentpicture_control_set_ptr->target_bits_best_pred_qp,
                (int)parentpicture_control_set_ptr->target_bits_rc,
                (int)rate_control_layer_ptr->channel_bit_rate,
                (int)rate_control_layer_ptr->bit_constraint,
                (double)rate_control_layer_ptr->c_coeff,
                (double)rate_control_layer_ptr->k_coeff,
                (double)parentpicture_control_set_ptr->sad_me,
                (double)context_ptr->extra_bits_gen,
                (int)rate_control_param_ptr->virtual_buffer_level,
                (int)context_ptr->extra_bits);
    }
#endif
}

void high_level_rc_feed_back_picture(PictureParentControlSet *pcs_ptr,
                                     SequenceControlSet *     scs_ptr) {
    // Queue variables
    HlRateControlHistogramEntry *hl_rate_control_histogram_ptr_temp;
    uint32_t                     queue_entry_index_head_temp;

    //SVT_LOG("\nOut:%d Slidings: ",pcs_ptr->picture_number);
    if (scs_ptr->static_config.look_ahead_distance != 0) {
        // Update the coded rate in the histogram queue
        if (pcs_ptr->picture_number >=
            scs_ptr->encode_context_ptr
                ->hl_rate_control_historgram_queue
                    [scs_ptr->encode_context_ptr->hl_rate_control_historgram_queue_head_index]
                ->picture_number) {
            queue_entry_index_head_temp = (int32_t)(
                pcs_ptr->picture_number -
                scs_ptr->encode_context_ptr
                    ->hl_rate_control_historgram_queue
                        [scs_ptr->encode_context_ptr->hl_rate_control_historgram_queue_head_index]
                    ->picture_number);
            queue_entry_index_head_temp +=
                scs_ptr->encode_context_ptr->hl_rate_control_historgram_queue_head_index;
            queue_entry_index_head_temp =
                (queue_entry_index_head_temp >
                 HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH - 1)
                    ? queue_entry_index_head_temp -
                          HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH
                    : queue_entry_index_head_temp;

            hl_rate_control_histogram_ptr_temp =
                (scs_ptr->encode_context_ptr
                     ->hl_rate_control_historgram_queue[queue_entry_index_head_temp]);
            if (hl_rate_control_histogram_ptr_temp->picture_number == pcs_ptr->picture_number &&
                hl_rate_control_histogram_ptr_temp->passed_to_hlrc) {
                eb_block_on_mutex(
                    scs_ptr->encode_context_ptr->hl_rate_control_historgram_queue_mutex);
                hl_rate_control_histogram_ptr_temp->total_num_bits_coded = pcs_ptr->total_num_bits;
                hl_rate_control_histogram_ptr_temp->is_coded             = EB_TRUE;
                eb_release_mutex(
                    scs_ptr->encode_context_ptr->hl_rate_control_historgram_queue_mutex);
            }
        }
    }
}
// rate control QP refinement
void rate_control_refinement(PictureControlSet *pcs_ptr, SequenceControlSet *scs_ptr,
                             RateControlIntervalParamContext *rate_control_param_ptr,
                             RateControlIntervalParamContext *prev_gop_rate_control_param_ptr,
                             RateControlIntervalParamContext *next_gop_rate_control_param_ptr) {
    if (pcs_ptr->picture_number == rate_control_param_ptr->first_poc &&
        pcs_ptr->picture_number != 0 && !prev_gop_rate_control_param_ptr->scene_change_in_gop) {
        int16_t delta_ap_qp = (int16_t)prev_gop_rate_control_param_ptr->first_pic_actual_qp -
                              (int16_t)prev_gop_rate_control_param_ptr->first_pic_pred_qp;
        int64_t extra_ap_bit_ratio =
            (prev_gop_rate_control_param_ptr->first_pic_pred_bits != 0)
                ? (((int64_t)prev_gop_rate_control_param_ptr->first_pic_actual_bits -
                    (int64_t)prev_gop_rate_control_param_ptr->first_pic_pred_bits) *
                   100) /
                      ((int64_t)prev_gop_rate_control_param_ptr->first_pic_pred_bits)
                : 0;
        extra_ap_bit_ratio += (int64_t)delta_ap_qp * 15;
        if (extra_ap_bit_ratio > 200)
            pcs_ptr->picture_qp = pcs_ptr->picture_qp + 3;
        else if (extra_ap_bit_ratio > 100)
            pcs_ptr->picture_qp = pcs_ptr->picture_qp + 2;
        else if (extra_ap_bit_ratio > 50)
            pcs_ptr->picture_qp++;
    }

    if (pcs_ptr->picture_number == rate_control_param_ptr->first_poc &&
        pcs_ptr->picture_number != 0) {
        uint8_t qp_inc_allowed = 3;
        uint8_t qp_dec_allowed = 4;
        if (pcs_ptr->parent_pcs_ptr->intra_selected_org_qp + 10 <=
            prev_gop_rate_control_param_ptr->first_pic_actual_qp)
            qp_dec_allowed = (uint8_t)(prev_gop_rate_control_param_ptr->first_pic_actual_qp -
                                       pcs_ptr->parent_pcs_ptr->intra_selected_org_qp) >>
                             1;
        if (pcs_ptr->parent_pcs_ptr->intra_selected_org_qp >=
            prev_gop_rate_control_param_ptr->first_pic_actual_qp + 10) {
            qp_inc_allowed = (uint8_t)(pcs_ptr->parent_pcs_ptr->intra_selected_org_qp -
                                       prev_gop_rate_control_param_ptr->first_pic_actual_qp) *
                             2 / 3;
            if (prev_gop_rate_control_param_ptr->first_pic_actual_qp <= 15)
                qp_inc_allowed += 5;
            else if (prev_gop_rate_control_param_ptr->first_pic_actual_qp <= 20)
                qp_inc_allowed += 4;
            else if (prev_gop_rate_control_param_ptr->first_pic_actual_qp <= 25)
                qp_inc_allowed += 3;
        } else if (prev_gop_rate_control_param_ptr->scene_change_in_gop)
            qp_inc_allowed = 5;
        if (pcs_ptr->parent_pcs_ptr->end_of_sequence_region) {
            qp_inc_allowed += 2;
            qp_dec_allowed += 4;
        }
        pcs_ptr->picture_qp = (uint8_t)CLIP3(
            (uint32_t)MAX((int32_t)prev_gop_rate_control_param_ptr->first_pic_actual_qp -
                              (int32_t)qp_dec_allowed,
                          0),
            (uint32_t)prev_gop_rate_control_param_ptr->first_pic_actual_qp + qp_inc_allowed,
            pcs_ptr->picture_qp);
    }

    // Scene change
    if (pcs_ptr->slice_type == I_SLICE &&
        pcs_ptr->picture_number != rate_control_param_ptr->first_poc) {
        if (next_gop_rate_control_param_ptr->first_pic_actual_qp_assigned) {
            pcs_ptr->picture_qp = (uint8_t)CLIP3(
                (uint32_t)MAX(
                    (int32_t)next_gop_rate_control_param_ptr->first_pic_actual_qp - (int32_t)1, 0),
                (uint32_t)next_gop_rate_control_param_ptr->first_pic_actual_qp + 8,
                pcs_ptr->picture_qp);
        } else {
            if (rate_control_param_ptr->first_pic_actual_qp < 20) {
                pcs_ptr->picture_qp = (uint8_t)CLIP3(
                    (uint32_t)MAX((int32_t)rate_control_param_ptr->first_pic_actual_qp - (int32_t)4,
                                  0),
                    (uint32_t)rate_control_param_ptr->first_pic_actual_qp + 10,
                    pcs_ptr->picture_qp);
            } else {
                pcs_ptr->picture_qp = (uint8_t)CLIP3(
                    (uint32_t)MAX((int32_t)rate_control_param_ptr->first_pic_actual_qp - (int32_t)4,
                                  0),
                    (uint32_t)rate_control_param_ptr->first_pic_actual_qp + 8,
                    pcs_ptr->picture_qp);
            }
        }
    }

    if (scs_ptr->intra_period_length != -1 && pcs_ptr->parent_pcs_ptr->hierarchical_levels < 2 &&
        (int32_t)pcs_ptr->temporal_layer_index == 0 && pcs_ptr->slice_type != I_SLICE) {
        if (next_gop_rate_control_param_ptr->first_pic_actual_qp_assigned ||
            next_gop_rate_control_param_ptr->was_used) {
            pcs_ptr->picture_qp = (uint8_t)CLIP3(
                (uint32_t)MAX(
                    (int32_t)next_gop_rate_control_param_ptr->first_pic_actual_qp - (int32_t)4, 0),
                (uint32_t)pcs_ptr->picture_qp,
                pcs_ptr->picture_qp);
        } else {
            pcs_ptr->picture_qp = (uint8_t)CLIP3(
                (uint32_t)MAX((int32_t)rate_control_param_ptr->first_pic_actual_qp - (int32_t)4, 0),
                (uint32_t)pcs_ptr->picture_qp,
                pcs_ptr->picture_qp);
        }
    }
}
// initialize the rate control parameter at the beginning
void init_rc(RateControlContext *context_ptr, PictureControlSet *pcs_ptr,
             SequenceControlSet *scs_ptr) {
    context_ptr->high_level_rate_control_ptr->target_bit_rate =
        scs_ptr->static_config.target_bit_rate;
    context_ptr->high_level_rate_control_ptr->frame_rate                 = scs_ptr->frame_rate;
    context_ptr->high_level_rate_control_ptr->channel_bit_rate_per_frame = (uint64_t)MAX(
        (int64_t)1,
        (int64_t)((context_ptr->high_level_rate_control_ptr->target_bit_rate << RC_PRECISION) /
                  context_ptr->high_level_rate_control_ptr->frame_rate));

    context_ptr->high_level_rate_control_ptr->channel_bit_rate_per_sw =
        context_ptr->high_level_rate_control_ptr->channel_bit_rate_per_frame *
        (scs_ptr->static_config.look_ahead_distance + 1);
    context_ptr->high_level_rate_control_ptr->bit_constraint_per_sw =
        context_ptr->high_level_rate_control_ptr->channel_bit_rate_per_sw;
    context_ptr->high_level_rate_control_ptr->previous_updated_bit_constraint_per_sw =
        context_ptr->high_level_rate_control_ptr->channel_bit_rate_per_sw;

    int32_t  total_frame_in_interval = scs_ptr->intra_period_length;
    uint32_t gop_period              = (1 << pcs_ptr->parent_pcs_ptr->hierarchical_levels);
    context_ptr->frame_rate          = scs_ptr->frame_rate;
    while (total_frame_in_interval >= 0) {
        if (total_frame_in_interval % (gop_period) == 0)
            context_ptr->frames_in_interval[0]++;
        else if (total_frame_in_interval % (gop_period >> 1) == 0)
            context_ptr->frames_in_interval[1]++;
        else if (total_frame_in_interval % (gop_period >> 2) == 0)
            context_ptr->frames_in_interval[2]++;
        else if (total_frame_in_interval % (gop_period >> 3) == 0)
            context_ptr->frames_in_interval[3]++;
        else if (total_frame_in_interval % (gop_period >> 4) == 0)
            context_ptr->frames_in_interval[4]++;
        else if (total_frame_in_interval % (gop_period >> 5) == 0)
            context_ptr->frames_in_interval[5]++;
        total_frame_in_interval--;
    }
    if (scs_ptr->static_config.rate_control_mode == 1) { // VBR
        context_ptr->virtual_buffer_size =
            (((uint64_t)scs_ptr->static_config.target_bit_rate * 3) << RC_PRECISION) /
            (context_ptr->frame_rate);
        context_ptr->rate_average_periodin_frames =
            (uint64_t)scs_ptr->static_config.intra_period_length + 1;
        context_ptr->virtual_buffer_level_initial_value = context_ptr->virtual_buffer_size >> 1;
        context_ptr->virtual_buffer_level               = context_ptr->virtual_buffer_size >> 1;
        context_ptr->previous_virtual_buffer_level      = context_ptr->virtual_buffer_size >> 1;
        context_ptr->vb_fill_threshold1             = (context_ptr->virtual_buffer_size * 6) >> 3;
        context_ptr->vb_fill_threshold2             = (context_ptr->virtual_buffer_size << 3) >> 3;
        context_ptr->base_layer_frames_avg_qp       = scs_ptr->static_config.qp;
        context_ptr->base_layer_intra_frames_avg_qp = scs_ptr->static_config.qp;
    } else if (scs_ptr->static_config.rate_control_mode == 2) {
        if (scs_ptr->static_config.vbv_bufsize > 0)
            context_ptr->virtual_buffer_size =
                ((uint64_t)scs_ptr->static_config.vbv_bufsize); // vbv_buf_size);
        else
            context_ptr->virtual_buffer_size =
                ((uint64_t)scs_ptr->static_config.target_bit_rate); // vbv_buf_size);
        context_ptr->rate_average_periodin_frames =
            (uint64_t)scs_ptr->static_config.intra_period_length + 1;
        context_ptr->virtual_buffer_level_initial_value = context_ptr->virtual_buffer_size >> 1;
        context_ptr->virtual_buffer_level               = context_ptr->virtual_buffer_size >> 1;
        context_ptr->previous_virtual_buffer_level      = context_ptr->virtual_buffer_size >> 1;
        context_ptr->vb_fill_threshold1 = context_ptr->virtual_buffer_level_initial_value +
                                          (context_ptr->virtual_buffer_size / 4);
        context_ptr->vb_fill_threshold2 = context_ptr->virtual_buffer_level_initial_value +
                                          (context_ptr->virtual_buffer_size / 3);
        context_ptr->base_layer_frames_avg_qp       = scs_ptr->static_config.qp;
        context_ptr->base_layer_intra_frames_avg_qp = scs_ptr->static_config.qp;
    }

    for (uint32_t base_qp = 0; base_qp < MAX_REF_QP_NUM; base_qp++) {
        if (base_qp < 64) {
            context_ptr->qp_scaling_map_i_slice[base_qp] =
                qp_scaling_calc(scs_ptr, I_SLICE, 0, base_qp);
        } else
            context_ptr->qp_scaling_map_i_slice[base_qp] = (uint32_t)CLIP3(
                0, 63, (int)base_qp - (63 - (int)context_ptr->qp_scaling_map_i_slice[63]));
        for (uint32_t temporal_layer_index = 0;
             temporal_layer_index < scs_ptr->static_config.hierarchical_levels + 1;
             temporal_layer_index++) {
            if (base_qp < 64) {
                context_ptr->qp_scaling_map[temporal_layer_index][base_qp] =
                    qp_scaling_calc(scs_ptr, 0, temporal_layer_index, base_qp);
            } else
                context_ptr->qp_scaling_map[temporal_layer_index][base_qp] = (uint32_t)CLIP3(
                    0,
                    63,
                    (int)base_qp -
                        (63 - (int)context_ptr->qp_scaling_map[temporal_layer_index][63]));
        }
    }
}

#define MAX_Q_INDEX 255
#define MIN_Q_INDEX 0

extern int16_t eb_av1_ac_quant_q3(int32_t qindex, int32_t delta, AomBitDepth bit_depth);
// These functions use formulaic calculations to make playing with the
// quantizer tables easier. If necessary they can be replaced by lookup
// tables if and when things settle down in the experimental Bitstream

double eb_av1_convert_qindex_to_q(int32_t qindex, AomBitDepth bit_depth) {
    // Convert the index to a real Q value (scaled down to match old Q values)
    switch (bit_depth) {
    case AOM_BITS_8: return eb_av1_ac_quant_q3(qindex, 0, bit_depth) / 4.0;
    case AOM_BITS_10: return eb_av1_ac_quant_q3(qindex, 0, bit_depth) / 16.0;
    case AOM_BITS_12: return eb_av1_ac_quant_q3(qindex, 0, bit_depth) / 64.0;
    default: assert(0 && "bit_depth should be AOM_BITS_8, AOM_BITS_10 or AOM_BITS_12"); return -1.0;
    }
}
int32_t eb_av1_compute_qdelta(double qstart, double qtarget, AomBitDepth bit_depth) {
    int32_t start_index  = MAX_Q_INDEX;
    int32_t target_index = MAX_Q_INDEX;
    int32_t i;

    // Convert the average q value to an index.
    for (i = MIN_Q_INDEX; i < MAX_Q_INDEX; ++i) {
        start_index = i;
        if (eb_av1_convert_qindex_to_q(i, bit_depth) >= qstart) break;
    }

    // Convert the q target to an index
    for (i = MIN_Q_INDEX; i < MAX_Q_INDEX; ++i) {
        target_index = i;
        if (eb_av1_convert_qindex_to_q(i, bit_depth) >= qtarget) break;
    }

    return target_index - start_index;
}
// calculate the QP based on the QP scaling
uint32_t qp_scaling_calc(SequenceControlSet *scs_ptr, EB_SLICE slice_type,
                         uint32_t temporal_layer_index, uint32_t base_qp) {
    // AMIR to fix
    uint32_t scaled_qp = 0;
    int      base_qindex;

    const double delta_rate_new[2][6] = {{0.40, 0.7, 0.85, 1.0, 1.0, 1.0},
                                         {0.35, 0.6, 0.8, 0.9, 1.0, 1.0}};

    int          qindex = quantizer_to_qindex[base_qp];
    const double q =
        eb_av1_convert_qindex_to_q(qindex, (AomBitDepth)scs_ptr->static_config.encoder_bit_depth);
    int delta_qindex;

    if (slice_type == I_SLICE) {
        delta_qindex = eb_av1_compute_qdelta(
            q, q * 0.25, (AomBitDepth)scs_ptr->static_config.encoder_bit_depth);
    } else {
        delta_qindex = eb_av1_compute_qdelta(
            q,
            q * delta_rate_new[scs_ptr->static_config.hierarchical_levels == 4]
                              [temporal_layer_index], // RC does not support 5L
            //q* delta_rate_new[0][temporal_layer_index], // RC does not support 5L
            (AomBitDepth)scs_ptr->static_config.encoder_bit_depth);
    }

    base_qindex = MAX(qindex + delta_qindex, MIN_Q_INDEX);
    scaled_qp   = (uint32_t)(base_qindex) >> 2;

    return scaled_qp;
}
typedef struct {
    // Rate targetting variables
    int base_frame_target; // A baseline frame target before adjustment
        // for previous under or over shoot.
    int this_frame_target; // Actual frame target after rc adjustment.
    int projected_frame_size;
    int sb64_target_rate;
    int last_q[FRAME_TYPES]; // Separate values for Intra/Inter
    int last_boosted_qindex; // Last boosted GF/KF/ARF q
    int last_kf_qindex; // Q index of the last key frame coded.

    int gfu_boost;
    int kf_boost;

    // double rate_correction_factors[RATE_FACTOR_LEVELS];

    int frames_since_golden;
    int frames_till_gf_update_due;
    int min_gf_interval;
    int max_gf_interval;
    int static_scene_max_gf_interval;
    int baseline_gf_interval;
    int constrained_gf_group;
    int frames_to_key;
    int frames_since_key;
    int this_key_frame_forced;
    int next_key_frame_forced;
    int source_alt_ref_pending;
    int source_alt_ref_active;
    int is_src_frame_alt_ref;
    int sframe_due;

    // Length of the bi-predictive frame group interval
    int bipred_group_interval;

    // NOTE: Different types of frames may have different bits allocated
    //       accordingly, aiming to achieve the overall optimal RD performance.
    int is_bwd_ref_frame;
    int is_last_bipred_frame;
    int is_bipred_frame;
    int is_src_frame_ext_arf;

    int avg_frame_bandwidth; // Average frame size target for clip
    int min_frame_bandwidth; // Minimum allocation used for any frame
    int max_frame_bandwidth; // Maximum burst rate allowed for a frame.

    int    ni_av_qi;
    int    ni_tot_qi;
    int    ni_frames;
    int    avg_frame_qindex[FRAME_TYPES];
    double tot_q;
    double avg_q;

    int64_t buffer_level;
    int64_t bits_off_target;
    int64_t vbr_bits_off_target;
    int64_t vbr_bits_off_target_fast;

    int decimation_factor;
    int decimation_count;

    int rolling_target_bits;
    int rolling_actual_bits;

    int long_rolling_target_bits;
    int long_rolling_actual_bits;

    int rate_error_estimate;

    int64_t total_actual_bits;
    int64_t total_target_bits;
    int64_t total_target_vs_actual;

    int worst_quality;
    int best_quality;

    int64_t starting_buffer_level;
    int64_t optimal_buffer_level;
    int64_t maximum_buffer_size;

    // rate control history for last frame(1) and the frame before(2).
    // -1: undershot
    //  1: overshoot
    //  0: not initialized.
    int rc_1_frame;
    int rc_2_frame;
    int q_1_frame;
    int q_2_frame;

    // Auto frame-scaling variables.
    //   int rf_level_maxq[RATE_FACTOR_LEVELS];
    float_t arf_boost_factor;
    // Q index used for ALT frame
    int arf_q;
} RATE_CONTROL;
#define STATIC_MOTION_THRESH 95

enum {
    INTER_NORMAL       = 0,
    INTER_LOW          = 1,
    INTER_HIGH         = 2,
    GF_ARF_LOW         = 3,
    GF_ARF_STD         = 4,
    KF_STD             = 5,
    RATE_FACTOR_LEVELS = 6
} rate_factor_level;

enum {
    KF_UPDATE            = 0,
    LF_UPDATE            = 1,
    GF_UPDATE            = 2,
    ARF_UPDATE           = 3,
    OVERLAY_UPDATE       = 4,
    BRF_UPDATE           = 5, // Backward Reference Frame
    LAST_BIPRED_UPDATE   = 6, // Last Bi-predictive Frame
    BIPRED_UPDATE        = 7, // Bi-predictive Frame, but not the last one
    INTNL_OVERLAY_UPDATE = 8, // Internal Overlay Frame
    INTNL_ARF_UPDATE     = 9, // Internal Altref Frame (candidate for ALTREF2)
    FRAME_UPDATE_TYPES   = 10
} frame_update_type;

// that are not marked as coded with 0,0 motion in the first pass.
#define FAST_MOVING_KF_GROUP_THRESH 5
#define MEDIUM_MOVING_KF_GROUP_THRESH 30
#define STATIC_KF_GROUP_THRESH 70
#define MAX_QPS_COMP_I 150
#define MAX_QPS_COMP_I_LR 42
#define MAX_QPS_COMP_NONI 300
#define HIGH_QPS_COMP_THRESHOLD 80
#define LOW_QPS_COMP_THRESHOLD 40
#define HIGH_FILTERED_THRESHOLD (4 << 8) // 8 bit precision
#define LOW_FILTERED_THRESHOLD (2 << 8) // 8 bit precision
#define QPS_SW_THRESH 8 // 100 to shut QPS/QPM (i.e. CORE only)
#define MAX_REF_AREA_I 50 // Max ref area for I slice
#define MAX_REF_AREA_NONI 50 // Max ref area for Non I slice
#define MAX_REF_AREA_NONI_LOW_RES 40 // Max ref area for Non I slice in low resolution
#define REF_AREA_DIF_THRESHOLD 10 // Difference threshold for ref area between two frames
#define REF_AREA_LOW_THRESHOLD 8 // Low threshold for ref area
#define REF_AREA_MED_THRESHOLD 16 // Medium threshold for ref area
#define ME_SAD_LOW_THRESHOLD1 15 // Low sad_ threshold1 for me distortion (very low)
#define ME_SAD_LOW_THRESHOLD2 25 // Low sad_ threshold2 for me distortion (low)
#define ME_SAD_HIGH_THRESHOLD 80 // High sad_ threshold2 for me distortion (high)

#define ASSIGN_MINQ_TABLE(bit_depth, name)                       \
    do {                                                         \
        switch (bit_depth) {                                     \
        case AOM_BITS_8: name = name##_8; break;                 \
        case AOM_BITS_10: name = name##_10; break;               \
        case AOM_BITS_12: name = name##_12; break;               \
        default:                                                 \
            assert(0 &&                                          \
                   "bit_depth should be AOM_BITS_8, AOM_BITS_10" \
                   " or AOM_BITS_12");                           \
            name = NULL;                                         \
        }                                                        \
    } while (0)
static int kf_low_motion_minq_cqp_8[QINDEX_RANGE] = {
    0,    0,    0,    0,
    0,    0,    0,    0,
    0,    0,    0,    0,
    0,    0,    0,    0,
    0,    0,    0,    0,
    0,    0,    0,    0,
    0,    0,    0,    0,
    0,    0,    0,    0,
    0,    0,    0,    0,
    0,    0,    0,    0,
    0,    0,    0,    0,
    0,    0,    0,    0,
    0,    2,    2,    2,
    2,    2,    2,    2,
    3,    3,    3,    3,
    3,    3,    3,    4,
    4,    4,    4,    4,
    4,    4,    4,    5,
    5,    5,    5,    5,
    5,    5,    6,    6,
    6,    6,    6,    6,
    6,    7,    7,    7,
    7,    7,    7,    7,
    7,    8,    8,    8,
    8,    8,    9,    9,
    9,    9,    10,    10,
    10,    10,    11,    11,
    11,    11,    12,    12,
    12,    12,    13,    13,
    13,    13,    14,    14,
    14,    15,    15,    15,
    16,    16,    16,    17,
    17,    18,    18,    18,
    19,    19,    19,    20,
    20,    20,    21,    21,
    22,    22,    23,    23,
    24,    24,    24,    25,
    25,    26,    26,    27,
    27,    28,    28,    29,
    30,    30,    31,    31,
    32,    32,    33,    34,
    34,    35,    36,    36,
    37,    37,    38,    39,
    39,    40,    41,    42,
    42,    43,    44,    45,
    45,    46,    47,    48,
    49,    50,    51,    51,
    52,    53,    54,    55,
    56,    57,    58,    59,
    60,    61,    62,    64,
    65,    66,    67,    69,
    70,    71,    72,    74,
    75,    77,    78,    80,
    82,    83,    85,    87,
    89,    91,    93,    95,
    96,    97,    99,    100,
    101,    103,    104,    105,
    107,    109,    110,    112,
    114,    116,    118,    120,
    122,    124,    125,    127,
    129,    131,    134,    136,
    138,    140,    142,    144,
    147,    149,    151,    154,
    156,    158,    161,    163
};

static int kf_high_motion_minq_cqp_8[QINDEX_RANGE] = {
    0,    0,    0,    0,
    0,    0,    0,    0,
    0,    0,    0,    0,
    2,    2,    3,    3,
    4,    4,    5,    5,
    5,    6,    6,    7,
    7,    8,    8,    8,
    9,    9,    10,    10,
    11,    11,    11,    12,
    12,    13,    13,    14,
    14,    14,    15,    15,
    16,    16,    16,    17,
    17,    18,    18,    19,
    19,    19,    20,    20,
    21,    21,    21,    22,
    22,    23,    23,    24,
    24,    24,    25,    25,
    26,    26,    26,    27,
    27,    28,    28,    28,
    29,    29,    30,    30,
    30,    31,    31,    32,
    32,    32,    33,    33,
    34,    34,    34,    35,
    35,    36,    36,    36,
    37,    38,    39,    39,
    40,    41,    42,    42,
    43,    44,    45,    46,
    46,    47,    48,    49,
    49,    50,    51,    51,
    52,    53,    54,    54,
    55,    56,    57,    58,
    59,    61,    62,    63,
    64,    65,    66,    67,
    68,    69,    70,    71,
    72,    73,    74,    76,
    77,    78,    80,    81,
    82,    84,    85,    86,
    88,    89,    90,    92,
    93,    95,    96,    97,
    97,    98,    99,    100,
    100,    101,    102,    103,
    104,    105,    106,    107,
    107,    108,    109,    110,
    111,    112,    113,    114,
    115,    116,    117,    118,
    119,    120,    121,    121,
    122,    123,    124,    124,
    125,    126,    127,    127,
    128,    129,    130,    130,
    131,    132,    133,    134,
    135,    135,    136,    137,
    138,    139,    139,    140,
    141,    141,    142,    143,
    144,    144,    145,    146,
    147,    148,    149,    149,
    150,    151,    152,    153,
    154,    154,    155,    156,
    157,    158,    159,    160,
    161,    162,    163,    164,
    166,    167,    168,    169,
    171,    172,    173,    175,
    176,    178,    179,    181,
    183,    184,    186,    188,
    190,    191,    193,    195
};

static int kf_low_motion_minq_cqp_10[QINDEX_RANGE] = {
        0,      0,      0,      0,
        0,      0,      0,      0,
        0,      0,      0,      0,
        0,      0,      0,      0,
        0,      0,      0,      0,
        0,      0,      0,      0,
        0,      0,      0,      0,
        0,      0,      0,      0,
        0,      0,      0,      0,
        0,      0,      0,      0,
        0,      0,      0,      0,
        0,      0,      0,      0,
        0,      0,      0,      0,
        0,      0,      10,     10,
        11,     11,     11,     11,
        11,     11,     12,     12,
        12,     12,     12,     13,
        13,     13,     13,     13,
        13,     13,     14,     14,
        14,     14,     14,     14,
        14,     15,     15,     15,
        15,     15,     16,     16,
        16,     16,     16,     16,
        16,     17,     17,     17,
        17,     17,     18,     18,
        18,     18,     19,     19,
        19,     19,     20,     20,
        20,     21,     21,     21,
        21,     22,     22,     22,
        22,     23,     23,     23,
        23,     24,     24,     24,
        25,     25,     25,     26,
        26,     26,     27,     27,
        27,     28,     28,     28,
        29,     29,     29,     30,
        30,     31,     31,     32,
        32,     32,     33,     33,
        34,     34,     34,     35,
        35,     36,     36,     37,
        37,     38,     38,     39,
        39,     40,     40,     41,
        41,     42,     42,     43,
        44,     44,     45,     46,
        46,     47,     47,     48,
        49,     49,     50,     51,
        51,     52,     53,     54,
        54,     55,     56,     57,
        58,     58,     59,     60,
        61,     62,     63,     64,
        65,     66,     67,     68,
        69,     70,     71,     72,
        73,     74,     76,     77,
        78,     80,     81,     83,
        84,     86,     87,     89,
        91,     93,     95,     96,
        97,     98,     100,    101,
        102,    103,    105,    106,
        108,    109,    111,    113,
        115,    117,    119,    121,
        122,    124,    126,    128,
        130,    132,    134,    136,
        138,    140,    142,    144,
        147,    149,    151,    154,
        156,    159,    161,    163
};

static int kf_high_motion_minq_cqp_10[QINDEX_RANGE] = {
        0,      0,      0,      0,
        0,      0,      0,      0,
        0,      0,      0,      0,
        0,      0,      0,      0,
        0,      0,      0,      0,
        0,      11,     11,     11,
        12,     13,     13,     14,
        14,     15,     15,     16,
        16,     17,     17,     18,
        18,     19,     19,     20,
        20,     21,     21,     22,
        22,     22,     23,     23,
        24,     24,     25,     25,
        26,     26,     27,     27,
        27,     28,     28,     29,
        29,     29,     30,     30,
        31,     31,     32,     32,
        32,     33,     33,     33,
        34,     34,     35,     35,
        35,     36,     36,     37,
        37,     37,     38,     38,
        39,     39,     39,     40,
        40,     41,     41,     41,
        42,     42,     42,     43,
        43,     44,     45,     45,
        46,     47,     48,     48,
        49,     50,     50,     51,
        52,     52,     53,     54,
        54,     55,     56,     56,
        57,     58,     58,     59,
        60,     61,     62,     63,
        64,     64,     66,     67,
        67,     69,     69,     70,
        71,     72,     73,     74,
        75,     76,     77,     79,
        80,     81,     82,     84,
        85,     86,     87,     88,
        90,     91,     92,     94,
        95,     96,     97,     97,
        98,     99,     100,    101,
        101,    102,    103,    104,
        105,    105,    106,    107,
        108,    109,    110,    111,
        112,    113,    114,    114,
        115,    116,    117,    118,
        119,    120,    121,    122,
        123,    123,    124,    125,
        125,    126,    127,    128,
        128,    129,    130,    131,
        132,    132,    133,    134,
        135,    136,    136,    137,
        138,    139,    139,    140,
        141,    142,    142,    143,
        144,    144,    145,    146,
        147,    148,    149,    150,
        150,    151,    152,    153,
        154,    154,    155,    156,
        157,    158,    159,    160,
        161,    162,    163,    165,
        166,    167,    168,    169,
        171,    172,    173,    175,
        176,    178,    179,    181,
        183,    184,    186,    188,
        190,    191,    193,    195
};

static int kf_low_motion_minq_cqp_12[QINDEX_RANGE] = {
        0,      0,      0,      0,
        0,      0,      0,      0,
        0,      0,      0,      0,
        0,      0,      0,      0,
        0,      0,      0,      0,
        0,      0,      0,      0,
        0,      0,      0,      0,
        0,      0,      0,      0,
        0,      0,      0,      0,
        0,      0,      0,      0,
        0,      0,      0,      0,
        0,      0,      0,      0,
        0,      0,      0,      0,
        0,      0,      0,      0,
        13,     13,     13,     13,
        14,     14,     14,     14,
        14,     14,     15,     15,
        15,     15,     15,     16,
        16,     16,     16,     16,
        16,     16,     17,     17,
        17,     17,     17,     17,
        18,     18,     18,     18,
        18,     18,     18,     19,
        19,     19,     19,     19,
        19,     20,     20,     20,
        20,     21,     21,     21,
        21,     22,     22,     22,
        22,     23,     23,     23,
        23,     24,     24,     24,
        24,     25,     25,     25,
        25,     26,     26,     26,
        27,     27,     27,     28,
        28,     28,     29,     29,
        29,     30,     30,     30,
        31,     31,     31,     32,
        32,     33,     33,     33,
        34,     34,     35,     35,
        35,     36,     36,     37,
        37,     38,     38,     39,
        39,     39,     40,     40,
        41,     41,     42,     42,
        43,     44,     44,     45,
        45,     46,     46,     47,
        48,     48,     49,     49,
        50,     51,     51,     52,
        53,     53,     54,     55,
        56,     56,     57,     58,
        59,     59,     60,     61,
        62,     63,     64,     65,
        66,     67,     68,     69,
        70,     71,     72,     73,
        74,     75,     76,     78,
        79,     80,     82,     83,
        85,     86,     88,     90,
        91,     93,     95,     96,
        97,     99,     100,    101,
        102,    104,    105,    106,
        108,    110,    111,    113,
        115,    117,    119,    121,
        122,    124,    126,    128,
        130,    132,    134,    136,
        138,    140,    142,    144,
        147,    149,    152,    154,
        156,    159,    161,    163
};

static int kf_high_motion_minq_cqp_12[QINDEX_RANGE] = {
        0,      0,      0,      0,
        0,      0,      0,      0,
        0,      0,      0,      0,
        0,      0,      0,      0,
        0,      0,      0,      0,
        0,      0,      0,      13,
        14,     14,     15,     15,
        16,     16,     17,     17,
        18,     18,     19,     19,
        20,     20,     21,     21,
        22,     22,     23,     23,
        23,     24,     24,     25,
        25,     26,     26,     27,
        27,     28,     28,     28,
        29,     29,     30,     30,
        31,     31,     31,     32,
        32,     33,     33,     33,
        34,     34,     35,     35,
        35,     36,     36,     37,
        37,     37,     38,     38,
        39,     39,     39,     40,
        40,     40,     41,     41,
        41,     42,     42,     43,
        43,     43,     44,     44,
        45,     45,     46,     47,
        47,     48,     49,     49,
        50,     51,     51,     52,
        53,     53,     54,     55,
        55,     56,     57,     57,
        58,     59,     59,     60,
        61,     62,     63,     64,
        64,     65,     66,     67,
        68,     69,     70,     71,
        72,     73,     74,     75,
        76,     77,     78,     79,
        80,     82,     83,     84,
        85,     86,     88,     89,
        90,     91,     92,     94,
        95,     96,     97,     98,
        98,     99,     100,    101,
        101,    102,    103,    104,
        105,    106,    107,    107,
        108,    109,    110,    111,
        112,    113,    114,    115,
        115,    116,    117,    118,
        119,    120,    121,    122,
        123,    123,    124,    125,
        125,    126,    127,    128,
        128,    129,    130,    131,
        132,    132,    133,    134,
        135,    136,    137,    137,
        138,    139,    139,    140,
        141,    142,    142,    143,
        144,    145,    145,    146,
        147,    148,    149,    150,
        151,    151,    152,    153,
        154,    155,    155,    156,
        157,    158,    159,    160,
        161,    162,    163,    165,
        166,    167,    168,    170,
        171,    172,    173,    175,
        176,    178,    179,    181,
        183,    184,    186,    188,
        190,    191,    193,    195
};
static int kf_low_motion_minq_8[QINDEX_RANGE] = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   2,   2,   2,   2,   2,   2,   2,   3,
    3,   3,   3,   3,   3,   3,   4,   4,   4,   4,   4,   4,   4,   4,   5,   5,   5,   5,   5,
    5,   5,   6,   6,   6,   6,   6,   6,   6,   7,   7,   7,   7,   7,   7,   7,   7,   8,   8,
    8,   8,   8,   9,   9,   9,   9,   10,  10,  10,  10,  11,  11,  11,  11,  12,  12,  12,  12,
    13,  13,  13,  13,  14,  14,  14,  15,  15,  15,  16,  16,  16,  17,  17,  18,  18,  18,  19,
    19,  19,  20,  20,  20,  21,  21,  22,  22,  23,  23,  24,  24,  24,  25,  25,  26,  26,  27,
    27,  28,  28,  29,  30,  30,  31,  31,  32,  32,  33,  34,  34,  35,  36,  36,  37,  37,  38,
    39,  39,  40,  41,  42,  42,  43,  44,  45,  45,  46,  47,  48,  49,  50,  51,  51,  52,  54,
    55,  56,  57,  58,  59,  61,  62,  63,  64,  67,  68,  69,  71,  73,  74,  76,  77,  80,  81,
    83,  85,  87,  90,  91,  94,  96,  99,  102, 104, 107, 108, 110, 113, 114, 116, 119, 121, 122,
    125, 128, 130, 133, 135, 138, 141, 144, 147, 150, 152, 155, 158, 161, 165, 168, 171, 174, 177,
    180, 184, 187, 190, 194, 197, 201, 205, 208};

static int kf_high_motion_minq_8[QINDEX_RANGE] = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   2,   2,   3,   3,   4,   4,   5,
    5,   5,   6,   6,   7,   7,   8,   8,   8,   9,   9,   10,  10,  11,  11,  11,  12,  12,  13,
    13,  14,  14,  14,  15,  15,  16,  16,  16,  17,  17,  18,  18,  19,  19,  19,  20,  20,  21,
    21,  21,  22,  22,  23,  23,  24,  24,  24,  25,  25,  26,  26,  26,  27,  27,  28,  28,  28,
    29,  29,  30,  30,  30,  31,  31,  32,  32,  32,  33,  33,  34,  34,  34,  35,  35,  36,  36,
    36,  37,  38,  39,  39,  40,  41,  42,  42,  43,  44,  45,  46,  46,  47,  48,  49,  49,  50,
    51,  51,  52,  53,  54,  54,  55,  56,  57,  58,  59,  61,  62,  63,  64,  65,  66,  67,  68,
    69,  70,  71,  72,  73,  74,  76,  77,  78,  80,  81,  82,  84,  85,  86,  88,  89,  90,  92,
    93,  95,  96,  97,  97,  98,  99,  100, 100, 101, 102, 103, 104, 105, 106, 107, 107, 108, 109,
    110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 121, 122, 123, 124, 124, 125, 126,
    128, 128, 129, 130, 131, 131, 132, 134, 135, 136, 137, 138, 139, 140, 141, 143, 143, 144, 146,
    146, 147, 149, 150, 151, 152, 153, 155, 156, 158, 158, 160, 161, 163, 164, 166, 166, 168, 170,
    171, 173, 174, 176, 178, 179, 181, 183, 185, 187, 189, 191, 193, 195, 197, 200, 201, 204, 206,
    209, 212, 214, 216, 219, 222, 224, 227, 230};

static int arfgf_low_motion_minq_8[QINDEX_RANGE] = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   2,   2,   2,   3,   3,   3,   3,   4,   4,   4,   5,   5,   5,   5,   6,   6,   6,
    7,   7,   7,   7,   8,   8,   8,   9,   9,   9,   9,   10,  10,  10,  10,  11,  11,  11,  12,
    12,  12,  12,  13,  13,  13,  13,  14,  14,  14,  15,  15,  15,  15,  16,  16,  16,  16,  17,
    17,  17,  17,  18,  18,  18,  18,  19,  19,  19,  20,  20,  20,  20,  21,  21,  21,  21,  22,
    22,  22,  23,  23,  24,  24,  25,  25,  26,  26,  27,  27,  28,  28,  29,  29,  30,  30,  31,
    31,  32,  32,  33,  33,  34,  34,  35,  36,  36,  37,  38,  38,  39,  40,  41,  41,  42,  43,
    43,  44,  45,  45,  46,  47,  48,  49,  49,  50,  51,  52,  53,  54,  54,  55,  56,  57,  58,
    59,  60,  61,  62,  63,  64,  65,  66,  67,  68,  69,  70,  71,  72,  73,  74,  75,  77,  78,
    79,  80,  81,  83,  84,  85,  86,  87,  89,  90,  91,  92,  94,  95,  96,  97,  97,  98,  100,
    100, 101, 102, 102, 103, 105, 106, 106, 107, 109, 110, 110, 112, 113, 114, 116, 116, 118, 119,
    120, 122, 123, 125, 125, 127, 128, 130, 132, 133, 134, 135, 137, 139, 140, 141, 143, 145, 146,
    148, 150, 152, 154, 155, 158, 160, 162, 164, 166, 168, 171, 173, 176, 178, 181, 183, 186, 188,
    191, 194, 197, 200, 203, 206, 210, 213, 216};

static int arfgf_high_motion_minq_8[QINDEX_RANGE] = {
    0,   0,   0,   0,   0,   0,   0,   0,   2,   2,   3,   3,   4,   4,   5,   5,   6,   7,   7,
    8,   8,   9,   9,   10,  10,  11,  11,  12,  12,  13,  13,  14,  14,  15,  16,  16,  17,  17,
    18,  18,  19,  19,  20,  20,  21,  21,  22,  22,  23,  23,  24,  24,  25,  25,  26,  26,  27,
    27,  28,  28,  29,  29,  30,  31,  31,  32,  32,  33,  33,  34,  34,  35,  35,  36,  36,  37,
    37,  38,  38,  39,  39,  40,  40,  41,  41,  42,  42,  43,  43,  44,  44,  45,  45,  46,  46,
    46,  47,  48,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,  60,  61,  62,  63,  64,
    65,  66,  67,  68,  68,  69,  70,  72,  73,  74,  76,  77,  79,  80,  81,  83,  84,  85,  87,
    88,  89,  91,  92,  93,  95,  96,  97,  98,  99,  100, 100, 101, 102, 103, 104, 105, 106, 107,
    108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 123, 124, 125,
    126, 127, 128, 129, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 139, 140, 141, 142,
    144, 144, 145, 146, 147, 148, 149, 151, 151, 152, 153, 155, 156, 156, 157, 159, 160, 161, 162,
    163, 164, 166, 167, 169, 169, 170, 172, 173, 175, 176, 178, 179, 180, 181, 183, 184, 186, 188,
    189, 191, 192, 194, 196, 197, 199, 201, 202, 204, 206, 209, 210, 212, 214, 217, 218, 220, 223,
    225, 228, 230, 232, 234, 237, 239, 242, 245};
/*
static int inter_minq_8[QINDEX_RANGE] = {
        0, 0, 2, 2, 3, 4, 5, 6, 7, 8, 9, 10, 10, 11, 12,
        13, 14, 15, 16, 17, 18, 18, 19, 20, 21, 22, 23, 24, 25, 26,
        26, 27, 28, 29, 30, 31, 32, 33, 33, 34, 35, 36, 37, 38, 39,
        40, 40, 41, 42, 43, 44, 45, 46, 47, 47, 48, 49, 50, 51, 52,
        53, 53, 54, 55, 56, 57, 58, 59, 59, 60, 61, 62, 63, 64, 65,
        65, 66, 67, 68, 69, 70, 71, 71, 72, 73, 74, 75, 76, 77, 77,
        78, 79, 80, 81, 82, 83, 84, 86, 88, 89, 91, 93, 94, 96, 97,
        97, 98, 99, 100, 101, 102, 102, 103, 104, 105, 106, 107, 107, 108, 109,
        110, 111, 112, 114, 115, 116, 117, 119, 120, 121, 122, 122, 123, 124, 125,
        126, 127, 127, 128, 129, 131, 132, 133, 134, 135, 136, 137, 138, 139, 139,
        140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154,
        155, 156, 157, 157, 158, 159, 161, 161, 162, 163, 164, 165, 166, 167, 168,
        169, 170, 171, 172, 173, 174, 175, 176, 176, 177, 178, 179, 180, 181, 182,
        183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 196,
        197, 199, 199, 200, 201, 203, 203, 205, 206, 207, 208, 209, 210, 211, 212,
        213, 214, 215, 216, 217, 219, 220, 221, 222, 223, 225, 226, 227, 228, 230,
        231, 232, 234, 235, 236, 238, 239, 240, 242, 243, 245, 246, 248, 250, 251,
        253};

static int rtc_minq_8[QINDEX_RANGE] = {
        0, 0, 0, 0, 0, 2, 3, 3, 4, 5, 5, 6, 7, 7, 8,
        9, 9, 10, 11, 12, 12, 13, 14, 14, 15, 16, 16, 17, 18, 18,
        19, 20, 20, 21, 22, 22, 23, 24, 24, 25, 26, 26, 27, 28, 28,
        29, 30, 31, 31, 32, 33, 33, 34, 35, 35, 36, 37, 37, 38, 39,
        39, 40, 41, 41, 42, 42, 43, 44, 44, 45, 46, 46, 47, 48, 48,
        49, 50, 50, 51, 52, 52, 53, 54, 54, 55, 56, 56, 57, 58, 58,
        59, 60, 60, 61, 61, 62, 63, 65, 66, 67, 69, 70, 71, 72, 74,
        75, 76, 78, 79, 80, 81, 83, 84, 85, 86, 88, 89, 90, 91, 93,
        94, 96, 97, 98, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108,
        109, 110, 110, 112, 113, 114, 115, 116, 118, 119, 120, 121, 122, 123, 123,
        124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138,
        139, 140, 141, 142, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152,
        153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 162, 163, 164, 165, 166,
        167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181,
        182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196,
        197, 199, 200, 201, 202, 203, 205, 206, 207, 208, 210, 211, 212, 214, 215,
        216, 218, 219, 221, 222, 224, 225, 227, 229, 230, 232, 234, 235, 237, 239,
        241};
*/

static int kf_low_motion_minq_10[QINDEX_RANGE] = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   10,  10,  11,
    11,  11,  11,  11,  11,  12,  12,  12,  12,  12,  13,  13,  13,  13,  13,  13,  13,  14,  14,
    14,  14,  14,  14,  14,  15,  15,  15,  15,  15,  16,  16,  16,  16,  16,  16,  16,  17,  17,
    17,  17,  17,  18,  18,  18,  18,  19,  19,  19,  19,  20,  20,  20,  21,  21,  21,  21,  22,
    22,  22,  22,  23,  23,  23,  23,  24,  24,  24,  25,  25,  25,  26,  26,  26,  27,  27,  27,
    28,  28,  28,  29,  29,  29,  30,  30,  31,  31,  32,  32,  32,  33,  33,  34,  34,  34,  35,
    35,  36,  36,  37,  37,  38,  38,  39,  39,  40,  40,  41,  41,  42,  42,  43,  44,  44,  45,
    46,  46,  47,  47,  48,  49,  49,  50,  51,  51,  52,  53,  54,  54,  55,  56,  57,  58,  59,
    60,  61,  62,  63,  64,  66,  67,  68,  69,  71,  72,  73,  75,  76,  77,  79,  81,  83,  84,
    86,  88,  90,  92,  94,  96,  98,  101, 104, 106, 108, 109, 111, 114, 115, 117, 119, 122, 123,
    126, 128, 131, 134, 136, 139, 142, 145, 147, 150, 153, 156, 159, 162, 165, 168, 171, 174, 177,
    180, 184, 187, 190, 194, 197, 202, 205, 208};

static int kf_high_motion_minq_10[QINDEX_RANGE] = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   11,  11,  11,  12,  13,  13,  14,  14,  15,  15,  16,  16,  17,  17,  18,  18,  19,
    19,  20,  20,  21,  21,  22,  22,  22,  23,  23,  24,  24,  25,  25,  26,  26,  27,  27,  27,
    28,  28,  29,  29,  29,  30,  30,  31,  31,  32,  32,  32,  33,  33,  33,  34,  34,  35,  35,
    35,  36,  36,  37,  37,  37,  38,  38,  39,  39,  39,  40,  40,  41,  41,  41,  42,  42,  42,
    43,  43,  44,  45,  45,  46,  47,  48,  48,  49,  50,  50,  51,  52,  52,  53,  54,  54,  55,
    56,  56,  57,  58,  58,  59,  60,  61,  62,  63,  64,  64,  66,  67,  67,  69,  69,  70,  71,
    72,  73,  74,  75,  76,  77,  79,  80,  81,  82,  84,  85,  86,  87,  88,  90,  91,  92,  94,
    95,  96,  97,  97,  98,  99,  100, 101, 101, 102, 103, 104, 105, 105, 106, 107, 108, 109, 110,
    111, 112, 113, 114, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 123, 124, 125, 125, 126,
    128, 129, 129, 130, 131, 132, 133, 134, 135, 136, 137, 139, 139, 140, 141, 143, 143, 144, 146,
    147, 147, 149, 150, 151, 152, 153, 155, 156, 158, 159, 160, 161, 163, 164, 166, 166, 168, 170,
    171, 173, 174, 176, 178, 179, 181, 184, 185, 187, 189, 191, 193, 195, 197, 200, 201, 204, 206,
    209, 212, 214, 216, 219, 222, 224, 227, 230};

static int arfgf_low_motion_minq_10[QINDEX_RANGE] = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   10,  11,  11,  11,  12,  12,  12,  13,  13,
    13,  14,  14,  14,  15,  15,  16,  16,  16,  17,  17,  17,  17,  18,  18,  18,  19,  19,  19,
    20,  20,  20,  21,  21,  21,  21,  22,  22,  22,  23,  23,  23,  24,  24,  24,  24,  25,  25,
    25,  25,  26,  26,  26,  26,  27,  27,  27,  28,  28,  28,  28,  28,  29,  29,  29,  30,  30,
    30,  30,  31,  31,  32,  32,  33,  33,  34,  34,  35,  35,  36,  36,  37,  37,  37,  38,  38,
    39,  39,  40,  40,  41,  41,  41,  42,  42,  43,  44,  44,  45,  46,  46,  47,  48,  48,  49,
    49,  50,  50,  51,  52,  52,  53,  54,  55,  56,  56,  57,  58,  59,  59,  60,  61,  62,  62,
    63,  64,  65,  66,  67,  68,  69,  69,  70,  72,  72,  73,  74,  75,  77,  77,  78,  79,  80,
    82,  83,  84,  85,  86,  87,  88,  90,  91,  92,  93,  94,  95,  96,  97,  98,  98,  99,  101,
    101, 102, 103, 103, 104, 106, 106, 107, 108, 110, 110, 111, 113, 114, 114, 116, 117, 119, 120,
    121, 122, 123, 125, 126, 128, 129, 131, 132, 133, 135, 136, 137, 139, 140, 142, 144, 145, 146,
    148, 150, 152, 154, 156, 158, 160, 162, 164, 166, 169, 171, 173, 176, 178, 181, 184, 186, 189,
    191, 194, 197, 200, 203, 206, 210, 213, 216};

static int arfgf_high_motion_minq_10[QINDEX_RANGE] = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   10,  11,
    11,  12,  13,  13,  14,  14,  15,  16,  16,  17,  18,  18,  19,  19,  20,  20,  21,  22,  22,
    23,  23,  24,  24,  25,  26,  26,  27,  27,  28,  28,  29,  30,  30,  30,  31,  32,  32,  33,
    33,  34,  34,  35,  35,  36,  36,  37,  37,  38,  38,  39,  39,  40,  40,  41,  41,  42,  42,
    42,  43,  44,  44,  44,  45,  45,  46,  46,  47,  47,  48,  48,  49,  49,  50,  50,  51,  51,
    52,  52,  53,  54,  55,  56,  57,  58,  59,  60,  60,  61,  62,  63,  64,  65,  66,  67,  67,
    68,  69,  70,  71,  72,  72,  73,  75,  76,  77,  78,  80,  81,  82,  84,  85,  86,  87,  89,
    90,  91,  92,  94,  95,  96,  97,  98,  99,  99,  100, 101, 102, 103, 104, 105, 105, 106, 107,
    108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 120, 121, 121, 122, 123, 124, 125, 125,
    126, 127, 128, 129, 130, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 140, 141, 142,
    144, 145, 145, 146, 147, 148, 149, 151, 152, 152, 153, 155, 156, 156, 157, 159, 160, 161, 163,
    163, 164, 166, 167, 169, 170, 170, 172, 173, 175, 176, 178, 179, 181, 181, 183, 184, 186, 188,
    189, 191, 192, 194, 196, 197, 199, 201, 202, 204, 206, 209, 210, 212, 214, 217, 218, 220, 223,
    225, 228, 230, 232, 234, 237, 240, 242, 245};
/*
static int inter_minq_10[QINDEX_RANGE] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 11, 11, 12, 13,
        14, 15, 16, 17, 18, 19, 20, 20, 21, 22, 23, 24, 25, 26, 27,
        28, 29, 29, 30, 31, 32, 33, 34, 35, 36, 37, 37, 39, 39, 40,
        41, 42, 43, 44, 44, 45, 46, 47, 48, 49, 50, 51, 51, 52, 53,
        54, 55, 56, 57, 58, 58, 59, 60, 61, 62, 62, 63, 64, 65, 66,
        67, 68, 69, 69, 70, 71, 72, 73, 73, 74, 75, 76, 77, 78, 79,
        79, 80, 81, 82, 83, 84, 85, 87, 88, 90, 92, 93, 95, 96, 97,
        98, 99, 99, 100, 101, 102, 103, 104, 104, 105, 106, 107, 108, 109, 109,
        110, 111, 113, 114, 115, 116, 118, 119, 120, 121, 122, 123, 123, 124, 125,
        126, 127, 127, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140,
        140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154,
        155, 156, 157, 158, 158, 160, 161, 161, 162, 163, 164, 165, 166, 167, 168,
        169, 170, 171, 172, 173, 174, 175, 176, 177, 177, 178, 179, 180, 181, 182,
        183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 196,
        197, 199, 199, 200, 201, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212,
        213, 214, 215, 216, 218, 219, 220, 221, 222, 223, 225, 226, 227, 228, 230,
        231, 232, 234, 235, 236, 238, 239, 240, 242, 243, 245, 246, 248, 250, 251,
        253};

static int rtc_minq_10[QINDEX_RANGE] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 11,
        11, 12, 13, 13, 14, 15, 16, 16, 17, 18, 19, 19, 20, 21, 22,
        22, 23, 24, 24, 25, 26, 27, 28, 28, 29, 29, 30, 31, 32, 32,
        33, 34, 34, 35, 36, 36, 37, 38, 38, 39, 40, 41, 41, 42, 42,
        43, 44, 44, 45, 46, 46, 47, 48, 48, 49, 50, 50, 51, 51, 52,
        53, 53, 54, 55, 55, 56, 56, 57, 58, 58, 59, 60, 60, 61, 62,
        62, 63, 63, 64, 64, 65, 67, 68, 69, 70, 71, 72, 74, 75, 76,
        77, 78, 80, 81, 82, 83, 84, 86, 87, 88, 89, 90, 91, 93, 94,
        95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 105, 106, 107, 108,
        109, 110, 111, 112, 113, 114, 116, 117, 118, 119, 120, 121, 122, 123, 124,
        124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138,
        139, 140, 141, 142, 143, 144, 144, 145, 146, 147, 148, 149, 150, 151, 152,
        153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 163, 164, 165, 166,
        167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181,
        182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196,
        198, 199, 200, 201, 202, 203, 205, 206, 207, 208, 210, 211, 212, 214, 215,
        216, 218, 219, 221, 222, 224, 225, 227, 229, 230, 232, 234, 235, 237, 239,
        241};
*/

static int kf_low_motion_minq_12[QINDEX_RANGE] = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   13,
    13,  13,  13,  14,  14,  14,  14,  14,  14,  15,  15,  15,  15,  15,  16,  16,  16,  16,  16,
    16,  16,  17,  17,  17,  17,  17,  17,  18,  18,  18,  18,  18,  18,  18,  19,  19,  19,  19,
    19,  19,  20,  20,  20,  20,  21,  21,  21,  21,  22,  22,  22,  22,  23,  23,  23,  23,  24,
    24,  24,  24,  25,  25,  25,  25,  26,  26,  26,  27,  27,  27,  28,  28,  28,  29,  29,  29,
    30,  30,  30,  31,  31,  31,  32,  32,  33,  33,  33,  34,  34,  35,  35,  35,  36,  36,  37,
    37,  38,  38,  39,  39,  39,  40,  40,  41,  41,  42,  42,  43,  44,  44,  45,  45,  46,  46,
    47,  48,  48,  49,  49,  50,  51,  51,  52,  53,  53,  54,  55,  56,  56,  57,  58,  59,  59,
    60,  61,  62,  63,  64,  65,  66,  67,  68,  69,  70,  71,  72,  73,  74,  75,  76,  78,  79,
    80,  82,  83,  85,  86,  88,  90,  91,  93,  95,  96,  97,  99,  100, 101, 102, 104, 105, 106,
    108, 110, 111, 113, 115, 117, 119, 121, 122, 124, 126, 128, 130, 132, 134, 136, 138, 140, 142,
    144, 147, 149, 152, 154, 156, 159, 161, 163};

static int kf_high_motion_minq_12[QINDEX_RANGE] = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   13,  14,  14,  15,  15,  16,  16,  17,  17,  18,  18,  19,  19,  20,  20,
    21,  21,  22,  22,  23,  23,  23,  24,  24,  25,  25,  26,  26,  27,  27,  28,  28,  28,  29,
    29,  30,  30,  31,  31,  31,  32,  32,  33,  33,  33,  34,  34,  35,  35,  35,  36,  36,  37,
    37,  37,  38,  38,  39,  39,  39,  40,  40,  40,  41,  41,  41,  42,  42,  43,  43,  43,  44,
    44,  45,  45,  46,  47,  47,  48,  49,  49,  50,  51,  51,  52,  53,  53,  54,  55,  55,  56,
    57,  57,  58,  59,  59,  60,  61,  62,  63,  64,  64,  65,  66,  67,  68,  69,  70,  71,  72,
    73,  74,  75,  76,  77,  78,  79,  80,  82,  83,  84,  85,  86,  88,  89,  90,  91,  92,  94,
    95,  96,  97,  98,  98,  99,  100, 101, 101, 102, 103, 104, 105, 106, 107, 107, 108, 109, 110,
    111, 112, 113, 114, 115, 115, 116, 117, 118, 119, 120, 121, 122, 123, 123, 124, 125, 125, 126,
    127, 128, 128, 129, 130, 131, 132, 132, 133, 134, 135, 136, 137, 137, 138, 139, 139, 140, 141,
    142, 142, 143, 144, 145, 145, 146, 147, 148, 149, 150, 151, 151, 152, 153, 154, 155, 155, 156,
    157, 158, 159, 160, 161, 162, 163, 165, 166, 167, 168, 170, 171, 172, 173, 175, 176, 178, 179,
    181, 183, 184, 186, 188, 190, 191, 193, 195};

static int arfgf_low_motion_minq_12[QINDEX_RANGE] = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   13,  13,  14,  14,  14,  15,  15,
    15,  16,  16,  16,  17,  17,  17,  18,  18,  18,  19,  19,  19,  20,  20,  20,  21,  21,  21,
    22,  22,  22,  22,  23,  23,  23,  24,  24,  24,  25,  25,  25,  25,  26,  26,  26,  26,  27,
    27,  27,  28,  28,  28,  28,  29,  29,  29,  29,  30,  30,  30,  30,  31,  31,  31,  31,  32,
    32,  32,  33,  33,  34,  34,  35,  35,  35,  36,  36,  37,  37,  38,  38,  39,  39,  39,  40,
    40,  41,  41,  42,  42,  42,  43,  43,  44,  45,  45,  46,  46,  47,  48,  48,  49,  49,  50,
    51,  51,  52,  52,  53,  54,  54,  55,  56,  57,  57,  58,  59,  60,  60,  61,  62,  63,  63,
    64,  65,  66,  67,  68,  69,  70,  70,  71,  72,  73,  74,  75,  76,  77,  78,  79,  80,  81,
    82,  83,  84,  86,  87,  88,  89,  90,  91,  92,  94,  95,  96,  96,  97,  98,  98,  99,  100,
    100, 101, 102, 102, 103, 104, 105, 105, 106, 107, 108, 108, 109, 110, 111, 111, 112, 113, 114,
    115, 115, 116, 117, 118, 119, 120, 121, 122, 122, 123, 124, 124, 125, 126, 127, 128, 129, 129,
    130, 131, 132, 134, 135, 136, 137, 138, 139, 141, 142, 143, 144, 146, 147, 149, 151, 152, 154,
    155, 157, 159, 161, 163, 165, 167, 169, 171};

static int arfgf_high_motion_minq_12[QINDEX_RANGE] = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   13,  14,  14,  15,  16,  16,  17,  17,  18,  19,  19,  20,  20,  21,  22,  22,  23,  23,
    24,  25,  25,  26,  26,  27,  27,  28,  28,  29,  30,  30,  31,  31,  32,  32,  33,  33,  34,
    34,  35,  35,  36,  36,  37,  37,  38,  38,  39,  39,  40,  40,  41,  41,  42,  42,  43,  43,
    44,  44,  45,  45,  46,  46,  47,  47,  48,  48,  49,  49,  49,  50,  50,  51,  51,  52,  52,
    53,  53,  54,  55,  56,  57,  58,  59,  59,  60,  61,  62,  63,  64,  65,  65,  66,  67,  68,
    69,  70,  71,  71,  72,  73,  74,  75,  77,  78,  79,  80,  82,  83,  84,  85,  87,  88,  89,
    90,  92,  93,  94,  95,  96,  97,  98,  99,  100, 101, 101, 102, 103, 104, 105, 106, 106, 107,
    108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 119, 120, 121, 122, 122, 123, 124, 125, 125,
    126, 127, 128, 129, 130, 131, 132, 132, 133, 134, 135, 136, 137, 138, 139, 140, 140, 141, 142,
    143, 144, 144, 145, 146, 147, 148, 149, 150, 150, 151, 152, 153, 154, 154, 155, 156, 157, 158,
    158, 159, 160, 161, 162, 163, 163, 164, 165, 166, 167, 168, 169, 170, 170, 171, 172, 173, 174,
    175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 187, 188, 189, 190, 192, 193, 194, 196,
    197, 199, 200, 202, 203, 205, 207, 208, 210};

/*
static int inter_minq_12[QINDEX_RANGE] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 13,
        14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 23, 24, 25, 26, 27,
        28, 29, 30, 31, 32, 32, 33, 34, 35, 36, 37, 38, 39, 40, 40,
        41, 42, 43, 44, 45, 46, 47, 47, 48, 49, 50, 51, 52, 53, 53,
        54, 55, 56, 57, 58, 59, 59, 60, 61, 62, 63, 64, 65, 65, 66,
        67, 68, 69, 70, 70, 71, 72, 73, 74, 75, 76, 76, 77, 78, 79,
        80, 80, 81, 82, 83, 84, 85, 87, 89, 90, 92, 93, 95, 96, 97,
        98, 99, 99, 100, 101, 102, 103, 104, 104, 105, 106, 107, 108, 109, 109,
        110, 111, 113, 114, 115, 116, 118, 119, 120, 121, 122, 123, 123, 124, 125,
        126, 127, 127, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140,
        140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154,
        155, 156, 157, 158, 158, 160, 161, 161, 162, 163, 164, 165, 166, 167, 168,
        169, 170, 171, 172, 173, 174, 175, 176, 177, 177, 178, 179, 180, 181, 182,
        183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 196,
        197, 199, 199, 200, 201, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212,
        213, 214, 215, 216, 217, 219, 220, 221, 222, 223, 225, 226, 227, 228, 230,
        231, 232, 234, 235, 236, 238, 239, 240, 242, 243, 245, 246, 248, 250, 251,
        253};

static int rtc_minq_12[QINDEX_RANGE] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 13, 14, 15, 16, 16, 17, 18, 19, 19, 20, 21, 22, 22,
        23, 24, 25, 25, 26, 27, 28, 28, 29, 30, 30, 31, 32, 32, 33,
        34, 34, 35, 36, 37, 37, 38, 39, 39, 40, 41, 41, 42, 43, 43,
        44, 45, 45, 46, 46, 47, 48, 48, 49, 50, 50, 51, 52, 52, 53,
        54, 54, 55, 55, 56, 57, 57, 58, 58, 59, 60, 60, 61, 62, 62,
        63, 63, 64, 65, 65, 66, 67, 68, 69, 71, 72, 73, 74, 75, 76,
        78, 79, 80, 81, 82, 84, 85, 86, 87, 88, 90, 91, 92, 93, 94,
        95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 107, 108,
        109, 110, 111, 112, 113, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124,
        124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138,
        139, 140, 141, 142, 143, 144, 145, 146, 146, 147, 148, 149, 150, 151, 152,
        153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 163, 164, 165, 166,
        167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181,
        182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196,
        197, 199, 200, 201, 202, 203, 205, 206, 207, 208, 210, 211, 212, 214, 215,
        216, 218, 219, 221, 222, 224, 225, 227, 229, 230, 232, 234, 235, 237, 239,
        241};
*/

static int gf_high = 2000;
static int gf_low  = 400;
static int kf_high = 5000;
static int kf_low  = 400;

static int get_active_quality(int q, int gfu_boost, int low, int high, int *low_motion_minq,
                              int *high_motion_minq) {
    if (gfu_boost > high)
        return low_motion_minq[q];
    else if (gfu_boost < low)
        return high_motion_minq[q];
    else {
        const int gap        = high - low;
        const int offset     = high - gfu_boost;
        const int qdiff      = high_motion_minq[q] - low_motion_minq[q];
        const int adjustment = ((offset * qdiff) + (gap >> 1)) / gap;
        return low_motion_minq[q] + adjustment;
    }
}
static int get_kf_active_quality_cqp(const RATE_CONTROL *const rc, int q,
    AomBitDepth bit_depth) {
    int *kf_low_motion_minq_cqp;
    int *kf_high_motion_minq_cqp;
    ASSIGN_MINQ_TABLE(bit_depth, kf_low_motion_minq_cqp);
    ASSIGN_MINQ_TABLE(bit_depth, kf_high_motion_minq_cqp);
    return get_active_quality(q, rc->kf_boost, kf_low, kf_high,
        kf_low_motion_minq_cqp, kf_high_motion_minq_cqp);
}
static int get_kf_active_quality(const RATE_CONTROL *const rc, int q, AomBitDepth bit_depth) {
    int *kf_low_motion_minq;
    int *kf_high_motion_minq;
    ASSIGN_MINQ_TABLE(bit_depth, kf_low_motion_minq);
    ASSIGN_MINQ_TABLE(bit_depth, kf_high_motion_minq);
    return get_active_quality(
        q, rc->kf_boost, kf_low, kf_high, kf_low_motion_minq, kf_high_motion_minq);
}

static int get_gf_active_quality(const RATE_CONTROL *const rc, int q, AomBitDepth bit_depth) {
    int *arfgf_low_motion_minq;
    int *arfgf_high_motion_minq;
    ASSIGN_MINQ_TABLE(bit_depth, arfgf_low_motion_minq);
    ASSIGN_MINQ_TABLE(bit_depth, arfgf_high_motion_minq);
    return get_active_quality(
        q, rc->gfu_boost, gf_low, gf_high, arfgf_low_motion_minq, arfgf_high_motion_minq);
}

static int get_gf_high_motion_quality(int q, AomBitDepth bit_depth) {
    int *arfgf_high_motion_minq;
    ASSIGN_MINQ_TABLE(bit_depth, arfgf_high_motion_minq);
    return arfgf_high_motion_minq[q];
}

#if CUTREE_LA
int16_t eb_av1_dc_quant_qtx(int32_t qindex, int32_t delta, AomBitDepth bit_depth);

static int get_kf_boost_from_r0(double r0, int frames_to_key) {
    double factor = sqrt((double)frames_to_key);
    factor = AOMMIN(factor, 10.0);
    factor = AOMMAX(factor, 4.0);
    const int boost = (int)rint((75.0 + 14.0 * factor) / r0);
    return boost;
}

int combine_prior_with_tpl_boost(int prior_boost, int tpl_boost,
    int frames_to_key) {
    double factor = sqrt((double)frames_to_key);
    factor = AOMMIN(factor, 12.0);
    factor = AOMMAX(factor, 4.0);
    factor -= 4.0;
    int boost = (int)((factor * prior_boost + (8.0 - factor) * tpl_boost) / 8.0);
    return boost;
}

int av1_get_deltaq_offset(AomBitDepth bit_depth, int qindex, double beta) {
    assert(beta > 0.0);
    int q = eb_av1_dc_quant_qtx(qindex, 0, bit_depth);
    int newq = (int)rint(q / sqrt(beta));
    int orig_qindex = qindex;
    if (newq < q) {
        do {
            qindex--;
            q = eb_av1_dc_quant_qtx(qindex, 0, bit_depth);
        } while (newq < q && qindex > 0);
    }
    else {
        do {
            qindex++;
            q = eb_av1_dc_quant_qtx(qindex, 0, bit_depth);
        } while (newq > q && qindex < MAXQ);
    }
    return qindex - orig_qindex;
}

int av1_get_adaptive_rdmult(AomBitDepth bit_depth, int base_qindex, double beta) {
  assert(beta > 0.0);
  int64_t q = eb_av1_dc_quant_qtx(base_qindex, 0, bit_depth);
  int64_t rdmult = 0;

  switch (bit_depth) {
    case AOM_BITS_8: rdmult = (int)((88 * q * q / beta) / 24); break;
    case AOM_BITS_10:
      rdmult = ROUND_POWER_OF_TWO((int)((88 * q * q / beta) / 24), 4);
      break;
    default:
      assert(bit_depth == AOM_BITS_12);
      rdmult = ROUND_POWER_OF_TWO((int)((88 * q * q / beta) / 24), 8);
      break;
  }

  return (int)rdmult;
}

static const int rd_boost_factor[16] = { 64, 32, 32, 32, 24, 16, 12, 12,
                                         8,  8,  4,  4,  2,  2,  1,  0 };
static const int rd_frame_type_factor[FRAME_UPDATE_TYPES] = { 128, 144, 128, 128, 144,
                                                              128, 128, 128, //0, 0, 0,
                                                              144, 128, };
#endif

/******************************************************
 * adaptive_qindex_calc_two_pass
 * assigns the q_index per frame using average reference area per frame.
 * used in the second pass of two pass encoding
 ******************************************************/
static int adaptive_qindex_calc_two_pass(PictureControlSet *pcs_ptr, RATE_CONTROL *rc, int qindex) {
    SequenceControlSet *scs_ptr              = pcs_ptr->parent_pcs_ptr->scs_ptr;
    const int           cq_level             = qindex;
    int                 active_best_quality  = 0;
    int                 active_worst_quality = qindex;
    rc->arf_q                                = 0;
    int q;
    int is_src_frame_alt_ref, refresh_golden_frame, refresh_alt_ref_frame, is_intrl_arf_boost,
        rf_level, update_type;
#if CUTREE_LA && CUTREE_LA_QPS
    EbBool enable_tpl = scs_ptr->static_config.look_ahead_distance != 0 &&
                        scs_ptr->static_config.enable_cutree_in_la &&
                        pcs_ptr->parent_pcs_ptr->temporal_layer_index == 0 &&
                        pcs_ptr->parent_pcs_ptr->picture_number<48;
#endif
    is_src_frame_alt_ref  = 0;
    refresh_golden_frame  = frame_is_intra_only(pcs_ptr->parent_pcs_ptr) ? 1 : 0;
    refresh_alt_ref_frame = (pcs_ptr->parent_pcs_ptr->temporal_layer_index == 0) ? 1 : 0;
    is_intrl_arf_boost    = (pcs_ptr->parent_pcs_ptr->temporal_layer_index > 0 &&
                          pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag)
                             ? 1
                             : 0;
    rf_level =
        (frame_is_intra_only(pcs_ptr->parent_pcs_ptr))
            ? KF_STD
            : (pcs_ptr->parent_pcs_ptr->temporal_layer_index == 0)
                  ? GF_ARF_STD
                  : pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag ? GF_ARF_LOW : INTER_NORMAL;

    update_type = (frame_is_intra_only(pcs_ptr->parent_pcs_ptr))
                      ? KF_UPDATE
                      : (pcs_ptr->parent_pcs_ptr->temporal_layer_index == 0)
                            ? ARF_UPDATE
                            : pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag ? INTNL_ARF_UPDATE
                                                                                 : LF_UPDATE;
    const int bit_depth = scs_ptr->static_config.encoder_bit_depth;
    // Since many frames can be processed at the same time, storing/using arf_q in rc param is not sufficient and will create a run to run.
    // So, for each frame, arf_q is updated based on the qp of its references.
    rc->arf_q = MAX(rc->arf_q, ((pcs_ptr->ref_pic_qp_array[0][0] << 2) + 2));
    if (pcs_ptr->slice_type == B_SLICE)
        rc->arf_q = MAX(rc->arf_q, ((pcs_ptr->ref_pic_qp_array[1][0] << 2) + 2));
    uint64_t referenced_area_avg = pcs_ptr->parent_pcs_ptr->referenced_area_avg;
    uint64_t referenced_area_max = 64;

    if (frame_is_intra_only(pcs_ptr->parent_pcs_ptr)) {
        // Not forced keyframe.
        double q_adj_factor = 1.0;
        double q_val;
        rc->worst_quality   = MAXQ;
        rc->best_quality    = MINQ;
        referenced_area_max = MAX_REF_AREA_I;

        if (referenced_area_avg <= 16) referenced_area_avg = 0;
        // cross multiplication to derive kf_boost from referenced area; kf_boost range is [kf_low,kf_high], and referenced range [0,referenced_area_max]
        rc->kf_boost =
            (int)((referenced_area_avg * (kf_high - kf_low)) / referenced_area_max) + kf_low;

#if CUTREE_LA && CUTREE_LA_QPS
        if(enable_tpl) {
            int frames_to_key = 60;
            const int new_kf_boost = get_kf_boost_from_r0(pcs_ptr->parent_pcs_ptr->r0, frames_to_key);

            printf("old kf boost %d new kf boost %d [%d] poc=%ld r0=%f\n", rc->kf_boost, new_kf_boost, frames_to_key, pcs_ptr->parent_pcs_ptr->picture_number, pcs_ptr->parent_pcs_ptr->r0);

            rc->kf_boost = combine_prior_with_tpl_boost(rc->kf_boost, new_kf_boost, frames_to_key);
        }
#endif
        // Baseline value derived from cpi->active_worst_quality and kf boost.
        active_best_quality = get_kf_active_quality(rc, active_worst_quality, bit_depth);
#if QPS_UPDATE
        // Allow somewhat lower kf minq with small image formats.
        if ((pcs_ptr->parent_pcs_ptr->av1_cm->frm_size.frame_width *
             pcs_ptr->parent_pcs_ptr->av1_cm->frm_size.frame_height) <= (352 * 288))
            q_adj_factor -= 0.25;
#endif
        // Make a further adjustment based on the kf zero motion measure.
        q_adj_factor +=
            0.05 - (0.001 * (double)pcs_ptr->parent_pcs_ptr
                                ->kf_zeromotion_pct /*(double)cpi->twopass.kf_zeromotion_pct*/);

        // Convert the adjustment factor to a qindex delta
        // on active_best_quality.
        q_val = eb_av1_convert_qindex_to_q(active_best_quality, bit_depth);
        active_best_quality += eb_av1_compute_qdelta(q_val, q_val * q_adj_factor, bit_depth);
    } else if (!is_src_frame_alt_ref &&
               (refresh_golden_frame || is_intrl_arf_boost || refresh_alt_ref_frame)) {
        referenced_area_max = scs_ptr->input_resolution < 2
                                  ? MAX_REF_AREA_NONI_LOW_RES
                                  : ((int)referenced_area_avg -
                                         (int)pcs_ptr->ref_pic_referenced_area_avg_array[0][0] >=
                                     REF_AREA_DIF_THRESHOLD)
                                        ? MAX_REF_AREA_NONI_LOW_RES
                                        : MAX_REF_AREA_NONI;

        // Clip the complexity of highly complex pictures to maximum.
#if QPS_UPDATE
        if (pcs_ptr->parent_pcs_ptr->qp_scaling_average_complexity > HIGH_QPS_COMP_THRESHOLD &&
            referenced_area_avg < 20)
#else
        if (pcs_ptr->parent_pcs_ptr->qp_scaling_average_complexity > HIGH_QPS_COMP_THRESHOLD)
#endif
            referenced_area_avg = 0;

        rc->arf_boost_factor =
            ((int)referenced_area_avg - (int)pcs_ptr->ref_pic_referenced_area_avg_array[0][0] >=
                 REF_AREA_DIF_THRESHOLD &&
             referenced_area_avg > 20 && pcs_ptr->ref_pic_referenced_area_avg_array[0][0] <= 20)
                ? (float_t)1.3
                : (float_t)1;
        rc->gfu_boost =
            (int)(((referenced_area_avg) * (gf_high - gf_low)) / referenced_area_max) + gf_low;

#if CUTREE_LA && CUTREE_LA_QPS
        if(enable_tpl) {
            int frames_to_key = 60 - pcs_ptr->parent_pcs_ptr->picture_number + 15;
            const int new_gfu_boost = (int)(200.0 / pcs_ptr->parent_pcs_ptr->r0);
            rc->arf_boost_factor = 1;

            printf("old gfu boost %d new gfu boost %d [%d] poc=%ld r0=%f\n", rc->gfu_boost, new_gfu_boost, frames_to_key, pcs_ptr->parent_pcs_ptr->picture_number, pcs_ptr->parent_pcs_ptr->r0);

            rc->gfu_boost = combine_prior_with_tpl_boost(rc->gfu_boost, new_gfu_boost, frames_to_key);
        }
#endif
        q = active_worst_quality;

        // non ref frame or repeated frames with re-encode
        if (!refresh_alt_ref_frame && !is_intrl_arf_boost)
            active_best_quality = cq_level;
        else {
            // base layer
            if (update_type == ARF_UPDATE) {
                active_best_quality = get_gf_active_quality(rc, q, bit_depth);
                rc->arf_q           = active_best_quality;
                const int min_boost = get_gf_high_motion_quality(q, bit_depth);
                const int boost     = min_boost - active_best_quality;

                active_best_quality = min_boost - (int)(boost * rc->arf_boost_factor);
                if (pcs_ptr->parent_pcs_ptr->sad_me / pcs_ptr->sb_total_count / 256 <
                    ME_SAD_LOW_THRESHOLD1)
                    active_best_quality = active_best_quality * 130 / 100;
                else if (pcs_ptr->parent_pcs_ptr->sad_me / pcs_ptr->sb_total_count / 256 <
                         ME_SAD_LOW_THRESHOLD2)
                    active_best_quality = active_best_quality * 115 / 100;
            } else
                active_best_quality = rc->arf_q;
            // active_best_quality is updated with the q index of the reference
            if (rf_level == GF_ARF_LOW)
                active_best_quality = (active_best_quality + cq_level + 1) / 2;
        }
    } else
        active_best_quality = cq_level;
    q = active_best_quality;
    clamp(q, active_best_quality, active_worst_quality);

    return q;
}

#define DEFAULT_KF_BOOST 2700
#define DEFAULT_GF_BOOST 1350
/******************************************************
 * cqp_qindex_calc
 * Assign the q_index per frame.
 * Used in the one pass encoding with no look ahead
 ******************************************************/
static int cqp_qindex_calc(
    PictureControlSet         *pcs_ptr,
    RATE_CONTROL                *rc,
    int                          qindex) {
    SequenceControlSet        *scs_ptr = pcs_ptr->parent_pcs_ptr->scs_ptr;
    const Av1Common  *const cm = pcs_ptr->parent_pcs_ptr->av1_cm;

    int active_best_quality = 0;
    int active_worst_quality = qindex;
    double q_val;
    int q;
    const int bit_depth = scs_ptr->static_config.encoder_bit_depth;
    // Since many frames can be processed at the same time, storing/using arf_q in rc param is not sufficient and will create a run to run.
    // So, for each frame, arf_q is updated based on the qp of its references.
    rc->arf_q = 0;
    if (pcs_ptr->ref_slice_type_array[0][0] != I_SLICE)
        rc->arf_q = MAX(rc->arf_q, ((pcs_ptr->ref_pic_qp_array[0][0] << 2) + 2));
    if ((pcs_ptr->slice_type == B_SLICE) && (pcs_ptr->ref_slice_type_array[1][0] != I_SLICE))
        rc->arf_q = MAX(rc->arf_q, ((pcs_ptr->ref_pic_qp_array[1][0] << 2) + 2));

    if (frame_is_intra_only(pcs_ptr->parent_pcs_ptr)) {
        // Not forced keyframe.
        double q_adj_factor = 1.0;

        rc->worst_quality = MAXQ;
        rc->best_quality = MINQ;

        // cross multiplication to derive kf_boost from non_moving_average_score; kf_boost range is [kf_low,kf_high], and non_moving_average_score range [0,max_qp_scaling_avg_comp_I]
        rc->kf_boost = DEFAULT_KF_BOOST;
        // Baseline value derived from cpi->active_worst_quality and kf boost.
        active_best_quality =
            get_kf_active_quality_cqp(rc, active_worst_quality, bit_depth);
        // Allow somewhat lower kf minq with small image formats.
        if ((cm->frm_size.frame_width * cm->frm_size.frame_height) <= (352 * 288))
            q_adj_factor -= 0.25;

        // Convert the adjustment factor to a qindex delta
        // on active_best_quality.
        q_val = eb_av1_convert_qindex_to_q(active_best_quality, bit_depth);
        active_best_quality +=
            eb_av1_compute_qdelta(q_val, q_val * q_adj_factor, bit_depth);
    }
    else{
        const  double delta_rate_new[7][6] =
        {
            { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 }, // 1L
            { 0.6, 1.0, 1.0, 1.0, 1.0, 1.0 }, // 2L
            { 0.6, 0.8, 1.0, 1.0, 1.0, 1.0 }, // 3L
            { 0.6 , 0.8, 0.9, 1.0, 1.0, 1.0 }, // 4L
            { 0.35, 0.6, 0.8,  0.9, 1.0, 1.0},  //5L
            { 0.35, 0.6, 0.8,  0.9, 0.95, 1.0}  //6L
        };
        q_val = eb_av1_convert_qindex_to_q(qindex, bit_depth);
        const int32_t delta_qindex = eb_av1_compute_qdelta(
            q_val,
            q_val * delta_rate_new[pcs_ptr->parent_pcs_ptr->hierarchical_levels]
            [pcs_ptr->parent_pcs_ptr->temporal_layer_index],
            bit_depth);
#if QPS_UPDATE
        if (scs_ptr->use_input_stat_file && pcs_ptr->parent_pcs_ptr->frames_in_sw < QPS_SW_THRESH)
            active_best_quality =
                MAX((int32_t)(qindex + delta_qindex), (pcs_ptr->ref_pic_qp_array[0][0] << 2) + 2);
        else
            active_best_quality = (int32_t)(qindex + delta_qindex);
#else
        active_best_quality = (int32_t)(qindex + delta_qindex);
#endif
    }
    q = active_best_quality;
    clamp(q, active_best_quality, active_worst_quality);

    return q;
}
/******************************************************
 * sb_qp_derivation_two_pass
 * Calculates the QP per SB based on the referenced area
 * used in the second pass of two pass encoding
 ******************************************************/
static void sb_qp_derivation_two_pass(PictureControlSet *pcs_ptr) {
    SequenceControlSet *scs_ptr = pcs_ptr->parent_pcs_ptr->scs_ptr;
    SuperBlock *        sb_ptr;
    uint32_t            sb_addr;
#if QP2QINDEX
    uint8_t             sb_qp;
#endif

    pcs_ptr->parent_pcs_ptr->average_qp = 0;
    if (scs_ptr->use_input_stat_file && pcs_ptr->temporal_layer_index <= 0)
        pcs_ptr->parent_pcs_ptr->frm_hdr.delta_q_params.delta_q_present = 1;
    else
        pcs_ptr->parent_pcs_ptr->frm_hdr.delta_q_params.delta_q_present = 0;

    if (pcs_ptr->parent_pcs_ptr->frm_hdr.delta_q_params.delta_q_present) {
        const int bit_depth            = scs_ptr->static_config.encoder_bit_depth;
        int       active_worst_quality = quantizer_to_qindex[(uint8_t)scs_ptr->static_config.qp];
        int *     kf_low_motion_minq;
        int *     kf_high_motion_minq;
        ASSIGN_MINQ_TABLE(bit_depth, kf_low_motion_minq);
        ASSIGN_MINQ_TABLE(bit_depth, kf_high_motion_minq);

        uint32_t me_sb_size = scs_ptr->sb_sz;
        uint32_t me_pic_width_in_sb =
            (pcs_ptr->parent_pcs_ptr->aligned_width + scs_ptr->sb_sz - 1) / me_sb_size;
        uint32_t me_pic_height_in_sb =
            (pcs_ptr->parent_pcs_ptr->aligned_height + me_sb_size - 1) / me_sb_size;

        int *arfgf_low_motion_minq;
        int *arfgf_high_motion_minq;
        ASSIGN_MINQ_TABLE(bit_depth, arfgf_low_motion_minq);
        ASSIGN_MINQ_TABLE(bit_depth, arfgf_high_motion_minq);

        int max_delta_qp = (pcs_ptr->slice_type == 2)
                               ? ((kf_high_motion_minq[active_worst_quality] -
                                   kf_low_motion_minq[active_worst_quality] + 2) >>
                                  2) /
                                     2
                               : ((arfgf_high_motion_minq[active_worst_quality] -
                                   arfgf_low_motion_minq[active_worst_quality] + 2) >>
                                  2) /
                                     2;

        for (sb_addr = 0; sb_addr < pcs_ptr->sb_total_count_pix; ++sb_addr) {
            sb_ptr            = pcs_ptr->sb_ptr_array[sb_addr];
            int      delta_qp = 0;
            uint16_t variance_sb;
            uint32_t referenced_area_sb, me_distortion;

            if (scs_ptr->seq_header.sb_size == BLOCK_128X128) {
                uint32_t me_sb_x      = (sb_ptr->origin_x / me_sb_size);
                uint32_t me_sb_y      = (sb_ptr->origin_y / me_sb_size);
                uint32_t me_sb_addr_0 = me_sb_x + me_sb_y * me_pic_width_in_sb;
                uint32_t me_sb_addr_1 = (me_sb_x + 1) < me_pic_width_in_sb
                                            ? (me_sb_x + 1) + ((me_sb_y + 0) * me_pic_width_in_sb)
                                            : me_sb_addr_0;
                uint32_t me_sb_addr_2 = (me_sb_y + 1) < me_pic_height_in_sb
                                            ? (me_sb_x + 0) + ((me_sb_y + 1) * me_pic_width_in_sb)
                                            : me_sb_addr_0;
                uint32_t me_sb_addr_3 =
                    ((me_sb_x + 1) < me_pic_width_in_sb) && ((me_sb_y + 1) < me_pic_height_in_sb)
                        ? (me_sb_x + 1) + ((me_sb_y + 1) * me_pic_width_in_sb)
                        : me_sb_addr_0;

                variance_sb =
                    (pcs_ptr->parent_pcs_ptr->variance[me_sb_addr_0][ME_TIER_ZERO_PU_64x64] +
                     pcs_ptr->parent_pcs_ptr->variance[me_sb_addr_1][ME_TIER_ZERO_PU_64x64] +
                     pcs_ptr->parent_pcs_ptr->variance[me_sb_addr_2][ME_TIER_ZERO_PU_64x64] +
                     pcs_ptr->parent_pcs_ptr->variance[me_sb_addr_3][ME_TIER_ZERO_PU_64x64] + 2) >>
                    2;

                referenced_area_sb =
                    (pcs_ptr->parent_pcs_ptr->stat_struct.referenced_area[me_sb_addr_0] /
                         pcs_ptr->parent_pcs_ptr->sb_params_array[me_sb_addr_0].width /
                         pcs_ptr->parent_pcs_ptr->sb_params_array[me_sb_addr_0].height +
                     pcs_ptr->parent_pcs_ptr->stat_struct.referenced_area[me_sb_addr_1] /
                         pcs_ptr->parent_pcs_ptr->sb_params_array[me_sb_addr_1].width /
                         pcs_ptr->parent_pcs_ptr->sb_params_array[me_sb_addr_1].height +
                     pcs_ptr->parent_pcs_ptr->stat_struct.referenced_area[me_sb_addr_2] /
                         pcs_ptr->parent_pcs_ptr->sb_params_array[me_sb_addr_2].width /
                         pcs_ptr->parent_pcs_ptr->sb_params_array[me_sb_addr_2].height +
                     pcs_ptr->parent_pcs_ptr->stat_struct.referenced_area[me_sb_addr_3] /
                         pcs_ptr->parent_pcs_ptr->sb_params_array[me_sb_addr_3].width /
                         pcs_ptr->parent_pcs_ptr->sb_params_array[me_sb_addr_3].height +
                     2) >>
                    2;
                me_distortion = (pcs_ptr->parent_pcs_ptr->rc_me_distortion[me_sb_addr_0] +
                                 pcs_ptr->parent_pcs_ptr->rc_me_distortion[me_sb_addr_1] +
                                 pcs_ptr->parent_pcs_ptr->rc_me_distortion[me_sb_addr_2] +
                                 pcs_ptr->parent_pcs_ptr->rc_me_distortion[me_sb_addr_3] + 2) >>
                                2;
                me_distortion >>= 8;
            } else {
                variance_sb = pcs_ptr->parent_pcs_ptr->variance[sb_addr][ME_TIER_ZERO_PU_64x64];
                referenced_area_sb = pcs_ptr->parent_pcs_ptr->stat_struct.referenced_area[sb_addr] /
                                     pcs_ptr->parent_pcs_ptr->sb_params_array[sb_addr].width /
                                     pcs_ptr->parent_pcs_ptr->sb_params_array[sb_addr].height;
                me_distortion = pcs_ptr->parent_pcs_ptr->rc_me_distortion[sb_addr] >> 8;
            }
            delta_qp = 0;

            if (pcs_ptr->slice_type == 2) {
                referenced_area_sb =
                    MIN(REF_AREA_MED_THRESHOLD + REF_AREA_LOW_THRESHOLD, referenced_area_sb);
                if (referenced_area_sb >= REF_AREA_MED_THRESHOLD)
                    delta_qp = -(max_delta_qp * ((int)referenced_area_sb - REF_AREA_MED_THRESHOLD) /
                                 (REF_AREA_MED_THRESHOLD));
                else
                    delta_qp = max_delta_qp;

                if (delta_qp < 0 && variance_sb < IS_COMPLEX_SB_FLAT_VARIANCE_TH) delta_qp = 0;
            } else if (pcs_ptr->temporal_layer_index == 0) {
                if (referenced_area_sb < REF_AREA_LOW_THRESHOLD) delta_qp = max_delta_qp >> 1;
            }

            if (pcs_ptr->slice_type == 2)
#if QP2QINDEX
                sb_qp =
#else
                sb_ptr->qp =
#endif
                    CLIP3(MIN(pcs_ptr->parent_pcs_ptr->picture_qp,
                              ((kf_low_motion_minq[active_worst_quality] + 2) >> 2)),
                          MAX(pcs_ptr->parent_pcs_ptr->picture_qp,
                              ((kf_high_motion_minq[active_worst_quality] + 2) >> 2)) +
                              3,
                          ((int16_t)pcs_ptr->parent_pcs_ptr->picture_qp + (int16_t)delta_qp));
            else
#if QP2QINDEX
                sb_qp =
#else
                sb_ptr->qp =
#endif
                    CLIP3(MIN(pcs_ptr->parent_pcs_ptr->picture_qp,
                              ((arfgf_low_motion_minq[active_worst_quality] + 2) >> 2)) -
                              1,
                          MAX(pcs_ptr->parent_pcs_ptr->picture_qp,
                              ((arfgf_high_motion_minq[active_worst_quality] + 2) >> 2)) +
                              3,
                          ((int16_t)pcs_ptr->parent_pcs_ptr->picture_qp + (int16_t)delta_qp));

#if QP2QINDEX
            sb_qp       = CLIP3(scs_ptr->static_config.min_qp_allowed,
                               scs_ptr->static_config.max_qp_allowed,
                               sb_qp);
            sb_ptr->qindex = quantizer_to_qindex[sb_qp];
            pcs_ptr->parent_pcs_ptr->average_qp += sb_qp;
#else
            sb_ptr->qp       = CLIP3(scs_ptr->static_config.min_qp_allowed,
                               scs_ptr->static_config.max_qp_allowed,
                               sb_ptr->qp);
            sb_ptr->delta_qp = (int)pcs_ptr->parent_pcs_ptr->picture_qp - (int)sb_ptr->qp;
            pcs_ptr->parent_pcs_ptr->average_qp += sb_ptr->qp;
#endif
        }
    } else {
        for (sb_addr = 0; sb_addr < pcs_ptr->sb_total_count_pix; ++sb_addr) {
            sb_ptr           = pcs_ptr->sb_ptr_array[sb_addr];
#if QP2QINDEX
            sb_qp            = (uint8_t)pcs_ptr->picture_qp;
            sb_ptr->qindex = quantizer_to_qindex[sb_qp];
            pcs_ptr->parent_pcs_ptr->average_qp += sb_qp;
#else
            sb_ptr->qp       = (uint8_t)pcs_ptr->picture_qp;
            sb_ptr->delta_qp = 0;
            pcs_ptr->parent_pcs_ptr->average_qp += sb_ptr->qp;
#endif
        }
    }
}

#if CUTREE_LA
#if LAMBDA_SCALING
//TODO: refine these
static const int16_t dc_qlookup_QTX[QINDEX_RANGE] = {
  4,    8,    8,    9,    10,  11,  12,  12,  13,  14,  15,   16,   17,   18,
  19,   19,   20,   21,   22,  23,  24,  25,  26,  26,  27,   28,   29,   30,
  31,   32,   32,   33,   34,  35,  36,  37,  38,  38,  39,   40,   41,   42,
  43,   43,   44,   45,   46,  47,  48,  48,  49,  50,  51,   52,   53,   53,
  54,   55,   56,   57,   57,  58,  59,  60,  61,  62,  62,   63,   64,   65,
  66,   66,   67,   68,   69,  70,  70,  71,  72,  73,  74,   74,   75,   76,
  77,   78,   78,   79,   80,  81,  81,  82,  83,  84,  85,   85,   87,   88,
  90,   92,   93,   95,   96,  98,  99,  101, 102, 104, 105,  107,  108,  110,
  111,  113,  114,  116,  117, 118, 120, 121, 123, 125, 127,  129,  131,  134,
  136,  138,  140,  142,  144, 146, 148, 150, 152, 154, 156,  158,  161,  164,
  166,  169,  172,  174,  177, 180, 182, 185, 187, 190, 192,  195,  199,  202,
  205,  208,  211,  214,  217, 220, 223, 226, 230, 233, 237,  240,  243,  247,
  250,  253,  257,  261,  265, 269, 272, 276, 280, 284, 288,  292,  296,  300,
  304,  309,  313,  317,  322, 326, 330, 335, 340, 344, 349,  354,  359,  364,
  369,  374,  379,  384,  389, 395, 400, 406, 411, 417, 423,  429,  435,  441,
  447,  454,  461,  467,  475, 482, 489, 497, 505, 513, 522,  530,  539,  549,
  559,  569,  579,  590,  602, 614, 626, 640, 654, 668, 684,  700,  717,  736,
  755,  775,  796,  819,  843, 869, 896, 925, 955, 988, 1022, 1058, 1098, 1139,
  1184, 1232, 1282, 1336,
};

static const int16_t dc_qlookup_10_QTX[QINDEX_RANGE] = {
  4,    9,    10,   13,   15,   17,   20,   22,   25,   28,   31,   34,   37,
  40,   43,   47,   50,   53,   57,   60,   64,   68,   71,   75,   78,   82,
  86,   90,   93,   97,   101,  105,  109,  113,  116,  120,  124,  128,  132,
  136,  140,  143,  147,  151,  155,  159,  163,  166,  170,  174,  178,  182,
  185,  189,  193,  197,  200,  204,  208,  212,  215,  219,  223,  226,  230,
  233,  237,  241,  244,  248,  251,  255,  259,  262,  266,  269,  273,  276,
  280,  283,  287,  290,  293,  297,  300,  304,  307,  310,  314,  317,  321,
  324,  327,  331,  334,  337,  343,  350,  356,  362,  369,  375,  381,  387,
  394,  400,  406,  412,  418,  424,  430,  436,  442,  448,  454,  460,  466,
  472,  478,  484,  490,  499,  507,  516,  525,  533,  542,  550,  559,  567,
  576,  584,  592,  601,  609,  617,  625,  634,  644,  655,  666,  676,  687,
  698,  708,  718,  729,  739,  749,  759,  770,  782,  795,  807,  819,  831,
  844,  856,  868,  880,  891,  906,  920,  933,  947,  961,  975,  988,  1001,
  1015, 1030, 1045, 1061, 1076, 1090, 1105, 1120, 1137, 1153, 1170, 1186, 1202,
  1218, 1236, 1253, 1271, 1288, 1306, 1323, 1342, 1361, 1379, 1398, 1416, 1436,
  1456, 1476, 1496, 1516, 1537, 1559, 1580, 1601, 1624, 1647, 1670, 1692, 1717,
  1741, 1766, 1791, 1817, 1844, 1871, 1900, 1929, 1958, 1990, 2021, 2054, 2088,
  2123, 2159, 2197, 2236, 2276, 2319, 2363, 2410, 2458, 2508, 2561, 2616, 2675,
  2737, 2802, 2871, 2944, 3020, 3102, 3188, 3280, 3375, 3478, 3586, 3702, 3823,
  3953, 4089, 4236, 4394, 4559, 4737, 4929, 5130, 5347,
};

static const int16_t dc_qlookup_12_QTX[QINDEX_RANGE] = {
  4,     12,    18,    25,    33,    41,    50,    60,    70,    80,    91,
  103,   115,   127,   140,   153,   166,   180,   194,   208,   222,   237,
  251,   266,   281,   296,   312,   327,   343,   358,   374,   390,   405,
  421,   437,   453,   469,   484,   500,   516,   532,   548,   564,   580,
  596,   611,   627,   643,   659,   674,   690,   706,   721,   737,   752,
  768,   783,   798,   814,   829,   844,   859,   874,   889,   904,   919,
  934,   949,   964,   978,   993,   1008,  1022,  1037,  1051,  1065,  1080,
  1094,  1108,  1122,  1136,  1151,  1165,  1179,  1192,  1206,  1220,  1234,
  1248,  1261,  1275,  1288,  1302,  1315,  1329,  1342,  1368,  1393,  1419,
  1444,  1469,  1494,  1519,  1544,  1569,  1594,  1618,  1643,  1668,  1692,
  1717,  1741,  1765,  1789,  1814,  1838,  1862,  1885,  1909,  1933,  1957,
  1992,  2027,  2061,  2096,  2130,  2165,  2199,  2233,  2267,  2300,  2334,
  2367,  2400,  2434,  2467,  2499,  2532,  2575,  2618,  2661,  2704,  2746,
  2788,  2830,  2872,  2913,  2954,  2995,  3036,  3076,  3127,  3177,  3226,
  3275,  3324,  3373,  3421,  3469,  3517,  3565,  3621,  3677,  3733,  3788,
  3843,  3897,  3951,  4005,  4058,  4119,  4181,  4241,  4301,  4361,  4420,
  4479,  4546,  4612,  4677,  4742,  4807,  4871,  4942,  5013,  5083,  5153,
  5222,  5291,  5367,  5442,  5517,  5591,  5665,  5745,  5825,  5905,  5984,
  6063,  6149,  6234,  6319,  6404,  6495,  6587,  6678,  6769,  6867,  6966,
  7064,  7163,  7269,  7376,  7483,  7599,  7715,  7832,  7958,  8085,  8214,
  8352,  8492,  8635,  8788,  8945,  9104,  9275,  9450,  9639,  9832,  10031,
  10245, 10465, 10702, 10946, 11210, 11482, 11776, 12081, 12409, 12750, 13118,
  13501, 13913, 14343, 14807, 15290, 15812, 16356, 16943, 17575, 18237, 18949,
  19718, 20521, 21387,
};


static int16_t av1_dc_quant_qtx(int qindex, int delta, AomBitDepth bit_depth) {
  const int q_clamped = clamp(qindex + delta, 0, MAXQ);
  switch (bit_depth) {
    case AOM_BITS_8: return dc_qlookup_QTX[q_clamped];
    case AOM_BITS_10: return dc_qlookup_10_QTX[q_clamped];
    case AOM_BITS_12: return dc_qlookup_12_QTX[q_clamped];
    default:
      assert(0 && "bit_depth should be AOM_BITS_8, AOM_BITS_10 or AOM_BITS_12");
      return -1;
  }
}

static int av1_compute_rd_mult_based_on_qindex(AomBitDepth bit_depth, int qindex) {
  const int q = av1_dc_quant_qtx(qindex, 0, bit_depth);
  //const int q = eb_av1_dc_quant_Q3(qindex, 0, bit_depth);
  int rdmult = q * q;
  rdmult = rdmult * 3 + (rdmult * 2 / 3);
  switch (bit_depth) {
    case AOM_BITS_8: break;
    case AOM_BITS_10: rdmult = ROUND_POWER_OF_TWO(rdmult, 4); break;
    case AOM_BITS_12: rdmult = ROUND_POWER_OF_TWO(rdmult, 8); break;
    default:
      assert(0 && "bit_depth should be AOM_BITS_8, AOM_BITS_10 or AOM_BITS_12");
      return -1;
  }
  return rdmult > 0 ? rdmult : 1;
}

static int av1_compute_rd_mult(AomBitDepth bit_depth, int qindex) {
  int64_t rdmult = av1_compute_rd_mult_based_on_qindex(bit_depth, qindex);
  /*
  if (is_stat_consumption_stage(cpi) &&
      (cpi->common.current_frame.frame_type != KEY_FRAME)) {
    const GF_GROUP *const gf_group = &cpi->gf_group;
    const FRAME_UPDATE_TYPE frame_type = gf_group->update_type[gf_group->index];
    const int boost_index = AOMMIN(15, (cpi->rc.gfu_boost / 100));

    rdmult = (rdmult * rd_frame_type_factor[frame_type]) >> 7;
    rdmult += ((rdmult * rd_boost_factor[boost_index]) >> 7);
  }
  */
  return (int)rdmult;
}

static void sb_setup_lambda(PictureControlSet *pcs_ptr,
        SuperBlock *sb_ptr) {
    const Av1Common  *const cm = pcs_ptr->parent_pcs_ptr->av1_cm;
    PictureParentControlSet   *ppcs_ptr = pcs_ptr->parent_pcs_ptr;
    SequenceControlSet        *scs_ptr = ppcs_ptr->scs_ptr;
    const int bsize_base = BLOCK_16X16;
    const int num_mi_w = mi_size_wide[bsize_base];
    const int num_mi_h = mi_size_high[bsize_base];
    const int num_cols = (cm->mi_cols + num_mi_w - 1) / num_mi_w;
    const int num_rows = (cm->mi_rows + num_mi_h - 1) / num_mi_h;
    const int num_bcols = (mi_size_wide[scs_ptr->seq_header.sb_size] + num_mi_w - 1) / num_mi_w;
    const int num_brows = (mi_size_high[scs_ptr->seq_header.sb_size] + num_mi_h - 1) / num_mi_h;
    int mi_col = sb_ptr->origin_x / 4;
    int mi_row = sb_ptr->origin_y / 4;
    int row, col;

    double base_block_count = 0.0;
    double log_sum = 0.0;

    for (row = mi_row / num_mi_w;
            row < num_rows && row < mi_row / num_mi_w + num_brows; ++row) {
        for (col = mi_col / num_mi_h;
                col < num_cols && col < mi_col / num_mi_h + num_bcols; ++col) {
            const int index = row * num_cols + col;
            log_sum += log(ppcs_ptr->tpl_rdmult_scaling_factors[index]);
            base_block_count += 1.0;
        }
    }

    const int orig_rdmult =
        av1_compute_rd_mult(8, ppcs_ptr->frm_hdr.quantization_params.base_q_idx);
    const int new_rdmult = av1_compute_rd_mult(8, sb_ptr->qindex);
    const double scaling_factor = (double)new_rdmult / (double)orig_rdmult;
    double scale_adj = log(scaling_factor) - log_sum / base_block_count;
    scale_adj = exp(scale_adj);
    int dbg = 0;
    if (pcs_ptr->picture_number == 48 && mi_row == 0 && mi_col == 0 && 0) {
        dbg = 1;
        printf("rdmult_setup_sb (0, 0): base_block_count %f, pic_qindex %d, sb_qindex %d, lambda %d/%d, sb_size %d\n",
                base_block_count, ppcs_ptr->frm_hdr.quantization_params.base_q_idx,
                sb_ptr->qindex, orig_rdmult, new_rdmult,
                scs_ptr->seq_header.sb_size);
    }

    for (row = mi_row / num_mi_w;
            row < num_rows && row < mi_row / num_mi_w + num_brows; ++row) {
        for (col = mi_col / num_mi_h;
                col < num_cols && col < mi_col / num_mi_h + num_bcols; ++col) {
            const int index = row * num_cols + col;
            ppcs_ptr->tpl_sb_rdmult_scaling_factors[index] =
                scale_adj * ppcs_ptr->tpl_rdmult_scaling_factors[index];
            if (dbg) {
                printf("\tindex %d, factor is %f, scale_adj is %f\n",
                        index, ppcs_ptr->tpl_rdmult_scaling_factors[index], scale_adj);
            }
        }
    }
    ppcs_ptr->blk_lambda_tuning = EB_TRUE;
}

#endif
/******************************************************
 * sb_qp_derivation_tpl_la
 * Calculates the QP per SB based on the referenced area
 * used in the second pass of two pass encoding
 ******************************************************/
static void sb_qp_derivation_tpl_la(
    RATE_CONTROL              *rc,
    PictureControlSet         *pcs_ptr) {

    PictureParentControlSet   *ppcs_ptr = pcs_ptr->parent_pcs_ptr;
    SequenceControlSet        *scs_ptr = pcs_ptr->parent_pcs_ptr->scs_ptr;
    const Av1Common           *const cm = pcs_ptr->parent_pcs_ptr->av1_cm;
    SuperBlock                *sb_ptr;
    uint32_t                  sb_addr;
#if QP2QINDEX
    uint8_t sb_qp = 0;
#endif
    int update_type = (frame_is_intra_only(pcs_ptr->parent_pcs_ptr)) ? KF_UPDATE :
        (pcs_ptr->parent_pcs_ptr->temporal_layer_index == 0) ? ARF_UPDATE :
        pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag ? INTNL_ARF_UPDATE : LF_UPDATE;

    pcs_ptr->parent_pcs_ptr->average_qp = 0;
    if (pcs_ptr->temporal_layer_index <= 0)
        pcs_ptr->parent_pcs_ptr->frm_hdr.delta_q_params.delta_q_present = 1;
    else
        pcs_ptr->parent_pcs_ptr->frm_hdr.delta_q_params.delta_q_present = 0;

    if (pcs_ptr->parent_pcs_ptr->frm_hdr.delta_q_params.delta_q_present) {
        const int bit_depth = scs_ptr->static_config.encoder_bit_depth;
        int active_worst_quality = quantizer_to_qindex[(uint8_t)scs_ptr->static_config.qp];
        int *kf_low_motion_minq;
        int *kf_high_motion_minq;
        ASSIGN_MINQ_TABLE(bit_depth, kf_low_motion_minq);
        ASSIGN_MINQ_TABLE(bit_depth, kf_high_motion_minq);

        uint32_t me_sb_size = scs_ptr->sb_sz;
        uint32_t me_pic_width_in_sb = (scs_ptr->seq_header.max_frame_width + scs_ptr->sb_sz - 1) / me_sb_size;
        uint32_t me_pic_height_in_sb = (scs_ptr->seq_header.max_frame_height + me_sb_size - 1) / me_sb_size;

        int *arfgf_low_motion_minq;
        int *arfgf_high_motion_minq;
        ASSIGN_MINQ_TABLE(bit_depth, arfgf_low_motion_minq);
        ASSIGN_MINQ_TABLE(bit_depth, arfgf_high_motion_minq);

#if 0
        int max_delta_qp = (pcs_ptr->slice_type == 2) ?
            ((kf_high_motion_minq[active_worst_quality] - kf_low_motion_minq[active_worst_quality] + 2) >> 2) / 2 :
            ((arfgf_high_motion_minq[active_worst_quality] - arfgf_low_motion_minq[active_worst_quality] + 2) >> 2) / 2;
#endif

        for (sb_addr = 0; sb_addr < scs_ptr->sb_tot_cnt; ++sb_addr) {
            sb_ptr = pcs_ptr->sb_ptr_array[sb_addr];
            int delta_qp = 0;
            uint16_t variance_sb;
            uint32_t me_distortion;

            if (scs_ptr->seq_header.sb_size == BLOCK_128X128) {
                uint32_t me_sb_x = (sb_ptr->origin_x / me_sb_size);
                uint32_t me_sb_y = (sb_ptr->origin_y / me_sb_size);
                uint32_t me_sb_addr_0 = me_sb_x + me_sb_y * me_pic_width_in_sb;
                uint32_t me_sb_addr_1 = (me_sb_x + 1) < me_pic_width_in_sb ? (me_sb_x + 1) + ((me_sb_y + 0) * me_pic_width_in_sb) : me_sb_addr_0;
                uint32_t me_sb_addr_2 = (me_sb_y + 1) < me_pic_height_in_sb ? (me_sb_x + 0) + ((me_sb_y + 1) * me_pic_width_in_sb) : me_sb_addr_0;
                uint32_t me_sb_addr_3 = ((me_sb_x + 1) < me_pic_width_in_sb) && ((me_sb_y + 1) < me_pic_height_in_sb) ?
                    (me_sb_x + 1) + ((me_sb_y + 1) * me_pic_width_in_sb) : me_sb_addr_0;

                variance_sb =
                    (pcs_ptr->parent_pcs_ptr->variance[me_sb_addr_0][ME_TIER_ZERO_PU_64x64] +
                        pcs_ptr->parent_pcs_ptr->variance[me_sb_addr_1][ME_TIER_ZERO_PU_64x64] +
                        pcs_ptr->parent_pcs_ptr->variance[me_sb_addr_2][ME_TIER_ZERO_PU_64x64] +
                        pcs_ptr->parent_pcs_ptr->variance[me_sb_addr_3][ME_TIER_ZERO_PU_64x64] + 2) >> 2;

                 me_distortion =
                    (pcs_ptr->parent_pcs_ptr->rc_me_distortion[me_sb_addr_0] +
                        pcs_ptr->parent_pcs_ptr->rc_me_distortion[me_sb_addr_1] +
                        pcs_ptr->parent_pcs_ptr->rc_me_distortion[me_sb_addr_2] +
                        pcs_ptr->parent_pcs_ptr->rc_me_distortion[me_sb_addr_3] + 2) >> 2;
                me_distortion >>= 8;
            }
            else {
                variance_sb = pcs_ptr->parent_pcs_ptr->variance[sb_addr][ME_TIER_ZERO_PU_64x64];
                me_distortion = pcs_ptr->parent_pcs_ptr->rc_me_distortion[sb_addr] >> 8;
            }

            double beta = ppcs_ptr->cutree_beta[sb_addr];
            int offset = av1_get_deltaq_offset(scs_ptr->static_config.encoder_bit_depth, ppcs_ptr->frm_hdr.quantization_params.base_q_idx, beta);
            offset = AOMMIN(offset,  pcs_ptr->parent_pcs_ptr->frm_hdr.delta_q_params.delta_q_res * 9*2 - 1);
            offset = AOMMAX(offset, -pcs_ptr->parent_pcs_ptr->frm_hdr.delta_q_params.delta_q_res * 9*2 + 1);

            delta_qp = offset>>2;
            //if(pcs_ptr->parent_pcs_ptr->picture_number == 0)
            //printf("rc qpm: poc%ld sb_addr=%d r0=%f delta_qp=%d offset=%d, beta=%f, base_rdmult=%d, base_q_idx=%d\n", pcs_ptr->parent_pcs_ptr->picture_number, sb_addr, ppcs_ptr->r0, delta_qp, offset, beta, ppcs_ptr->base_rdmult, ppcs_ptr->frm_hdr.quantization_params.base_q_idx);

#if 0
            if (pcs_ptr->slice_type == 2) {
                if (delta_qp < 0 && variance_sb < IS_COMPLEX_SB_FLAT_VARIANCE_TH)
                    delta_qp = 0;
            }
#endif
            if (pcs_ptr->slice_type == 2)
#if QP2QINDEX
                sb_qp = CLIP3(
#else
                sb_ptr->qp = CLIP3(
#endif
                    MIN(pcs_ptr->parent_pcs_ptr->picture_qp, ((kf_low_motion_minq[active_worst_quality] + 2) >> 2)),
                    MAX(pcs_ptr->parent_pcs_ptr->picture_qp, ((kf_high_motion_minq[active_worst_quality] + 2) >> 2)) + 3,
                    ((int16_t)pcs_ptr->parent_pcs_ptr->picture_qp + (int16_t)delta_qp));
            else
#if QP2QINDEX
                sb_qp = CLIP3(
#else
                sb_ptr->qp = CLIP3(
#endif
                    MIN(pcs_ptr->parent_pcs_ptr->picture_qp, ((arfgf_low_motion_minq[active_worst_quality] + 2) >> 2)) - 1,
                    MAX(pcs_ptr->parent_pcs_ptr->picture_qp, ((arfgf_high_motion_minq[active_worst_quality] + 2) >> 2)) + 3,
                    ((int16_t)pcs_ptr->parent_pcs_ptr->picture_qp + (int16_t)delta_qp));

#if QP2QINDEX
            sb_qp = CLIP3(
                scs_ptr->static_config.min_qp_allowed,
                scs_ptr->static_config.max_qp_allowed,
                sb_qp);
#else
            sb_ptr->qp = CLIP3(
                scs_ptr->static_config.min_qp_allowed,
                scs_ptr->static_config.max_qp_allowed,
                sb_ptr->qp);
#endif
#if !QP2QINDEX
            sb_ptr->delta_qp = (int)pcs_ptr->parent_pcs_ptr->picture_qp - (int)sb_ptr->qp;
#endif

#if QP2QINDEX
            sb_ptr->qindex = quantizer_to_qindex[sb_qp];
            pcs_ptr->parent_pcs_ptr->average_qp += sb_qp;
#else
            pcs_ptr->parent_pcs_ptr->average_qp += sb_ptr->qp;
#endif

#if LAMBDA_SCALING
			sb_setup_lambda(pcs_ptr, sb_ptr);
#endif


            //int qp_index = quantizer_to_qindex[sb_ptr->qp];
            int qp_index = quantizer_to_qindex[(int)pcs_ptr->parent_pcs_ptr->picture_qp];
            //pcs_ptr->parent_pcs_ptr->frm_hdr.delta_q_params.delta_q_present ? (uint8_t)quantizer_to_qindex[sb_qp] : (uint8_t)pcs_ptr->parent_pcs_ptr->frm_hdr.quantization_params.base_q_idx;
#if !QP2QINDEX
            if(qp_index<0 || qp_index>255)
                printf("sb_qp_derivation_tpl_la poc%ld, sb_index%d, sb_qp=%d, update_type=%d, gfu_boost=%d, qp_index=%d\n", pcs_ptr->picture_number, sb_addr, sb_ptr->qp, update_type, rc->gfu_boost, qp_index);
#endif
            int rdmult = av1_get_adaptive_rdmult(scs_ptr->static_config.encoder_bit_depth, qp_index, beta);
            //if(pcs_ptr->picture_number == 56)
            //    printf("sb_qp_derivation_tpl_la poc%ld, sb_index%d, sb_qp=%d, rdmult=%d, update_type=%d, gfu_boost=%d\n", pcs_ptr->picture_number, sb_addr, sb_ptr->qp, rdmult, update_type, rc->gfu_boost);
#if 0
            if (pcs_ptr->slice_type != I_SLICE) {
              const int boost_index = AOMMIN(15, (rc->gfu_boost / 100));

              rdmult = (rdmult * rd_frame_type_factor[update_type]) >> 7;
              //if(pcs_ptr->picture_number == 16)
              //    printf("sb_qp_derivation_tpl_la poc%ld, sb_index%d, sb_qp=%d, rdmult=%d, update_type=%d, gfu_boost=%d, boost_index=%d\n", pcs_ptr->picture_number, sb_addr, sb_ptr->qp, rdmult, update_type, rc->gfu_boost, boost_index);
              rdmult += ((rdmult * rd_boost_factor[boost_index]) >> 7);
            }
#endif
            if (rdmult < 1) rdmult = 1;
            pcs_ptr->sb_ptr_array[sb_addr]->rdmult = rdmult;
        }
    }
    else {
        for (sb_addr = 0; sb_addr < scs_ptr->sb_tot_cnt; ++sb_addr) {
            sb_ptr = pcs_ptr->sb_ptr_array[sb_addr];
#if QP2QINDEX
            sb_ptr->qindex = quantizer_to_qindex[pcs_ptr->picture_qp];
            pcs_ptr->parent_pcs_ptr->average_qp += pcs_ptr->picture_qp;
#else
            sb_ptr->qp = (uint8_t)pcs_ptr->picture_qp;
            sb_ptr->delta_qp = 0;
            pcs_ptr->parent_pcs_ptr->average_qp += sb_ptr->qp;
#endif
        }
    }
}
#endif

// Calculates the QP per SB based on the non moving index. For now, only active for I Slice.
static void sb_qp_derivation(PictureControlSet *pcs_ptr) {
    SequenceControlSet *scs_ptr = pcs_ptr->parent_pcs_ptr->scs_ptr;
    SuperBlock *        sb_ptr;
    uint32_t            sb_addr;
#if QP2QINDEX
    uint8_t             sb_qp = 0;
#endif
    RATE_CONTROL        rc;
    pcs_ptr->parent_pcs_ptr->average_qp = 0;
    if (pcs_ptr->slice_type == 2)
        pcs_ptr->parent_pcs_ptr->frm_hdr.delta_q_params.delta_q_present = 1;
    else
        pcs_ptr->parent_pcs_ptr->frm_hdr.delta_q_params.delta_q_present = 0;

    if (pcs_ptr->parent_pcs_ptr->frm_hdr.delta_q_params.delta_q_present) {
        const int bit_depth            = scs_ptr->static_config.encoder_bit_depth;
        int       active_best_quality  = 0;
        int       active_worst_quality = quantizer_to_qindex[(uint8_t)scs_ptr->static_config.qp];
        int *     kf_low_motion_minq;
        int *     kf_high_motion_minq;
        ASSIGN_MINQ_TABLE(bit_depth, kf_low_motion_minq);
        ASSIGN_MINQ_TABLE(bit_depth, kf_high_motion_minq);
        double   q_val, picture_q_val;
        uint32_t me_sb_size = scs_ptr->sb_sz;
        uint32_t me_pic_width_in_sb =
            (pcs_ptr->parent_pcs_ptr->aligned_width + scs_ptr->sb_sz - 1) / me_sb_size;
        uint32_t me_pic_height_in_sb =
            (pcs_ptr->parent_pcs_ptr->aligned_height + me_sb_size - 1) / me_sb_size;
        int max_qp_scaling_avg_comp =
            MAX(1,
                pcs_ptr->parent_pcs_ptr->non_moving_index_min_distance +
                    pcs_ptr->parent_pcs_ptr->non_moving_index_max_distance);
        // Calculate the QP per frames
        rc.kf_boost =
            (((max_qp_scaling_avg_comp - pcs_ptr->parent_pcs_ptr->non_moving_index_average) *
              (kf_high - kf_low)) /
             max_qp_scaling_avg_comp) +
            kf_low;
        active_best_quality = get_kf_active_quality(&rc, active_worst_quality, bit_depth);
        // Convert the adjustment factor to a qindex delta
        // on active_best_quality.
        picture_q_val = eb_av1_convert_qindex_to_q(active_best_quality, bit_depth);
        for (sb_addr = 0; sb_addr < pcs_ptr->sb_total_count_pix; ++sb_addr) {
            sb_ptr              = pcs_ptr->sb_ptr_array[sb_addr];
            int       delta_qp  = 0;
            SbParams *sb_params = &pcs_ptr->parent_pcs_ptr->sb_params_array[sb_addr];
            uint8_t   non_moving_index_sb;
            uint16_t  variance_sb;
            if (scs_ptr->seq_header.sb_size == BLOCK_128X128) {
                uint32_t me_sb_x      = (sb_ptr->origin_x / me_sb_size);
                uint32_t me_sb_y      = (sb_ptr->origin_y / me_sb_size);
                uint32_t me_sb_addr_0 = me_sb_x + me_sb_y * me_pic_width_in_sb;
                uint32_t me_sb_addr_1 = (me_sb_x + 1) < me_pic_width_in_sb
                                            ? (me_sb_x + 1) + ((me_sb_y + 0) * me_pic_width_in_sb)
                                            : me_sb_addr_0;
                uint32_t me_sb_addr_2 = (me_sb_y + 1) < me_pic_height_in_sb
                                            ? (me_sb_x + 0) + ((me_sb_y + 1) * me_pic_width_in_sb)
                                            : me_sb_addr_0;
                uint32_t me_sb_addr_3 =
                    ((me_sb_x + 1) < me_pic_width_in_sb) && ((me_sb_y + 1) < me_pic_height_in_sb)
                        ? (me_sb_x + 1) + ((me_sb_y + 1) * me_pic_width_in_sb)
                        : me_sb_addr_0;
                non_moving_index_sb =
                    (pcs_ptr->parent_pcs_ptr->non_moving_index_array[me_sb_addr_0] +
                     pcs_ptr->parent_pcs_ptr->non_moving_index_array[me_sb_addr_1] +
                     pcs_ptr->parent_pcs_ptr->non_moving_index_array[me_sb_addr_2] +
                     pcs_ptr->parent_pcs_ptr->non_moving_index_array[me_sb_addr_3] + 2) >>
                    2;
                variance_sb =
                    (pcs_ptr->parent_pcs_ptr->variance[me_sb_addr_0][ME_TIER_ZERO_PU_64x64] +
                     pcs_ptr->parent_pcs_ptr->variance[me_sb_addr_1][ME_TIER_ZERO_PU_64x64] +
                     pcs_ptr->parent_pcs_ptr->variance[me_sb_addr_2][ME_TIER_ZERO_PU_64x64] +
                     pcs_ptr->parent_pcs_ptr->variance[me_sb_addr_3][ME_TIER_ZERO_PU_64x64] + 2) >>
                    2;
            } else {
                non_moving_index_sb = pcs_ptr->parent_pcs_ptr->non_moving_index_array[sb_addr];
                variance_sb = pcs_ptr->parent_pcs_ptr->variance[sb_addr][ME_TIER_ZERO_PU_64x64];
            }
            if (sb_params->is_complete_sb && max_qp_scaling_avg_comp >= 10 &&
                pcs_ptr->parent_pcs_ptr->non_moving_index_average < 20 &&
                non_moving_index_sb < pcs_ptr->parent_pcs_ptr->non_moving_index_average) {
                if (variance_sb < IS_COMPLEX_SB_FLAT_VARIANCE_TH)
                    delta_qp = 3;
                else {
                    // Calculate the QP of each block to find the delta
                    rc.kf_boost =
                        (((max_qp_scaling_avg_comp - non_moving_index_sb) * (kf_high - kf_low)) /
                         max_qp_scaling_avg_comp) +
                        kf_low;
                    // Baseline value derived from cpi->active_worst_quality and kf boost.
                    active_best_quality =
                        get_kf_active_quality(&rc, active_worst_quality, bit_depth);
                    // Convert the adjustment factor to a qindex delta
                    // on active_best_quality.
                    q_val    = eb_av1_convert_qindex_to_q(active_best_quality, bit_depth);
                    delta_qp = (int16_t)q_val - (int16_t)picture_q_val;
                }
            }
#if QP2QINDEX
            sb_qp       = CLIP3(MIN(pcs_ptr->parent_pcs_ptr->picture_qp,
                                   ((kf_low_motion_minq[active_worst_quality] + 2) >> 2)),
                               MAX(pcs_ptr->parent_pcs_ptr->picture_qp,
                                   ((kf_high_motion_minq[active_worst_quality] + 2) >> 2)) +
                                   3,
                               ((int16_t)pcs_ptr->parent_pcs_ptr->picture_qp + (int16_t)delta_qp));
            sb_qp       = CLIP3(scs_ptr->static_config.min_qp_allowed,
                               scs_ptr->static_config.max_qp_allowed,
                               sb_qp);
            sb_ptr->qindex = quantizer_to_qindex[sb_qp];
            pcs_ptr->parent_pcs_ptr->average_qp += sb_qp;
#else
            sb_ptr->qp       = CLIP3(MIN(pcs_ptr->parent_pcs_ptr->picture_qp,
                                   ((kf_low_motion_minq[active_worst_quality] + 2) >> 2)),
                               MAX(pcs_ptr->parent_pcs_ptr->picture_qp,
                                   ((kf_high_motion_minq[active_worst_quality] + 2) >> 2)) +
                                   3,
                               ((int16_t)pcs_ptr->parent_pcs_ptr->picture_qp + (int16_t)delta_qp));
            sb_ptr->qp       = CLIP3(scs_ptr->static_config.min_qp_allowed,
                               scs_ptr->static_config.max_qp_allowed,
                               sb_ptr->qp);
            sb_ptr->delta_qp = (int)pcs_ptr->parent_pcs_ptr->picture_qp - (int)sb_ptr->qp;
            pcs_ptr->parent_pcs_ptr->average_qp += sb_ptr->qp;
#endif
        }
    } else {
        for (sb_addr = 0; sb_addr < pcs_ptr->sb_total_count_pix; ++sb_addr) {
            sb_ptr           = pcs_ptr->sb_ptr_array[sb_addr];
#if QP2QINDEX
            sb_ptr->qindex   = quantizer_to_qindex[pcs_ptr->picture_qp];
            pcs_ptr->parent_pcs_ptr->average_qp += pcs_ptr->picture_qp;
#else
            sb_ptr->qp       = (uint8_t)pcs_ptr->picture_qp;
            sb_ptr->delta_qp = 0;
            pcs_ptr->parent_pcs_ptr->average_qp += sb_ptr->qp;
#endif
        }
    }
}
void *rate_control_kernel(void *input_ptr) {
    // Context
    EbThreadContext *   thread_context_ptr = (EbThreadContext *)input_ptr;
    RateControlContext *context_ptr        = (RateControlContext *)thread_context_ptr->priv;

    RateControlIntervalParamContext *rate_control_param_ptr;

    RateControlIntervalParamContext *prev_gop_rate_control_param_ptr;
    RateControlIntervalParamContext *next_gop_rate_control_param_ptr;

    PictureControlSet *      pcs_ptr;
    PictureParentControlSet *parentpicture_control_set_ptr;

    // Config
    SequenceControlSet *scs_ptr;

    // Input
    EbObjectWrapper * rate_control_tasks_wrapper_ptr;
    RateControlTasks *rate_control_tasks_ptr;

    // Output
    EbObjectWrapper *   rate_control_results_wrapper_ptr;
    RateControlResults *rate_control_results_ptr;

    RateControlLayerContext *rate_control_layer_ptr;

    uint64_t total_number_of_fb_frames = 0;

    RateControlTaskTypes task_type;
    RATE_CONTROL         rc;

    for (;;) {
        // Get RateControl Task
        eb_get_full_object(context_ptr->rate_control_input_tasks_fifo_ptr,
                           &rate_control_tasks_wrapper_ptr);

        rate_control_tasks_ptr = (RateControlTasks *)rate_control_tasks_wrapper_ptr->object_ptr;
        task_type              = rate_control_tasks_ptr->task_type;

        // Modify these for different temporal layers later
        switch (task_type) {
        case RC_PICTURE_MANAGER_RESULT:

            pcs_ptr = (PictureControlSet *)rate_control_tasks_ptr->pcs_wrapper_ptr->object_ptr;
            scs_ptr = (SequenceControlSet *)pcs_ptr->scs_wrapper_ptr->object_ptr;
            FrameHeader *frm_hdr = &pcs_ptr->parent_pcs_ptr->frm_hdr;
#if LAMBDA_SCALING
            pcs_ptr->parent_pcs_ptr->blk_lambda_tuning = EB_FALSE;
#endif

            if (pcs_ptr->picture_number == 0) {
                //init rate control parameters
                init_rc(context_ptr, pcs_ptr, scs_ptr);
            }
            // SB Loop
            pcs_ptr->parent_pcs_ptr->sad_me = 0;
            if (pcs_ptr->slice_type != 2)
                for (int sb_addr = 0; sb_addr < pcs_ptr->sb_total_count; ++sb_addr) {
                    pcs_ptr->parent_pcs_ptr->sad_me +=
                        pcs_ptr->parent_pcs_ptr->rc_me_distortion[sb_addr];
                }
            if (scs_ptr->static_config.rate_control_mode) {
                pcs_ptr->parent_pcs_ptr->intra_selected_org_qp = 0;
                // High level RC
                if (scs_ptr->static_config.rate_control_mode == 1)

                    high_level_rc_input_picture_vbr(pcs_ptr->parent_pcs_ptr,
                                                    scs_ptr,
                                                    scs_ptr->encode_context_ptr,
                                                    context_ptr,
                                                    context_ptr->high_level_rate_control_ptr);
                else if (scs_ptr->static_config.rate_control_mode == 2)
                    high_level_rc_input_picture_cvbr(pcs_ptr->parent_pcs_ptr,
                                                     scs_ptr,
                                                     scs_ptr->encode_context_ptr,
                                                     context_ptr,
                                                     context_ptr->high_level_rate_control_ptr);
            }

            // Frame level RC. Find the ParamPtr for the current GOP
            if (scs_ptr->intra_period_length == -1 ||
                scs_ptr->static_config.rate_control_mode == 0) {
                rate_control_param_ptr          = context_ptr->rate_control_param_queue[0];
                prev_gop_rate_control_param_ptr = context_ptr->rate_control_param_queue[0];
                next_gop_rate_control_param_ptr = context_ptr->rate_control_param_queue[0];
            } else {
                uint32_t interval_index_temp = 0;
                EbBool   interval_found      = EB_FALSE;
                while ((interval_index_temp < PARALLEL_GOP_MAX_NUMBER) && !interval_found) {
                    if (pcs_ptr->picture_number >=
                            context_ptr->rate_control_param_queue[interval_index_temp]->first_poc &&
                        pcs_ptr->picture_number <=
                            context_ptr->rate_control_param_queue[interval_index_temp]->last_poc) {
                        interval_found = EB_TRUE;
                    } else
                        interval_index_temp++;
                }
                CHECK_REPORT_ERROR(interval_index_temp != PARALLEL_GOP_MAX_NUMBER,
                                   scs_ptr->encode_context_ptr->app_callback_ptr,
                                   EB_ENC_RC_ERROR2);

                rate_control_param_ptr = context_ptr->rate_control_param_queue[interval_index_temp];

                prev_gop_rate_control_param_ptr =
                    (interval_index_temp == 0)
                        ? context_ptr->rate_control_param_queue[PARALLEL_GOP_MAX_NUMBER - 1]
                        : context_ptr->rate_control_param_queue[interval_index_temp - 1];
                next_gop_rate_control_param_ptr =
                    (interval_index_temp == PARALLEL_GOP_MAX_NUMBER - 1)
                        ? context_ptr->rate_control_param_queue[0]
                        : context_ptr->rate_control_param_queue[interval_index_temp + 1];
            }

            rate_control_layer_ptr =
                rate_control_param_ptr->rate_control_layer_array[pcs_ptr->temporal_layer_index];

            if (scs_ptr->static_config.rate_control_mode == 0) {
                // if RC mode is 0,  fixed QP is used
                // QP scaling based on POC number for Flat IPPP structure
                frm_hdr->quantization_params.base_q_idx = quantizer_to_qindex[pcs_ptr->picture_qp];

                if (scs_ptr->static_config.enable_qp_scaling_flag &&
                    pcs_ptr->parent_pcs_ptr->qp_on_the_fly == EB_FALSE) {
                    const int32_t qindex = quantizer_to_qindex[(uint8_t)scs_ptr->static_config.qp];
                    // if there are need enough pictures in the LAD/SlidingWindow, the adaptive QP scaling is not used
#if QPS_UPDATE
                    if (pcs_ptr->temporal_layer_index == 0)
                        SVT_LOG("POC:%d\t%d\t%d\n",
                            pcs_ptr->picture_number,
                            pcs_ptr->parent_pcs_ptr->referenced_area_avg,
                            pcs_ptr->parent_pcs_ptr->qp_scaling_average_complexity);
#endif
                    int32_t new_qindex;
                    if (!scs_ptr->use_output_stat_file &&
                        pcs_ptr->parent_pcs_ptr->frames_in_sw >= QPS_SW_THRESH) {
                        // Content adaptive qp assignment
                        if (scs_ptr->use_input_stat_file &&
                            !pcs_ptr->parent_pcs_ptr->sc_content_detected &&
                            pcs_ptr->parent_pcs_ptr->referenced_area_has_non_zero)
                            new_qindex = adaptive_qindex_calc_two_pass(pcs_ptr, &rc, qindex);
                        else
                            new_qindex = cqp_qindex_calc(pcs_ptr, &rc, qindex);
                    }
                    else {
                        new_qindex = cqp_qindex_calc(
                            pcs_ptr,
                            &rc,
                            qindex);
                    }
                    frm_hdr->quantization_params.base_q_idx = (uint8_t)CLIP3(
                        (int32_t)quantizer_to_qindex[scs_ptr->static_config.min_qp_allowed],
                        (int32_t)quantizer_to_qindex[scs_ptr->static_config.max_qp_allowed],
                        (int32_t)(new_qindex));

                    pcs_ptr->picture_qp =
                        (uint8_t)CLIP3((int32_t)scs_ptr->static_config.min_qp_allowed,
                                       (int32_t)scs_ptr->static_config.max_qp_allowed,
                                       (frm_hdr->quantization_params.base_q_idx + 2) >> 2);
                }

                else if (pcs_ptr->parent_pcs_ptr->qp_on_the_fly == EB_TRUE) {
                    pcs_ptr->picture_qp =
                        (uint8_t)CLIP3((int32_t)scs_ptr->static_config.min_qp_allowed,
                                       (int32_t)scs_ptr->static_config.max_qp_allowed,
                                       pcs_ptr->parent_pcs_ptr->picture_qp);
                    frm_hdr->quantization_params.base_q_idx =
                        quantizer_to_qindex[pcs_ptr->picture_qp];
                }

                pcs_ptr->parent_pcs_ptr->picture_qp = pcs_ptr->picture_qp;
                setup_segmentation(pcs_ptr, scs_ptr, rate_control_layer_ptr);
            } else {
                // ***Rate Control***
                if (scs_ptr->static_config.rate_control_mode == 1) {
                    frame_level_rc_input_picture_vbr(pcs_ptr,
                                                     scs_ptr,
                                                     context_ptr,
                                                     rate_control_layer_ptr,
                                                     rate_control_param_ptr);

                    // rate control QP refinement
                    rate_control_refinement(pcs_ptr,
                                            scs_ptr,
                                            rate_control_param_ptr,
                                            prev_gop_rate_control_param_ptr,
                                            next_gop_rate_control_param_ptr);
                } else if (scs_ptr->static_config.rate_control_mode == 2) {
                    frame_level_rc_input_picture_cvbr(pcs_ptr,
                                                      scs_ptr,
                                                      context_ptr,
                                                      rate_control_layer_ptr,
                                                      rate_control_param_ptr);
                }
                pcs_ptr->picture_qp = (uint8_t)CLIP3(scs_ptr->static_config.min_qp_allowed,
                                                     scs_ptr->static_config.max_qp_allowed,
                                                     pcs_ptr->picture_qp);
                frm_hdr->quantization_params.base_q_idx = quantizer_to_qindex[pcs_ptr->picture_qp];
            }

            pcs_ptr->parent_pcs_ptr->picture_qp = pcs_ptr->picture_qp;

            if (pcs_ptr->parent_pcs_ptr->temporal_layer_index == 0 &&
                scs_ptr->static_config.look_ahead_distance != 0)
                context_ptr->base_layer_frames_avg_qp =
                    (3 * context_ptr->base_layer_frames_avg_qp + pcs_ptr->picture_qp + 2) >> 2;
            if (pcs_ptr->slice_type == I_SLICE) {
                if (pcs_ptr->picture_number == rate_control_param_ptr->first_poc) {
                    rate_control_param_ptr->first_pic_pred_qp =
                        (uint16_t)pcs_ptr->parent_pcs_ptr->best_pred_qp;
                    rate_control_param_ptr->first_pic_actual_qp = (uint16_t)pcs_ptr->picture_qp;
                    rate_control_param_ptr->scene_change_in_gop =
                        pcs_ptr->parent_pcs_ptr->scene_change_in_gop;
                    rate_control_param_ptr->first_pic_actual_qp_assigned = EB_TRUE;
                }
                {
                    if (pcs_ptr->picture_number == rate_control_param_ptr->first_poc) {
                        if (scs_ptr->static_config.look_ahead_distance != 0)
                            context_ptr->base_layer_intra_frames_avg_qp =
                                (3 * context_ptr->base_layer_intra_frames_avg_qp +
                                 pcs_ptr->picture_qp + 2) >>
                                2;
                    }

                    if (pcs_ptr->picture_number == rate_control_param_ptr->first_poc) {
                        rate_control_param_ptr->intra_frames_qp         = pcs_ptr->picture_qp;
                        rate_control_param_ptr->next_gop_intra_frame_qp = pcs_ptr->picture_qp;
                        rate_control_param_ptr->intra_frames_qp_bef_scal =
                            (uint8_t)scs_ptr->static_config.max_qp_allowed;
                        for (uint32_t qindex = scs_ptr->static_config.min_qp_allowed;
                             qindex <= scs_ptr->static_config.max_qp_allowed;
                             qindex++) {
                            if (rate_control_param_ptr->intra_frames_qp <=
                                context_ptr->qp_scaling_map_i_slice[qindex]) {
                                rate_control_param_ptr->intra_frames_qp_bef_scal = (uint8_t)qindex;
                                break;
                            }
                        }
                    }
                }
            }
            if (scs_ptr->static_config.enable_adaptive_quantization == 2 &&
                pcs_ptr->parent_pcs_ptr->frames_in_sw >= QPS_SW_THRESH &&
                !pcs_ptr->parent_pcs_ptr->sc_content_detected && !scs_ptr->use_output_stat_file &&
                scs_ptr->use_input_stat_file)
#if CUTREE_LA && CUTREE_LA_QPM
                if (scs_ptr->use_input_stat_file &&
                    scs_ptr->static_config.look_ahead_distance != 0 &&
                    scs_ptr->static_config.enable_cutree_in_la &&
                    pcs_ptr->parent_pcs_ptr->r0 != 0 && pcs_ptr->picture_number<48) // only QPM for poc<48 pictue for last pic seq diff with aom
                    // Content adaptive qp assignment
                    sb_qp_derivation_tpl_la(&rc, pcs_ptr);
                else
#endif

                if (scs_ptr->use_input_stat_file &&
                    pcs_ptr->parent_pcs_ptr->referenced_area_has_non_zero)
                    sb_qp_derivation_two_pass(pcs_ptr);
                else
                    sb_qp_derivation(pcs_ptr);
            else {
                pcs_ptr->parent_pcs_ptr->frm_hdr.delta_q_params.delta_q_present = 0;
                SuperBlock *sb_ptr;
                pcs_ptr->parent_pcs_ptr->average_qp = 0;
                for (int sb_addr = 0; sb_addr < pcs_ptr->sb_total_count_pix; ++sb_addr) {
                    sb_ptr           = pcs_ptr->sb_ptr_array[sb_addr];
#if QP2QINDEX
                    sb_ptr->qindex   = quantizer_to_qindex[pcs_ptr->picture_qp];
                    pcs_ptr->parent_pcs_ptr->average_qp += pcs_ptr->picture_qp;
#else
                    sb_ptr->qp       = (uint8_t)pcs_ptr->picture_qp;
                    sb_ptr->delta_qp = 0;
                    pcs_ptr->parent_pcs_ptr->average_qp += sb_ptr->qp;
#endif
                }
            }
            // Get Empty Rate Control Results Buffer
            eb_get_empty_object(context_ptr->rate_control_output_results_fifo_ptr,
                                &rate_control_results_wrapper_ptr);
            rate_control_results_ptr =
                (RateControlResults *)rate_control_results_wrapper_ptr->object_ptr;
            rate_control_results_ptr->pcs_wrapper_ptr = rate_control_tasks_ptr->pcs_wrapper_ptr;

            // Post Full Rate Control Results
            eb_post_full_object(rate_control_results_wrapper_ptr);

            // Release Rate Control Tasks
            eb_release_object(rate_control_tasks_wrapper_ptr);

            break;

        case RC_PACKETIZATION_FEEDBACK_RESULT:

            parentpicture_control_set_ptr =
                (PictureParentControlSet *)rate_control_tasks_ptr->pcs_wrapper_ptr->object_ptr;
            scs_ptr =
                (SequenceControlSet *)parentpicture_control_set_ptr->scs_wrapper_ptr->object_ptr;

            // Frame level RC
            if (scs_ptr->intra_period_length == -1 ||
                scs_ptr->static_config.rate_control_mode == 0) {
                rate_control_param_ptr          = context_ptr->rate_control_param_queue[0];
                prev_gop_rate_control_param_ptr = context_ptr->rate_control_param_queue[0];
                if (parentpicture_control_set_ptr->slice_type == I_SLICE) {
                    if (parentpicture_control_set_ptr->total_num_bits > MAX_BITS_PER_FRAME)
                        context_ptr->max_rate_adjust_delta_qp++;
                    else if (context_ptr->max_rate_adjust_delta_qp > 0 &&
                             parentpicture_control_set_ptr->total_num_bits <
                                 MAX_BITS_PER_FRAME * 85 / 100)
                        context_ptr->max_rate_adjust_delta_qp--;
                    context_ptr->max_rate_adjust_delta_qp =
                        CLIP3(0, 63, context_ptr->max_rate_adjust_delta_qp);
                    context_ptr->max_rate_adjust_delta_qp = 0;
                }
            } else {
                uint32_t interval_index_temp = 0;
                EbBool   interval_found      = EB_FALSE;
                while ((interval_index_temp < PARALLEL_GOP_MAX_NUMBER) && !interval_found) {
                    if (parentpicture_control_set_ptr->picture_number >=
                            context_ptr->rate_control_param_queue[interval_index_temp]->first_poc &&
                        parentpicture_control_set_ptr->picture_number <=
                            context_ptr->rate_control_param_queue[interval_index_temp]->last_poc) {
                        interval_found = EB_TRUE;
                    } else
                        interval_index_temp++;
                }
                CHECK_REPORT_ERROR(interval_index_temp != PARALLEL_GOP_MAX_NUMBER,
                                   scs_ptr->encode_context_ptr->app_callback_ptr,
                                   EB_ENC_RC_ERROR2);

                rate_control_param_ptr = context_ptr->rate_control_param_queue[interval_index_temp];

                prev_gop_rate_control_param_ptr =
                    (interval_index_temp == 0)
                        ? context_ptr->rate_control_param_queue[PARALLEL_GOP_MAX_NUMBER - 1]
                        : context_ptr->rate_control_param_queue[interval_index_temp - 1];
            }
            if (scs_ptr->static_config.rate_control_mode != 0) {
                context_ptr->previous_virtual_buffer_level = context_ptr->virtual_buffer_level;

                context_ptr->virtual_buffer_level =
                    (int64_t)context_ptr->previous_virtual_buffer_level +
                    (int64_t)parentpicture_control_set_ptr->total_num_bits -
                    (int64_t)context_ptr->high_level_rate_control_ptr->channel_bit_rate_per_frame;

                high_level_rc_feed_back_picture(parentpicture_control_set_ptr, scs_ptr);
                if (scs_ptr->static_config.rate_control_mode == 1)
                    frame_level_rc_feedback_picture_vbr(
                        parentpicture_control_set_ptr, scs_ptr, context_ptr);
                else if (scs_ptr->static_config.rate_control_mode == 2)
                    frame_level_rc_feedback_picture_cvbr(
                        parentpicture_control_set_ptr, scs_ptr, context_ptr);
                if (parentpicture_control_set_ptr->picture_number ==
                    rate_control_param_ptr->first_poc) {
                    rate_control_param_ptr->first_pic_pred_bits =
                        parentpicture_control_set_ptr->target_bits_best_pred_qp;
                    rate_control_param_ptr->first_pic_actual_bits =
                        parentpicture_control_set_ptr->total_num_bits;
                    {
                        int16_t delta_ap_qp = (int16_t)rate_control_param_ptr->first_pic_actual_qp -
                                              (int16_t)rate_control_param_ptr->first_pic_pred_qp;
                        rate_control_param_ptr->extra_ap_bit_ratio_i =
                            (rate_control_param_ptr->first_pic_pred_bits != 0)
                                ? (((int64_t)rate_control_param_ptr->first_pic_actual_bits -
                                    (int64_t)rate_control_param_ptr->first_pic_pred_bits) *
                                   100) /
                                      ((int64_t)rate_control_param_ptr->first_pic_pred_bits)
                                : 0;
                        rate_control_param_ptr->extra_ap_bit_ratio_i += (int64_t)delta_ap_qp * 15;
                    }
                }
            }

            // Queue variables
#if OVERSHOOT_STAT_PRINT
            if (scs_ptr->intra_period_length != -1) {
                int32_t                queue_entry_index;
                uint32_t               queue_entry_index_temp;
                uint32_t               queue_entry_index_temp2;
                CodedFramesStatsEntry *queue_entry_ptr;
                EbBool                 move_slide_window_flag = EB_TRUE;
                EbBool                 end_of_sequence_flag   = EB_TRUE;
                uint32_t               frames_in_sw;

                // Determine offset from the Head Ptr
                queue_entry_index = (int32_t)(
                    parentpicture_control_set_ptr->picture_number -
                    context_ptr
                        ->coded_frames_stat_queue[context_ptr->coded_frames_stat_queue_head_index]
                        ->picture_number);
                queue_entry_index += context_ptr->coded_frames_stat_queue_head_index;
                queue_entry_index = (queue_entry_index > CODED_FRAMES_STAT_QUEUE_MAX_DEPTH - 1)
                                        ? queue_entry_index - CODED_FRAMES_STAT_QUEUE_MAX_DEPTH
                                        : queue_entry_index;
                queue_entry_ptr = context_ptr->coded_frames_stat_queue[queue_entry_index];

                queue_entry_ptr->frame_total_bit_actual =
                    (uint64_t)parentpicture_control_set_ptr->total_num_bits;
                queue_entry_ptr->picture_number = parentpicture_control_set_ptr->picture_number;
                queue_entry_ptr->end_of_sequence_flag =
                    parentpicture_control_set_ptr->end_of_sequence_flag;
                context_ptr->rate_average_periodin_frames =
                    (uint64_t)scs_ptr->static_config.intra_period + 1;

                //SVT_LOG("\n0_POC: %d\n",
                //    queue_entry_ptr->picture_number);
                move_slide_wondow_flag = EB_TRUE;
                while (move_slide_wondow_flag) {
                    //  SVT_LOG("\n1_POC: %d\n",
                    //      queue_entry_ptr->picture_number);
                    // Check if the sliding window condition is valid
                    queue_entry_index_temp = context_ptr->coded_frames_stat_queue_head_index;
                    if (context_ptr->coded_frames_stat_queue[queue_entry_index_temp]
                            ->frame_total_bit_actual != -1)
                        end_of_sequence_flag =
                            context_ptr->coded_frames_stat_queue[queue_entry_index_temp]
                                ->end_of_sequence_flag;
                    else
                        end_of_sequence_flag = EB_FALSE;
                    while (move_slide_wondow_flag && !end_of_sequence_flag &&
                           queue_entry_index_temp <
                               context_ptr->coded_frames_stat_queue_head_index +
                                   context_ptr->rate_average_periodin_frames) {
                        // SVT_LOG("\n2_POC: %d\n",
                        //     queue_entry_ptr->picture_number);

                        queue_entry_index_temp2 =
                            (queue_entry_index_temp > CODED_FRAMES_STAT_QUEUE_MAX_DEPTH - 1)
                                ? queue_entry_index_temp - CODED_FRAMES_STAT_QUEUE_MAX_DEPTH
                                : queue_entry_index_temp;

                        move_slide_wondow_flag =
                            (EbBool)(move_slide_wondow_flag &&
                                     (context_ptr->coded_frames_stat_queue[queue_entry_index_temp2]
                                          ->frame_total_bit_actual != -1));

                        if (context_ptr->coded_frames_stat_queue[queue_entry_index_temp2]
                                ->frame_total_bit_actual != -1) {
                            // check if it is the last frame. If we have reached the last frame, we would output the buffered frames in the Queue.
                            end_of_sequence_flag =
                                context_ptr->coded_frames_stat_queue[queue_entry_index_temp]
                                    ->end_of_sequence_flag;
                        } else
                            end_of_sequence_flag = EB_FALSE;
                        queue_entry_index_temp =
                            (queue_entry_index_temp == CODED_FRAMES_STAT_QUEUE_MAX_DEPTH - 1)
                                ? 0
                                : queue_entry_index_temp + 1;
                    }

                    if (move_slide_wondow_flag) {
                        //get a new entry spot
                        queue_entry_ptr        = (context_ptr->coded_frames_stat_queue
                                               [context_ptr->coded_frames_stat_queue_head_index]);
                        queue_entry_index_temp = context_ptr->coded_frames_stat_queue_head_index;
                        // This is set to false, so the last frame would go inside the loop
                        end_of_sequence_flag                 = EB_FALSE;
                        frames_in_sw                         = 0;
                        context_ptr->total_bit_actual_per_sw = 0;

                        while (!end_of_sequence_flag &&
                               queue_entry_index_temp <
                                   context_ptr->coded_frames_stat_queue_head_index +
                                       context_ptr->rate_average_periodin_frames) {
                            frames_in_sw++;

                            queue_entry_index_temp2 =
                                (queue_entry_index_temp > CODED_FRAMES_STAT_QUEUE_MAX_DEPTH - 1)
                                    ? queue_entry_index_temp - CODED_FRAMES_STAT_QUEUE_MAX_DEPTH
                                    : queue_entry_index_temp;

                            context_ptr->total_bit_actual_per_sw +=
                                context_ptr->coded_frames_stat_queue[queue_entry_index_temp2]
                                    ->frame_total_bit_actual;
                            end_of_sequence_flag =
                                context_ptr->coded_frames_stat_queue[queue_entry_index_temp2]
                                    ->end_of_sequence_flag;

                            queue_entry_index_temp =
                                (queue_entry_index_temp == CODED_FRAMES_STAT_QUEUE_MAX_DEPTH - 1)
                                    ? 0
                                    : queue_entry_index_temp + 1;
                        }
                        //

                        //if(frames_in_sw == context_ptr->rate_average_periodin_frames)
                        //    SVT_LOG("POC:%d\t %.3f\n", queue_entry_ptr->picture_number, (double)context_ptr->total_bit_actual_per_sw*(scs_ptr->frame_rate>> RC_PRECISION)/(double)frames_in_sw/1000);
                        if (frames_in_sw == (uint32_t)scs_ptr->intra_period_length + 1) {
                            context_ptr->max_bit_actual_per_sw = MAX(
                                context_ptr->max_bit_actual_per_sw,
                                context_ptr->total_bit_actual_per_sw *
                                    (scs_ptr->frame_rate >> RC_PRECISION) / frames_in_sw / 1000);
                            if (queue_entry_ptr->picture_number %
                                    ((scs_ptr->intra_period_length + 1)) ==
                                0) {
                                context_ptr->max_bit_actual_per_gop =
                                    MAX(context_ptr->max_bit_actual_per_gop,
                                        context_ptr->total_bit_actual_per_sw *
                                            (scs_ptr->frame_rate >> RC_PRECISION) / frames_in_sw /
                                            1000);
                                context_ptr->min_bit_actual_per_gop =
                                    MIN(context_ptr->min_bit_actual_per_gop,
                                        context_ptr->total_bit_actual_per_sw *
                                            (scs_ptr->frame_rate >> RC_PRECISION) / frames_in_sw /
                                            1000);
                                //if (context_ptr->total_bit_actual_per_sw > scs_ptr->static_config.max_buffersize){
                                SVT_LOG(
                                    "POC:%d\t%.0f\t%.2f%% \n",
                                    (int)queue_entry_ptr->picture_number,
                                    (double)((int64_t)context_ptr->total_bit_actual_per_sw *
                                             (scs_ptr->frame_rate >> RC_PRECISION) / frames_in_sw /
                                             1000),
                                    (double)(100 * (double)context_ptr->total_bit_actual_per_sw *
                                             (scs_ptr->frame_rate >> RC_PRECISION) / frames_in_sw /
                                             (double)scs_ptr->static_config.target_bit_rate) -
                                        100);
                            }
                        }
                        if (frames_in_sw == context_ptr->rate_average_periodin_frames - 1) {
                            //SVT_LOG("\n%d MAX\n", (int32_t)context_ptr->max_bit_actual_per_sw);
                            SVT_LOG("\n%d GopMax\t", (int32_t)context_ptr->max_bit_actual_per_gop);
                            SVT_LOG("%d GopMin\n", (int32_t)context_ptr->min_bit_actual_per_gop);
                        }
                        // Reset the Queue Entry
                        queue_entry_ptr->picture_number += CODED_FRAMES_STAT_QUEUE_MAX_DEPTH;
                        queue_entry_ptr->frame_total_bit_actual = -1;

                        // Increment the Reorder Queue head Ptr
                        context_ptr->coded_frames_stat_queue_head_index =
                            (context_ptr->coded_frames_stat_queue_head_index ==
                             CODED_FRAMES_STAT_QUEUE_MAX_DEPTH - 1)
                                ? 0
                                : context_ptr->coded_frames_stat_queue_head_index + 1;

                        queue_entry_ptr = (context_ptr->coded_frames_stat_queue
                                               [context_ptr->coded_frames_stat_queue_head_index]);
                    }
                }
            }
#endif
            total_number_of_fb_frames++;

            // Release the SequenceControlSet
            eb_release_object(parentpicture_control_set_ptr->scs_wrapper_ptr);
            // Release the ParentPictureControlSet
            eb_release_object(parentpicture_control_set_ptr->input_picture_wrapper_ptr);
            eb_release_object(rate_control_tasks_ptr->pcs_wrapper_ptr);

            // Release Rate Control Tasks
            eb_release_object(rate_control_tasks_wrapper_ptr);
            break;

        case RC_ENTROPY_CODING_ROW_FEEDBACK_RESULT:

            // Extract bits-per-sb-row

            // Release Rate Control Tasks
            eb_release_object(rate_control_tasks_wrapper_ptr);

            break;

        default:
            pcs_ptr = (PictureControlSet *)rate_control_tasks_ptr->pcs_wrapper_ptr->object_ptr;
            scs_ptr = (SequenceControlSet *)pcs_ptr->scs_wrapper_ptr->object_ptr;

            break;
        }
    }

    return EB_NULL;
}
