/*
* Copyright(c) 2019 Intel Corporation
* Copyright (c) 2016, Alliance for Open Media. All rights reserved
*
* This source code is subject to the terms of the BSD 2 Clause License and
* the Alliance for Open Media Patent License 1.0. If the BSD 2 Clause License
* was not distributed with this source code in the LICENSE file, you can
* obtain it at https://www.aomedia.org/license/software-license. If the Alliance for Open
* Media Patent License 1.0 was not distributed with this source code in the
* PATENTS file, you can obtain it at https://www.aomedia.org/license/patent-license.
*/

#include "EbSegmentation.h"
#include "EbSegmentationParams.h"
#include "EbMotionEstimationContext.h"
#include "common_dsp_rtcd.h"

uint16_t get_variance_for_cu(const BlockGeom *blk_geom, uint16_t *variance_ptr) {
    int index0, index1;
    //Assumes max CU size is 64
    switch (blk_geom->bsize) {
    case BLOCK_4X4:
    case BLOCK_4X8:
    case BLOCK_8X4:
    case BLOCK_8X8:
        index0 = index1 = ME_TIER_ZERO_PU_8x8_0 + ((blk_geom->origin_x >> 3) + blk_geom->origin_y);
        break;

    case BLOCK_8X16:
        index0 = ME_TIER_ZERO_PU_8x8_0 + ((blk_geom->origin_x >> 3) + blk_geom->origin_y);
        index1 = index0 + 1;
        break;

    case BLOCK_16X8:
        index0 = ME_TIER_ZERO_PU_8x8_0 + ((blk_geom->origin_x >> 3) + blk_geom->origin_y);
        index1 = index0 + blk_geom->origin_y;
        break;

    case BLOCK_4X16:
    case BLOCK_16X4:
    case BLOCK_16X16:
        index0 = index1 =
            ME_TIER_ZERO_PU_16x16_0 + ((blk_geom->origin_x >> 4) + (blk_geom->origin_y >> 2));
        break;

    case BLOCK_16X32:
        index0 = ME_TIER_ZERO_PU_16x16_0 + ((blk_geom->origin_x >> 4) + (blk_geom->origin_y >> 2));
        index1 = index0 + 1;
        break;

    case BLOCK_32X16:
        index0 = ME_TIER_ZERO_PU_16x16_0 + ((blk_geom->origin_x >> 4) + (blk_geom->origin_y >> 2));
        index1 = index0 + (blk_geom->origin_y >> 2);
        break;

    case BLOCK_8X32:
    case BLOCK_32X8:
    case BLOCK_32X32:
        index0 = index1 =
            ME_TIER_ZERO_PU_32x32_0 + ((blk_geom->origin_x >> 5) + (blk_geom->origin_y >> 4));
        break;

    case BLOCK_32X64:
        index0 = ME_TIER_ZERO_PU_32x32_0 + ((blk_geom->origin_x >> 5) + (blk_geom->origin_y >> 4));
        index1 = index0 + 1;
        break;

    case BLOCK_64X32:
        index0 = ME_TIER_ZERO_PU_32x32_0 + ((blk_geom->origin_x >> 5) + (blk_geom->origin_y >> 4));
        index1 = index0 + (blk_geom->origin_y >> 4);
        break;

    case BLOCK_64X64:
    case BLOCK_16X64:
    case BLOCK_64X16:
    default: index0 = index1 = 0; break;
    }
    return (variance_ptr[index0] + variance_ptr[index1]) >> 1;
}

void apply_segmentation_based_quantization(const BlockGeom *blk_geom, PictureControlSet *pcs_ptr,
                                           SuperBlock *sb_ptr, BlkStruct *blk_ptr) {
    uint16_t *          variance_ptr        = pcs_ptr->parent_pcs_ptr->variance[sb_ptr->index];
    SegmentationParams *segmentation_params = &pcs_ptr->parent_pcs_ptr->frm_hdr.segmentation_params;
    uint16_t            variance            = get_variance_for_cu(blk_geom, variance_ptr);
    for (int i = 0; i < MAX_SEGMENTS; i++) {
        if (variance <= segmentation_params->variance_bin_edge[i]) {
            blk_ptr->segment_id = i;
            break;
        }
    }
    int32_t q_index = pcs_ptr->parent_pcs_ptr->frm_hdr.quantization_params.base_q_idx +
                      pcs_ptr->parent_pcs_ptr->frm_hdr.segmentation_params
                          .feature_data[blk_ptr->segment_id][SEG_LVL_ALT_Q];
    blk_ptr->qindex = q_index;
}

void setup_segmentation(PictureControlSet *pcs_ptr, SequenceControlSet *scs_ptr,
                        RateControlLayerContext *rateControlLayerPtr) {
    SegmentationParams *segmentation_params = &pcs_ptr->parent_pcs_ptr->frm_hdr.segmentation_params;
    segmentation_params->segmentation_enabled =
        (EbBool)(scs_ptr->static_config.enable_adaptive_quantization == 1);
    if (segmentation_params->segmentation_enabled) {
        int32_t segment_qps[MAX_SEGMENTS] = {0};
        segmentation_params->segmentation_update_data =
            1; //always updating for now. Need to set this based on actual deltas
        segmentation_params->segmentation_update_map = 1;
        segmentation_params->segmentation_temporal_update =
            EB_FALSE; //!(pcs_ptr->parent_pcs_ptr->av1FrameType == KEY_FRAME || pcs_ptr->parent_pcs_ptr->av1FrameType == INTRA_ONLY_FRAME);
        find_segment_qps(segmentation_params, pcs_ptr);
        temporally_update_qps(segment_qps,
                              rateControlLayerPtr->prev_segment_qps,
                              segmentation_params->segmentation_temporal_update);
        for (int i = 0; i < MAX_SEGMENTS; i++)
            segmentation_params->feature_enabled[i][SEG_LVL_ALT_Q] = 1;

        calculate_segmentation_data(segmentation_params);
    }
}

void calculate_segmentation_data(SegmentationParams *segmentation_params) {
    for (int i = 0; i < MAX_SEGMENTS; i++) {
        for (int j = 0; j < SEG_LVL_MAX; j++) {
            if (segmentation_params->feature_enabled[i][j]) {
                segmentation_params->last_active_seg_id = i;
                if (j >= SEG_LVL_REF_FRAME) { segmentation_params->seg_id_pre_skip = 1; }
            }
        }
    }
}

void find_segment_qps(SegmentationParams *segmentation_params,
                      PictureControlSet * pcs_ptr) { //QP needs to be specified as qpindex, not qp.

    uint16_t  min_var = UINT16_MAX, max_var = MIN_UNSIGNED_VALUE, avg_var = 0;
    const float strength = 2; //to tune

    // get range of variance
    for (uint32_t sb_idx = 0; sb_idx < pcs_ptr->sb_total_count; ++sb_idx) {
        uint16_t *variance_ptr = pcs_ptr->parent_pcs_ptr->variance[sb_idx];
        uint32_t var_index, local_avg = 0;
        // Loop over all 8x8s in a 64x64
        for (var_index = ME_TIER_ZERO_PU_8x8_0; var_index <= ME_TIER_ZERO_PU_8x8_63; var_index++) {
            max_var = MAX(max_var, variance_ptr[var_index]);
            min_var = MIN(min_var, variance_ptr[var_index]);
            local_avg += variance_ptr[var_index];
        }
        avg_var += (local_avg >> 6);
    }
    avg_var /= pcs_ptr->sb_total_count;
    avg_var = eb_log2f(avg_var);

    //get variance bin edges & QPs
    uint16_t min_var_log = eb_log2f(MAX(1, min_var));
    uint16_t max_var_log = eb_log2f(MAX(1, max_var));
    uint16_t step_size   = (uint16_t)(max_var_log - min_var_log) <= MAX_SEGMENTS
                             ? 1
                             : ROUND(((max_var_log - min_var_log)) / MAX_SEGMENTS);
    uint16_t bin_edge   = min_var_log + step_size;
    uint16_t bin_center = bin_edge >> 1;

    for (int i = 0; i < MAX_SEGMENTS; i++) {
        segmentation_params->variance_bin_edge[i] = POW2(bin_edge);
        segmentation_params->feature_data[i][SEG_LVL_ALT_Q] =
            ROUND((uint16_t)strength * (MAX(1, bin_center) - avg_var));
        bin_edge += step_size;
        bin_center += step_size;
    }
}

void temporally_update_qps(int32_t *segment_qp_ptr, int32_t *prev_segment_qp_ptr,
                           EbBool temporal_update) {
    for (int i = 0; i < MAX_SEGMENTS; i++) {
        int32_t diff           = segment_qp_ptr[i] - prev_segment_qp_ptr[i];
        prev_segment_qp_ptr[i] = segment_qp_ptr[i];
        segment_qp_ptr[i]      = temporal_update ? diff : segment_qp_ptr[i];
    }
}

void init_enc_dec_segement(PictureParentControlSet *parentpicture_control_set_ptr) {
    SequenceControlSet *scs_ptr = (SequenceControlSet *)parentpicture_control_set_ptr->scs_wrapper_ptr->object_ptr;
    uint8_t pic_width_in_sb = (uint8_t)((parentpicture_control_set_ptr->aligned_width +
                scs_ptr->sb_size_pix - 1) /
            scs_ptr->sb_size_pix);
    uint8_t picture_height_in_sb = (uint8_t)((parentpicture_control_set_ptr->aligned_height +
                scs_ptr->sb_size_pix - 1) /
            scs_ptr->sb_size_pix);
    set_tile_info(parentpicture_control_set_ptr);
    int sb_size_log2 = scs_ptr->seq_header.sb_size_log2;
    uint32_t enc_dec_seg_col_cnt = scs_ptr->enc_dec_segment_col_count_array[parentpicture_control_set_ptr->temporal_layer_index];
    uint32_t enc_dec_seg_row_cnt = scs_ptr->enc_dec_segment_row_count_array[parentpicture_control_set_ptr->temporal_layer_index];
    Av1Common *const cm = parentpicture_control_set_ptr->av1_cm;
    uint16_t tile_row, tile_col;
    uint32_t x_sb_index, y_sb_index;
    const int tile_cols = parentpicture_control_set_ptr->av1_cm->tiles_info.tile_cols;
    const int tile_rows = parentpicture_control_set_ptr->av1_cm->tiles_info.tile_rows;
    TileInfo  tile_info;
    uint8_t tile_group_cols = MIN(
            tile_cols,
            scs_ptr
            ->tile_group_col_count_array[parentpicture_control_set_ptr->temporal_layer_index]);
    uint8_t tile_group_rows = MIN(
            tile_rows,
            scs_ptr
            ->tile_group_row_count_array[parentpicture_control_set_ptr->temporal_layer_index]);

    if (tile_group_cols * tile_group_rows > 1) {
        enc_dec_seg_col_cnt = MIN(enc_dec_seg_col_cnt,
                (uint8_t)(pic_width_in_sb / tile_group_cols));
        enc_dec_seg_row_cnt = MIN(
                enc_dec_seg_row_cnt,
                (uint8_t)(picture_height_in_sb / tile_group_rows));
    }
    parentpicture_control_set_ptr->tile_group_cols = tile_group_cols;
    parentpicture_control_set_ptr->tile_group_rows = tile_group_rows;

    uint8_t tile_group_col_start_tile_idx[1024];
    uint8_t tile_group_row_start_tile_idx[1024];

    // Get the tile start index for tile group
    for (uint8_t c = 0; c <= tile_group_cols; c++) {
        tile_group_col_start_tile_idx[c] = c * tile_cols / tile_group_cols;
    }
    for (uint8_t r = 0; r <= tile_group_rows; r++) {
        tile_group_row_start_tile_idx[r] = r * tile_rows / tile_group_rows;
    }
    for (uint8_t r = 0; r < tile_group_rows; r++) {
        for (uint8_t c = 0; c < tile_group_cols; c++) {
            uint16_t tile_group_idx        = r * tile_group_cols + c;
            uint16_t top_left_tile_col_idx = tile_group_col_start_tile_idx[c];
            uint16_t top_left_tile_row_idx = tile_group_row_start_tile_idx[r];
            uint16_t bottom_right_tile_col_idx =
                tile_group_col_start_tile_idx[c + 1];
            uint16_t bottom_right_tile_row_idx =
                tile_group_row_start_tile_idx[r + 1];

            TileGroupInfo *tg_info_ptr =
                &parentpicture_control_set_ptr->tile_group_info[tile_group_idx];

            tg_info_ptr->tile_group_tile_start_x = top_left_tile_col_idx;
            tg_info_ptr->tile_group_tile_end_x   = bottom_right_tile_col_idx;

            tg_info_ptr->tile_group_tile_start_y = top_left_tile_row_idx;
            tg_info_ptr->tile_group_tile_end_y   = bottom_right_tile_row_idx;

            tg_info_ptr->tile_group_sb_start_x =
                cm->tiles_info.tile_col_start_mi[top_left_tile_col_idx] >>
                sb_size_log2;
            tg_info_ptr->tile_group_sb_start_y =
                cm->tiles_info.tile_row_start_mi[top_left_tile_row_idx] >>
                sb_size_log2;

            // Get the SB end of the bottom right tile
            tg_info_ptr->tile_group_sb_end_x =
                (cm->tiles_info.tile_col_start_mi[bottom_right_tile_col_idx] >>
                 sb_size_log2);
            tg_info_ptr->tile_group_sb_end_y =
                (cm->tiles_info.tile_row_start_mi[bottom_right_tile_row_idx] >>
                 sb_size_log2);

            // Get the width/height of tile group in SB
            tg_info_ptr->tile_group_height_in_sb =
                tg_info_ptr->tile_group_sb_end_y -
                tg_info_ptr->tile_group_sb_start_y;
            tg_info_ptr->tile_group_width_in_sb =
                tg_info_ptr->tile_group_sb_end_x -
                tg_info_ptr->tile_group_sb_start_x;

            // Init segments within the tile group
            enc_dec_segments_init(
                    parentpicture_control_set_ptr->child_pcs->enc_dec_segment_ctrl[tile_group_idx],
                    enc_dec_seg_col_cnt,
                    enc_dec_seg_row_cnt,
                    tg_info_ptr->tile_group_width_in_sb,
                    tg_info_ptr->tile_group_height_in_sb);

            // Enable tile parallelism in Entropy Coding stage
            for (uint16_t r = top_left_tile_row_idx;
                    r < bottom_right_tile_row_idx;
                    r++) {
                for (uint16_t c = top_left_tile_col_idx;
                        c < bottom_right_tile_col_idx;
                        c++) {
                    uint16_t tileIdx = r * tile_cols + c;
                    parentpicture_control_set_ptr->child_pcs->entropy_coding_info[tileIdx]
                        ->entropy_coding_current_row = 0;
                    parentpicture_control_set_ptr->child_pcs->entropy_coding_info[tileIdx]
                        ->entropy_coding_current_available_row = 0;
                    parentpicture_control_set_ptr->child_pcs->entropy_coding_info[tileIdx]
                        ->entropy_coding_row_count =
                        (cm->tiles_info.tile_row_start_mi[r + 1] -
                         cm->tiles_info.tile_row_start_mi[r]) >>
                        sb_size_log2;
                    parentpicture_control_set_ptr->child_pcs->entropy_coding_info[tileIdx]
                        ->entropy_coding_in_progress = EB_FALSE;
                    parentpicture_control_set_ptr->child_pcs->entropy_coding_info[tileIdx]
                        ->entropy_coding_tile_done = EB_FALSE;

                    for (unsigned rowIndex = 0; rowIndex < MAX_SB_ROWS;
                            ++rowIndex) {
                        parentpicture_control_set_ptr->child_pcs->entropy_coding_info[tileIdx]
                            ->entropy_coding_row_array[rowIndex] = EB_FALSE;
                    }
                }
            }
            parentpicture_control_set_ptr->child_pcs->entropy_coding_pic_reset_flag = EB_TRUE;
        }
    }
}
