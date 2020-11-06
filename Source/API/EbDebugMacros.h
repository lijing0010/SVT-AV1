/*
* Copyright(c) 2020 Intel Corporation
*
* This source code is subject to the terms of the BSD 2 Clause License and
* the Alliance for Open Media Patent License 1.0. If the BSD 2 Clause License
* was not distributed with this source code in the LICENSE file, you can
* obtain it at https://www.aomedia.org/license/software-license. If the Alliance for Open
* Media Patent License 1.0 was not distributed with this source code in the
* PATENTS file, you can obtain it at https://www.aomedia.org/license/patent-license.
*/


/*
* This file contains only debug macros that are used during the development
* and are supposed to be cleaned up every tag cycle
* all macros must have the following format:
* - enabling a feature should be prefixed by ENABLE_
* - disableing a feature should be prefixed by DISABLE_
* - tuning a feature should be prefixed by TUNE_
* - adding a new feature should be prefixed by FEATURE_
* - bug fixes should be prefixed by FIX_
* - all macros must have a coherent comment explaining what the MACRO is doing
* - #if 0 / #if 1 are not to be used
*/

#ifndef EbDebugMacros_h
#define EbDebugMacros_h

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

// undefining this macro would allow the AVX512 optimization to be enabled by default
#ifndef NON_AVX512_SUPPORT
#define NON_AVX512_SUPPORT
#endif

// START  svt-03 /////////////////////////////////////////////////////////
#define FEATURE_MDS2 1 // TXT @ MDS2 if CLASS_0_3, and TXS/RDOQ @ MDS2 if CLASS_1_2
#define PR1481       1 //Fix memory leaks from valgrind
#define PR1485       1 //Fix mismatch C/AVX2 kernel svt_av1_apply_temporal_filter_planewise()

#define FIX_RC_BUG 1 // PR:1484 Fix the one pass QP assignment using frames_to_be_encoded
#define FIX_VBR_BUG 1 // PR:1484 Fix 1st pass bug (bug from rebasing the branch)
#define FIX_10BIT     1 // PR:1484 fix 1st pass for 10bit input
#define FIX_RC_TOKEN     1 // PR:1484 fix RC token check to include double dash
//***************************************************************//
#define FIX_PAD_CHROMA_AFTER_MCTF     1 // Padding chroma after altref
#define FEATURE_NEW_DELAY             1 // Change delay some sorts of I in PicDecision
#if FEATURE_NEW_DELAY
#define FIX_LAD_DEADLOCK              1 // Fix deadlock when lad>0 + iperiod>0
#define TUNE_NEW_DELAY_DBG_MSG        0 // Turn off debug message
#define SCD_LAD            6  //number of future frames
#define PD_WINDOW_SIZE     (SCD_LAD +2) //adding previous+current to future
#define MAX_TPL_GROUP_SIZE 64 //enough to cover 6L gop
#endif

#define FEATURE_INL_ME                1 //Enable in-loop ME
#if FEATURE_INL_ME
#define TUNE_IME_REUSE_TPL_RESULT     1 // Reuse TPL results for iLoopME
#define TUNE_INL_TPL_ENHANCEMENT      1 // Refinement for TPL
#define TUNE_INL_TPL_ME_DBG_MSG       0 // Turn off debug message
#define TUNE_INL_ME_RECON_INPUT       1 // Perform ME on input/recon: 1 on input, 0 on recon
#if TUNE_INL_ME_RECON_INPUT
#define TUNE_INL_ME_ON_INPUT          1 // Perform ME on input
#define TUNE_INL_GM_ON_INPUT          1 // Perform GM on input
#define TUNE_INL_TPL_ON_INPUT         1 // Perform TPL on input
#define TUNE_INL_ME_MEM_OPT           1 // Optimize memory usage when perform ME on input, only use 8bit luma
#define TUNE_INL_ME_DECODE_ORDER      1 // Force decode order for inloopME
#endif
#if !TUNE_IME_REUSE_TPL_RESULT
#define TUNE_SIGNAL_TPL_ME_OQ         1 // A separate signal_xxx_oq for TPL ME
#endif
#define RE_ENCODE_SUPPORT     1 // re-encode support
#if RE_ENCODE_SUPPORT
#define RE_ENCODE_SUPPORT_RC 1 // re-encode decision maker in RC kernel
#define RE_ENCODE_PCS_SB 1 // pcs sb_ptr_array update with re-encode new q
#define RE_ENCODE_FRAME_SIZE_SCALE 1 // scale rc->projected_frame_size with *0.8 before re-encode check
#define RE_ENCODE_MAX_LOOP3 0 // set max reencode loop to 3
#define RE_ENCODE_ONLY_KEY_FRAME 1 // re-encode only key frame
#endif
#define FIX_VBR_GF_INTERVAL 0 // fix 2nd pass min/max_gf_interval error
#define FIX_VBR_LAST_GOP_BITS 1 // fix 2nd pass last small group too big frame size error
#define ONE_MIN_QP_ALLOWED 1 // set default min_qp_allowed=1
#define ALLOW_SB128_2PASS_VBR 1 // allow SB128x128 for 2pass VBR
#define TWOPASS_VBR_4L_SUPPORT 1 // 2pass VBR 4L support in impose_gf_length and construct_multi_layer_gf_structure
#define FIRST_PASS_GM_FIX      1 // Fix the GM setting for the first pass
#endif
//***************************************************************//

#define FEATURE_IN_LOOP_TPL 1 // Moving TPL to in loop
#if FEATURE_IN_LOOP_TPL
#define ENABLE_TPL_ZERO_LAD 1 // Enable TPL in loop to work with zero LAD
#define TUNE_TPL 1   // Tuning TPL algorithm for QP assignment
#define ENABLE_TPL_TRAILING 1 //enable trailing pictures for TPL
#endif

#define FEATURE_NIC_SCALING_PER_STAGE            1 // Add ability to scale NICs per stage; improve current trade-offs
#define TUNE_NICS                                1 // Tune settings for NIC scaling/pruning/# of stages to improve trade-offs with new scaling
#define PARTIAL_FREQUENCY                        1 //Calculate partial frequency transforms N2 or N4
#define TUNE_SC_QPS_IMP                          1 // Improve QP assignment for SC
#define FEATURE_REMOVE_CIRCULAR                  1 // Remove circular actions from current NSQ feautres; replace them with non-circular levels
#define FEATURE_NEW_INTER_COMP_LEVELS            1 // Add new levels and controls for inter compound; remove old levels
#define FEATURE_NEW_OBMC_LEVELS                  1 // Add new levels and controls for OBMC
#define TUNE_CDF                                 1 // Update CDF Levels
#define TUNE_TX_TYPE_LEVELS                      1 // Add Tx Type Grouping Levels
#define TUNE_INIT_FAST_LOOP_OPT                  0 // Fast loop optimizations
#define TUNE_REMOVE_UNUSED_NEIG_ARRAY            1 // Removes unused neighbor array
#define INIT_BLOCK_OPT                           1 // optimize block initialization
#define ME_IDX_LUPT                              1 // get index using lookuptable
#define REFACTOR_MD_BLOCK_LOOP                   1 // Refactor the loop that iterates over all blocks at MD
#define FEATURE_INTER_INTRA_LEVELS               1 // Cleanup and modify inter-intra levels
#define TUNE_QPS_QPM       1 // Improve the QPS settings for Keyframe. Improve QPM for nonI base frames
#define TUNE_CDEF_FILTER                         1 // Added new fast search for CDEF
#define FIX_ME_IDX_LUPT                          1 // bug fix stops encoder from deadlocking on >=360p clips


#define FIX_OPTIMIZE_BUILD_QUANTIZER                 1 // Optimize eb_av1_build_quantizer():  called for each single frame (while the generated data does not change per frame). Moved buffer to sps, and performed 1 @ 1st frame only.
#define FEATURE_OPT_IFS                              1 // Reverse IFS search order; regular to be performed last since the most probable then bypass the last evaluation if regular is the winner. 1 chroma compensation could be avoided if we add the ability to do chroma only when calling inter_comp.
#define FIX_REMOVE_UNUSED_CODE                       1 // Remove unused code
#define FIX_BYPASS_USELESS_OPERATIONS                1 // Bypass useless operations when fast rate is OFF
#define FIX_USE_MDS_CNT_INIT                         1 // Use the actual number of candidates @ the init of cand_buff_indices
#define FIX_MOVE_PME_RES_INIT_UNDER_PME              1 // Bypass useless pme_res init
#define FIX_REMOVE_MD_SKIP_COEFF_CIRCUITERY          1 // Remove MD skip_coeff_context circuitery
#define FIX_REMOVE_MVP_MEMSET                        1 // Remove MVP generation useless memset()
#define FIX_OPT_FAST_COST_INIT                       1 // Use the actual number of md_stage_0 candidates @ fast_cost_array init
#define FIX_TF_REFACTOR                              1 // Refactor tf
#define FIX_TUNIFY_SORTING_ARRAY                     1 // Unify MD sorting arrays into 1
#define FIX_IFS                                      1 // Fix IFS to use the actual motion_mode and the actual is_inter_intra
#define FEATURE_COST_BASED_PRED_REFINEMENT           1 // Add an offset to sub_to_current_th and parent_to_current_th on the cost range of the predicted block; use default ths for high cost(s) and more aggressive TH(s) for low cost(s)
#define FEATURE_PD0_CUT_DEPTH                        1 // Shut 16x16 & lower depth(s) based on the 64x64 distortion if sb_64x64
#define FEATURE_PD0_SHUT_SKIP_DC_SIGN_UPDATE         1 // Skip dc_sign derivation/update, and bypass useless mi_info updates
#define FEATURE_OPT_RDOQ                             1 // Use default_quant for chroma and rdoq_bypass = f(satd)
#define FEATURE_OPT_TF                               1 // Add the ability to perform luma only @ tf, control tf_16x16 using tf_32x32 pred error, apply tf @ base only
#define TUNE_CFL_REF_ONLY                            1 // CFL only @ REF
#define TUNE_ENABLE_GM_FOR_ALL_PRESETS               1 // GM @ REF, bipred only, rotzoom model omly
#define FEATURE_GM_OPT                               1 // GM @ REF, bipred only, rotzoom model omly
#define TUNE_HME_ME_TUNING                           1 // HME/ME:HME_L1=8x3 instead of 16x16, HME_L2=8x3 instead of 16x16, MAX_ME=64x32 instead 64x64

#define FEATURE_RDOQ_OPT                             1 // lossless, early exit rdo, disables last md search tools (rdoq, txtype search, interpolation search)
#define DC_ONLY_AT_NON_REF                       1 // use only intra dc at no reference frame
#define TUNE_PALETTE_LEVEL                       1 // palette level will only be 6 for temporal layer == 0, not encode preset <=M3
#define FIX_TPL_TRAILING_FRAME_BUG               1 // fix bug related to ENABLE_TPL_TRAILING
#define TUNE_TPL_OIS                             1 // move ois to inloop TPL, can be done in me kernel with scs_ptr->in_loop_ois = 0
#define TUNE_TPL_RATE                            1 // remove  uncessary rate calculation

#define FIX_NIC_1_CLEAN_UP                           1 // Code clean-up/unification; Use scale to signal all PD NIC(s) and 1 NIC @ mds3
#define FEATURE_MDS0_ELIMINATE_CAND                  1 // Eliminate candidates based on the estimated cost of the distortion in mds0.
#define TUNE_TPL_TOWARD_CHROMA                       1 //Tune TPL for better chroma. Only for 240P
#define TUNE_NEW_PRESETS                             1 // Preset tuning for M0-M7
#define FIX_10BIT_CRASH                              1 // Fixed bug that caused encoder to crash with 10-bit clips
#define FIX_GM_BUG                                   1 // FIX GM r2r difference
#define FIX_ME_IDX_LUPT_ASSERT                       1 // change location of assert statement, code cleanup
#define FIX_IFS_10BIT                                1 // fix bug relating to IFS 10 bit error
#define FIX_GM_PARAMS_UPDATE                         1 // Fix GM r2r related to improper setting of GM params for NREF frames when GM is used for REF only
// END  svt-03 /////////////////////////////////////////////////////////

//FOR DEBUGGING - Do not remove
#define NO_ENCDEC         0 // bypass encDec to test cmpliance of MD. complained achieved when skip_flag is OFF. Port sample code from VCI-SW_AV1_Candidate1 branch
#define TUNE_CHROMA_SSIM  0 // Enable better Chroma/SSIM
#ifdef __cplusplus
}
#endif // __cplusplus

#endif // EbDebugMacros_h
