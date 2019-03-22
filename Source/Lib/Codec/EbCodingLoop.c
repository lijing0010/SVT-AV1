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
#include <string.h>

#include "EbDefinitions.h"
#include "EbUtility.h"
#include "EbTransformUnit.h"
#include "EbRateDistortionCost.h"
#include "EbDeblockingFilter.h"
#include "EbPictureOperators.h"

#include "EbModeDecisionProcess.h"
#include "EbEncDecProcess.h"
#include "EbErrorCodes.h"
#include "EbTransforms.h"
#include "EbModeDecisionConfiguration.h"
#include "EbIntraPrediction.h"
#include "aom_dsp_rtcd.h"
#if TX_SEARCH_LEVELS
#include "EbCodingLoop.h"
#endif

#define DEBUG_REF_INFO
#ifdef DEBUG_REF_INFO
static void dump_buf_desc_to_file(EbPictureBufferDesc_t* reconBuffer, const char* filename, int POC)
{
    const int bitDepth = reconBuffer->bit_depth;
    const int unitSize = ((bitDepth == 8) ? 1 : 2);
    const int colorFormat = reconBuffer->color_format;    // Chroma format
    const int subWidthCMinus1 = (colorFormat == EB_YUV444 ? 1 : 2) - 1;
    const int subHeightCMinus1 = (colorFormat >= EB_YUV422 ? 1 : 2) - 1;

    if (POC == 0) {
        FILE* tmp=fopen(filename, "w");
        fclose(tmp);
    }
    FILE* fp = fopen(filename, "r+");
    assert(fp);
    long descSize = reconBuffer->height * reconBuffer->width; //Luma
    descSize += 2 * ((reconBuffer->height * reconBuffer->width) >> (3 - reconBuffer->color_format)); //Chroma
    descSize = descSize * unitSize;
    long offset = descSize * POC;
    fseek(fp, 0, SEEK_END);
    long fileSize = ftell(fp);
    if (offset > fileSize) {
        int count = (offset - fileSize) / descSize;
        char *tmpBuf = (char*)malloc(descSize);
        for (int i=0;i<count;i++) {
            fwrite(tmpBuf, 1, descSize, fp);
        }
        free(tmpBuf);
    }
    fseek(fp, offset, SEEK_SET);
    assert(ftell(fp) == offset);


    unsigned char* luma_ptr = reconBuffer->bufferY +
        ((reconBuffer->strideY * (reconBuffer->origin_y) + reconBuffer->origin_x) * unitSize);
    unsigned char* cb_ptr = reconBuffer->bufferCb +
        ((reconBuffer->strideCb * (reconBuffer->origin_y >> subHeightCMinus1) + (reconBuffer->origin_x>>subWidthCMinus1)) * unitSize);
    unsigned char* cr_ptr = reconBuffer->bufferCr +
        ((reconBuffer->strideCr * (reconBuffer->origin_y >> subHeightCMinus1) + (reconBuffer->origin_x>>subWidthCMinus1)) * unitSize);

    for (int i=0; i<reconBuffer->height; i++) {
        fwrite(luma_ptr, 1, reconBuffer->width * unitSize, fp);
        luma_ptr += reconBuffer->strideY * unitSize;
    }

    for (int i=0; i<reconBuffer->height >> subHeightCMinus1 ;i++) {
        fwrite(cb_ptr, 1, (reconBuffer->width >> subWidthCMinus1) * unitSize, fp);
        cb_ptr += reconBuffer->strideCb * unitSize;
    }

    for (int i=0;i<reconBuffer->height>>subHeightCMinus1;i++) {
        fwrite(cr_ptr, 1, (reconBuffer->width >> subWidthCMinus1) * unitSize, fp);
        cr_ptr += reconBuffer->strideCr * unitSize;
    }
    fseek(fp, 0, SEEK_END);
    //printf("After write POC %d, filesize %d\n", POC, ftell(fp));
    fclose(fp);
}

static void dump_left_array(NeighborArrayUnit_t *neighbor, int y_pos, int size)
{
    int unitSize = neighbor->unitSize;
    printf("*Dump left array\n");
    for (int i=0; i<size; i++) {
        if (unitSize == 1) {
            printf("%3u ", neighbor->leftArray[(i+y_pos)*unitSize]);
        } else {
            printf("%3u ", *((uint16_t*)(neighbor->leftArray+(i+y_pos)*unitSize)));
        }
    }
    printf("\n----------------------\n");
}

static void dump_intra_ref(void* ref, int size, int mask, EbBool is16bit)
{
    
    unsigned char* ptr = NULL;
    unsigned int unitSize = is16bit ? 2 : 1;
    if (mask==0) {
        if (!is16bit) {
            ptr = ((IntraReferenceSamples_t*)ref)->y_intra_reference_array;
        } else {
            ptr = ((IntraReference16bitSamples_t*)ref)->y_intra_reference_array;
        }
    } else if (mask == 1) {
        if (!is16bit) {
            ptr = ((IntraReferenceSamples_t*)ref)->cbIntraReferenceArray;
        } else {
            ptr = ((IntraReference16bitSamples_t*)ref)->cbIntraReferenceArray;
        }
    } else if (mask ==2) {
        if (!is16bit) {
            ptr = ((IntraReferenceSamples_t*)ref)->crIntraReferenceArray;
        } else {
            ptr = ((IntraReference16bitSamples_t*)ref)->crIntraReferenceArray;
        }
    } else {
        assert(0);
    }

    printf("*Dumping intra reference array for component %d\n", mask);
    for (int i=0; i<size; i++) {
        if (is16bit) {
            printf("%3u ", *((uint16_t*)(ptr+i*unitSize)));
        } else {
            printf("%3u ", ptr[i]);
        }
    }
    printf("\n----------------------\n");
}

static void dump_block_from_desc(int txw, int txh, EbPictureBufferDesc_t *buf_tmp, int startX, int startY, int componentMask)
{
    unsigned char* buf=NULL;
    int stride=0;
    int bitDepth = buf_tmp->bit_depth;
    int val=(bitDepth==8)?1:2;
    EbColorFormat colorFormat = buf_tmp->color_format;    // Chroma format
    uint16_t subWidthCMinus1 = (colorFormat==EB_YUV444?1:2)-1;
    uint16_t subHeightCMinus1 = (colorFormat>=EB_YUV422?1:2)-1;
    if (componentMask ==0) {
        buf=buf_tmp->bufferY;
        stride=buf_tmp->strideY;
        subWidthCMinus1=0;
        subHeightCMinus1=0;
    } else if (componentMask == 1) {
        buf=buf_tmp->bufferCb;
        stride=buf_tmp->strideCb;
        startX=ROUND_UV_EX(startX, subWidthCMinus1);
        startY=ROUND_UV_EX(startY, subHeightCMinus1);
    } else if (componentMask == 2) {
        buf=buf_tmp->bufferCr;
        stride=buf_tmp->strideCr;
        startX=ROUND_UV_EX(startX, subWidthCMinus1);
        startY=ROUND_UV_EX(startY, subHeightCMinus1);
    } else {
        assert(0);
    }

    int offset=((stride*((buf_tmp->origin_y>>subHeightCMinus1) + startY))) + (startX+(buf_tmp->origin_x>>subWidthCMinus1));
    printf("bitDepth is %d, dump block size %dx%d at offset %d, (%d, %d), component is %s\n",
            bitDepth, txw, txh, offset, startX, startY, componentMask==0?"luma":(componentMask==1?"Cb":"Cr"));
            unsigned char* start_tmp=buf+offset*val;
            for (int i=0;i<txh;i++) {
                for (int j=0;j<txw+1;j++) {
                    if (j==txw) {
                        printf("|||");
                    } else if (j%4 == 0) {
                        printf("|");
                    }

                    if (bitDepth == 8) {
                        printf("%4u ", start_tmp[j]);
                    } else if (bitDepth == 10 || bitDepth == 16) {
                        printf("%4d ", *((int16_t*)start_tmp + j));
                    } else if (bitDepth == 32) {
                        printf("%4d ", *((int32_t*)start_tmp + j));
                    } else {
                        printf("bitDepth is %d\n", bitDepth);
                        assert(0);
                    }
                }
                printf("\n");
                start_tmp += stride*val;
            }
    printf("------------------------\n");
}
#endif
static const uint32_t me2Nx2NOffset[4] = { 0, 1, 5, 21 };
extern void av1_predict_intra_block_new(
#if TILES   
    TileInfo                    *tile,
#endif
#if INTRA_CORE_OPT
    ModeDecisionContext_t                  *md_context_ptr,
#endif
    STAGE       stage,
    uint8_t                     intra_luma_left_mode,
    uint8_t                     intra_luma_top_mode,
    uint8_t                     intra_chroma_left_mode,
    uint8_t                     intra_chroma_top_mode,
    const BlockGeom            *blk_geom,
    const Av1Common *cm,
    int32_t wpx,
    int32_t hpx,
    TxSize tx_size,
    PredictionMode mode,
    int32_t angle_delta,
    int32_t use_palette,
    FILTER_INTRA_MODE filter_intra_mode,
    uint8_t* topNeighArray,
    uint8_t* leftNeighArray,
    EbPictureBufferDesc_t  *reconBuffer,
#if !INTRA_CORE_OPT
    int32_t col_off,
    int32_t row_off,
#endif
    int32_t plane,
    BlockSize bsize,
    uint32_t bl_org_x_pict,
    uint32_t bl_org_y_pict,
    uint32_t bl_org_x_mb,
    uint32_t bl_org_y_mb);

extern void av1_predict_intra_block(
#if TILES   
    TileInfo                    *tile,
#endif
#if INTRA_CORE_OPT
    ModeDecisionContext_t                  *md_context_ptr,
#endif
    STAGE       stage,
    uint8_t                     intra_luma_left_mode,
    uint8_t                     intra_luma_top_mode,
    uint8_t                     intra_chroma_left_mode,
    uint8_t                     intra_chroma_top_mode,
    const BlockGeom            *blk_geom,
    const Av1Common *cm,
    int32_t wpx,
    int32_t hpx,
    TxSize tx_size,
    PredictionMode mode,
    int32_t angle_delta,
    int32_t use_palette,
    FILTER_INTRA_MODE filter_intra_mode,
    uint8_t* topNeighArray,
    uint8_t* leftNeighArray,
    EbPictureBufferDesc_t  *reconBuffer,
#if !INTRA_CORE_OPT
    int32_t col_off,
    int32_t row_off,
#endif
    int32_t plane,
    BlockSize bsize,
    uint32_t bl_org_x_pict,
    uint32_t bl_org_y_pict,
    uint32_t bl_org_x_mb,
    uint32_t bl_org_y_mb);

#if INTRA_10BIT_SUPPORT
void av1_predict_intra_block_16bit(
#if TILES   
    TileInfo               *tile,
#endif
    EncDecContext_t         *context_ptr,
    CodingUnit_t *cu_ptr,
    const Av1Common *cm,
    int32_t wpx,
    int32_t hpx,
    TxSize tx_size,
    PredictionMode mode,
    int32_t angle_delta,
    int32_t use_palette,
    FILTER_INTRA_MODE filter_intra_mode,
    uint16_t* topNeighArray,
    uint16_t* leftNeighArray,
    EbPictureBufferDesc_t  *reconBuffer,
    int32_t col_off,
    int32_t row_off,
    int32_t plane,
    BlockSize bsize,
    uint32_t bl_org_x_pict,
    uint32_t bl_org_y_pict);
#endif

/*******************************************
* set Penalize Skip Flag
*
* Summary: Set the PenalizeSkipFlag to true
* When there is luminance/chrominance change
* or in noisy clip with low motion at meduim
* varince area
*
*******************************************/

#define S32 32*32
#define S16 16*16
#define S8  8*8
#define S4  4*4

typedef void(*EB_AV1_INIT_NEIGHBOR_FUNC_PTR)(
    EncDecContext_t           *context_ptr,
    BlockGeom * blk_geom,
    NeighborArrayUnit_t *recon_neighbor_array,
    uint8_t      plane,
    uint8_t      subsampling_x,
    uint8_t      subsampling_y,
    void    *top_neigh_array,
    void    *left_neigh_array);

typedef void(*EB_AV1_ENCODE_LOOP_FUNC_PTR)(
#if ENCDEC_TX_SEARCH
    PictureControlSet_t    *picture_control_set_ptr,
#endif
    EncDecContext_t       *context_ptr,
    LargestCodingUnit_t   *sb_ptr,
    uint32_t                 origin_x,
    uint32_t                 origin_y,
    uint32_t                 offset_x,   //offset within a CU, for 422 chroma
    uint32_t                 offset_y,   //Offset within a CU
    uint32_t                 cbQp,
    EbPictureBufferDesc_t *predSamples,             // no basis/offset
    EbPictureBufferDesc_t *coeffSamplesTB,          // lcu based
    EbPictureBufferDesc_t *residual16bit,           // no basis/offset
    EbPictureBufferDesc_t *transform16bit,          // no basis/offset
    EbPictureBufferDesc_t *inverse_quant_buffer,
    int16_t                *transformScratchBuffer,
    EbAsm                 asm_type,
    uint32_t                  *count_non_zero_coeffs,
    uint32_t                 plane,
    uint32_t                   use_delta_qp,
    uint32_t                 dZoffset,
    uint16_t                 *eob,
    MacroblockPlane       *candidate_plane);


typedef void(*EB_AV1_GENERATE_RECON_FUNC_PTR)(
    EncDecContext_t       *context_ptr,
    uint32_t                 origin_x,
    uint32_t                 origin_y,
    EbPictureBufferDesc_t *predSamples,     // no basis/offset
    EbPictureBufferDesc_t *residual16bit,    // no basis/offset
    int16_t                *transformScratchBuffer,
    uint32_t                 plane,
    EbColorFormat            color_format,
    uint16_t                *eob,
    EbAsm                 asm_type);


typedef void(*EB_GENERATE_RECON_FUNC_PTR)(
    EncDecContext_t       *context_ptr,
    uint32_t                 origin_x,
    uint32_t                 origin_y,
    EbPictureBufferDesc_t *predSamples,     // no basis/offset
    EbPictureBufferDesc_t *residual16bit,    // no basis/offset
    int16_t                *transformScratchBuffer,
    EbAsm                 asm_type);

typedef void(*EB_GENERATE_RECON_INTRA_4x4_FUNC_PTR)(
    EncDecContext_t       *context_ptr,
    uint32_t                 origin_x,
    uint32_t                 origin_y,
    EbPictureBufferDesc_t *predSamples,     // no basis/offset
    EbPictureBufferDesc_t *residual16bit,    // no basis/offset
    int16_t                *transformScratchBuffer,
    uint32_t                 component_mask,
    EbAsm                 asm_type);

typedef EbErrorType(*EB_GENERATE_INTRA_SAMPLES_FUNC_PTR)(
    EbBool                         *is_left_availble,
    EbBool                         *is_above_availble,
    EbBool                     constrained_intra_flag,   //input parameter, indicates if constrained intra is switched on/off
    EbBool                     strongIntraSmoothingFlag,
    uint32_t                      origin_x,
    uint32_t                      origin_y,
    uint32_t                      size,
    uint32_t                      cu_depth,
    NeighborArrayUnit_t        *mode_type_neighbor_array,
    NeighborArrayUnit_t        *luma_recon_neighbor_array,
    NeighborArrayUnit_t        *cb_recon_neighbor_array,
    NeighborArrayUnit_t        *cr_recon_neighbor_array,
    void                       *refWrapperPtr,
    EbBool                     pictureLeftBoundary,
    EbBool                     pictureTopBoundary,
    EbBool                     pictureRightBoundary);
typedef EbErrorType(*EB_ENC_PASS_INTRA_FUNC_PTR)(
    uint8_t                          upsample_left,
    uint8_t                          upsample_above,
    uint8_t                          upsample_left_chroma,
    uint8_t                          upsample_above_chroma,
    EbBool                         is_left_availble,
    EbBool                         is_above_availble,
    void                       *referenceSamples,
    uint32_t                      origin_x,
    uint32_t                      origin_y,
    uint32_t                      puSize,
    EbPictureBufferDesc_t      *prediction_ptr,
    uint32_t                      lumaMode,
    uint32_t                      chroma_mode,
    int32_t                      angle_delta,
    uint16_t                      bitdepth,
    EbAsm                      asm_type);


/***************************************************
* Update Intra Mode Neighbor Arrays
***************************************************/
static void EncodePassUpdateIntraModeNeighborArrays(
    NeighborArrayUnit_t     *mode_type_neighbor_array,
    NeighborArrayUnit_t     *intra_mode_neighbor_array,
    uint8_t                  intra_mode,
    uint8_t                  plane,
    uint32_t                 pu_origin_x,
    uint32_t                 pu_origin_y,
    uint32_t                 tx_width,
    uint32_t                 tx_height)
{
    uint8_t modeType = INTRA_MODE;

    if (plane == 0) {
        // Mode Type Update
        NeighborArrayUnitModeWrite(
            mode_type_neighbor_array,
            &modeType,
            pu_origin_x,
            pu_origin_y,
            tx_width,
            tx_height,
            NEIGHBOR_ARRAY_UNIT_FULL_MASK);
    }

    // Intra Mode Update
    NeighborArrayUnitModeWrite(
            intra_mode_neighbor_array,
            &intra_mode,
            pu_origin_x,
            pu_origin_y,
            tx_width,
            tx_height,
            NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
    return;
}

/***************************************************
* Update Inter Mode Neighbor Arrays
***************************************************/
static void EncodePassUpdateInterModeNeighborArrays(
    NeighborArrayUnit_t     *mode_type_neighbor_array,
    NeighborArrayUnit_t     *mv_neighbor_array,
    NeighborArrayUnit_t     *skipNeighborArray,
    MvUnit_t                *mv_unit,
    uint8_t                 *skip_flag,
    uint32_t                 origin_x,
    uint32_t                 origin_y,
    uint32_t                 bwidth,
    uint32_t                 bheight)
{
    uint8_t modeType = INTER_MODE;

    // Mode Type Update
    NeighborArrayUnitModeWrite(
        mode_type_neighbor_array,
        &modeType,
        origin_x,
        origin_y,
        bwidth,
        bheight,
        NEIGHBOR_ARRAY_UNIT_FULL_MASK);

    // Motion Vector Unit
    NeighborArrayUnitModeWrite(
        mv_neighbor_array,
        (uint8_t*)mv_unit,
        origin_x,
        origin_y,
        bwidth,
        bheight,
        NEIGHBOR_ARRAY_UNIT_FULL_MASK);

    // Skip Flag
    NeighborArrayUnitModeWrite(
        skipNeighborArray,
        skip_flag,
        origin_x,
        origin_y,
        bwidth,
        bheight,
        NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);

    return;
}

/***************************************************
* Update Recon Samples Neighbor Arrays
***************************************************/
static void EncodePassUpdateReconSampleNeighborArrays(
    NeighborArrayUnit_t     *reconSampleNeighborArray,
    EbPictureBufferDesc_t   *reconBuffer,
    uint32_t                 plane,
    uint32_t                 pu_origin_x,
    uint32_t                 pu_origin_y,
    uint32_t                 tx_width,
    uint32_t                 tx_height,
    EbBool                   is16bit)
{
    EbByte recon_ptr;
    uint32_t recon_stride;
    EbByte src_origin_x; 
    EbByte src_origin_y; 

    if (plane == 0) {
        recon_ptr = reconBuffer->bufferY;
        recon_stride = reconBuffer->strideY;
        src_origin_x = reconBuffer->origin_x + pu_origin_x;
        src_origin_y = reconBuffer->origin_y + pu_origin_y;
    } else if (plane == 1) {
        const uint8_t subsampling_x = (reconBuffer->color_format == EB_YUV444 ? 1 : 2) - 1;
        const uint8_t subsampling_y = (reconBuffer->color_format >= EB_YUV422 ? 1 : 2) - 1;
        recon_ptr = reconBuffer->bufferCb;
        recon_stride = reconBuffer->strideCb;
        src_origin_x = (reconBuffer->origin_x >> subsampling_x) + pu_origin_x;
        src_origin_y = (reconBuffer->origin_y >> subsampling_y) + pu_origin_y;
    } else {
        const uint8_t subsampling_x = (reconBuffer->color_format == EB_YUV444 ? 1 : 2) - 1;
        const uint8_t subsampling_y = (reconBuffer->color_format >= EB_YUV422 ? 1 : 2) - 1;
        recon_ptr = reconBuffer->bufferCr;
        recon_stride = reconBuffer->strideCr;
        src_origin_x = (reconBuffer->origin_x >> subsampling_x) + pu_origin_x;
        src_origin_y = (reconBuffer->origin_y >> subsampling_y) + pu_origin_y;
    }
    //Get2dOrigin(pu_origin_x, pu_origin_y, reconBuffer, plane, &recon_ptr, &recon_stride);

    if (is16bit == EB_TRUE) {
        NeighborArrayUnit16bitSampleWrite(
                reconSampleNeighborArray,
                (uint16_t*)(recon_ptr),
                recon_stride,
                src_origin_x,
                src_origin_y,
                pu_origin_x,
                pu_origin_y,
                tx_width,
                tx_height,
                NEIGHBOR_ARRAY_UNIT_FULL_MASK);
    } else {
        NeighborArrayUnitSampleWrite(
                reconSampleNeighborArray,
                recon_ptr,
                recon_stride,
                src_origin_x,
                src_origin_y,
                pu_origin_x,
                pu_origin_y,
                tx_width,
                tx_height,
                NEIGHBOR_ARRAY_UNIT_FULL_MASK);
    }

    return;
}
static void GeneratePuNeighborModes(
    EncDecContext_t         *context_ptr,
    uint32_t                 pu_origin_x,
    uint32_t                 pu_origin_y,
    NeighborArrayUnit_t     *mode_type_neighbor_array)
{
    uint32_t modeTypeLeftNeighborIndex = GetNeighborArrayUnitLeftIndex(
        mode_type_neighbor_array,
        pu_origin_y);

    uint32_t modeTypeTopNeighborIndex = GetNeighborArrayUnitTopIndex(
        mode_type_neighbor_array,
        pu_origin_x);

    context_ptr->cu_left_mode_type = mode_type_neighbor_array->leftArray[modeTypeLeftNeighborIndex];
    context_ptr->cu_top_mode_type = mode_type_neighbor_array->topArray[modeTypeTopNeighborIndex];

    return;
}

static void GeneratePuIntraNeighborModes(
    EncDecContext_t         *context_ptr,
    CodingUnit_t            *cu_ptr,
    uint32_t                 plane,
    uint32_t                 pu_origin_x,
    uint32_t                 pu_origin_y,
    NeighborArrayUnit_t     *intra_neighbor_array)
{
    uint32_t intraModeLeftNeighborIndex = GetNeighborArrayUnitLeftIndex(
        intra_neighbor_array,
        pu_origin_y);
        
    uint32_t intraModeTopNeighborIndex = GetNeighborArrayUnitTopIndex(
        intra_neighbor_array,
        pu_origin_x);

    if (plane == 0) {
        (&cu_ptr->prediction_unit_array[0])->intra_luma_left_mode = (uint32_t)(
            (context_ptr->cu_left_mode_type != INTRA_MODE) ? DC_PRED/*EB_INTRA_DC*/ :
            (uint32_t)intra_neighbor_array->leftArray[intraModeLeftNeighborIndex]);

        (&cu_ptr->prediction_unit_array[0])->intra_luma_top_mode = (uint32_t)(
            (context_ptr->cu_top_mode_type != INTRA_MODE) ? DC_PRED/*EB_INTRA_DC*/ :
            (uint32_t)intra_neighbor_array->topArray[intraModeTopNeighborIndex]);       //   use DC. This seems like we could use a LCU-width
    } else {
        (&cu_ptr->prediction_unit_array[0])->intra_chroma_left_mode = (uint32_t)(
            (context_ptr->cu_left_mode_type != INTRA_MODE) ? UV_DC_PRED :
            (uint32_t)intra_neighbor_array->leftArray[intraModeLeftNeighborIndex]);

        (&cu_ptr->prediction_unit_array[0])->intra_chroma_top_mode = (uint32_t)(
            (context_ptr->cu_top_mode_type != INTRA_MODE) ? UV_DC_PRED :
            (uint32_t)intra_neighbor_array->topArray[intraModeTopNeighborIndex]);       //   use DC. This seems like we could use a LCU-width
    }
    return;
}

void PfZeroOutUselessQuadrants(
    int16_t* transformCoeffBuffer,
    uint32_t  transformCoeffStride,
    uint32_t  quadrantSize,
    EbAsm  asm_type) {

    PicZeroOutCoef_funcPtrArray[asm_type][quadrantSize >> 3](
        transformCoeffBuffer,
        transformCoeffStride,
        quadrantSize,
        quadrantSize,
        quadrantSize);

    PicZeroOutCoef_funcPtrArray[asm_type][quadrantSize >> 3](
        transformCoeffBuffer,
        transformCoeffStride,
        quadrantSize * transformCoeffStride,
        quadrantSize,
        quadrantSize);

    PicZeroOutCoef_funcPtrArray[asm_type][quadrantSize >> 3](
        transformCoeffBuffer,
        transformCoeffStride,
        quadrantSize * transformCoeffStride + quadrantSize,
        quadrantSize,
        quadrantSize);

}

void encode_pass_tx_search(
    PictureControlSet_t            *picture_control_set_ptr,
    EncDecContext_t                *context_ptr,
    LargestCodingUnit_t            *sb_ptr,
    uint32_t                       cbQp,
    EbPictureBufferDesc_t          *coeffSamplesTB,
    EbPictureBufferDesc_t          *residual16bit,
    EbPictureBufferDesc_t          *transform16bit,
    EbPictureBufferDesc_t          *inverse_quant_buffer,
    int16_t                        *transformScratchBuffer,
    EbAsm                          asm_type,
    uint32_t                       *count_non_zero_coeffs,
    uint32_t                       component_mask,
    uint32_t                       use_delta_qp,
    uint32_t                       dZoffset,
    uint16_t                       *eob,
    MacroblockPlane                *candidate_plane);

static void InitNeighborArray16bit(
    EncDecContext_t           *context_ptr,
    BlockGeom * blk_geom,
    NeighborArrayUnit_t *recon_neighbor_array,
    uint8_t      plane,
    uint8_t      subsampling_x,
    uint8_t      subsampling_y,
    uint16_t*    top_neigh_array,
    uint16_t*    left_neigh_array)
{
    uint32_t cu_originy_uv = (context_ptr->cu_origin_y >> 3 << 3) >> subsampling_y;
    uint32_t cu_originx_uv = (context_ptr->cu_origin_x >> 3 << 3) >> subsampling_x;
    if (plane == 0) {
        if (context_ptr->cu_origin_y != 0)
            memcpy((uint16_t*)top_neigh_array + 1, (uint16_t*)(recon_neighbor_array->topArray) + context_ptr->cu_origin_x, blk_geom->bwidth * 2 * sizeof(uint16_t));
        if (context_ptr->cu_origin_x != 0)
            memcpy((uint16_t*)left_neigh_array + 1, (uint16_t*)(recon_neighbor_array->leftArray) + context_ptr->cu_origin_y, blk_geom->bheight * 2 * sizeof(uint16_t));
        if (context_ptr->cu_origin_y != 0 && context_ptr->cu_origin_x != 0)
            top_neigh_array[0] = left_neigh_array[0] = ((uint16_t*)(recon_neighbor_array->topLeftArray) + MAX_PICTURE_HEIGHT_SIZE + context_ptr->cu_origin_x - context_ptr->cu_origin_y)[0];
    } else if (plane == 1) {
        if (cu_originy_uv != 0)
            memcpy((uint16_t*)top_neigh_array+ 1, (uint16_t*)(recon_neighbor_array->topArray) + cu_originx_uv, blk_geom->bwidth_uv_ex * 2 * sizeof(uint16_t));
        if (cu_originx_uv != 0)
            memcpy((uint16_t*)left_neigh_array + 1, (uint16_t*)(recon_neighbor_array->leftArray) + cu_originy_uv, blk_geom->bheight_uv_ex * 2 * sizeof(uint16_t));
        if (cu_originy_uv != 0 && cu_originx_uv != 0)
            top_neigh_array[0] = left_neigh_array[0] = ((uint16_t*)(recon_neighbor_array->topLeftArray) + (MAX_PICTURE_HEIGHT_SIZE >> subsampling_y) + cu_originx_uv - cu_originy_uv)[0];
    } else {
        if (cu_originy_uv != 0)
            memcpy((uint16_t*)top_neigh_array + 1, (uint16_t*)(recon_neighbor_array->topArray) + cu_originx_uv, blk_geom->bwidth_uv_ex * 2 * sizeof(uint16_t));
        if (cu_originx_uv != 0)
            memcpy((uint16_t*)left_neigh_array + 1, (uint16_t*)(recon_neighbor_array->leftArray) + cu_originy_uv, blk_geom->bheight_uv_ex * 2 * sizeof(uint16_t));
        if (cu_originy_uv != 0 && cu_originx_uv != 0)
            top_neigh_array[0] = left_neigh_array[0] = ((uint16_t*)(recon_neighbor_array->topLeftArray) + (MAX_PICTURE_HEIGHT_SIZE >> subsampling_y) + cu_originx_uv - cu_originy_uv)[0];
    }
}

static void InitNeighborArray(
    EncDecContext_t     *context_ptr,
    const BlockGeom     *blk_geom,
    NeighborArrayUnit_t *recon_neighbor_array,
    uint8_t      plane,
    uint8_t      subsampling_x,
    uint8_t      subsampling_y,
    uint8_t     *top_neigh_array,
    uint8_t     *left_neigh_array)
{
    uint32_t cu_originx_uv = ROUND_UV_EX(context_ptr->cu_origin_x ,subsampling_x);
    uint32_t cu_originy_uv = ROUND_UV_EX(context_ptr->cu_origin_y ,subsampling_y);
    if (plane == 0) {
        if (context_ptr->cu_origin_y != 0)
            memcpy((uint8_t*)top_neigh_array + 1, recon_neighbor_array->topArray + context_ptr->cu_origin_x, blk_geom->bwidth * 2);
        if (context_ptr->cu_origin_x != 0)
            memcpy((uint8_t*)left_neigh_array + 1, recon_neighbor_array->leftArray + context_ptr->cu_origin_y, blk_geom->bheight * 2);
        if (context_ptr->cu_origin_y != 0 && context_ptr->cu_origin_x != 0)
            top_neigh_array[0] = left_neigh_array[0] = ((recon_neighbor_array->topLeftArray) + MAX_PICTURE_HEIGHT_SIZE + context_ptr->cu_origin_x - context_ptr->cu_origin_y)[0];
    } else {
        if (cu_originy_uv != 0)
            memcpy((uint8_t*)top_neigh_array + 1, recon_neighbor_array->topArray + cu_originx_uv, blk_geom->bwidth_uv_ex * 2);
        if (cu_originx_uv != 0)
            memcpy((uint8_t*)left_neigh_array + 1, recon_neighbor_array->leftArray + cu_originy_uv, blk_geom->bheight_uv_ex * 2);
        if (cu_originy_uv != 0 && cu_originx_uv != 0)
            top_neigh_array[0] = left_neigh_array[0] = (recon_neighbor_array->topLeftArray + (MAX_PICTURE_HEIGHT_SIZE >> subsampling_y) + cu_originx_uv - cu_originy_uv)[0];
    }
}

static void Get1dOrigin(uint32_t offset, EbPictureBufferDesc_t *desc, uint8_t p, EbByte *ptr)
{
    uint8_t subsampling_x = (p == 0) ? 0 : ((desc->color_format == EB_YUV444 ? 1 : 2) - 1);
    uint8_t subsampling_y = (p == 0) ? 0 : ((desc->color_format >= EB_YUV422 ? 1 : 2) - 1);
    uint8_t desc_depth_size = (desc->bit_depth == EB_8BIT) ? 1 :
                               (desc->bit_depth == EB_32BIT) ? 4 : 2;

    uint32_t inputOffset = offset * desc_depth_size;
    EbByte desc_buf = (p == 0) ? desc->bufferY: 
                      (p == 1) ? desc->bufferCb : desc->bufferCr;
    *ptr = desc_buf + inputOffset;
}

static void Get2dOrigin(uint32_t origin_x, uint32_t origin_y, EbPictureBufferDesc_t* desc, uint8_t p, EbByte *ptr, uint32_t* stride)
{
    uint8_t subsampling_x = (p == 0) ? 0 : ((desc->color_format == EB_YUV444 ? 1 : 2) - 1);
    uint8_t subsampling_y = (p == 0) ? 0 : ((desc->color_format >= EB_YUV422 ? 1 : 2) - 1);

    uint32_t desc_origin_x = desc->origin_x >> subsampling_x;
    uint32_t desc_origin_y = desc->origin_y >> subsampling_y;
    uint32_t plane_stride = (p == 0) ? desc->strideY:
                            (p == 1) ? desc->strideCb: desc->strideCr;
    *stride = plane_stride;

    uint8_t desc_depth_size = (desc->bit_depth == EB_8BIT) ? 1 :
                               (desc->bit_depth == EB_32BIT) ? 4 : 2;

    uint32_t inputOffset = ((origin_y + desc_origin_y) * plane_stride + (origin_x + desc_origin_x)) * desc_depth_size;

    EbByte desc_buf = (p == 0) ? desc->bufferY: 
                      (p == 1) ? desc->bufferCb : desc->bufferCr;
    *ptr = desc_buf + inputOffset;
}

/**********************************************************
* Encode Loop
*
* Summary: Performs a H.265 conformant
*   Transform, Quantization  and Inverse Quantization of a TU.
*
* Inputs:
*   origin_x
*   origin_y
*   txb_size
*   sb_sz
*   input - input samples (position sensitive)
*   pred - prediction samples (position independent)
*
* Outputs:
*   Inverse quantized coeff - quantization indices (position sensitive)
*
**********************************************************/
static void Av1EncodeLoop(
#if ENCDEC_TX_SEARCH
    PictureControlSet_t    *picture_control_set_ptr,
#endif
    EncDecContext_t       *context_ptr,
    LargestCodingUnit_t   *sb_ptr,
    uint32_t               sb_origin_x,   //pic based tx org x
    uint32_t               sb_origin_y,   //pic based tx org y
    uint32_t               origin_x,   //pic based tx org x
    uint32_t               origin_y,   //pic based tx org y
    TxSize                 tx_size, 
    uint32_t               cbQp,
    EbPictureBufferDesc_t *predSamples,             // no basis/offset
    EbPictureBufferDesc_t *coeffSamplesTB,          // lcu based
    EbPictureBufferDesc_t *residual16bit,           // no basis/offset
    EbPictureBufferDesc_t *transform32bit,          // no basis/offset
    EbPictureBufferDesc_t *inverse_quant_buffer,
    int16_t                *transformScratchBuffer,
    EbAsm                 asm_type,
    uint32_t                  *count_non_zero_coeffs,
    uint32_t                 plane,
    uint32_t                   use_delta_qp,
    uint32_t                 dZoffset,
    uint16_t                  txb_itr,
    uint16_t                 *eob,
    MacroblockPlane       *candidate_plane)
{

    (void)dZoffset;
    (void)use_delta_qp;
    (void)cbQp;

    const int32_t txw = tx_size_wide[tx_size];
    const int32_t txh = tx_size_high[tx_size];
    const EbColorFormat             color_format = context_ptr->color_format;
    const uint16_t subsampling_x = (color_format == EB_YUV444 ? 1 : 2) - 1;
    const uint16_t subsampling_y = (color_format >= EB_YUV422 ? 1 : 2) - 1;
    //    uint32_t                 chroma_qp = cbQp;
    CodingUnit_t          *cu_ptr = context_ptr->cu_ptr;

    //Jing:
    //TODO: need to have more than 1 tu for 422 & 444
    TransformUnit_t       *txb_ptr = &cu_ptr->transform_unit_array[txb_itr];
    //    EB_SLICE               slice_type = sb_ptr->picture_control_set_ptr->slice_type;
    //    uint32_t                 temporal_layer_index = sb_ptr->picture_control_set_ptr->temporal_layer_index;
    uint32_t                 qp = cu_ptr->qp;
    EbPictureBufferDesc_t  *input_samples = context_ptr->input_samples;

    //2D
    EbByte input_ptr[3];
    EbByte pred_ptr[3];
    EbByte res_ptr[3];
    uint32_t input_stride[3];
    uint32_t pred_stride[3];
    uint32_t res_stride[3];

    //1D
    EbByte trans_ptr[3];
    EbByte coeff_ptr[3];
    EbByte inverse_ptr[3];

    for (int p = 0; p < 3; p++) {
        uint32_t sb_plane_origin_x = (p == 0) ? sb_origin_x : sb_origin_x >> subsampling_x;
        uint32_t sb_plane_origin_y = (p == 0) ? sb_origin_y : sb_origin_y >> subsampling_y;
        //uint32_t offset1d = (p == 0) ? context_ptr->coded_area_sb : context_ptr->coded_area_sb_uv[p];
        Get2dOrigin(origin_x, origin_y, input_samples, p, &input_ptr[p], &input_stride[p]);
        Get2dOrigin(origin_x, origin_y, predSamples, p, &pred_ptr[p], &pred_stride[p]);
        Get2dOrigin(origin_x - sb_plane_origin_x, origin_y - sb_plane_origin_y, residual16bit, p, &res_ptr[p], &res_stride[p]);

        Get1dOrigin(context_ptr->coded_area_sb[p], transform32bit, p, &trans_ptr[p]);
        Get1dOrigin(context_ptr->coded_area_sb[p], coeffSamplesTB, p, &coeff_ptr[p]);
        Get1dOrigin(context_ptr->coded_area_sb[p], inverse_quant_buffer, p, &inverse_ptr[p]);
    }

    EbBool cleanSparseCoeffFlag = EB_FALSE;

    context_ptr->three_quad_energy = 0;

    {
        PLANE_TYPE p_type = (plane == 0) ? PLANE_TYPE_Y : PLANE_TYPE_UV;
        COMPONENT_TYPE c_type = (plane == 0) ? COMPONENT_LUMA : (plane == 1) ? COMPONENT_CHROMA_CB: COMPONENT_CHROMA_CR;
        EB_TRANS_COEFF_SHAPE trans_shape = (plane == 0) ? context_ptr->trans_coeff_shape_luma : context_ptr->trans_coeff_shape_chroma;

        ResidualKernel(
            input_ptr[plane],
            input_stride[plane],
            pred_ptr[plane],
            pred_stride[plane],
            res_ptr[plane],
            res_stride[plane],
            txw,
            txh);

        Av1EstimateTransform(
            res_ptr[plane],
            res_stride[plane],
            trans_ptr[plane],
            NOT_USED_VALUE,
            tx_size,
            &context_ptr->three_quad_energy,
            transformScratchBuffer,
            BIT_INCREMENT_8BIT,
            txb_ptr->transform_type[p_type],
            asm_type,
            p_type,
            trans_shape);
            
        Av1QuantizeInvQuantize(
            sb_ptr->picture_control_set_ptr,
            trans_ptr[plane],
            NOT_USED_VALUE,
            coeff_ptr[plane],
            inverse_ptr[plane],
            qp,
            txw, txh, tx_size,
            &eob[plane],
            candidate_plane[plane],
            asm_type,
            &(count_non_zero_coeffs[plane]),
            0,
            0,
            c_type,
            BIT_INCREMENT_8BIT,
            txb_ptr->transform_type[p_type],
            cleanSparseCoeffFlag);

        {
            //debug code
            //uint32_t coeff_num = txw * txh;
            //uint32_t nz_count = 0;
            //for (int i=0; i < coeff_num; i++) {
            //    uint32_t* ptr = (uint32_t*)(coeff_ptr[plane]);
            //    if (ptr[i] != 0) {
            //        nz_count++;
            //    }
            //}
            //assert(nz_count == count_non_zero_coeffs[plane]);
        }

        //Jing: TODO double check here
        if (plane == 0) {
            txb_ptr->y_has_coeff = count_non_zero_coeffs[0] ? EB_TRUE : EB_FALSE;
            txb_ptr->trans_coeff_shape_luma = context_ptr->trans_coeff_shape_luma;
        } else if (plane == 1) {
            txb_ptr->u_has_coeff = count_non_zero_coeffs[1] ? EB_TRUE : EB_FALSE;
            txb_ptr->trans_coeff_shape_chroma = context_ptr->trans_coeff_shape_chroma;
        } else {
            txb_ptr->v_has_coeff = count_non_zero_coeffs[2] ? EB_TRUE : EB_FALSE;
            txb_ptr->trans_coeff_shape_chroma = context_ptr->trans_coeff_shape_chroma;
        }
        txb_ptr->nz_coef_count[plane] = (uint16_t)count_non_zero_coeffs[plane];
    }

#ifdef DEBUG_REF_INFO
    if (sb_origin_x == 0 && sb_origin_y == 1) {
        printf("shape_luma %d, shape chroma %d, nz_coef_count (%d, %d, %d), txb_itr is %d\n",
                txb_ptr->trans_coeff_shape_luma,
                txb_ptr->trans_coeff_shape_chroma,
                txb_ptr->nz_coef_count[0],
                txb_ptr->nz_coef_count[1],
                txb_ptr->nz_coef_count[2], txb_itr);
        {
            int originX = origin_x; 
            int originY = origin_y; 
            printf("\nAbout to dump coeff for (%d, %d) at plane %d, size %dx%d\n",
                    originX, originY, plane, txw, txh);
            //dump_block_from_desc(txw, txh, coeffSamplesTB, originX, originY, plane);
        }
    }
#endif
    return;
}

void encode_pass_tx_search_hbd(
    PictureControlSet_t            *picture_control_set_ptr,
    EncDecContext_t                *context_ptr,
    LargestCodingUnit_t            *sb_ptr,
    uint32_t                       cbQp,
    EbPictureBufferDesc_t          *coeffSamplesTB,
    EbPictureBufferDesc_t          *residual16bit,
    EbPictureBufferDesc_t          *transform16bit,
    EbPictureBufferDesc_t          *inverse_quant_buffer,
    int16_t                        *transformScratchBuffer,
    EbAsm                          asm_type,
    uint32_t                       *count_non_zero_coeffs,
    uint32_t                       component_mask,
    uint32_t                       use_delta_qp,
    uint32_t                       dZoffset,
    uint16_t                       *eob,
    MacroblockPlane                *candidate_plane);



/**********************************************************
* Encode Loop
*
* Summary: Performs a H.265 conformant
*   Transform, Quantization  and Inverse Quantization of a TU.
*
* Inputs:
*   origin_x
*   origin_y
*   txb_size
*   sb_sz
*   input - input samples (position sensitive)
*   pred - prediction samples (position independent)
*
* Outputs:
*   Inverse quantized coeff - quantization indices (position sensitive)
*
**********************************************************/
static void Av1EncodeLoop16bit(
#if ENCDEC_TX_SEARCH
    PictureControlSet_t    *picture_control_set_ptr,
#endif
    EncDecContext_t       *context_ptr,
    LargestCodingUnit_t   *sb_ptr,
    uint32_t                 origin_x,
    uint32_t                 origin_y,
    uint32_t                 offset_x,   //offset within a CU, for 422 chroma
    uint32_t                 offset_y,   //Offset within a CU
    uint32_t                 cbQp,
    EbPictureBufferDesc_t *predSamples,         // no basis/offset
    EbPictureBufferDesc_t *coeffSamplesTB,      // lcu based
    EbPictureBufferDesc_t *residual16bit,       // no basis/offset
    EbPictureBufferDesc_t *transform16bit,      // no basis/offset
    EbPictureBufferDesc_t *inverse_quant_buffer,
    int16_t                *transformScratchBuffer,
    EbAsm                 asm_type,
    uint32_t                  *count_non_zero_coeffs,
    uint32_t                 component_mask,
    uint32_t                   use_delta_qp,
    uint32_t                 dZoffset,
    uint16_t                 *eob,
    MacroblockPlane       *candidate_plane)

{
    (void)use_delta_qp;
    (void)dZoffset;
#if QT_10BIT_SUPPORT
    (void)cbQp;
#else
    uint32_t                 chroma_qp = cbQp;
#endif

    CodingUnit_t          *cu_ptr = context_ptr->cu_ptr;
    TransformUnit_t       *txb_ptr = &cu_ptr->transform_unit_array[context_ptr->txb_itr];
    //    EB_SLICE               slice_type = sb_ptr->picture_control_set_ptr->slice_type;
    //    uint32_t                 temporal_layer_index = sb_ptr->picture_control_set_ptr->temporal_layer_index;
    uint32_t                 qp = cu_ptr->qp;

    EbPictureBufferDesc_t *inputSamples16bit = context_ptr->input_sample16bit_buffer;
    EbPictureBufferDesc_t *predSamples16bit = predSamples;
    uint32_t                 round_origin_x = (origin_x >> 3) << 3;// for Chroma blocks with size of 4
    uint32_t                 round_origin_y = (origin_y >> 3) << 3;// for Chroma blocks with size of 4
    const uint32_t           inputLumaOffset = context_ptr->blk_geom->tx_org_x[context_ptr->txb_itr] + context_ptr->blk_geom->tx_org_y[context_ptr->txb_itr] * SB_STRIDE_Y;
    const uint32_t           inputCbOffset = ROUND_UV(context_ptr->blk_geom->tx_org_x[context_ptr->txb_itr]) / 2 + ROUND_UV(context_ptr->blk_geom->tx_org_y[context_ptr->txb_itr]) / 2 * SB_STRIDE_UV;
    const uint32_t           inputCrOffset = ROUND_UV(context_ptr->blk_geom->tx_org_x[context_ptr->txb_itr]) / 2 + ROUND_UV(context_ptr->blk_geom->tx_org_y[context_ptr->txb_itr]) / 2 * SB_STRIDE_UV;
    const uint32_t           predLumaOffset = ((predSamples16bit->origin_y + origin_y)        * predSamples16bit->strideY) + (predSamples16bit->origin_x + origin_x);
    const uint32_t           predCbOffset = (((predSamples16bit->origin_y + round_origin_y) >> 1)  * predSamples16bit->strideCb) + ((predSamples16bit->origin_x + round_origin_x) >> 1);
    const uint32_t           predCrOffset = (((predSamples16bit->origin_y + round_origin_y) >> 1)  * predSamples16bit->strideCr) + ((predSamples16bit->origin_x + round_origin_x) >> 1);
    const uint32_t scratchLumaOffset = context_ptr->blk_geom->origin_x + context_ptr->blk_geom->origin_y * SB_STRIDE_Y;
    const uint32_t scratchCbOffset = ROUND_UV(context_ptr->blk_geom->origin_x) / 2 + ROUND_UV(context_ptr->blk_geom->origin_y) / 2 * SB_STRIDE_UV;
    const uint32_t scratchCrOffset = ROUND_UV(context_ptr->blk_geom->origin_x) / 2 + ROUND_UV(context_ptr->blk_geom->origin_y) / 2 * SB_STRIDE_UV;

#if QT_10BIT_SUPPORT
    const uint32_t coeff1dOffset = context_ptr->coded_area_sb;
    const uint32_t coeff1dOffsetChroma = context_ptr->coded_area_sb_uv;
    UNUSED(coeff1dOffsetChroma);
#endif

    EbBool cleanSparseCoeffFlag = EB_FALSE;

    //Update QP for Quant
    qp += QP_BD_OFFSET;
#if !QT_10BIT_SUPPORT
    chroma_qp += QP_BD_OFFSET;
#endif

    {

        //**********************************
        // Luma
        //**********************************
        if (component_mask == PICTURE_BUFFER_DESC_FULL_MASK || component_mask == PICTURE_BUFFER_DESC_LUMA_MASK) {

            ResidualKernel16bit(
                ((uint16_t*)inputSamples16bit->bufferY) + inputLumaOffset,
                inputSamples16bit->strideY,
                ((uint16_t*)predSamples16bit->bufferY) + predLumaOffset,
                predSamples16bit->strideY,
                ((int16_t*)residual16bit->bufferY) + scratchLumaOffset,
                residual16bit->strideY,
                context_ptr->blk_geom->tx_width[context_ptr->txb_itr],
                context_ptr->blk_geom->tx_height[context_ptr->txb_itr]);

#if TX_SEARCH_LEVELS
            uint8_t  tx_search_skip_fag = picture_control_set_ptr->parent_pcs_ptr->tx_search_level == TX_SEARCH_ENC_DEC ? get_skip_tx_search_flag(
                context_ptr->blk_geom->sq_size,
                MAX_MODE_COST,
                0,
                1) : 1;

            if (!tx_search_skip_fag) {
#else
#if ENCDEC_TX_SEARCH
#if ENCODER_MODE_CLEANUP
            if (picture_control_set_ptr->enc_mode > ENC_M1) {
#endif
                if (context_ptr->blk_geom->sq_size < 128) //no tx search for 128x128 for now
#endif
#endif
                    encode_pass_tx_search_hbd(
                        picture_control_set_ptr,
                        context_ptr,
                        sb_ptr,
                        cbQp,
                        coeffSamplesTB,
                        residual16bit,
                        transform16bit,
                        inverse_quant_buffer,
                        transformScratchBuffer,
                        asm_type,
                        count_non_zero_coeffs,
                        component_mask,
                        use_delta_qp,
                        dZoffset,
                        eob,
                        candidate_plane);
#if ENCODER_MODE_CLEANUP
            }
#endif


            Av1EstimateTransform(
                ((int16_t*)residual16bit->bufferY) + scratchLumaOffset,
                residual16bit->strideY,
                ((tran_low_t*)transform16bit->bufferY) + coeff1dOffset,
                NOT_USED_VALUE,
                context_ptr->blk_geom->txsize[context_ptr->txb_itr],
                &context_ptr->three_quad_energy,
                transformScratchBuffer,
                BIT_INCREMENT_10BIT,
                txb_ptr->transform_type[PLANE_TYPE_Y],
                asm_type,
                PLANE_TYPE_Y,
                context_ptr->trans_coeff_shape_luma);

            Av1QuantizeInvQuantize(
                sb_ptr->picture_control_set_ptr,
                ((int32_t*)transform16bit->bufferY) + coeff1dOffset,
                NOT_USED_VALUE,
#if QT_10BIT_SUPPORT
                ((int32_t*)coeffSamplesTB->bufferY) + coeff1dOffset,
#else
                ((int32_t*)coeffSamplesTB->bufferY) + scratchLumaOffset,
#endif
                ((int32_t*)inverse_quant_buffer->bufferY) + coeff1dOffset,
                qp,
                context_ptr->blk_geom->tx_width[context_ptr->txb_itr],
                context_ptr->blk_geom->tx_height[context_ptr->txb_itr],
                context_ptr->blk_geom->txsize[context_ptr->txb_itr],
                &eob[0],
                candidate_plane[0],
                asm_type,
                &(count_non_zero_coeffs[0]),
                0,
                0,
                COMPONENT_LUMA,
#if QT_10BIT_SUPPORT
                BIT_INCREMENT_10BIT,
#endif
                txb_ptr->transform_type[PLANE_TYPE_Y],
                cleanSparseCoeffFlag);
            txb_ptr->y_has_coeff = count_non_zero_coeffs[0] ? EB_TRUE : EB_FALSE;
#if TX_TYPE_FIX
            if (count_non_zero_coeffs[0] == 0) {
                // INTER. Chroma follows Luma in transform type
                if (cu_ptr->prediction_mode_flag == INTER_MODE) {
                    txb_ptr->transform_type[PLANE_TYPE_Y] = DCT_DCT;
                    txb_ptr->transform_type[PLANE_TYPE_UV] = DCT_DCT;
                }
                else { // INTRA
                    txb_ptr->transform_type[PLANE_TYPE_Y] = DCT_DCT;
                }
            }
#endif

        }

        if (component_mask == PICTURE_BUFFER_DESC_FULL_MASK || component_mask == PICTURE_BUFFER_DESC_CHROMA_MASK) {

            if (cu_ptr->prediction_mode_flag == INTRA_MODE && cu_ptr->prediction_unit_array->intra_chroma_mode == UV_CFL_PRED) {
                EbPictureBufferDesc_t *reconSamples = predSamples16bit;
                uint32_t reconLumaOffset = (reconSamples->origin_y + origin_y)            * reconSamples->strideY + (reconSamples->origin_x + origin_x);
                if (txb_ptr->y_has_coeff == EB_TRUE && cu_ptr->skip_flag == EB_FALSE) {
#if QT_10BIT_SUPPORT
                    uint16_t     *predBuffer = ((uint16_t*)predSamples16bit->bufferY) + predLumaOffset;
                    Av1InvTransformRecon(
                        ((int32_t*)inverse_quant_buffer->bufferY) + coeff1dOffset,
                        CONVERT_TO_BYTEPTR(predBuffer),
                        predSamples->strideY,
                        context_ptr->blk_geom->txsize[context_ptr->txb_itr],
                        BIT_INCREMENT_10BIT,
                        txb_ptr->transform_type[PLANE_TYPE_Y],
                        PLANE_TYPE_Y,
                        eob[0]);
#else
                    Av1EstimateInvTransform(
                        ((int32_t*)inverse_quant_buffer->bufferY) + scratchLumaOffset,
                        64,
                        ((int32_t*)inverse_quant_buffer->bufferY) + scratchLumaOffset,
                        64,
                        txb_size,
                        transformScratchBuffer,
                        BIT_INCREMENT_10BIT,
                        txb_ptr->transform_type[PLANE_TYPE_Y],
                        eob[0],
                        asm_type,
                        0);

                    PictureAdditionKernel16Bit(
                        ((uint16_t*)predSamples16bit->bufferY) + predLumaOffset,
                        predSamples16bit->strideY,
                        ((int32_t*)inverse_quant_buffer->bufferY) + scratchLumaOffset,
                        64,
                        ((uint16_t*)reconSamples->bufferY) + reconLumaOffset,
                        reconSamples->strideY,
                        txb_size,
                        txb_size,
                        10);
#endif
                }

                // Down sample Luma
                cfl_luma_subsampling_420_hbd_c(
                    ((uint16_t*)reconSamples->bufferY) + reconLumaOffset,
                    reconSamples->strideY,              
#if CHROMA_BLIND
                    context_ptr->md_context->pred_buf_q3,
#else
                    context_ptr->pred_buf_q3,
#endif
                    context_ptr->blk_geom->tx_width[context_ptr->txb_itr],
                    context_ptr->blk_geom->tx_height[context_ptr->txb_itr]);

                int32_t round_offset = ((context_ptr->blk_geom->tx_width_uv[context_ptr->txb_itr])*(context_ptr->blk_geom->tx_height_uv[context_ptr->txb_itr])) / 2;


                subtract_average(
#if CHROMA_BLIND
                    context_ptr->md_context->pred_buf_q3,
#else
                    context_ptr->pred_buf_q3,
#endif
                    context_ptr->blk_geom->tx_width_uv[context_ptr->txb_itr],
                    context_ptr->blk_geom->tx_height_uv[context_ptr->txb_itr],
                    round_offset,
                    LOG2F(context_ptr->blk_geom->tx_width_uv[context_ptr->txb_itr]) + LOG2F(context_ptr->blk_geom->tx_height_uv[context_ptr->txb_itr]));



                int32_t alpha_q3 =
                    cfl_idx_to_alpha(cu_ptr->prediction_unit_array->cfl_alpha_idx, cu_ptr->prediction_unit_array->cfl_alpha_signs, CFL_PRED_U); // once for U, once for V
                // TOCHANGE
                // assert(chromaSize * CFL_BUF_LINE + chromaSize <=                CFL_BUF_SQUARE);

                cfl_predict_hbd(
#if CHROMA_BLIND
                    context_ptr->md_context->pred_buf_q3,
#else
                    context_ptr->pred_buf_q3,
#endif
                    ((uint16_t*)predSamples16bit->bufferCb) + predCbOffset,
                    predSamples16bit->strideCb,
                    ((uint16_t*)predSamples16bit->bufferCb) + predCbOffset,
                    predSamples16bit->strideCb,
                    alpha_q3,
                    10,
                    context_ptr->blk_geom->tx_width_uv[context_ptr->txb_itr],
                    context_ptr->blk_geom->tx_height_uv[context_ptr->txb_itr]);

                alpha_q3 =
                    cfl_idx_to_alpha(cu_ptr->prediction_unit_array->cfl_alpha_idx, cu_ptr->prediction_unit_array->cfl_alpha_signs, CFL_PRED_V); // once for U, once for V
                // TOCHANGE
                //assert(chromaSize * CFL_BUF_LINE + chromaSize <=                CFL_BUF_SQUARE);

                cfl_predict_hbd(
#if CHROMA_BLIND
                    context_ptr->md_context->pred_buf_q3,
#else
                    context_ptr->pred_buf_q3,
#endif
                    ((uint16_t*)predSamples16bit->bufferCr) + predCrOffset,
                    predSamples16bit->strideCr,
                    ((uint16_t*)predSamples16bit->bufferCr) + predCrOffset,
                    predSamples16bit->strideCr,
                    alpha_q3,
                    10,
                    context_ptr->blk_geom->tx_width_uv[context_ptr->txb_itr],
                    context_ptr->blk_geom->tx_height_uv[context_ptr->txb_itr]);
            }

        }

        {
            //**********************************
            // Cb
            //**********************************
            ResidualKernel16bit(
                ((uint16_t*)inputSamples16bit->bufferCb) + inputCbOffset,
                inputSamples16bit->strideCb,
                ((uint16_t*)predSamples16bit->bufferCb) + predCbOffset,
                predSamples16bit->strideCb,
                ((int16_t*)residual16bit->bufferCb) + scratchCbOffset,
                residual16bit->strideCb,
                context_ptr->blk_geom->tx_width_uv[context_ptr->txb_itr],
                context_ptr->blk_geom->tx_height_uv[context_ptr->txb_itr]);


            ResidualKernel16bit(

                ((uint16_t*)inputSamples16bit->bufferCr) + inputCrOffset,
                inputSamples16bit->strideCr,
                ((uint16_t*)predSamples16bit->bufferCr) + predCrOffset,
                predSamples16bit->strideCr,
                ((int16_t*)residual16bit->bufferCr) + scratchCrOffset,
                residual16bit->strideCr,
                context_ptr->blk_geom->tx_width_uv[context_ptr->txb_itr],
                context_ptr->blk_geom->tx_height_uv[context_ptr->txb_itr]);


            Av1EstimateTransform(
                ((int16_t*)residual16bit->bufferCb) + scratchCbOffset,
                residual16bit->strideCb,

                ((tran_low_t*)transform16bit->bufferCb) + context_ptr->coded_area_sb_uv,
                NOT_USED_VALUE,
                context_ptr->blk_geom->txsize_uv[context_ptr->txb_itr],
                &context_ptr->three_quad_energy,
                transformScratchBuffer,
                BIT_INCREMENT_10BIT,
                txb_ptr->transform_type[PLANE_TYPE_UV],
                asm_type,
                PLANE_TYPE_UV,
                context_ptr->trans_coeff_shape_chroma);


            Av1QuantizeInvQuantize(
                sb_ptr->picture_control_set_ptr,
                ((int32_t*)transform16bit->bufferCb) + context_ptr->coded_area_sb_uv,
                NOT_USED_VALUE,

#if QT_10BIT_SUPPORT
                ((int32_t*)coeffSamplesTB->bufferCb) + context_ptr->coded_area_sb_uv,
#else
                ((int32_t*)coeffSamplesTB->bufferCb) + scratchCbOffset,
#endif
                ((int32_t*)inverse_quant_buffer->bufferCb) + context_ptr->coded_area_sb_uv,
                qp,
                context_ptr->blk_geom->tx_width_uv[context_ptr->txb_itr],
                context_ptr->blk_geom->tx_height_uv[context_ptr->txb_itr],
                context_ptr->blk_geom->txsize_uv[context_ptr->txb_itr],
                &eob[1],
                candidate_plane[1],
                asm_type,
                &(count_non_zero_coeffs[1]),
                0,
                0,
                COMPONENT_CHROMA_CB,
#if QT_10BIT_SUPPORT
                BIT_INCREMENT_10BIT,
#endif
                txb_ptr->transform_type[PLANE_TYPE_UV],
                cleanSparseCoeffFlag);

            txb_ptr->u_has_coeff = count_non_zero_coeffs[1] ? EB_TRUE : EB_FALSE;

            //**********************************
            // Cr
            //**********************************
#if !QT_10BIT_SUPPORT
            EncodeTransform(
                ((int16_t*)residual16bit->bufferCr) + scratchCrOffset,
                32,
                ((int16_t*)transform16bit->bufferCr) + scratchCrOffset,
                32,
                txb_size >> 1,
                transformScratchBuffer,
                BIT_INCREMENT_10BIT,
                EB_FALSE,
                context_ptr->trans_coeff_shape_chroma,
                asm_type);
#endif

            Av1EstimateTransform(
                ((int16_t*)residual16bit->bufferCr) + scratchCbOffset,

                residual16bit->strideCr,

                ((tran_low_t*)transform16bit->bufferCr) + context_ptr->coded_area_sb_uv,
                NOT_USED_VALUE,


                context_ptr->blk_geom->txsize_uv[context_ptr->txb_itr],
                &context_ptr->three_quad_energy,
                transformScratchBuffer,
                BIT_INCREMENT_10BIT,
                txb_ptr->transform_type[PLANE_TYPE_UV],
                asm_type,
                PLANE_TYPE_UV,
                context_ptr->trans_coeff_shape_chroma);


            Av1QuantizeInvQuantize(
                sb_ptr->picture_control_set_ptr,
                ((int32_t*)transform16bit->bufferCr) + context_ptr->coded_area_sb_uv,
                NOT_USED_VALUE,
#if QT_10BIT_SUPPORT
                ((int32_t*)coeffSamplesTB->bufferCr) + context_ptr->coded_area_sb_uv,
#else
                ((int32_t*)coeffSamplesTB->bufferCr) + scratchCbOffset,
#endif
                ((int32_t*)inverse_quant_buffer->bufferCr) + context_ptr->coded_area_sb_uv,
                qp,
                context_ptr->blk_geom->tx_width_uv[context_ptr->txb_itr],
                context_ptr->blk_geom->tx_height_uv[context_ptr->txb_itr],
                context_ptr->blk_geom->txsize_uv[context_ptr->txb_itr],
                &eob[2],
                candidate_plane[2],
                asm_type,
                &(count_non_zero_coeffs[2]),
                0,
                0,
                COMPONENT_CHROMA_CR,
#if QT_10BIT_SUPPORT
                BIT_INCREMENT_10BIT,
#endif
                txb_ptr->transform_type[PLANE_TYPE_UV],
                cleanSparseCoeffFlag);
            txb_ptr->v_has_coeff = count_non_zero_coeffs[2] ? EB_TRUE : EB_FALSE;

        }
    }


    txb_ptr->trans_coeff_shape_luma = context_ptr->trans_coeff_shape_luma;
    txb_ptr->trans_coeff_shape_chroma = context_ptr->trans_coeff_shape_chroma;
    txb_ptr->nz_coef_count[0] = (uint16_t)count_non_zero_coeffs[0];
    txb_ptr->nz_coef_count[1] = (uint16_t)count_non_zero_coeffs[1];
    txb_ptr->nz_coef_count[2] = (uint16_t)count_non_zero_coeffs[2];
    return;
}


/**********************************************************
* Encode Generate Recon
*
* Summary: Performs a H.265 conformant
*   Inverse Transform and generate
*   the reconstructed samples of a TU.
*
* Inputs:
*   origin_x
*   origin_y
*   txb_size
*   sb_sz
*   input - Inverse Qunatized Coeff (position sensitive)
*   pred - prediction samples (position independent)
*
* Outputs:
*   Recon  (position independent)
*
**********************************************************/
static void Av1EncodeGenerateRecon(
    EncDecContext_t       *context_ptr,
    uint32_t               origin_x,
    uint32_t               origin_y,
    TxSize                 tx_size, 
    EbPictureBufferDesc_t *predSamples,     // no basis/offset
    EbPictureBufferDesc_t *inverse_quant_buffer,    // no basis/offset
    int16_t               *transformScratchBuffer,
    uint32_t               plane,
    uint16_t               txb_itr,
    uint16_t              *eob,
    EbAsm                  asm_type)
{
    CodingUnit_t          *cu_ptr = context_ptr->cu_ptr;
    TransformUnit_t       *txb_ptr = &cu_ptr->transform_unit_array[txb_itr];
    PLANE_TYPE p_type = (plane == 0) ? PLANE_TYPE_Y : PLANE_TYPE_UV;

    //2D
    EbByte pred_ptr;
    uint32_t pred_stride;
    
    //1D
    EbByte inverse_ptr;

    //uint32_t offset1d = (plane == 0) ? context_ptr->coded_area_sb : context_ptr->coded_area_sb_uv;
    uint32_t offset1d = context_ptr->coded_area_sb[plane];

    EbBool has_coeff = (plane == 0) ? txb_ptr->y_has_coeff :
                        (plane == 1) ? txb_ptr->u_has_coeff :txb_ptr->v_has_coeff;

    Get1dOrigin(offset1d, inverse_quant_buffer, plane, &inverse_ptr);
    Get2dOrigin(origin_x, origin_y, predSamples, plane, &pred_ptr, &pred_stride);

    if (plane || (plane == 0 && (cu_ptr->prediction_mode_flag != INTRA_MODE || (cu_ptr->prediction_unit_array->intra_chroma_mode != UV_CFL_PRED && context_ptr->evaluate_cfl_ep == EB_FALSE)))) { 
        if (has_coeff && cu_ptr->skip_flag == EB_FALSE) {
            Av1InvTransformRecon8bit(
                    inverse_ptr,
                    pred_ptr,
                    pred_stride,
                    tx_size,
                    txb_ptr->transform_type[p_type],
                    p_type,
                    eob[plane]);
        }
    }
    return;
}


/**********************************************************
* Encode Generate Recon
*
* Summary: Performs a H.265 conformant
*   Inverse Transform and generate
*   the reconstructed samples of a TU.
*
* Inputs:
*   origin_x
*   origin_y
*   txb_size
*   sb_sz
*   input - Inverse Qunatized Coeff (position sensitive)
*   pred - prediction samples (position independent)
*
* Outputs:
*   Recon  (position independent)
*
**********************************************************/
static void Av1EncodeGenerateRecon16bit(
    EncDecContext_t         *context_ptr,
    uint32_t                 origin_x,
    uint32_t                 origin_y,
    EbPictureBufferDesc_t   *predSamples,     // no basis/offset
    EbPictureBufferDesc_t   *residual16bit,    // no basis/offset
    int16_t                 *transformScratchBuffer,
    uint32_t                 component_mask,
    EbColorFormat            color_format,
    uint16_t                *eob,
    EbAsm                    asm_type)
{

    //Jing: TODO change it
    uint32_t predLumaOffset;
    uint32_t predChromaOffset;
#if !QT_10BIT_SUPPORT
    uint32_t scratchLumaOffset;
    uint32_t scratchChromaOffset;
    uint32_t reconLumaOffset;
    uint32_t reconChromaOffset;
#endif

    CodingUnit_t          *cu_ptr = context_ptr->cu_ptr;
    TransformUnit_t       *txb_ptr = &cu_ptr->transform_unit_array[context_ptr->txb_itr];

#if QT_10BIT_SUPPORT
    (void)asm_type;
    (void)transformScratchBuffer;
#endif
    //**********************************
    // Luma
    //**********************************
    if (component_mask & PICTURE_BUFFER_DESC_LUMA_MASK) {
        if (cu_ptr->prediction_mode_flag != INTRA_MODE || cu_ptr->prediction_unit_array->intra_chroma_mode != UV_CFL_PRED)

        {
            predLumaOffset = (predSamples->origin_y + origin_y)* predSamples->strideY + (predSamples->origin_x + origin_x);
#if !QT_10BIT_SUPPORT
            scratchLumaOffset = context_ptr->blk_geom->tx_org_x[context_ptr->txb_itr] + context_ptr->blk_geom->tx_org_y[context_ptr->txb_itr] * SB_STRIDE_Y;
            reconLumaOffset = (predSamples->origin_y + origin_y)* predSamples->strideY + (predSamples->origin_x + origin_x);
#endif
            if (txb_ptr->y_has_coeff == EB_TRUE && cu_ptr->skip_flag == EB_FALSE) {

#if QT_10BIT_SUPPORT
                uint16_t     *predBuffer = ((uint16_t*)predSamples->bufferY) + predLumaOffset;
                Av1InvTransformRecon(
                    ((int32_t*)residual16bit->bufferY) + context_ptr->coded_area_sb[0],
                    CONVERT_TO_BYTEPTR(predBuffer),
                    predSamples->strideY,
                    context_ptr->blk_geom->txsize[context_ptr->txb_itr],
                    BIT_INCREMENT_10BIT,
                    txb_ptr->transform_type[PLANE_TYPE_Y],
                    PLANE_TYPE_Y,
                    eob[0]
                );
#else
                Av1EstimateInvTransform(
                    ((int32_t*)residual16bit->bufferY) + scratchLumaOffset,
                    64,
                    ((int32_t*)residual16bit->bufferY) + scratchLumaOffset,
                    64,
                    txb_size,
                    transformScratchBuffer,
                    BIT_INCREMENT_10BIT,
                    txb_ptr->transform_type[PLANE_TYPE_Y],
                    eob[0],
                    asm_type,
                    0);

                PictureAdditionKernel16Bit(
                    (uint16_t*)predSamples->bufferY + predLumaOffset,
                    predSamples->strideY,
                    ((int32_t*)residual16bit->bufferY) + scratchLumaOffset,
                    64,
                    (uint16_t*)predSamples->bufferY + reconLumaOffset,
                    predSamples->strideY,
                    txb_size,
                    txb_size,
                    10);
#endif
            }

        }
    }

    if (component_mask & PICTURE_BUFFER_DESC_CHROMA_MASK)

        //**********************************
        // Chroma
        //**********************************

    {

        //**********************************
        // Cb
        //**********************************

        uint32_t                 round_origin_x = (origin_x >> 3) << 3;// for Chroma blocks with size of 4
        uint32_t                 round_origin_y = (origin_y >> 3) << 3;// for Chroma blocks with size of 4

        predChromaOffset = (((predSamples->origin_y + round_origin_y) >> 1)           * predSamples->strideCb) + ((predSamples->origin_x + round_origin_x) >> 1);
#if !QT_10BIT_SUPPORT
        scratchChromaOffset = ROUND_UV(context_ptr->blk_geom->tx_org_x[context_ptr->txb_itr]) / 2 + ROUND_UV(context_ptr->blk_geom->tx_org_y[context_ptr->txb_itr]) / 2 * SB_STRIDE_UV;
        reconChromaOffset = (((predSamples->origin_y + origin_y) >> 1) * predSamples->strideCb) + ((predSamples->origin_x + origin_x) >> 1);
#endif

        if (txb_ptr->u_has_coeff == EB_TRUE && cu_ptr->skip_flag == EB_FALSE) {


#if QT_10BIT_SUPPORT
            uint16_t     *predBuffer = ((uint16_t*)predSamples->bufferCb) + predChromaOffset;
            Av1InvTransformRecon(
                ((int32_t*)residual16bit->bufferCb) + context_ptr->coded_area_sb_uv,
                CONVERT_TO_BYTEPTR(predBuffer),
                predSamples->strideCb,
                context_ptr->blk_geom->txsize_uv[context_ptr->txb_itr],
                BIT_INCREMENT_10BIT,
                txb_ptr->transform_type[PLANE_TYPE_UV],
                PLANE_TYPE_UV,
                eob[1]);
#else
            Av1EstimateInvTransform(
                ((int32_t*)residual16bit->bufferCb) + scratchChromaOffset,
                32,
                ((int32_t*)residual16bit->bufferCb) + scratchChromaOffset,
                32,
                txb_size >> 1,
                transformScratchBuffer,
                BIT_INCREMENT_10BIT,
                txb_ptr->transform_type[PLANE_TYPE_UV],
                eob[1],
                asm_type,
                0);

            PictureAdditionKernel16Bit(
                (uint16_t*)predSamples->bufferCb + predChromaOffset,
                predSamples->strideCb,
                ((int32_t*)residual16bit->bufferCb) + scratchChromaOffset,
                32,
                (uint16_t*)predSamples->bufferCb + reconChromaOffset,
                predSamples->strideCb,
                txb_size >> 1,
                txb_size >> 1,
                10);
#endif

        }

        //**********************************
        // Cr
        //**********************************
        predChromaOffset = (((predSamples->origin_y + round_origin_y) >> 1)           * predSamples->strideCr) + ((predSamples->origin_x + round_origin_x) >> 1);
#if !QT_10BIT_SUPPORT
        scratchChromaOffset = ROUND_UV(context_ptr->blk_geom->tx_org_x[context_ptr->txb_itr]) / 2 + ROUND_UV(context_ptr->blk_geom->tx_org_y[context_ptr->txb_itr]) / 2 * SB_STRIDE_UV;
        reconChromaOffset = (((predSamples->origin_y + origin_y) >> 1) * predSamples->strideCr) + ((predSamples->origin_x + origin_x) >> 1);
#endif
        if (txb_ptr->v_has_coeff == EB_TRUE && cu_ptr->skip_flag == EB_FALSE) {

#if QT_10BIT_SUPPORT
            uint16_t     *predBuffer = ((uint16_t*)predSamples->bufferCr) + predChromaOffset;
            Av1InvTransformRecon(
                ((int32_t*)residual16bit->bufferCr) + context_ptr->coded_area_sb_uv,
                CONVERT_TO_BYTEPTR(predBuffer),
                predSamples->strideCr,
                context_ptr->blk_geom->txsize_uv[context_ptr->txb_itr],
                BIT_INCREMENT_10BIT,
                txb_ptr->transform_type[PLANE_TYPE_UV],
                PLANE_TYPE_UV,
                eob[2]);
#else
            Av1EstimateInvTransform(
                ((int32_t*)residual16bit->bufferCr) + scratchChromaOffset,
                32,
                ((int32_t*)residual16bit->bufferCr) + scratchChromaOffset,
                32,
                txb_size >> 1,
                transformScratchBuffer,
                BIT_INCREMENT_10BIT,
                txb_ptr->transform_type[PLANE_TYPE_UV],
                eob[2],
                asm_type,
                0);

            PictureAdditionKernel16Bit(
                (uint16_t*)predSamples->bufferCr + predChromaOffset,
                predSamples->strideCr,
                ((int32_t*)residual16bit->bufferCr) + scratchChromaOffset,
                32,
                (uint16_t*)predSamples->bufferCr + reconChromaOffset,
                predSamples->strideCr,
                txb_size >> 1,
                txb_size >> 1,
                10);
#endif


        }
    }

    return;
}
#if !QT_10BIT_SUPPORT
/**********************************************************
* Encode Generate Recon
*
* Summary: Performs a H.265 conformant
*   Inverse Transform and generate
*   the reconstructed samples of a TU.
*
* Inputs:
*   origin_x
*   origin_y
*   txb_size
*   sb_sz
*   input - Inverse Qunatized Coeff (position sensitive)
*   pred - prediction samples (position independent)
*
* Outputs:
*   Recon  (position independent)
*
**********************************************************/
static void EncodeGenerateRecon(
    EncDecContext_t       *context_ptr,
    uint32_t                 origin_x,
    uint32_t                 origin_y,
    EbPictureBufferDesc_t *predSamples,     // no basis/offset
    EbPictureBufferDesc_t *residual16bit,    // no basis/offset
    int16_t                *transformScratchBuffer,
    EbAsm                 asm_type)
{
    uint32_t predLumaOffset;
    uint32_t predChromaOffset;
    uint32_t scratchLumaOffset;
    uint32_t scratchChromaOffset;
    uint32_t reconLumaOffset;
    uint32_t reconChromaOffset;

    CodingUnit_t          *cu_ptr = context_ptr->cu_ptr;
    TransformUnit_t       *txb_ptr = &cu_ptr->transform_unit_array[context_ptr->txb_itr];
    uint32_t                 txb_size = context_ptr->cu_stats->size;

    EbPictureBufferDesc_t *reconSamples = predSamples;
    // *Note - The prediction is built in-place in the Recon buffer. It is overwritten with Reconstructed
    //   samples if the CBF==1 && SKIP==False

    //**********************************
    // Luma
    //**********************************

    {
        predLumaOffset = (predSamples->origin_y + origin_y)             * predSamples->strideY + (predSamples->origin_x + origin_x);
        scratchLumaOffset = ((origin_y & (63)) * 64) + (origin_x & (63));
        reconLumaOffset = (reconSamples->origin_y + origin_y)            * reconSamples->strideY + (reconSamples->origin_x + origin_x);
        if (txb_ptr->lumaCbf == EB_TRUE && cu_ptr->skip_flag == EB_FALSE) {

            EncodeInvTransform(
                txb_ptr->trans_coeff_shape_luma == ONLY_DC_SHAPE || txb_ptr->is_only_dc[0],
                ((int16_t*)residual16bit->bufferY) + scratchLumaOffset,
                64,
                ((int16_t*)residual16bit->bufferY) + scratchLumaOffset,
                64,
                txb_size,
                transformScratchBuffer,
                BIT_INCREMENT_8BIT,
                (EbBool)(txb_size == MIN_PU_SIZE),
                asm_type);

            AdditionKernel_funcPtrArray[asm_type][txb_size >> 3](
                predSamples->bufferY + predLumaOffset,
                predSamples->strideY,
                ((int16_t*)residual16bit->bufferY) + scratchLumaOffset,
                64,
                reconSamples->bufferY + reconLumaOffset,
                reconSamples->strideY,
                txb_size,
                txb_size);
        }
    }

    //**********************************
    // Chroma
    //**********************************

    {
        predChromaOffset = (((predSamples->origin_y + origin_y) >> 1)           * predSamples->strideCb) + ((predSamples->origin_x + origin_x) >> 1);
        scratchChromaOffset = (((origin_y & (63)) >> 1) * 32) + ((origin_x & (63)) >> 1);
        reconChromaOffset = (((reconSamples->origin_y + origin_y) >> 1)          * reconSamples->strideCb) + ((reconSamples->origin_x + origin_x) >> 1);
        //**********************************
        // Cb
        //**********************************
        if (txb_ptr->cbCbf == EB_TRUE && cu_ptr->skip_flag == EB_FALSE) {

            EncodeInvTransform(
                txb_ptr->trans_coeff_shape_chroma == ONLY_DC_SHAPE || txb_ptr->is_only_dc[1],
                ((int16_t*)residual16bit->bufferCb) + scratchChromaOffset,
                32,
                ((int16_t*)residual16bit->bufferCb) + scratchChromaOffset,
                32,
                txb_size >> 1,
                transformScratchBuffer,
                BIT_INCREMENT_8BIT,
                EB_FALSE,
                asm_type);

            AdditionKernel_funcPtrArray[asm_type][txb_size >> 4](
                predSamples->bufferCb + predChromaOffset,
                predSamples->strideCb,
                ((int16_t*)residual16bit->bufferCb) + scratchChromaOffset,
                32,
                reconSamples->bufferCb + reconChromaOffset,
                reconSamples->strideCb,
                txb_size >> 1,
                txb_size >> 1);
        }

        //**********************************
        // Cr
        //**********************************
        predChromaOffset = (((predSamples->origin_y + origin_y) >> 1)           * predSamples->strideCr) + ((predSamples->origin_x + origin_x) >> 1);
        scratchChromaOffset = (((origin_y & (63)) >> 1) * 32) + ((origin_x & (63)) >> 1);
        reconChromaOffset = (((reconSamples->origin_y + origin_y) >> 1)          * reconSamples->strideCr) + ((reconSamples->origin_x + origin_x) >> 1);
        if (txb_ptr->crCbf == EB_TRUE && cu_ptr->skip_flag == EB_FALSE) {

            EncodeInvTransform(
                txb_ptr->trans_coeff_shape_chroma == ONLY_DC_SHAPE || txb_ptr->is_only_dc[2],
                ((int16_t*)residual16bit->bufferCr) + scratchChromaOffset,
                32,
                ((int16_t*)residual16bit->bufferCr) + scratchChromaOffset,
                32,
                txb_size >> 1,
                transformScratchBuffer,
                BIT_INCREMENT_8BIT,
                EB_FALSE,
                asm_type);

            AdditionKernel_funcPtrArray[asm_type][txb_size >> 4](
                predSamples->bufferCr + predChromaOffset,
                predSamples->strideCr,
                ((int16_t*)residual16bit->bufferCr) + scratchChromaOffset,
                32,
                reconSamples->bufferCr + reconChromaOffset,
                reconSamples->strideCr,
                txb_size >> 1,
                txb_size >> 1);
        }
    }

    return;
}

/**********************************************************
* Encode Generate Recon
*
* Summary: Performs a H.265 conformant
*   Inverse Transform and generate
*   the reconstructed samples of a TU.
*
* Inputs:
*   origin_x
*   origin_y
*   txb_size
*   sb_sz
*   input - Inverse Qunatized Coeff (position sensitive)
*   pred - prediction samples (position independent)
*
* Outputs:
*   Recon  (position independent)
*
**********************************************************/
static void EncodeGenerateRecon16bit(
    EncDecContext_t       *context_ptr,
    uint32_t                 origin_x,
    uint32_t                 origin_y,
    EbPictureBufferDesc_t *predSamples,     // no basis/offset
    EbPictureBufferDesc_t *residual16bit,    // no basis/offset
    int16_t                *transformScratchBuffer,
    EbAsm                 asm_type)
{

    uint32_t predLumaOffset;
    uint32_t predChromaOffset;
    uint32_t scratchLumaOffset;
    uint32_t scratchChromaOffset;
    uint32_t reconLumaOffset;
    uint32_t reconChromaOffset;

    CodingUnit_t          *cu_ptr = context_ptr->cu_ptr;
    TransformUnit_t       *txb_ptr = &cu_ptr->transform_unit_array[context_ptr->txb_itr];
    uint32_t                 txb_size = context_ptr->cu_stats->size;

    //**********************************
    // Luma
    //**********************************

    {
        predLumaOffset = (predSamples->origin_y + origin_y)* predSamples->strideY + (predSamples->origin_x + origin_x);
        scratchLumaOffset = ((origin_y & (63)) * 64) + (origin_x & (63));
        reconLumaOffset = (predSamples->origin_y + origin_y)* predSamples->strideY + (predSamples->origin_x + origin_x);
        if (txb_ptr->lumaCbf == EB_TRUE && cu_ptr->skip_flag == EB_FALSE) {

            EncodeInvTransform(
                txb_ptr->trans_coeff_shape_luma == ONLY_DC_SHAPE || txb_ptr->is_only_dc[0],
                ((int16_t*)residual16bit->bufferY) + scratchLumaOffset,
                64,
                ((int16_t*)residual16bit->bufferY) + scratchLumaOffset,
                64,
                txb_size,
                transformScratchBuffer,
                BIT_INCREMENT_10BIT,
                (EbBool)(txb_size == MIN_PU_SIZE),
                asm_type);

            AdditionKernel_funcPtrArray16bit[asm_type](
                (uint16_t*)predSamples->bufferY + predLumaOffset,
                predSamples->strideY,
                ((int16_t*)residual16bit->bufferY) + scratchLumaOffset,
                64,
                (uint16_t*)predSamples->bufferY + reconLumaOffset,
                predSamples->strideY,
                txb_size,
                txb_size);

        }

    }

    //**********************************
    // Chroma
    //**********************************

    {

        //**********************************
        // Cb
        //**********************************
        predChromaOffset = (((predSamples->origin_y + origin_y) >> 1)  * predSamples->strideCb) + ((predSamples->origin_x + origin_x) >> 1);
        scratchChromaOffset = (((origin_y & (63)) >> 1) * 32) + ((origin_x & (63)) >> 1);
        reconChromaOffset = (((predSamples->origin_y + origin_y) >> 1) * predSamples->strideCb) + ((predSamples->origin_x + origin_x) >> 1);
        if (txb_ptr->cbCbf == EB_TRUE && cu_ptr->skip_flag == EB_FALSE) {

            EncodeInvTransform(
                txb_ptr->trans_coeff_shape_chroma == ONLY_DC_SHAPE || txb_ptr->is_only_dc[1],
                ((int16_t*)residual16bit->bufferCb) + scratchChromaOffset,
                32,
                ((int16_t*)residual16bit->bufferCb) + scratchChromaOffset,
                32,
                txb_size >> 1,
                transformScratchBuffer,
                BIT_INCREMENT_10BIT,
                EB_FALSE,
                asm_type);

            AdditionKernel_funcPtrArray16bit[asm_type](
                (uint16_t*)predSamples->bufferCb + predChromaOffset,
                predSamples->strideCb,
                ((int16_t*)residual16bit->bufferCb) + scratchChromaOffset,
                32,
                (uint16_t*)predSamples->bufferCb + reconChromaOffset,
                predSamples->strideCb,
                txb_size >> 1,
                txb_size >> 1);

        }

        //**********************************
        // Cr
        //**********************************
        predChromaOffset = (((predSamples->origin_y + origin_y) >> 1)  * predSamples->strideCr) + ((predSamples->origin_x + origin_x) >> 1);
        scratchChromaOffset = (((origin_y & (63)) >> 1) * 32) + ((origin_x & (63)) >> 1);
        reconChromaOffset = (((predSamples->origin_y + origin_y) >> 1) * predSamples->strideCr) + ((predSamples->origin_x + origin_x) >> 1);
        if (txb_ptr->crCbf == EB_TRUE && cu_ptr->skip_flag == EB_FALSE) {

            EncodeInvTransform(
                txb_ptr->trans_coeff_shape_chroma == ONLY_DC_SHAPE || txb_ptr->is_only_dc[2],
                ((int16_t*)residual16bit->bufferCr) + scratchChromaOffset,
                32,
                ((int16_t*)residual16bit->bufferCr) + scratchChromaOffset,
                32,
                txb_size >> 1,
                transformScratchBuffer,
                BIT_INCREMENT_10BIT,
                EB_FALSE,
                asm_type);

            AdditionKernel_funcPtrArray16bit[asm_type](
                (uint16_t*)predSamples->bufferCr + predChromaOffset,
                predSamples->strideCr,
                ((int16_t*)residual16bit->bufferCr) + scratchChromaOffset,
                32,
                (uint16_t*)predSamples->bufferCr + reconChromaOffset,
                predSamples->strideCr,
                txb_size >> 1,
                txb_size >> 1);

        }
    }

    return;
}

#endif
static EB_AV1_ENCODE_LOOP_FUNC_PTR   Av1EncodeLoopFunctionTable[2] =
{
    Av1EncodeLoop16bit
};

static EB_AV1_INIT_NEIGHBOR_FUNC_PTR Av1InitNeighborFunctionTable[2] =
{
    InitNeighborArray,
    InitNeighborArray16bit
};

EB_AV1_GENERATE_RECON_FUNC_PTR   Av1EncodeGenerateReconFunctionPtr[2] =
{
    Av1EncodeGenerateRecon,
    Av1EncodeGenerateRecon16bit
};
#if !QT_10BIT_SUPPORT

EB_GENERATE_RECON_FUNC_PTR   EncodeGenerateReconFunctionPtr[2] =
{
    EncodeGenerateRecon,
    EncodeGenerateRecon16bit
};
#endif

#if !QT_10BIT_SUPPORT
EB_GENERATE_RECON_INTRA_4x4_FUNC_PTR   EncodeGenerateReconIntra4x4FunctionPtr[2] =
{
    EncodeGenerateReconIntra4x4,
    EncodeGenerateReconIntra4x416bit
};

EB_GENERATE_INTRA_SAMPLES_FUNC_PTR GenerateIntraReferenceSamplesFuncTable[2] =
{
    GenerateIntraReferenceSamplesEncodePass,
    GenerateIntraReference16bitSamplesEncodePass
};

EB_ENC_PASS_INTRA_FUNC_PTR EncodePassIntraPredictionFuncTable[2] =
{
    EncodePassIntraPrediction,
    EncodePassIntraPrediction16bit
};
#endif

/*******************************************
* Encode Pass - Assign Delta Qp
*******************************************/
static void EncodePassUpdateQp(
    PictureControlSet_t     *picture_control_set_ptr,
    EncDecContext_t         *context_ptr,
    EbBool                  availableCoeff,
    EbBool                  isDeltaQpEnable,
    EbBool                 *isDeltaQpNotCoded,
    uint32_t                   dif_cu_delta_qp_depth,
    uint8_t                   *prev_coded_qp,
    uint8_t                   *prev_quant_group_coded_qp,
    uint32_t                   sb_qp
)
{

    uint32_t ref_qp;
    uint8_t qp;

    uint32_t  log2MinCuQpDeltaSize = LOG2F_MAX_LCU_SIZE - dif_cu_delta_qp_depth;
    int32_t  qpTopNeighbor = 0;
    int32_t  qpLeftNeighbor = 0;
    EbBool newQuantGroup;
    uint32_t  quantGroupX = context_ptr->cu_origin_x - (context_ptr->cu_origin_x & ((1 << log2MinCuQpDeltaSize) - 1));
    uint32_t  quantGroupY = context_ptr->cu_origin_y - (context_ptr->cu_origin_y & ((1 << log2MinCuQpDeltaSize) - 1));
    EbBool sameLcuCheckTop = (((quantGroupY - 1) >> LOG2F_MAX_LCU_SIZE) == ((quantGroupY) >> LOG2F_MAX_LCU_SIZE)) ? EB_TRUE : EB_FALSE;
    EbBool sameLcuCheckLeft = (((quantGroupX - 1) >> LOG2F_MAX_LCU_SIZE) == ((quantGroupX) >> LOG2F_MAX_LCU_SIZE)) ? EB_TRUE : EB_FALSE;
    // Neighbor Array
    uint32_t qpLeftNeighborIndex = 0;
    uint32_t qpTopNeighborIndex = 0;

    // CU larger than the quantization group

    if (Log2f(context_ptr->blk_geom->bwidth) >= log2MinCuQpDeltaSize) {

        *isDeltaQpNotCoded = EB_TRUE;
    }

    // At the beginning of a new quantization group
    if (((context_ptr->cu_origin_x & ((1 << log2MinCuQpDeltaSize) - 1)) == 0) &&
        ((context_ptr->cu_origin_y & ((1 << log2MinCuQpDeltaSize) - 1)) == 0))
    {
        *isDeltaQpNotCoded = EB_TRUE;
        newQuantGroup = EB_TRUE;
    }
    else {
        newQuantGroup = EB_FALSE;
    }

    // setting the previous Quantization Group QP
    if (newQuantGroup == EB_TRUE) {
        *prev_coded_qp = *prev_quant_group_coded_qp;
    }

    if (sameLcuCheckTop) {
        qpTopNeighborIndex =
            LUMA_SAMPLE_PIC_WISE_LOCATION_TO_QP_ARRAY_IDX(
                quantGroupX,
                quantGroupY - 1,
                picture_control_set_ptr->qp_array_stride);
        qpTopNeighbor = picture_control_set_ptr->qp_array[qpTopNeighborIndex];
    }
    else {
        qpTopNeighbor = *prev_coded_qp;
    }

    if (sameLcuCheckLeft) {
        qpLeftNeighborIndex =
            LUMA_SAMPLE_PIC_WISE_LOCATION_TO_QP_ARRAY_IDX(
                quantGroupX - 1,
                quantGroupY,
                picture_control_set_ptr->qp_array_stride);

        qpLeftNeighbor = picture_control_set_ptr->qp_array[qpLeftNeighborIndex];
    }
    else {
        qpLeftNeighbor = *prev_coded_qp;
    }

    ref_qp = (qpLeftNeighbor + qpTopNeighbor + 1) >> 1;

    qp = (uint8_t)context_ptr->cu_ptr->qp;
    // Update the State info
    if (isDeltaQpEnable) {
        if (*isDeltaQpNotCoded) {
            if (availableCoeff) {
                qp = (uint8_t)context_ptr->cu_ptr->qp;
                *prev_coded_qp = qp;
                *prev_quant_group_coded_qp = qp;
                *isDeltaQpNotCoded = EB_FALSE;
            }
            else {
                qp = (uint8_t)ref_qp;
                *prev_quant_group_coded_qp = qp;
            }
        }
    }
    else {
        qp = (uint8_t)sb_qp;
    }
    context_ptr->cu_ptr->qp = qp;
    return;
}



EbErrorType QpmDeriveBeaAndSkipQpmFlagLcu(
    SequenceControlSet_t                   *sequence_control_set_ptr,
    PictureControlSet_t                    *picture_control_set_ptr,
    LargestCodingUnit_t                    *sb_ptr,
    uint32_t                                 sb_index,
    EncDecContext_t                        *context_ptr)
{

    EbErrorType                    return_error = EB_ErrorNone;
#if ADD_DELTA_QP_SUPPORT
    uint16_t                           picture_qp = picture_control_set_ptr->parent_pcs_ptr->base_qindex;
    uint16_t                           min_qp_allowed = 0;
    uint16_t                           max_qp_allowed = 255;
    uint16_t                           deltaQpRes = (uint16_t)picture_control_set_ptr->parent_pcs_ptr->delta_q_res;
#else
    uint8_t                           picture_qp = picture_control_set_ptr->picture_qp;

    uint8_t                           min_qp_allowed = (uint8_t)sequence_control_set_ptr->static_config.min_qp_allowed;
    uint8_t                           max_qp_allowed = (uint8_t)sequence_control_set_ptr->static_config.max_qp_allowed;
#endif


    context_ptr->qpmQp = picture_qp;

    SbStat_t *sb_stat_ptr = &(picture_control_set_ptr->parent_pcs_ptr->sb_stat_array[sb_index]);


    context_ptr->non_moving_delta_qp = 0;

    context_ptr->grass_enhancement_flag = ((picture_control_set_ptr->scene_caracteristic_id == EB_FRAME_CARAC_1) && (sb_stat_ptr->cu_stat_array[0].grass_area)
        && (sb_ptr->picture_control_set_ptr->parent_pcs_ptr->edge_results_ptr[sb_index].edge_block_num > 0))

        ? EB_TRUE : EB_FALSE;


    context_ptr->backgorund_enhancement = EB_FALSE;


    context_ptr->skip_qpm_flag = sequence_control_set_ptr->static_config.improve_sharpness ? EB_FALSE : EB_TRUE;

    if ((picture_control_set_ptr->parent_pcs_ptr->logo_pic_flag == EB_FALSE) && ((picture_control_set_ptr->parent_pcs_ptr->pic_noise_class >= PIC_NOISE_CLASS_3_1) || (picture_control_set_ptr->parent_pcs_ptr->high_dark_low_light_area_density_flag) || (picture_control_set_ptr->parent_pcs_ptr->intra_coded_block_probability > 90))) {
        context_ptr->skip_qpm_flag = EB_TRUE;
    }

    if (sequence_control_set_ptr->input_resolution < INPUT_SIZE_4K_RANGE) {
        context_ptr->skip_qpm_flag = EB_TRUE;
    }

#if ADD_DELTA_QP_SUPPORT
    context_ptr->skip_qpm_flag = EB_FALSE;
#endif

    if (context_ptr->skip_qpm_flag == EB_FALSE) {
        if (picture_control_set_ptr->parent_pcs_ptr->pic_homogenous_over_time_sb_percentage > 30 && picture_control_set_ptr->slice_type != I_SLICE) {
#if ADD_DELTA_QP_SUPPORT
            context_ptr->qpmQp = CLIP3(min_qp_allowed, max_qp_allowed, picture_qp + deltaQpRes);
#else
            context_ptr->qpmQp = CLIP3(min_qp_allowed, max_qp_allowed, picture_qp + 1);
#endif
        }
    }


    return return_error;
}
#if ADD_DELTA_QP_SUPPORT
/*****************************************************************************
* NM - Note: Clean up
* AV1 QPM is SB based and all sub-Lcu buffers needs to be removed
******************************************************************************/
EbErrorType Av1QpModulationLcu(
    SequenceControlSet_t                   *sequence_control_set_ptr,
    PictureControlSet_t                    *picture_control_set_ptr,
    LargestCodingUnit_t                    *sb_ptr,
    uint32_t                                  sb_index,
    uint8_t                                   type,
    EncDecContext_t                        *context_ptr)
{
    EbErrorType                            return_error = EB_ErrorNone;

    int64_t                                  complexityDistance;
    int8_t                                   delta_qp = 0;
    uint16_t                                  qpmQp = context_ptr->qpmQp;
    uint16_t                                  min_qp_allowed = 0;
    uint16_t                                  max_qp_allowed = 255;
    uint16_t                                  cu_qp;
    EbBool                                 acEnergyBasedAntiContouring = picture_control_set_ptr->slice_type == I_SLICE ? EB_TRUE : EB_FALSE;
    uint8_t                                   lowerQPClass;

    int8_t    non_moving_delta_qp = context_ptr->non_moving_delta_qp;
    int8_t    bea64x64DeltaQp;

    uint8_t   deltaQpRes = picture_control_set_ptr->parent_pcs_ptr->delta_q_res;

    cu_qp = qpmQp;
    sb_ptr->qp = qpmQp;

    uint32_t  distortion = 0;

    if (!context_ptr->skip_qpm_flag) {

        // INTRA MODE
        if (type == INTRA_MODE) {




            OisCu32Cu16Results_t  *oisCu32Cu16ResultsPtr = picture_control_set_ptr->parent_pcs_ptr->ois_cu32_cu16_results[sb_index];
            //OisCu8Results_t         *oisCu8ResultsPtr = picture_control_set_ptr->parent_pcs_ptr->ois_cu8_results[sb_index];

            distortion =
                oisCu32Cu16ResultsPtr->sorted_ois_candidate[1][0].distortion +
                oisCu32Cu16ResultsPtr->sorted_ois_candidate[2][0].distortion +
                oisCu32Cu16ResultsPtr->sorted_ois_candidate[3][0].distortion +
                oisCu32Cu16ResultsPtr->sorted_ois_candidate[4][0].distortion;



            distortion = (uint32_t)CLIP3(picture_control_set_ptr->parent_pcs_ptr->intra_complexity_min[0], picture_control_set_ptr->parent_pcs_ptr->intra_complexity_max[0], distortion);
            complexityDistance = ((int32_t)distortion - (int32_t)picture_control_set_ptr->parent_pcs_ptr->intra_complexity_avg[0]);

            if (complexityDistance < 0) {

                delta_qp = (picture_control_set_ptr->parent_pcs_ptr->intra_min_distance[0] != 0) ? (int8_t)((context_ptr->min_delta_qp_weight * context_ptr->min_delta_qp[0] * complexityDistance) / (100 * picture_control_set_ptr->parent_pcs_ptr->intra_min_distance[0])) : 0;
            }
            else {

                delta_qp = (picture_control_set_ptr->parent_pcs_ptr->intra_max_distance[0] != 0) ? (int8_t)((context_ptr->max_delta_qp_weight * context_ptr->max_delta_qp[0] * complexityDistance) / (100 * picture_control_set_ptr->parent_pcs_ptr->intra_max_distance[0])) : 0;
            }



        }
        // INTER MODE
        else {


            distortion = picture_control_set_ptr->parent_pcs_ptr->me_results[sb_index][0].distortionDirection[0].distortion;


            distortion = (uint32_t)CLIP3(picture_control_set_ptr->parent_pcs_ptr->inter_complexity_min[0], picture_control_set_ptr->parent_pcs_ptr->inter_complexity_max[0], distortion);
            complexityDistance = ((int32_t)distortion - (int32_t)picture_control_set_ptr->parent_pcs_ptr->inter_complexity_avg[0]);

            if (complexityDistance < 0) {

                delta_qp = (picture_control_set_ptr->parent_pcs_ptr->inter_min_distance[0] != 0) ? (int8_t)((context_ptr->min_delta_qp_weight * context_ptr->min_delta_qp[0] * complexityDistance) / (100 * picture_control_set_ptr->parent_pcs_ptr->inter_min_distance[0])) : 0;
            }
            else {

                delta_qp = (picture_control_set_ptr->parent_pcs_ptr->inter_max_distance[0] != 0) ? (int8_t)((context_ptr->max_delta_qp_weight * context_ptr->max_delta_qp[0] * complexityDistance) / (100 * picture_control_set_ptr->parent_pcs_ptr->inter_max_distance[0])) : 0;
            }

        }

        if (context_ptr->backgorund_enhancement) {
            // Use the 8x8 background enhancement only for the Intra slice, otherwise, use the existing SB based BEA results
            bea64x64DeltaQp = non_moving_delta_qp;

            if ((picture_control_set_ptr->parent_pcs_ptr->yMean[sb_index][0] > ANTI_CONTOURING_LUMA_T2) || (picture_control_set_ptr->parent_pcs_ptr->yMean[sb_index][0] < ANTI_CONTOURING_LUMA_T1)) {

                if (bea64x64DeltaQp < 0) {
                    bea64x64DeltaQp = 0;
                }

            }

            delta_qp += bea64x64DeltaQp;
        }

        if ((picture_control_set_ptr->parent_pcs_ptr->logo_pic_flag)) {
            delta_qp = (delta_qp < context_ptr->min_delta_qp[0]) ? delta_qp : context_ptr->min_delta_qp[0];
        }

        SbStat_t *sb_stat_ptr = &(picture_control_set_ptr->parent_pcs_ptr->sb_stat_array[sb_index]);
        if (sb_stat_ptr->stationary_edge_over_time_flag && delta_qp > 0) {
            delta_qp = 0;
        }

        if (acEnergyBasedAntiContouring) {

            lowerQPClass = DeriveContouringClass(
                sb_ptr->picture_control_set_ptr->parent_pcs_ptr,
                sb_ptr->index,
                (uint8_t)1/*cu_index*/);

            if (lowerQPClass) {
                if (lowerQPClass == 3)
                    delta_qp = ANTI_CONTOURING_DELTA_QP_0;
                else if (lowerQPClass == 2)
                    delta_qp = ANTI_CONTOURING_DELTA_QP_1;
                else if (lowerQPClass == 1)
                    delta_qp = ANTI_CONTOURING_DELTA_QP_2;
            }
        }


        delta_qp -= context_ptr->grass_enhancement_flag ? 3 : 0;

        delta_qp *= deltaQpRes;


        if (sequence_control_set_ptr->static_config.rate_control_mode == 1 || sequence_control_set_ptr->static_config.rate_control_mode == 2) {

            if (qpmQp > (RC_QPMOD_MAXQP * deltaQpRes)) {
                delta_qp = MIN(0, delta_qp);
            }


            cu_qp = (uint32_t)(qpmQp + delta_qp);


            if ((qpmQp <= (RC_QPMOD_MAXQP *deltaQpRes))) {
                cu_qp = (uint8_t)CLIP3(
                    min_qp_allowed,
                    RC_QPMOD_MAXQP*deltaQpRes,
                    cu_qp);
            }
        }
        else {
            cu_qp = (uint8_t)(qpmQp + delta_qp);
        }

        cu_qp = (uint8_t)CLIP3(
            min_qp_allowed,
            max_qp_allowed,
            cu_qp);


    }

    sb_ptr->qp = sequence_control_set_ptr->static_config.improve_sharpness ? cu_qp : qpmQp;


    sb_ptr->delta_qp = (int16_t)sb_ptr->qp - (int16_t)qpmQp;

    sb_ptr->org_delta_qp = sb_ptr->delta_qp;

    if (sb_ptr->delta_qp % deltaQpRes != 0)
        printf("Qpm_error: delta_qp must be multiplier of deltaQpRes\n");
    if (sb_ptr->qp == 0)
        printf("Qpm_error: qp must be greater than 0 when use_delta_q is ON\n");

    return return_error;
}

#endif
EbErrorType EncQpmDeriveDeltaQPForEachLeafLcu(
    SequenceControlSet_t                   *sequence_control_set_ptr,
    PictureControlSet_t                    *picture_control_set_ptr,
    LargestCodingUnit_t                    *sb_ptr,
    uint32_t                                  sb_index,
    CodingUnit_t                           *cu_ptr,
    uint32_t                                  cu_depth,
    uint32_t                                  cu_index,
    uint32_t                                  cu_size,
    uint8_t                                   type,
    uint8_t                                   parent32x32Index,
    EncDecContext_t                        *context_ptr)
{
    EbErrorType                    return_error = EB_ErrorNone;


    //SbParams_t                        sb_params;
    int64_t                          complexityDistance;
    int8_t                           delta_qp = 0;
    uint8_t                           qpmQp = (uint8_t)context_ptr->qpmQp;
    uint8_t                           min_qp_allowed = (uint8_t)sequence_control_set_ptr->static_config.min_qp_allowed;
    uint8_t                           max_qp_allowed = (uint8_t)sequence_control_set_ptr->static_config.max_qp_allowed;
    uint8_t                           cu_qp;

#if ENCODER_MODE_CLEANUP
    EbBool  use16x16Stat = EB_FALSE;

#else
    EbBool  use16x16Stat = (sequence_control_set_ptr->input_resolution == INPUT_SIZE_4K_RANGE &&
        picture_control_set_ptr->enc_mode >= ENC_M3 &&
        picture_control_set_ptr->slice_type != I_SLICE && cu_size == 8);
#endif
    uint32_t usedDepth = cu_depth;
    if (use16x16Stat)
        usedDepth = 2;



    uint32_t cuIndexInRaterScan = MD_SCAN_TO_RASTER_SCAN[cu_index];



    EbBool                         acEnergyBasedAntiContouring = picture_control_set_ptr->slice_type == I_SLICE ? EB_TRUE : EB_FALSE;
    uint8_t                           lowerQPClass;

    int8_t    non_moving_delta_qp = context_ptr->non_moving_delta_qp;
    int8_t    bea64x64DeltaQp;

    cu_qp = qpmQp;
    cu_ptr->qp = qpmQp;

    uint32_t  distortion = 0;

    if (!context_ptr->skip_qpm_flag) {

        // INTRA MODE
        if (type == INTRA_MODE) {




            OisCu32Cu16Results_t  *oisCu32Cu16ResultsPtr = picture_control_set_ptr->parent_pcs_ptr->ois_cu32_cu16_results[sb_index];
            OisCu8Results_t         *oisCu8ResultsPtr = picture_control_set_ptr->parent_pcs_ptr->ois_cu8_results[sb_index];

            if (cu_size > 32) {
                distortion =
                    oisCu32Cu16ResultsPtr->sorted_ois_candidate[1][0].distortion +
                    oisCu32Cu16ResultsPtr->sorted_ois_candidate[2][0].distortion +
                    oisCu32Cu16ResultsPtr->sorted_ois_candidate[3][0].distortion +
                    oisCu32Cu16ResultsPtr->sorted_ois_candidate[4][0].distortion;
            }
            else if (cu_size == 32) {
                const uint32_t me2Nx2NTableOffset = context_ptr->cu_stats->cuNumInDepth + me2Nx2NOffset[context_ptr->cu_stats->depth];
                distortion = oisCu32Cu16ResultsPtr->sorted_ois_candidate[me2Nx2NTableOffset][0].distortion;
            }
            else {
                if (cu_size > 8) {
                    const uint32_t me2Nx2NTableOffset = context_ptr->cu_stats->cuNumInDepth + me2Nx2NOffset[context_ptr->cu_stats->depth];
                    distortion = oisCu32Cu16ResultsPtr->sorted_ois_candidate[me2Nx2NTableOffset][0].distortion;
                }
                else {


                    if (use16x16Stat) {

                        const CodedUnitStats_t  *cu_stats = GetCodedUnitStats(ParentBlockIndex[cu_index]);
                        const uint32_t me2Nx2NTableOffset = cu_stats->cuNumInDepth + me2Nx2NOffset[cu_stats->depth];

                        distortion = oisCu32Cu16ResultsPtr->sorted_ois_candidate[me2Nx2NTableOffset][0].distortion;
                    }
                    else {



                        const uint32_t me2Nx2NTableOffset = context_ptr->cu_stats->cuNumInDepth;

                        if (oisCu8ResultsPtr->sorted_ois_candidate[me2Nx2NTableOffset][0].valid_distortion) {
                            distortion = oisCu8ResultsPtr->sorted_ois_candidate[me2Nx2NTableOffset][0].distortion;
                        }
                        else {

                            const CodedUnitStats_t  *cu_stats = GetCodedUnitStats(ParentBlockIndex[cu_index]);
                            const uint32_t me2Nx2NTableOffset = cu_stats->cuNumInDepth + me2Nx2NOffset[cu_stats->depth];

                            if (oisCu32Cu16ResultsPtr->sorted_ois_candidate[me2Nx2NTableOffset][0].valid_distortion) {
                                distortion = oisCu32Cu16ResultsPtr->sorted_ois_candidate[me2Nx2NTableOffset][0].distortion;
                            }
                            else {
                                distortion = 0;
                            }
                        }

                    }


                }
            }






            distortion = (uint32_t)CLIP3(picture_control_set_ptr->parent_pcs_ptr->intra_complexity_min[usedDepth], picture_control_set_ptr->parent_pcs_ptr->intra_complexity_max[usedDepth], distortion);
            complexityDistance = ((int32_t)distortion - (int32_t)picture_control_set_ptr->parent_pcs_ptr->intra_complexity_avg[usedDepth]);

            if (complexityDistance < 0) {

                delta_qp = (picture_control_set_ptr->parent_pcs_ptr->intra_min_distance[usedDepth] != 0) ? (int8_t)((context_ptr->min_delta_qp_weight * context_ptr->min_delta_qp[usedDepth] * complexityDistance) / (100 * picture_control_set_ptr->parent_pcs_ptr->intra_min_distance[usedDepth])) : 0;
            }
            else {

                delta_qp = (picture_control_set_ptr->parent_pcs_ptr->intra_max_distance[usedDepth] != 0) ? (int8_t)((context_ptr->max_delta_qp_weight * context_ptr->max_delta_qp[usedDepth] * complexityDistance) / (100 * picture_control_set_ptr->parent_pcs_ptr->intra_max_distance[usedDepth])) : 0;
            }



        }
        // INTER MODE
        else {


            distortion = picture_control_set_ptr->parent_pcs_ptr->me_results[sb_index][cuIndexInRaterScan].distortionDirection[0].distortion;



            if (use16x16Stat) {
                uint32_t cuIndexRScan = MD_SCAN_TO_RASTER_SCAN[ParentBlockIndex[cu_index]];

                distortion = picture_control_set_ptr->parent_pcs_ptr->me_results[sb_index][cuIndexRScan].distortionDirection[0].distortion;

            }
            distortion = (uint32_t)CLIP3(picture_control_set_ptr->parent_pcs_ptr->inter_complexity_min[usedDepth], picture_control_set_ptr->parent_pcs_ptr->inter_complexity_max[usedDepth], distortion);
            complexityDistance = ((int32_t)distortion - (int32_t)picture_control_set_ptr->parent_pcs_ptr->inter_complexity_avg[usedDepth]);

            if (complexityDistance < 0) {

                delta_qp = (picture_control_set_ptr->parent_pcs_ptr->inter_min_distance[usedDepth] != 0) ? (int8_t)((context_ptr->min_delta_qp_weight * context_ptr->min_delta_qp[usedDepth] * complexityDistance) / (100 * picture_control_set_ptr->parent_pcs_ptr->inter_min_distance[usedDepth])) : 0;
            }
            else {

                delta_qp = (picture_control_set_ptr->parent_pcs_ptr->inter_max_distance[usedDepth] != 0) ? (int8_t)((context_ptr->max_delta_qp_weight * context_ptr->max_delta_qp[usedDepth] * complexityDistance) / (100 * picture_control_set_ptr->parent_pcs_ptr->inter_max_distance[usedDepth])) : 0;
            }



        }

        if (context_ptr->backgorund_enhancement) {
            // Use the 8x8 background enhancement only for the Intra slice, otherwise, use the existing SB based BEA results
            bea64x64DeltaQp = non_moving_delta_qp;

            if (((cu_index > 0) && ((picture_control_set_ptr->parent_pcs_ptr->yMean[sb_index][parent32x32Index]) > ANTI_CONTOURING_LUMA_T2 || (picture_control_set_ptr->parent_pcs_ptr->yMean[sb_index][parent32x32Index]) < ANTI_CONTOURING_LUMA_T1)) ||
                ((cu_index == 0) && ((picture_control_set_ptr->parent_pcs_ptr->yMean[sb_index][0]) > ANTI_CONTOURING_LUMA_T2 || (picture_control_set_ptr->parent_pcs_ptr->yMean[sb_index][0]) < ANTI_CONTOURING_LUMA_T1))) {

                if (bea64x64DeltaQp < 0) {
                    bea64x64DeltaQp = 0;
                }

            }

            delta_qp += bea64x64DeltaQp;
        }

        if ((picture_control_set_ptr->parent_pcs_ptr->logo_pic_flag)) {
            delta_qp = (delta_qp < context_ptr->min_delta_qp[0]) ? delta_qp : context_ptr->min_delta_qp[0];
        }

        SbStat_t *sb_stat_ptr = &(picture_control_set_ptr->parent_pcs_ptr->sb_stat_array[sb_index]);
        if (sb_stat_ptr->stationary_edge_over_time_flag && delta_qp > 0) {
            delta_qp = 0;
        }

        if (acEnergyBasedAntiContouring) {

            lowerQPClass = DeriveContouringClass(
                sb_ptr->picture_control_set_ptr->parent_pcs_ptr,
                sb_ptr->index,
                (uint8_t)cu_index);

            if (lowerQPClass) {
                if (lowerQPClass == 3)
                    delta_qp = ANTI_CONTOURING_DELTA_QP_0;
                else if (lowerQPClass == 2)
                    delta_qp = ANTI_CONTOURING_DELTA_QP_1;
                else if (lowerQPClass == 1)
                    delta_qp = ANTI_CONTOURING_DELTA_QP_2;
            }
        }


        delta_qp -= context_ptr->grass_enhancement_flag ? 3 : 0;

        if (sequence_control_set_ptr->static_config.rate_control_mode == 1 || sequence_control_set_ptr->static_config.rate_control_mode == 2) {

            if (qpmQp > RC_QPMOD_MAXQP) {
                delta_qp = MIN(0, delta_qp);
            }

            cu_qp = (uint32_t)(qpmQp + delta_qp);


            if ((qpmQp <= RC_QPMOD_MAXQP)) {
                cu_qp = (uint8_t)CLIP3(
                    min_qp_allowed,
                    RC_QPMOD_MAXQP,
                    cu_qp);
            }
        }
        else {
            cu_qp = (uint8_t)(qpmQp + delta_qp);
        }

        cu_qp = (uint8_t)CLIP3(
            min_qp_allowed,
            max_qp_allowed,
            cu_qp);


    }

    cu_ptr->qp = sequence_control_set_ptr->static_config.improve_sharpness ? cu_qp : qpmQp;

    sb_ptr->qp = (cu_size == 64) ? (uint8_t)cu_ptr->qp : sb_ptr->qp;


    cu_ptr->delta_qp = (int16_t)cu_ptr->qp - (int16_t)qpmQp;

    cu_ptr->org_delta_qp = cu_ptr->delta_qp;


    return return_error;
}

void Store16bitInputSrc(
    EncDecContext_t         *context_ptr,
    PictureControlSet_t     *picture_control_set_ptr,
    uint32_t                 lcuX,
    uint32_t                 lcuY,
    uint32_t                 lcuW,
    uint32_t                 lcuH ){

    uint32_t rowIt;
    uint16_t* fromPtr;
    uint16_t* toPtr;

    fromPtr = (uint16_t*)context_ptr->input_sample16bit_buffer->bufferY;
    toPtr = (uint16_t*)picture_control_set_ptr->input_frame16bit->bufferY + (lcuX + picture_control_set_ptr->input_frame16bit->origin_x) + (lcuY + picture_control_set_ptr->input_frame16bit->origin_y)*picture_control_set_ptr->input_frame16bit->strideY;

    for (rowIt = 0; rowIt < lcuH; rowIt++)
    {
        memcpy(toPtr + rowIt * picture_control_set_ptr->input_frame16bit->strideY, fromPtr + rowIt * context_ptr->input_sample16bit_buffer->strideY, lcuW * 2);
    }

    lcuX = lcuX / 2;
    lcuY = lcuY / 2;
    lcuW = lcuW / 2;
    lcuH = lcuH / 2;

    fromPtr = (uint16_t*)context_ptr->input_sample16bit_buffer->bufferCb;
    toPtr = (uint16_t*)picture_control_set_ptr->input_frame16bit->bufferCb + (lcuX + picture_control_set_ptr->input_frame16bit->origin_x / 2) + (lcuY + picture_control_set_ptr->input_frame16bit->origin_y / 2)*picture_control_set_ptr->input_frame16bit->strideCb;

    for (rowIt = 0; rowIt < lcuH; rowIt++)
    {
        memcpy(toPtr + rowIt * picture_control_set_ptr->input_frame16bit->strideCb, fromPtr + rowIt * context_ptr->input_sample16bit_buffer->strideCb, lcuW * 2);
    }

    fromPtr = (uint16_t*)context_ptr->input_sample16bit_buffer->bufferCr;
    toPtr = (uint16_t*)picture_control_set_ptr->input_frame16bit->bufferCr + (lcuX + picture_control_set_ptr->input_frame16bit->origin_x / 2) + (lcuY + picture_control_set_ptr->input_frame16bit->origin_y / 2)*picture_control_set_ptr->input_frame16bit->strideCb;

    for (rowIt = 0; rowIt < lcuH; rowIt++)
    {
        memcpy(toPtr + rowIt * picture_control_set_ptr->input_frame16bit->strideCr, fromPtr + rowIt * context_ptr->input_sample16bit_buffer->strideCr, lcuW * 2);
    }


}

void update_av1_mi_map(
    CodingUnit_t                   *cu_ptr,
    uint32_t                          cu_origin_x,
    uint32_t                          cu_origin_y,
    const BlockGeom                 *blk_geom,
    PictureControlSet_t            *picture_control_set_ptr);

void move_cu_data(
    CodingUnit_t *src_cu,
    CodingUnit_t *dst_cu);



/*******************************************
* Encode Pass
*
* Summary: Performs a H.265 conformant
*   reconstruction based on the LCU
*   mode decision.
*
* Inputs:
*   SourcePic
*   Coding Results
*   SB Location
*   Sequence Control Set
*   Picture Control Set
*
* Outputs:
*   Reconstructed Samples
*   Coefficient Samples
*
*******************************************/
EB_EXTERN void AV1EncodePass(
    SequenceControlSet_t      *sequence_control_set_ptr,
    PictureControlSet_t       *picture_control_set_ptr,
    LargestCodingUnit_t       *sb_ptr,
    uint32_t                   tbAddr,
    uint32_t                   sb_origin_x,
    uint32_t                   sb_origin_y,
    uint32_t                   sb_qp,
    EncDecContext_t           *context_ptr)
{
    const EbBool                    is16bit = context_ptr->is16bit;
    const EbColorFormat             color_format = context_ptr->color_format;
    EbPictureBufferDesc_t    *reconBuffer = is16bit ? picture_control_set_ptr->recon_picture16bit_ptr : picture_control_set_ptr->recon_picture_ptr;
    EbPictureBufferDesc_t    *coeff_buffer_sb = sb_ptr->quantized_coeff; //32bit
    EbPictureBufferDesc_t    *inputPicture;
    ModeDecisionContext_t    *mdcontextPtr;
    mdcontextPtr = context_ptr->md_context;
    inputPicture = context_ptr->input_samples = (EbPictureBufferDesc_t*)picture_control_set_ptr->parent_pcs_ptr->enhanced_picture_ptr;

    SbStat_t                *sb_stat_ptr = &(picture_control_set_ptr->parent_pcs_ptr->sb_stat_array[tbAddr]);

    EbBool                    availableCoeff;

    // QP Neighbor Arrays
    EbBool                    isDeltaQpNotCoded = EB_TRUE;

    // SB Stats
    uint32_t                  sb_width = MIN(sequence_control_set_ptr->sb_size_pix, sequence_control_set_ptr->luma_width - sb_origin_x);
    uint32_t                  sb_height = MIN(sequence_control_set_ptr->sb_size_pix, sequence_control_set_ptr->luma_height - sb_origin_y);
    // MV merge mode
    uint32_t                  y_has_coeff;
    uint32_t                  u_has_coeff;
    uint32_t                  v_has_coeff;
    EbAsm                     asm_type = sequence_control_set_ptr->encode_context_ptr->asm_type;
    uint64_t                  y_coeff_bits;
    uint64_t                  cb_coeff_bits;
    uint64_t                  cr_coeff_bits;
    uint64_t                  y_full_distortion[DIST_CALC_TOTAL];
    uint64_t                  yTuFullDistortion[DIST_CALC_TOTAL];
    uint32_t                  count_non_zero_coeffs[3];
    MacroblockPlane           cuPlane[3];
    uint16_t                  eobs[MAX_TXB_COUNT][3];
    uint64_t                  yTuCoeffBits;
    uint64_t                  cbTuCoeffBits;
    uint64_t                  crTuCoeffBits;
    EncodeContext_t          *encode_context_ptr;
    uint32_t                  lcuRowIndex = sb_origin_y / BLOCK_SIZE_64;

    // Dereferencing early
    NeighborArrayUnit_t      *ep_mode_type_neighbor_array = picture_control_set_ptr->ep_mode_type_neighbor_array;
#if 0
    NeighborArrayUnit_t      *ep_intra_luma_mode_neighbor_array = picture_control_set_ptr->ep_intra_luma_mode_neighbor_array;
    NeighborArrayUnit_t      *ep_intra_chroma_mode_neighbor_array = picture_control_set_ptr->ep_intra_chroma_mode_neighbor_array;
#else
    NeighborArrayUnit_t      *ep_intra_mode_neighbor_array[3];
    ep_intra_mode_neighbor_array[0] = picture_control_set_ptr->ep_intra_luma_mode_neighbor_array;
    ep_intra_mode_neighbor_array[1] = picture_control_set_ptr->ep_intra_chroma_mode_neighbor_array_cb;
    ep_intra_mode_neighbor_array[2] = picture_control_set_ptr->ep_intra_chroma_mode_neighbor_array_cr;
#endif
    NeighborArrayUnit_t      *ep_mv_neighbor_array = picture_control_set_ptr->ep_mv_neighbor_array;
#if 0
    NeighborArrayUnit_t      *ep_luma_recon_neighbor_array = is16bit ? picture_control_set_ptr->ep_luma_recon_neighbor_array16bit : picture_control_set_ptr->ep_luma_recon_neighbor_array;
    NeighborArrayUnit_t      *ep_cb_recon_neighbor_array = is16bit ? picture_control_set_ptr->ep_cb_recon_neighbor_array16bit : picture_control_set_ptr->ep_cb_recon_neighbor_array;
    NeighborArrayUnit_t      *ep_cr_recon_neighbor_array = is16bit ? picture_control_set_ptr->ep_cr_recon_neighbor_array16bit : picture_control_set_ptr->ep_cr_recon_neighbor_array;
#else
    NeighborArrayUnit_t      *recon_neighbor_array[3];
    recon_neighbor_array[0] = is16bit ? picture_control_set_ptr->ep_luma_recon_neighbor_array16bit : picture_control_set_ptr->ep_luma_recon_neighbor_array;
    recon_neighbor_array[1] = is16bit ? picture_control_set_ptr->ep_cb_recon_neighbor_array16bit : picture_control_set_ptr->ep_cb_recon_neighbor_array;
    recon_neighbor_array[2] = is16bit ? picture_control_set_ptr->ep_cr_recon_neighbor_array16bit : picture_control_set_ptr->ep_cr_recon_neighbor_array;
#endif
    NeighborArrayUnit_t      *ep_skip_flag_neighbor_array = picture_control_set_ptr->ep_skip_flag_neighbor_array;

    EbBool                 constrained_intra_flag = picture_control_set_ptr->constrained_intra_flag;

    EbBool dlfEnableFlag = (EbBool)(picture_control_set_ptr->parent_pcs_ptr->loop_filter_mode &&
        (picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag ||
            sequence_control_set_ptr->static_config.recon_enabled ||
            sequence_control_set_ptr->static_config.stat_report));

    const EbBool isIntraLCU = picture_control_set_ptr->limit_intra ? EB_FALSE : EB_TRUE;

    EbBool doRecon = (EbBool)(
        (picture_control_set_ptr->limit_intra == 0 || isIntraLCU == 1) ||
        picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag ||
        sequence_control_set_ptr->static_config.recon_enabled ||
        sequence_control_set_ptr->static_config.stat_report);

    EntropyCoder_t  *coeff_est_entropy_coder_ptr = picture_control_set_ptr->coeff_est_entropy_coder_ptr;

    uint32_t           dZoffset = 0;

    if (!sb_stat_ptr->stationary_edge_over_time_flag && sequence_control_set_ptr->static_config.improve_sharpness && picture_control_set_ptr->parent_pcs_ptr->pic_noise_class < PIC_NOISE_CLASS_3_1) {

        int16_t cuDeltaQp = (int16_t)(sb_ptr->qp - picture_control_set_ptr->parent_pcs_ptr->average_qp);
        uint32_t dzCondition = cuDeltaQp > 0 ? 0 : 1;

        if (sequence_control_set_ptr->input_resolution == INPUT_SIZE_4K_RANGE) {

            if (!(picture_control_set_ptr->parent_pcs_ptr->is_pan ||
                (picture_control_set_ptr->parent_pcs_ptr->non_moving_index_average < 10 && sb_ptr->aura_status_iii) ||
                (sb_stat_ptr->cu_stat_array[0].skin_area) ||
                (picture_control_set_ptr->parent_pcs_ptr->intra_coded_block_probability > 90) ||
                (picture_control_set_ptr->parent_pcs_ptr->high_dark_area_density_flag))) {

                if (picture_control_set_ptr->slice_type != I_SLICE &&
                    picture_control_set_ptr->temporal_layer_index == 0 &&
                    picture_control_set_ptr->parent_pcs_ptr->intra_coded_block_probability > 60 &&
                    !picture_control_set_ptr->parent_pcs_ptr->is_tilt &&
                    picture_control_set_ptr->parent_pcs_ptr->pic_homogenous_over_time_sb_percentage > 40)
                {
                    dZoffset = 10;
                }

                if (dzCondition) {
                    if (picture_control_set_ptr->scene_caracteristic_id == EB_FRAME_CARAC_1) {
                        if (picture_control_set_ptr->slice_type == I_SLICE) {
                            dZoffset = sb_stat_ptr->cu_stat_array[0].grass_area ? 10 : dZoffset;
                        }
                        else if (picture_control_set_ptr->temporal_layer_index == 0) {
                            dZoffset = sb_stat_ptr->cu_stat_array[0].grass_area ? 9 : dZoffset;
                        }
                        else if (picture_control_set_ptr->temporal_layer_index == 1) {
                            dZoffset = sb_stat_ptr->cu_stat_array[0].grass_area ? 5 : dZoffset;
                        }
                    }
                }
            }
        }
    }
    if (sequence_control_set_ptr->static_config.improve_sharpness) {

        QpmDeriveBeaAndSkipQpmFlagLcu(
            sequence_control_set_ptr,
            picture_control_set_ptr,
            sb_ptr,
            tbAddr,
            context_ptr);
    }
    else {
        context_ptr->skip_qpm_flag = EB_TRUE;
    }


    encode_context_ptr = ((SequenceControlSet_t*)(picture_control_set_ptr->sequence_control_set_wrapper_ptr->objectPtr))->encode_context_ptr;

    if (picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag == EB_TRUE) {

        //get the 16bit form of the input LCU
        if (is16bit) {

            reconBuffer = ((EbReferenceObject_t*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->objectPtr)->referencePicture16bit;

        }

        else {
            reconBuffer = ((EbReferenceObject_t*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->objectPtr)->referencePicture;
        }
    }
    else { // non ref pictures
        reconBuffer = is16bit ? picture_control_set_ptr->recon_picture16bit_ptr : picture_control_set_ptr->recon_picture_ptr;
    }


    EbBool use_delta_qp = (EbBool)sequence_control_set_ptr->static_config.improve_sharpness;
    EbBool oneSegment = (sequence_control_set_ptr->enc_dec_segment_col_count_array[picture_control_set_ptr->temporal_layer_index] == 1) && (sequence_control_set_ptr->enc_dec_segment_row_count_array[picture_control_set_ptr->temporal_layer_index] == 1);
    EbBool useDeltaQpSegments = oneSegment ? 0 : (EbBool)sequence_control_set_ptr->static_config.improve_sharpness;

    // DeriveZeroLumaCbf
    EbBool  highIntraRef = EB_FALSE;
    EbBool  checkZeroLumaCbf = EB_FALSE;

    if (is16bit) {


        //SB128_TODO change 10bit SB creation

        if ((sequence_control_set_ptr->static_config.ten_bit_format == 1) || (sequence_control_set_ptr->static_config.compressed_ten_bit_format == 1))
        {

            const uint32_t inputLumaOffset = ((sb_origin_y + inputPicture->origin_y)         * inputPicture->strideY) + (sb_origin_x + inputPicture->origin_x);
            const uint32_t inputCbOffset = (((sb_origin_y + inputPicture->origin_y) >> 1)  * inputPicture->strideCb) + ((sb_origin_x + inputPicture->origin_x) >> 1);
            const uint32_t inputCrOffset = (((sb_origin_y + inputPicture->origin_y) >> 1)  * inputPicture->strideCr) + ((sb_origin_x + inputPicture->origin_x) >> 1);
            const uint16_t luma2BitWidth = inputPicture->width / 4;
            const uint16_t chroma2BitWidth = inputPicture->width / 8;


            CompressedPackLcu(
                inputPicture->bufferY + inputLumaOffset,
                inputPicture->strideY,
                inputPicture->bufferBitIncY + sb_origin_y * luma2BitWidth + (sb_origin_x / 4)*sb_height,
                sb_width / 4,
                (uint16_t *)context_ptr->input_sample16bit_buffer->bufferY,
                context_ptr->input_sample16bit_buffer->strideY,
                sb_width,
                sb_height,
                asm_type);

            CompressedPackLcu(
                inputPicture->bufferCb + inputCbOffset,
                inputPicture->strideCb,
                inputPicture->bufferBitIncCb + sb_origin_y / 2 * chroma2BitWidth + (sb_origin_x / 8)*(sb_height / 2),
                sb_width / 8,
                (uint16_t *)context_ptr->input_sample16bit_buffer->bufferCb,
                context_ptr->input_sample16bit_buffer->strideCb,
                sb_width >> 1,
                sb_height >> 1,
                asm_type);

            CompressedPackLcu(
                inputPicture->bufferCr + inputCrOffset,
                inputPicture->strideCr,
                inputPicture->bufferBitIncCr + sb_origin_y / 2 * chroma2BitWidth + (sb_origin_x / 8)*(sb_height / 2),
                sb_width / 8,
                (uint16_t *)context_ptr->input_sample16bit_buffer->bufferCr,
                context_ptr->input_sample16bit_buffer->strideCr,
                sb_width >> 1,
                sb_height >> 1,
                asm_type);

        }
        else {

            const uint32_t inputLumaOffset = ((sb_origin_y + inputPicture->origin_y)         * inputPicture->strideY) + (sb_origin_x + inputPicture->origin_x);
            const uint32_t inputBitIncLumaOffset = ((sb_origin_y + inputPicture->origin_y)         * inputPicture->strideBitIncY) + (sb_origin_x + inputPicture->origin_x);
            const uint32_t inputCbOffset = (((sb_origin_y + inputPicture->origin_y) >> 1)  * inputPicture->strideCb) + ((sb_origin_x + inputPicture->origin_x) >> 1);
            const uint32_t inputBitIncCbOffset = (((sb_origin_y + inputPicture->origin_y) >> 1)  * inputPicture->strideBitIncCb) + ((sb_origin_x + inputPicture->origin_x) >> 1);
            const uint32_t inputCrOffset = (((sb_origin_y + inputPicture->origin_y) >> 1)  * inputPicture->strideCr) + ((sb_origin_x + inputPicture->origin_x) >> 1);
            const uint32_t inputBitIncCrOffset = (((sb_origin_y + inputPicture->origin_y) >> 1)  * inputPicture->strideBitIncCr) + ((sb_origin_x + inputPicture->origin_x) >> 1);

            Pack2D_SRC(
                inputPicture->bufferY + inputLumaOffset,
                inputPicture->strideY,
                inputPicture->bufferBitIncY + inputBitIncLumaOffset,
                inputPicture->strideBitIncY,
                (uint16_t *)context_ptr->input_sample16bit_buffer->bufferY,
                context_ptr->input_sample16bit_buffer->strideY,
                sb_width,
                sb_height,
                asm_type);


            Pack2D_SRC(
                inputPicture->bufferCb + inputCbOffset,
                inputPicture->strideCr,
                inputPicture->bufferBitIncCb + inputBitIncCbOffset,
                inputPicture->strideBitIncCr,
                (uint16_t *)context_ptr->input_sample16bit_buffer->bufferCb,
                context_ptr->input_sample16bit_buffer->strideCb,
                sb_width >> 1,
                sb_height >> 1,
                asm_type);


            Pack2D_SRC(
                inputPicture->bufferCr + inputCrOffset,
                inputPicture->strideCr,
                inputPicture->bufferBitIncCr + inputBitIncCrOffset,
                inputPicture->strideBitIncCr,
                (uint16_t *)context_ptr->input_sample16bit_buffer->bufferCr,
                context_ptr->input_sample16bit_buffer->strideCr,
                sb_width >> 1,
                sb_height >> 1,
                asm_type);

        }

        Store16bitInputSrc(context_ptr, picture_control_set_ptr, sb_origin_x, sb_origin_y, sb_width, sb_height);


    }

    if ((sequence_control_set_ptr->input_resolution == INPUT_SIZE_4K_RANGE) && !picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag) {
        if (!((sb_stat_ptr->stationary_edge_over_time_flag) || (picture_control_set_ptr->parent_pcs_ptr->logo_pic_flag)))
        {
            if (picture_control_set_ptr->slice_type == B_SLICE) {

                EbReferenceObject_t  *refObjL0 = (EbReferenceObject_t*)picture_control_set_ptr->ref_pic_ptr_array[REF_LIST_0]->objectPtr;
                EbReferenceObject_t  *refObjL1 = (EbReferenceObject_t*)picture_control_set_ptr->ref_pic_ptr_array[REF_LIST_1]->objectPtr;
                uint32_t const TH = (sequence_control_set_ptr->static_config.frame_rate >> 16) < 50 ? 25 : 30;

                if ((refObjL0->tmpLayerIdx == 2 && refObjL0->intra_coded_area > TH) || (refObjL1->tmpLayerIdx == 2 && refObjL1->intra_coded_area > TH))
                    highIntraRef = EB_TRUE;

            }
            if (highIntraRef == EB_FALSE) {

                checkZeroLumaCbf = EB_TRUE;
            }
        }
    }
    context_ptr->intra_coded_area_sb[tbAddr] = 0;

    context_ptr->trans_coeff_shape_luma = 0;
    context_ptr->trans_coeff_shape_chroma = 0;
    context_ptr->coded_area_sb[0] = 0;
    context_ptr->coded_area_sb[1] = 0;
    context_ptr->coded_area_sb[2] = 0;

#if AV1_LF 
    if (dlfEnableFlag && picture_control_set_ptr->parent_pcs_ptr->loop_filter_mode == 1){        
        if (tbAddr == 0) {
            av1_loop_filter_init(picture_control_set_ptr);

            av1_pick_filter_level(
#if FILT_PROC
                0,
#else
                context_ptr,
#endif
                (EbPictureBufferDesc_t*)picture_control_set_ptr->parent_pcs_ptr->enhanced_picture_ptr,
                picture_control_set_ptr,
                LPF_PICK_FROM_Q);

            av1_loop_filter_frame_init(picture_control_set_ptr, 0, 3);
        }
    }
#endif
#if ADD_DELTA_QP_SUPPORT
    if (context_ptr->skip_qpm_flag == EB_FALSE && sequence_control_set_ptr->static_config.improve_sharpness) {

        Av1QpModulationLcu(
            sequence_control_set_ptr,
            picture_control_set_ptr,
            sb_ptr,
            tbAddr,
            picture_control_set_ptr->slice_type == I_SLICE ? INTRA_MODE : INTER_MODE,
            context_ptr);
    }
#endif


    uint32_t final_cu_itr = 0;

    // CU Loop

    uint32_t    blk_it = 0;

    while (blk_it < sequence_control_set_ptr->max_block_cnt) {

        CodingUnit_t  *cu_ptr = context_ptr->cu_ptr = &context_ptr->md_context->md_cu_arr_nsq[blk_it];
        PartitionType part = cu_ptr->part;

        const BlockGeom * blk_geom = context_ptr->blk_geom = Get_blk_geom_mds(blk_it);
        UNUSED(blk_geom);

        assert(blk_geom->valid_block == 1);
        sb_ptr->cu_partition_array[blk_it] = context_ptr->md_context->md_cu_arr_nsq[blk_it].part;


        //Jing:
        // Check for valid partition for 422!!!
        if (part != PARTITION_SPLIT) {
            int32_t offset_d1 = ns_blk_offset[(int32_t)part]; //cu_ptr->best_d1_blk; // TOCKECK
            int32_t num_d1_block = ns_blk_num[(int32_t)part]; // context_ptr->blk_geom->totns; // TOCKECK
            assert(part == PARTITION_NONE);

            for (int32_t d1_itr = (int32_t)blk_it + offset_d1; d1_itr < (int32_t)blk_it + offset_d1 + num_d1_block; d1_itr++) {

                const BlockGeom * blk_geom = context_ptr->blk_geom = Get_blk_geom_mds(d1_itr);
                assert(blk_geom->valid_block == 1);
                assert(blk_geom->bwidth == blk_geom->bheight);

                // PU Stack variables
                PredictionUnit_t        *pu_ptr = (PredictionUnit_t *)EB_NULL; //  done
                EbPictureBufferDesc_t   *residual_buffer = context_ptr->residual_buffer;
                EbPictureBufferDesc_t   *transform_buffer = context_ptr->transform_buffer;
                EbPictureBufferDesc_t   *inverse_quant_buffer = context_ptr->inverse_quant_buffer;

                int16_t                  *transform_inner_array_ptr = context_ptr->transform_inner_array_ptr;

                CodingUnit_t            *cu_ptr = context_ptr->cu_ptr = &context_ptr->md_context->md_cu_arr_nsq[d1_itr];

                context_ptr->cu_origin_x = (uint16_t)(sb_origin_x + blk_geom->origin_x);
                context_ptr->cu_origin_y = (uint16_t)(sb_origin_y + blk_geom->origin_y);
                cu_ptr->delta_qp = 0;
                cu_ptr->block_has_coeff = 0;

#ifdef DEBUG_REF_INFO
                //assert(cu_ptr->skip_flag == 0);
                //printf("(%d, %d), mode %d, angle is (%d, %d)\n",
                //        context_ptr->cu_origin_x, context_ptr->cu_origin_y, cu_ptr->pred_mode,
                //        cu_ptr->prediction_unit_array->angle_delta[0], cu_ptr->prediction_unit_array->angle_delta[1]);
#endif

                // if(picture_control_set_ptr->picture_number==4 && context_ptr->cu_origin_x==0 && context_ptr->cu_origin_y==0)
                //     printf("CHEDD");
                uint32_t  coded_area_org = context_ptr->coded_area_sb[0];
                uint32_t  coded_area_org_uv = context_ptr->coded_area_sb[1]; //Jing: change here for inter

#if CHROMA_BLIND
                // Derive disable_cfl_flag as evaluate_cfl_ep = f(disable_cfl_flag)
                EbBool disable_cfl_flag = (context_ptr->blk_geom->sq_size > 32 ||
                    context_ptr->blk_geom->bwidth == 4 ||
                    context_ptr->blk_geom->bheight == 4) ? EB_TRUE : EB_FALSE;
                // Evaluate cfl @ EP if applicable, and not done @ MD 
                context_ptr->evaluate_cfl_ep = (disable_cfl_flag == EB_FALSE && context_ptr->md_context->chroma_level == CHROMA_MODE_1);
                //Jing: Disable cfl first
                context_ptr->evaluate_cfl_ep = 0;
#endif

#if ADD_DELTA_QP_SUPPORT
                if (context_ptr->skip_qpm_flag == EB_FALSE && sequence_control_set_ptr->static_config.improve_sharpness) {
                    cu_ptr->qp = sb_ptr->qp;
                    cu_ptr->delta_qp = sb_ptr->delta_qp;
                    cu_ptr->org_delta_qp = sb_ptr->org_delta_qp;
                }
                else {
                    uint16_t                           picture_qp = picture_control_set_ptr->parent_pcs_ptr->base_qindex;
                    sb_ptr->qp = picture_qp;
                    cu_ptr->qp = sb_ptr->qp;
                    cu_ptr->delta_qp = 0;
                    cu_ptr->org_delta_qp = 0;
                }

#else
                cu_ptr->qp = (sequence_control_set_ptr->static_config.improve_sharpness) ? context_ptr->qpmQp : picture_control_set_ptr->picture_qp;
                sb_ptr->qp = (sequence_control_set_ptr->static_config.improve_sharpness) ? context_ptr->qpmQp : picture_control_set_ptr->picture_qp;
                cu_ptr->org_delta_qp = cu_ptr->delta_qp;
#endif
#if !ADD_DELTA_QP_SUPPORT
                //CHKN remove usage of depth
                if (!context_ptr->skip_qpm_flag && (sequence_control_set_ptr->static_config.improve_sharpness) && (0xFF <= picture_control_set_ptr->dif_cu_delta_qp_depth)) {
                    EncQpmDeriveDeltaQPForEachLeafLcu(
                        sequence_control_set_ptr,
                        picture_control_set_ptr,
                        sb_ptr,
                        tbAddr,
                        cu_ptr,
                        0xFF, //This is obviously not ok
                        d1_itr, // TOCHECK OMK
                        context_ptr->blk_geom->bwidth, // TOCHECK
                        cu_ptr->prediction_mode_flag,
                        context_ptr->cu_stats->parent32x32Index, // TOCHECK not valid
                        context_ptr);
                }

#endif

                if (cu_ptr->prediction_mode_flag == INTRA_MODE) {

#if ENCDEC_TX_SEARCH
#if ENCODER_MODE_CLEANUP
                    if (picture_control_set_ptr->enc_mode > ENC_M1) 
#endif
                        context_ptr->is_inter = 0;
#endif

                    context_ptr->tot_intra_coded_area += blk_geom->bwidth * blk_geom->bheight;
                    if (picture_control_set_ptr->slice_type != I_SLICE) {
                        context_ptr->intra_coded_area_sb[tbAddr] += blk_geom->bwidth* blk_geom->bheight;
                    }

                    // *Note - Transforms are the same size as predictions
                    // Partition Loop
                    context_ptr->txb_itr = 0;
                    int32_t plane_end = blk_geom->has_uv_ex ? 2 : 0;

                    for (int32_t plane = 0; plane <= plane_end; ++plane) {
                        TxSize  tx_size = plane ? blk_geom->txsize_uv_ex[context_ptr->txb_itr] : blk_geom->txsize[context_ptr->txb_itr];
                        int32_t plane_width = plane ? blk_geom->bwidth_uv_ex : blk_geom->bwidth;
                        int32_t plane_height = plane ? blk_geom->bheight_uv_ex : blk_geom->bheight;
                        const int32_t txw = tx_size_wide[tx_size];
                        const int32_t txh = tx_size_high[tx_size];

                        //TU loop
                        uint16_t txb_itr[3] = {0};
                        for (int row = 0; row < plane_height; row += txh) {
                            for (int col = 0; col < plane_width; col += txw) {
                                uint32_t pu_block_origin_x = context_ptr->cu_origin_x + col;
                                uint32_t pu_block_origin_y = context_ptr->cu_origin_y + row;
                                uint8_t subsampling_x = 0;
                                uint8_t subsampling_y = 0;
                                if (plane == 0) {
                                    subsampling_x = subsampling_y = 0;
                                    //Generate neighbor mode for current CU
                                    GeneratePuNeighborModes(
                                            context_ptr,
                                            pu_block_origin_x,
                                            pu_block_origin_y,
                                            ep_mode_type_neighbor_array);
                                } else {
                                    // Jing: TODO:
                                    // 4x4 case for chroma, its neighbor mode is same as 1st luma or last luma?
                                    subsampling_x = (color_format == EB_YUV444 ? 1 : 2) - 1;
                                    subsampling_y = (color_format >= EB_YUV422 ? 1 : 2) - 1;
                                    pu_block_origin_x = ROUND_UV_EX(context_ptr->cu_origin_x, subsampling_x) + col;
                                    pu_block_origin_y = ROUND_UV_EX(context_ptr->cu_origin_y, subsampling_y) + row;
                                }
                                pu_ptr = cu_ptr->prediction_unit_array;

                                // Generate Intra Neighbor Modes
                                GeneratePuIntraNeighborModes(
                                        context_ptr,
                                        cu_ptr,
                                        plane,
                                        pu_block_origin_x,
                                        pu_block_origin_y,
                                        ep_intra_mode_neighbor_array[plane]);
#if INTRA_10BIT_SUPPORT
                                {
                                    //Generate pred
                                    //Jing: to check if separate to 8bit and 16bit
                                    uint8_t    topNeighArray[64 * 2 + 1];
                                    uint8_t    leftNeighArray[64 * 2 + 1];
                                    PredictionMode mode;

                                    //InitNeighborArray,
                                    if (pu_block_origin_y != 0) {
                                        memcpy(topNeighArray + 1,
                                                recon_neighbor_array[plane]->topArray + pu_block_origin_x,
                                                txw * 2);
                                    }
                                    if (pu_block_origin_x != 0) {
                                        memcpy(leftNeighArray + 1,
                                                recon_neighbor_array[plane]->leftArray + pu_block_origin_y,
                                                txh * 2);
                                    }
                                    if (pu_block_origin_y != 0 && pu_block_origin_x != 0) {
                                        topNeighArray[0] = leftNeighArray[0] = ((recon_neighbor_array[plane]->topLeftArray) + (MAX_PICTURE_HEIGHT_SIZE >> subsampling_y) + pu_block_origin_x - pu_block_origin_y)[0];
                                    }

                                    if (plane) {
                                        mode = (pu_ptr->intra_chroma_mode == UV_CFL_PRED) ? (PredictionMode)UV_DC_PRED : (PredictionMode)pu_ptr->intra_chroma_mode;
                                    } else {
                                        mode = cu_ptr->pred_mode; //PredictionMode mode,
                                    }

                                    //Jing: TODO: add 16bit
                                    av1_predict_intra_block(
#if TILES
                                            &sb_ptr->tile_info,
#endif
#if INTRA_CORE_OPT
                                            NULL,
#endif
                                            ED_STAGE,
                                            pu_ptr->intra_luma_left_mode,
                                            pu_ptr->intra_luma_top_mode,
                                            pu_ptr->intra_chroma_left_mode,
                                            pu_ptr->intra_chroma_top_mode,
                                            context_ptr->blk_geom,
                                            picture_control_set_ptr->parent_pcs_ptr->av1_cm,                  //const Av1Common *cm,
                                            plane ? blk_geom->bwidth_uv_ex : blk_geom->bwidth,                   //int32_t wpx,
                                            plane ? blk_geom->bheight_uv_ex : blk_geom->bheight,                  //int32_t hpx,
                                            tx_size,
                                            mode,                                                       //PredictionMode mode,
                                            plane ? 0 : pu_ptr->angle_delta[PLANE_TYPE_Y],                //int32_t angle_delta,
                                            0,                                                          //int32_t use_palette,
                                            FILTER_INTRA_MODES,                                         //CHKN FILTER_INTRA_MODE filter_intra_mode,
                                            topNeighArray + 1,
                                            leftNeighArray + 1,
                                            reconBuffer,                                                //uint8_t *dst,
                                            //int32_t dst_stride,
#if !INTRA_CORE_OPT
                                            0,                                                          //int32_t col_off,
                                            0,                                                          //int32_t row_off,
#endif
                                            plane,                                                      //int32_t plane,
                                            blk_geom->bsize,                  //uint32_t puSize,
                                            context_ptr->cu_origin_x,
                                            context_ptr->cu_origin_y,
                                            col, //re-use these 2 for the PU offset
                                            row);
#ifdef DEBUG_REF_INFO
                                    {
                                        int originX = context_ptr->cu_origin_x;
                                        int originY = context_ptr->cu_origin_y;
                                        if (originX == 0 && originY == 448 && plane == 1)
                                        {
                                            printf("\nAbout to dump pred for (%d, %d) at plane %d, size %dx%d, pu offset (%d, %d)\n",
                                                    originX, originY, plane, txw, txh, col, row);
                                            dump_block_from_desc(txw, txh, reconBuffer, originX, originY, plane);
                                        }
                                    }
#endif
                                }

                                uint8_t cbQp = cu_ptr->qp;
                                //Av1EncodeLoopFunctionTable[is16bit](
                                Av1EncodeLoop(
#if ENCDEC_TX_SEARCH
                                    picture_control_set_ptr,
#endif
                                    context_ptr,
                                    sb_ptr,
                                    sb_origin_x,
                                    sb_origin_y,
                                    pu_block_origin_x,
                                    pu_block_origin_y,
                                    tx_size,
                                    cbQp,
                                    reconBuffer,
                                    coeff_buffer_sb,
                                    residual_buffer,
                                    transform_buffer,
                                    inverse_quant_buffer,
                                    transform_inner_array_ptr,
                                    asm_type,
                                    count_non_zero_coeffs,
                                    plane,
                                    useDeltaQpSegments,
                                    cu_ptr->delta_qp > 0 ? 0 : dZoffset,
                                    txb_itr[plane],
                                    eobs[txb_itr[plane]],
                                    cuPlane);

#ifdef DEBUG_REF_INFO
                                    {
                                        int originX = context_ptr->cu_origin_x;
                                        int originY = context_ptr->cu_origin_y;
                                        if (originX == 128 && originY == 320)// && plane > 0)
                                        {
                                            printf("\nAbout to dump coeff for (%d, %d) at plane %d, tx size %d\n",
                                                    originX, originY, plane, tx_size);
                                            dump_block_from_desc(txw, txh, coeff_buffer_sb, originX, originY, plane);
                                        }
                                    }
#endif

                                //intra mode
                                //Av1EncodeGenerateReconFunctionPtr[is16bit](
                                  Av1EncodeGenerateRecon(
                                    context_ptr,
                                    pu_block_origin_x,
                                    pu_block_origin_y,
                                    tx_size,
                                    reconBuffer,
                                    inverse_quant_buffer,
                                    transform_inner_array_ptr,
                                    plane,
                                    txb_itr[plane],
                                    eobs[txb_itr[plane]],
                                    asm_type);
#ifdef DEBUG_REF_INFO
                                    {
                                        int originX = pu_block_origin_x;
                                        int originY = pu_block_origin_y;
                                        if (originX == 0 && originY == 448 && plane == 1)
                                        {
                                            printf("\nAbout to dump recon for (%d, %d) at plane %d\n", originX, originY, plane);
                                            dump_block_from_desc(txw, txh, reconBuffer, originX, originY, plane);
                                        }
                                    }
#endif

                                // Update the Intra-specific Neighbor Arrays
                                    EncodePassUpdateIntraModeNeighborArrays(
                                      ep_mode_type_neighbor_array,
                                      ep_intra_mode_neighbor_array[plane],
                                      plane == 0 ? (uint8_t)cu_ptr->pred_mode : (uint8_t)pu_ptr->intra_chroma_mode,
                                      plane,
                                      pu_block_origin_x,
                                      pu_block_origin_y,
                                      txw,
                                      txh);

                                // Update Recon Samples-INTRA-
                                EncodePassUpdateReconSampleNeighborArrays(
                                        recon_neighbor_array[plane],
                                        reconBuffer,
                                        plane,
                                        pu_block_origin_x,
                                        pu_block_origin_y,
                                        txw,
                                        txh,
                                        is16bit);

                                // Jing: TODO: check here for 422/444 for entropy
                                if (plane == 0) {
                                    cu_ptr->block_has_coeff = cu_ptr->block_has_coeff |
                                        cu_ptr->transform_unit_array[txb_itr[plane]].y_has_coeff;
                                } else if (plane == 1 && cu_ptr->transform_unit_array[txb_itr[plane]].u_has_coeff) {
                                        //Jing: why here? comment first, may be useful for inter with 128x128
                                        //cu_ptr->transform_unit_array[0].u_has_coeff = EB_TRUE;
                                        cu_ptr->block_has_coeff = cu_ptr->block_has_coeff |
                                            cu_ptr->transform_unit_array[txb_itr[plane]].u_has_coeff;
                                } else if (plane == 2 && cu_ptr->transform_unit_array[txb_itr[plane]].v_has_coeff) {
                                        //cu_ptr->transform_unit_array[0].v_has_coeff = EB_TRUE;
                                        cu_ptr->block_has_coeff = cu_ptr->block_has_coeff |
                                            cu_ptr->transform_unit_array[txb_itr[plane]].v_has_coeff;
                                }
#else
#endif
                                context_ptr->coded_area_sb[plane] += txw * txh;
                                txb_itr[plane] += 1;
                            }
                        }// Txb Loop
                    } // Plane Loop

                    //printf("Finished CU (%d, %d), coded_area_sb is %d, coded_area_sb_uv %d\n",
                    //        context_ptr->cu_origin_x, context_ptr->cu_origin_y, context_ptr->coded_area_sb, context_ptr->coded_area_sb_uv);
                } else if (cu_ptr->prediction_mode_flag == INTER_MODE) {
#if 0
Jing: Disable it first

#if ENCDEC_TX_SEARCH
                    context_ptr->is_inter = 1;
#endif

                    EbReferenceObject_t* refObj0 = (EbReferenceObject_t*)picture_control_set_ptr->ref_pic_ptr_array[REF_LIST_0]->objectPtr;
                    EbReferenceObject_t* refObj1 = picture_control_set_ptr->slice_type == B_SLICE ?
                        (EbReferenceObject_t*)picture_control_set_ptr->ref_pic_ptr_array[REF_LIST_1]->objectPtr : 0;

                    uint16_t  txb_origin_x;
                    uint16_t  txb_origin_y;
                    EbBool isCuSkip = EB_FALSE;

                    //********************************
                    //        INTER
                    //********************************

                    EbBool  zeroLumaCbfMD = EB_FALSE;
                    //EbBool doLumaMC = EB_TRUE;
                    EbBool doMVpred = EB_TRUE;
                    //if QPM and Segments are used, First Cu in SB row should have at least one coeff.
                    EbBool isFirstCUinRow = (use_delta_qp == 1) &&
                        !oneSegment &&
                        (context_ptr->cu_origin_x == 0 && context_ptr->cu_origin_y == sb_origin_y) ? EB_TRUE : EB_FALSE;
                    zeroLumaCbfMD = (EbBool)(checkZeroLumaCbf && ((&cu_ptr->prediction_unit_array[0])->merge_flag == EB_FALSE && cu_ptr->block_has_coeff == 0 && isFirstCUinRow == EB_FALSE));
                    zeroLumaCbfMD = EB_FALSE;


                    //Motion Compensation could be avoided in the case below
                    EbBool doMC = EB_TRUE;

                    // Perform Merge/Skip Decision if the mode coming from MD is merge. for the First CU in Row merge will remain as is.
                    if (cu_ptr->prediction_unit_array[0].merge_flag == EB_TRUE)
                    {
                        if (isFirstCUinRow == EB_FALSE)
                        {
                            isCuSkip = mdcontextPtr->md_ep_pipe_sb[cu_ptr->mds_idx].skip_cost <= mdcontextPtr->md_ep_pipe_sb[cu_ptr->mds_idx].merge_cost ? 1 : 0;
                        }
                    }

                    //MC could be avoided in some cases below
                    if (isFirstCUinRow == EB_FALSE) {

                        if (picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag == EB_FALSE && constrained_intra_flag == EB_TRUE &&
                            cu_ptr->prediction_unit_array[0].merge_flag == EB_TRUE)
                        {
                            if (isCuSkip)
                            {
                                //here merge is decided to be skip in nonRef frame.
                                doMC = EB_FALSE;
                                doMVpred = EB_FALSE;
                            }
                        }
                        else if (picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag == EB_FALSE && constrained_intra_flag == EB_TRUE &&
                            zeroLumaCbfMD == EB_TRUE)
                        {
                            //MV mode with no Coeff  in nonRef frame.
                            doMC = EB_FALSE;
                        }

                        else if (picture_control_set_ptr->limit_intra && isIntraLCU == EB_FALSE)
                        {
                            if (isCuSkip)
                            {
                                doMC = EB_FALSE;
                                doMVpred = EB_FALSE;
                            }
                        }


                    }


                    doMC = (EbBool)(doRecon | doMC);

                    doMVpred = (EbBool)(doRecon | doMVpred);

                    //IntMv  predmv[2];
                    enc_pass_av1_mv_pred(
#if TILES
                        &sb_ptr->tile_info,
#endif
                         context_ptr->md_context,
                        cu_ptr,
                        blk_geom,
                        context_ptr->cu_origin_x,
                        context_ptr->cu_origin_y,
                        picture_control_set_ptr,
                        cu_ptr->prediction_unit_array[0].ref_frame_type,
                        cu_ptr->prediction_unit_array[0].is_compound,
                        cu_ptr->pred_mode,
                        cu_ptr->predmv);
                    //out1:  predmv
                    //out2:   cu_ptr->inter_mode_ctx[ cu_ptr->prediction_unit_array[0].ref_frame_type ]

                    //keep final usefull mvp for entropy
                    memcpy(cu_ptr->av1xd->final_ref_mv_stack,
                       context_ptr->md_context->md_local_cu_unit[context_ptr->blk_geom->blkidx_mds].ed_ref_mv_stack[cu_ptr->prediction_unit_array[0].ref_frame_type],
                        sizeof(CandidateMv)*MAX_REF_MV_STACK_SIZE);

                    {
                        // 1st Partition Loop
                        pu_ptr = cu_ptr->prediction_unit_array;

                        // Set MvUnit
                        context_ptr->mv_unit.predDirection = (uint8_t)pu_ptr->inter_pred_direction_index;
                        context_ptr->mv_unit.mv[REF_LIST_0].mvUnion = pu_ptr->mv[REF_LIST_0].mvUnion;
                        context_ptr->mv_unit.mv[REF_LIST_1].mvUnion = pu_ptr->mv[REF_LIST_1].mvUnion;

                        // Inter Prediction
                        EbBool local_warp_valid = EB_FALSE;
                        if (pu_ptr->motion_mode == WARPED_CAUSAL) {
                            local_warp_valid = warped_motion_parameters(
                                picture_control_set_ptr,
                                cu_ptr,
                                &context_ptr->mv_unit,
                                blk_geom,
                                context_ptr->cu_origin_x,
                                context_ptr->cu_origin_y,
                                &cu_ptr->prediction_unit_array[0].wm_params,
                                &cu_ptr->prediction_unit_array[0].num_proj_ref);

                            int32_t mi_row = context_ptr->cu_origin_y >> MI_SIZE_LOG2;
                            int32_t mi_col = context_ptr->cu_origin_x >> MI_SIZE_LOG2;

                            av1_count_overlappable_neighbors(
                                picture_control_set_ptr,
                                cu_ptr,
                                blk_geom->bsize,
                                mi_row,
                                mi_col);

                            const EbBool overlappable_candidates =
                                cu_ptr->prediction_unit_array[0].overlappable_neighbors[0]!=0 ||
                                cu_ptr->prediction_unit_array[0].overlappable_neighbors[1]!=0;

                            local_warp_valid = local_warp_valid && overlappable_candidates;

                            if (doMC) {
                                if (local_warp_valid) {
                                    if (is16bit) {
                                        warped_motion_prediction(
                                            &context_ptr->mv_unit,
                                            context_ptr->cu_origin_x,
                                            context_ptr->cu_origin_y,
                                            cu_ptr,
                                            blk_geom,
                                            refObj0->referencePicture16bit,
                                            reconBuffer,
                                            context_ptr->cu_origin_x,
                                            context_ptr->cu_origin_y,
                                            &cu_ptr->prediction_unit_array[0].wm_params,
                                            (uint8_t) sequence_control_set_ptr->static_config.encoder_bit_depth,
#if CHROMA_BLIND
                                            EB_TRUE,
#endif
                                            asm_type);
                                    } else {
                                        warped_motion_prediction(
                                            &context_ptr->mv_unit,
                                            context_ptr->cu_origin_x,
                                            context_ptr->cu_origin_y,
                                            cu_ptr,
                                            blk_geom,
                                            refObj0->referencePicture,
                                            reconBuffer,
                                            context_ptr->cu_origin_x,
                                            context_ptr->cu_origin_y,
                                            &cu_ptr->prediction_unit_array[0].wm_params,
                                            (uint8_t) sequence_control_set_ptr->static_config.encoder_bit_depth,
#if CHROMA_BLIND
                                            EB_TRUE,
#endif
                                            asm_type);
                                    }
                                } else
                                    pu_ptr->motion_mode = SIMPLE_TRANSLATION;
                            }
                        }

                        if (doMC &&
                            (pu_ptr->motion_mode != WARPED_CAUSAL ||
                            (pu_ptr->motion_mode == WARPED_CAUSAL && local_warp_valid == EB_FALSE)))
                        {
                            if (is16bit) {
                                av1_inter_prediction_hbd(
                                    picture_control_set_ptr,
                                    cu_ptr->prediction_unit_array->ref_frame_type,
                                    cu_ptr,
                                    &context_ptr->mv_unit,
                                    context_ptr->cu_origin_x,
                                    context_ptr->cu_origin_y,
                                    blk_geom->bwidth,
                                    blk_geom->bheight,
                                    refObj0->referencePicture16bit,
                                    picture_control_set_ptr->slice_type == B_SLICE ? refObj1->referencePicture16bit : 0,
                                    reconBuffer,
                                    context_ptr->cu_origin_x,
                                    context_ptr->cu_origin_y,
                                    (uint8_t)sequence_control_set_ptr->static_config.encoder_bit_depth,
                                    asm_type);
                            } else {
                                av1_inter_prediction(
                                    picture_control_set_ptr,
                                    cu_ptr->interp_filters,
                                    cu_ptr,
                                    cu_ptr->prediction_unit_array->ref_frame_type,
                                    &context_ptr->mv_unit,
                                    context_ptr->cu_origin_x,
                                    context_ptr->cu_origin_y,
                                    blk_geom->bwidth,
                                    blk_geom->bheight,
                                    refObj0->referencePicture,
                                    picture_control_set_ptr->slice_type == B_SLICE ? refObj1->referencePicture : 0,
                                    reconBuffer,
                                    context_ptr->cu_origin_x,
                                    context_ptr->cu_origin_y,
#if CHROMA_BLIND
                                    EB_TRUE,
#endif
                                    asm_type);
                            }
                        }
                    }

                    context_ptr->txb_itr = 0;
                    // Transform Loop
                    cu_ptr->transform_unit_array[0].y_has_coeff = EB_FALSE;
                    cu_ptr->transform_unit_array[0].u_has_coeff = EB_FALSE;
                    cu_ptr->transform_unit_array[0].v_has_coeff = EB_FALSE;

                    // initialize TU Split
                    y_full_distortion[DIST_CALC_RESIDUAL] = 0;
                    y_full_distortion[DIST_CALC_PREDICTION] = 0;

                    y_coeff_bits = 0;
                    cb_coeff_bits = 0;
                    cr_coeff_bits = 0;


                    uint32_t totTu = context_ptr->blk_geom->txb_count[0];
                    uint8_t   tuIt;
                    uint8_t    cbQp = cu_ptr->qp;
                    uint32_t  component_mask = context_ptr->blk_geom->has_uv_ex ? PICTURE_BUFFER_DESC_FULL_MASK : PICTURE_BUFFER_DESC_LUMA_MASK;

                    if (cu_ptr->prediction_unit_array[0].merge_flag == EB_FALSE) {

                        for (tuIt = 0; tuIt < totTu; tuIt++) {
                            context_ptr->txb_itr = tuIt;
                            txb_origin_x = context_ptr->cu_origin_x + context_ptr->blk_geom->tx_boff_x[tuIt];
                            txb_origin_y = context_ptr->cu_origin_y + context_ptr->blk_geom->tx_boff_y[tuIt];
                            if (!zeroLumaCbfMD)
                                //inter mode  1
                                Av1EncodeLoopFunctionTable[is16bit](
#if ENCDEC_TX_SEARCH
                                    picture_control_set_ptr,
#endif
                                    context_ptr,
                                    sb_ptr,
                                    txb_origin_x,   //pic org
                                    txb_origin_y,
                                    cbQp,
                                    reconBuffer,
                                    coeff_buffer_sb,
                                    residual_buffer,
                                    transform_buffer,
                                    inverse_quant_buffer,
                                    transform_inner_array_ptr,
                                    asm_type,
                                    count_non_zero_coeffs,
                                    context_ptr->blk_geom->has_uv_ex ? PICTURE_BUFFER_DESC_FULL_MASK : PICTURE_BUFFER_DESC_LUMA_MASK,
                                    useDeltaQpSegments,
                                    cu_ptr->delta_qp > 0 ? 0 : dZoffset,
                                    eobs[context_ptr->txb_itr],
                                    cuPlane);

                            // SKIP the CBF zero mode for DC path. There are problems with cost calculations
                            if (context_ptr->trans_coeff_shape_luma != ONLY_DC_SHAPE) {
                                // Compute Tu distortion
                                if (!zeroLumaCbfMD)

                                    // LUMA DISTORTION
                                    PictureFullDistortion32Bits(
                                        transform_buffer,
                                        context_ptr->coded_area_sb,
                                        0,
                                        inverse_quant_buffer,
                                        context_ptr->coded_area_sb,
                                        0,
                                        blk_geom->tx_width[tuIt],
                                        blk_geom->tx_height[tuIt],
                                        context_ptr->blk_geom->bwidth_uv,
                                        context_ptr->blk_geom->bheight_uv,
                                        yTuFullDistortion,
                                        yTuFullDistortion,
                                        yTuFullDistortion,
                                        eobs[context_ptr->txb_itr][0],
                                        0,
                                        0,
                                        COMPONENT_LUMA,
                                        asm_type);
                                TxSize  txSize = blk_geom->txsize[context_ptr->txb_itr];
                                int32_t shift = (MAX_TX_SCALE - av1_get_tx_scale(txSize)) * 2;
                                yTuFullDistortion[DIST_CALC_RESIDUAL] = RIGHT_SIGNED_SHIFT(yTuFullDistortion[DIST_CALC_RESIDUAL], shift);
                                yTuFullDistortion[DIST_CALC_PREDICTION] = RIGHT_SIGNED_SHIFT(yTuFullDistortion[DIST_CALC_PREDICTION], shift);

                                yTuCoeffBits = 0;
                                cbTuCoeffBits = 0;
                                crTuCoeffBits = 0;

                                if (!zeroLumaCbfMD) {

                                    ModeDecisionCandidateBuffer_t         **candidateBufferPtrArrayBase = context_ptr->md_context->candidate_buffer_ptr_array;
                                    ModeDecisionCandidateBuffer_t         **candidate_buffer_ptr_array = &(candidateBufferPtrArrayBase[context_ptr->md_context->buffer_depth_index_start[0]]);
                                    ModeDecisionCandidateBuffer_t          *candidateBuffer;

                                    // Set the Candidate Buffer
                                    candidateBuffer = candidate_buffer_ptr_array[0];
                                    // Rate estimation function uses the values from CandidatePtr. The right values are copied from cu_ptr to CandidatePtr
                                    candidateBuffer->candidate_ptr->transform_type[PLANE_TYPE_Y] = cu_ptr->transform_unit_array[tuIt].transform_type[PLANE_TYPE_Y];
                                    candidateBuffer->candidate_ptr->transform_type[PLANE_TYPE_UV] = cu_ptr->transform_unit_array[tuIt].transform_type[PLANE_TYPE_UV];
                                    candidateBuffer->candidate_ptr->type = cu_ptr->prediction_mode_flag;

                                    const uint32_t coeff1dOffset = context_ptr->coded_area_sb;

                                    Av1TuEstimateCoeffBits(
                                        picture_control_set_ptr,
                                        candidateBuffer,
                                        cu_ptr,
                                        coeff1dOffset,
                                        context_ptr->coded_area_sb_uv,
                                        coeff_est_entropy_coder_ptr,
                                        coeff_buffer_sb,
                                        eobs[context_ptr->txb_itr][0],
                                        eobs[context_ptr->txb_itr][1],
                                        eobs[context_ptr->txb_itr][2],
                                        &yTuCoeffBits,
                                        &cbTuCoeffBits,
                                        &crTuCoeffBits,
                                        context_ptr->blk_geom->txsize[context_ptr->txb_itr],
                                        context_ptr->blk_geom->txsize_uv[context_ptr->txb_itr],
                                        context_ptr->blk_geom->has_uv_ex ? COMPONENT_ALL : COMPONENT_LUMA,
                                        asm_type);
                                }

                                // CBF Tu decision
                                if (zeroLumaCbfMD == EB_FALSE)

                                    Av1EncodeTuCalcCost(
                                        context_ptr,
                                        count_non_zero_coeffs,
                                        yTuFullDistortion,
                                        &yTuCoeffBits,
                                        component_mask);

                                else {
                                    cu_ptr->transform_unit_array[context_ptr->txb_itr].y_has_coeff = 0;
                                    cu_ptr->transform_unit_array[context_ptr->txb_itr].u_has_coeff = 0;
                                    cu_ptr->transform_unit_array[context_ptr->txb_itr].v_has_coeff = 0;
                                }
                                // Update count_non_zero_coeffs after CBF decision
                                if (cu_ptr->transform_unit_array[context_ptr->txb_itr].y_has_coeff == EB_FALSE)
                                    count_non_zero_coeffs[0] = 0;
                                if (context_ptr->blk_geom->has_uv_ex) {
                                    if (cu_ptr->transform_unit_array[context_ptr->txb_itr].u_has_coeff == EB_FALSE)
                                        count_non_zero_coeffs[1] = 0;
                                    if (cu_ptr->transform_unit_array[context_ptr->txb_itr].v_has_coeff == EB_FALSE)
                                        count_non_zero_coeffs[2] = 0;
                                }

                                // Update TU count_non_zero_coeffs
                                cu_ptr->transform_unit_array[context_ptr->txb_itr].nz_coef_count[0] = (uint16_t)count_non_zero_coeffs[0];
                                cu_ptr->transform_unit_array[context_ptr->txb_itr].nz_coef_count[1] = (uint16_t)count_non_zero_coeffs[1];
                                cu_ptr->transform_unit_array[context_ptr->txb_itr].nz_coef_count[2] = (uint16_t)count_non_zero_coeffs[2];

                                y_coeff_bits += yTuCoeffBits;

                                if (context_ptr->blk_geom->has_uv_ex) {
                                    cb_coeff_bits += cbTuCoeffBits;
                                    cr_coeff_bits += crTuCoeffBits;
                                }

                                y_full_distortion[DIST_CALC_RESIDUAL] += yTuFullDistortion[DIST_CALC_RESIDUAL];
                                y_full_distortion[DIST_CALC_PREDICTION] += yTuFullDistortion[DIST_CALC_PREDICTION];

                            }
                            context_ptr->coded_area_sb += blk_geom->tx_width[tuIt] * blk_geom->tx_height[tuIt];
                            if (blk_geom->has_uv_ex)
                                context_ptr->coded_area_sb_uv += blk_geom->tx_width_uv[tuIt] * blk_geom->tx_height_uv[tuIt];

                        } // Transform Loop

                    }

                    //Set Final CU data flags after skip/Merge decision.
                    if (isFirstCUinRow == EB_FALSE) {

                        if (cu_ptr->prediction_unit_array[0].merge_flag == EB_TRUE) {

                            cu_ptr->skip_flag = (isCuSkip) ? EB_TRUE : EB_FALSE;
                            cu_ptr->prediction_unit_array[0].merge_flag = (isCuSkip) ? EB_FALSE : EB_TRUE;

                        }
                    }


                    // Initialize the Transform Loop

                    context_ptr->txb_itr = 0;
                    y_has_coeff = 0;
                    u_has_coeff = 0;
                    v_has_coeff = 0;
                    totTu = context_ptr->blk_geom->txb_count[0];


                    //reset coeff buffer offsets at the start of a new Tx loop
                    context_ptr->coded_area_sb = coded_area_org;
                    context_ptr->coded_area_sb_uv = coded_area_org_uv;
                    for (tuIt = 0; tuIt < totTu; tuIt++)
                    {
                        context_ptr->txb_itr = tuIt;
                        txb_origin_x = context_ptr->cu_origin_x + context_ptr->blk_geom->tx_boff_x[tuIt];
                        txb_origin_y = context_ptr->cu_origin_y + context_ptr->blk_geom->tx_boff_y[tuIt];
                        if (cu_ptr->skip_flag == EB_TRUE) {
                            cu_ptr->transform_unit_array[context_ptr->txb_itr].y_has_coeff = EB_FALSE;
                            cu_ptr->transform_unit_array[context_ptr->txb_itr].u_has_coeff = EB_FALSE;
                            cu_ptr->transform_unit_array[context_ptr->txb_itr].v_has_coeff = EB_FALSE;


                        }
                        else if ((&cu_ptr->prediction_unit_array[0])->merge_flag == EB_TRUE) {

                            //inter mode  2

                            Av1EncodeLoopFunctionTable[is16bit](
#if ENCDEC_TX_SEARCH
                                picture_control_set_ptr,
#endif
                                context_ptr,
                                sb_ptr,
                                txb_origin_x, //pic offset
                                txb_origin_y,
                                cbQp,
                                reconBuffer,
                                coeff_buffer_sb,
                                residual_buffer,
                                transform_buffer,
                                inverse_quant_buffer,
                                transform_inner_array_ptr,
                                asm_type,
                                count_non_zero_coeffs,
                                context_ptr->blk_geom->has_uv_ex ? PICTURE_BUFFER_DESC_FULL_MASK : PICTURE_BUFFER_DESC_LUMA_MASK,
                                useDeltaQpSegments,
                                cu_ptr->delta_qp > 0 ? 0 : dZoffset,
                                eobs[context_ptr->txb_itr],
                                cuPlane);



                        }

                        if (context_ptr->blk_geom->has_uv_ex) {
                            cu_ptr->block_has_coeff = cu_ptr->block_has_coeff |
                                cu_ptr->transform_unit_array[context_ptr->txb_itr].y_has_coeff |
                                cu_ptr->transform_unit_array[context_ptr->txb_itr].u_has_coeff |
                                cu_ptr->transform_unit_array[context_ptr->txb_itr].v_has_coeff;

                        }
                        else {
                            cu_ptr->block_has_coeff = cu_ptr->block_has_coeff |
                                cu_ptr->transform_unit_array[context_ptr->txb_itr].y_has_coeff;
                        }

                        //inter mode
                        if (doRecon)

                            Av1EncodeGenerateReconFunctionPtr[is16bit](
                                context_ptr,
                                txb_origin_x,  //pic offset
                                txb_origin_y,
                                reconBuffer,
                                inverse_quant_buffer,
                                transform_inner_array_ptr,
                                context_ptr->blk_geom->has_uv_ex ? PICTURE_BUFFER_DESC_FULL_MASK : PICTURE_BUFFER_DESC_LUMA_MASK,
                                eobs[context_ptr->txb_itr],
                                asm_type);
                        if (context_ptr->blk_geom->has_uv_ex) {
                            y_has_coeff |= cu_ptr->transform_unit_array[context_ptr->txb_itr].y_has_coeff;
                            u_has_coeff |= cu_ptr->transform_unit_array[context_ptr->txb_itr].u_has_coeff;
                            v_has_coeff |= cu_ptr->transform_unit_array[context_ptr->txb_itr].v_has_coeff;
                        }
                        else {
                            y_has_coeff |= cu_ptr->transform_unit_array[context_ptr->txb_itr].y_has_coeff;
                        }


                        context_ptr->coded_area_sb += blk_geom->tx_width[tuIt] * blk_geom->tx_height[tuIt];
                        if (blk_geom->has_uv_ex)
                            context_ptr->coded_area_sb_uv += blk_geom->tx_width_uv[tuIt] * blk_geom->tx_height_uv[tuIt];


                    } // Transform Loop

                    // Calculate Root CBF
                    if (context_ptr->blk_geom->has_uv_ex)
                        cu_ptr->block_has_coeff = (y_has_coeff | u_has_coeff | v_has_coeff) ? EB_TRUE : EB_FALSE;
                    else
                        cu_ptr->block_has_coeff = (y_has_coeff) ? EB_TRUE : EB_FALSE;


                    // Force Skip if MergeFlag == TRUE && RootCbf == 0

                    if (cu_ptr->skip_flag == EB_FALSE &&
                        cu_ptr->prediction_unit_array[0].merge_flag == EB_TRUE && cu_ptr->block_has_coeff == EB_FALSE)
                    {
                        cu_ptr->skip_flag = EB_TRUE;
                    }

                    {
                        // Set the PU Loop Variables
                        pu_ptr = cu_ptr->prediction_unit_array;

                        // Set MvUnit
                        context_ptr->mv_unit.predDirection = (uint8_t)pu_ptr->inter_pred_direction_index;
                        context_ptr->mv_unit.mv[REF_LIST_0].mvUnion = pu_ptr->mv[REF_LIST_0].mvUnion;
                        context_ptr->mv_unit.mv[REF_LIST_1].mvUnion = pu_ptr->mv[REF_LIST_1].mvUnion;

                        // Update Neighbor Arrays (Mode Type, MVs, SKIP)
                        {
                            uint8_t skip_flag = (uint8_t)cu_ptr->skip_flag;
                            EncodePassUpdateInterModeNeighborArrays(
                                ep_mode_type_neighbor_array,
                                ep_mv_neighbor_array,
                                ep_skip_flag_neighbor_array,
                                &context_ptr->mv_unit,
                                &skip_flag,
                                context_ptr->cu_origin_x,
                                context_ptr->cu_origin_y,
                                blk_geom->bwidth,
                                blk_geom->bheight);

                        }

                    } // 2nd Partition Loop


                    // Update Recon Samples Neighbor Arrays -INTER-

                    if (doRecon)
                        EncodePassUpdateReconSampleNeighborArrays(
                            ep_luma_recon_neighbor_array,
                            ep_cb_recon_neighbor_array,
                            ep_cr_recon_neighbor_array,
                            reconBuffer,
                            context_ptr->cu_origin_x,
                            context_ptr->cu_origin_y,
                            context_ptr->blk_geom->bwidth,
                            context_ptr->blk_geom->bheight,
                            context_ptr->blk_geom->bwidth_uv,
                            context_ptr->blk_geom->bheight_uv,
                            context_ptr->blk_geom->has_uv_ex ? PICTURE_BUFFER_DESC_FULL_MASK : PICTURE_BUFFER_DESC_LUMA_MASK,
                            is16bit);
#endif
                }
                else {
                    CHECK_REPORT_ERROR_NC(
                        encode_context_ptr->app_callback_ptr,
                        EB_ENC_CL_ERROR2);
                }

                update_av1_mi_map(
                    cu_ptr,
                    context_ptr->cu_origin_x,
                    context_ptr->cu_origin_y,
                    blk_geom,
                    picture_control_set_ptr);

                if (dlfEnableFlag)
                {

                    assert(0);
                    if (blk_geom->has_uv_ex) {
                        availableCoeff = (cu_ptr->prediction_mode_flag == INTER_MODE) ? (EbBool)cu_ptr->block_has_coeff :
                            (cu_ptr->transform_unit_array[0].y_has_coeff ||
                                cu_ptr->transform_unit_array[0].v_has_coeff ||
                                cu_ptr->transform_unit_array[0].u_has_coeff) ? EB_TRUE : EB_FALSE;
                    }
                    else {
                        availableCoeff = (cu_ptr->transform_unit_array[0].y_has_coeff) ? EB_TRUE : EB_FALSE;
                    }


                    // Assign the LCU-level QP
                    //NM - To be revisited
                    EncodePassUpdateQp(
                        picture_control_set_ptr,
                        context_ptr,
                        availableCoeff,
                        use_delta_qp,
                        &isDeltaQpNotCoded,
                        picture_control_set_ptr->dif_cu_delta_qp_depth,
                        &(picture_control_set_ptr->enc_prev_coded_qp[oneSegment ? 0 : lcuRowIndex]),
                        &(picture_control_set_ptr->enc_prev_quant_group_coded_qp[oneSegment ? 0 : lcuRowIndex]),
                        sb_qp);

                }

                {
                    {
                        // Set the PU Loop Variables
                        pu_ptr = cu_ptr->prediction_unit_array;
                        // Set MvUnit
                        context_ptr->mv_unit.predDirection = (uint8_t)pu_ptr->inter_pred_direction_index;
                        context_ptr->mv_unit.mv[REF_LIST_0].mvUnion = pu_ptr->mv[REF_LIST_0].mvUnion;
                        context_ptr->mv_unit.mv[REF_LIST_1].mvUnion = pu_ptr->mv[REF_LIST_1].mvUnion;
                    }

                }


                {

                    CodingUnit_t *src_cu = &context_ptr->md_context->md_cu_arr_nsq[d1_itr];

                    CodingUnit_t *dst_cu = &sb_ptr->final_cu_arr[final_cu_itr++];

                    move_cu_data(src_cu, dst_cu);
                }
            }
            blk_it += ns_depth_offset[sequence_control_set_ptr->sb_size == BLOCK_128X128][context_ptr->blk_geom->depth];
        }
        else {
            blk_it += d1_depth_offset[sequence_control_set_ptr->sb_size == BLOCK_128X128][context_ptr->blk_geom->depth];
        }
    } // CU Loop

    sb_ptr->tot_final_cu = final_cu_itr;

#ifdef DEBUG_REF_INFO
    static int sb_index = 0;
    if (sb_index == sequence_control_set_ptr->sb_tot_cnt - 1) {
        //dump_buf_desc_to_file(reconBuffer, "internal_recon.yuv", 0);
    } else {
        sb_index++;
    }
#endif
#if AV1_LF
    // First Pass Deblocking
    if (dlfEnableFlag && picture_control_set_ptr->parent_pcs_ptr->loop_filter_mode == 1) {
        assert(0);
        if (picture_control_set_ptr->parent_pcs_ptr->lf.filter_level[0] || picture_control_set_ptr->parent_pcs_ptr->lf.filter_level[1]) {
            uint8_t LastCol = ((sb_origin_x)+sb_width == sequence_control_set_ptr->luma_width) ? 1 : 0;
            loop_filter_sb(
                reconBuffer,
                picture_control_set_ptr,
                NULL,
                sb_origin_y >> 2,
                sb_origin_x >> 2,
                0,
                3,
                LastCol);
        }
    }
#endif

    return;
}

#if NO_ENCDEC
EB_EXTERN void no_enc_dec_pass(
    SequenceControlSet_t    *sequence_control_set_ptr,
    PictureControlSet_t     *picture_control_set_ptr,
    LargestCodingUnit_t     *sb_ptr,
    uint32_t                   tbAddr,
    uint32_t                   sb_origin_x,
    uint32_t                   sb_origin_y,
    uint32_t                   sb_qp,
    EncDecContext_t         *context_ptr)
{

    context_ptr->coded_area_sb = 0;
    context_ptr->coded_area_sb_uv = 0;

    uint32_t      final_cu_itr = 0;


    uint32_t    blk_it = 0;

    while (blk_it < sequence_control_set_ptr->max_block_cnt) {


        CodingUnit_t  *cu_ptr = context_ptr->cu_ptr = &context_ptr->md_context->md_cu_arr_nsq[blk_it];
        PartitionType part = cu_ptr->part;
        const BlockGeom * blk_geom = context_ptr->blk_geom = Get_blk_geom_mds(blk_it);


        sb_ptr->cu_partition_array[blk_it] = context_ptr->md_context->md_cu_arr_nsq[blk_it].part;

        if (part != PARTITION_SPLIT) {



            int32_t offset_d1 = ns_blk_offset[(int32_t)part]; //cu_ptr->best_d1_blk; // TOCKECK
            int32_t num_d1_block = ns_blk_num[(int32_t)part]; // context_ptr->blk_geom->totns; // TOCKECK

            for (int32_t d1_itr = blk_it + offset_d1; d1_itr < blk_it + offset_d1 + num_d1_block; d1_itr++) {

                const BlockGeom * blk_geom = context_ptr->blk_geom = Get_blk_geom_mds(d1_itr);
                CodingUnit_t            *cu_ptr = context_ptr->cu_ptr = &context_ptr->md_context->md_cu_arr_nsq[d1_itr];


                cu_ptr->delta_qp = 0;
                cu_ptr->qp = (sequence_control_set_ptr->static_config.improve_sharpness) ? context_ptr->qpmQp : picture_control_set_ptr->picture_qp;
                sb_ptr->qp = (sequence_control_set_ptr->static_config.improve_sharpness) ? context_ptr->qpmQp : picture_control_set_ptr->picture_qp;
                cu_ptr->org_delta_qp = cu_ptr->delta_qp;


                {
                    CodingUnit_t *src_cu = &context_ptr->md_context->md_cu_arr_nsq[d1_itr];
                    CodingUnit_t *dst_cu = &sb_ptr->final_cu_arr[final_cu_itr++];

                    move_cu_data(src_cu, dst_cu);
                }


                //copy coeff
                int32_t txb_1d_offset = 0, txb_1d_offset_uv = 0;

                int32_t txb_itr = 0;
                do
                {

                    uint32_t  bwidth = context_ptr->blk_geom->tx_width[txb_itr] < 64 ? context_ptr->blk_geom->tx_width[txb_itr] : 32;
                    uint32_t  bheight = context_ptr->blk_geom->tx_height[txb_itr] < 64 ? context_ptr->blk_geom->tx_height[txb_itr] : 32;

                    int32_t* srcPtr = &(((int32_t*)context_ptr->cu_ptr->coeff_tmp->bufferY)[txb_1d_offset]);
                    int32_t* dstPtr = &(((int32_t*)sb_ptr->quantized_coeff->bufferY)[context_ptr->coded_area_sb]);

                    uint32_t j;
                    for (j = 0; j < bheight; j++)
                    {
                        memcpy(dstPtr + j * bwidth, srcPtr + j * bwidth, bwidth * sizeof(int32_t));
                    }

                    if (context_ptr->blk_geom->has_uv_ex)
                    {
                        // Cb
                        bwidth = context_ptr->blk_geom->tx_width_uv[txb_itr];
                        bheight = context_ptr->blk_geom->tx_height_uv[txb_itr];

                        srcPtr = &(((int32_t*)context_ptr->cu_ptr->coeff_tmp->bufferCb)[txb_1d_offset_uv]);
                        dstPtr = &(((int32_t*)sb_ptr->quantized_coeff->bufferCb)[context_ptr->coded_area_sb_uv]);

                        for (j = 0; j < bheight; j++)
                        {
                            memcpy(dstPtr + j * bwidth, srcPtr + j * bwidth, bwidth * sizeof(int32_t));
                        }

                        //Cr
                        srcPtr = &(((int32_t*)context_ptr->cu_ptr->coeff_tmp->bufferCr)[txb_1d_offset_uv]);
                        dstPtr = &(((int32_t*)sb_ptr->quantized_coeff->bufferCr)[context_ptr->coded_area_sb_uv]);

                        for (j = 0; j < bheight; j++)
                        {
                            memcpy(dstPtr + j * bwidth, srcPtr + j * bwidth, bwidth * sizeof(int32_t));
                        }

                    }

                    context_ptr->coded_area_sb += context_ptr->blk_geom->tx_width[txb_itr] * context_ptr->blk_geom->tx_height[txb_itr];
                    if (context_ptr->blk_geom->has_uv_ex)
                        context_ptr->coded_area_sb_uv += context_ptr->blk_geom->tx_width_uv[txb_itr] * context_ptr->blk_geom->tx_height_uv[txb_itr];

                    txb_1d_offset += context_ptr->blk_geom->tx_width[txb_itr] * context_ptr->blk_geom->tx_height[txb_itr];
                    if (context_ptr->blk_geom->has_uv_ex)
                        txb_1d_offset_uv += context_ptr->blk_geom->tx_width_uv[txb_itr] * context_ptr->blk_geom->tx_height_uv[txb_itr];

                    txb_itr++;

                } while (txb_itr < context_ptr->blk_geom->txb_count[0]);






                //copy recon
                {
                    EbPictureBufferDesc_t          *ref_pic;
                    if (picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag)
                    {
                        EbReferenceObject_t* refObj = (EbReferenceObject_t*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->objectPtr;
                        ref_pic = refObj->referencePicture;
                    }
                    else
                    {
                        ref_pic = picture_control_set_ptr->recon_picture_ptr;
                    }

                    context_ptr->cu_origin_x = sb_origin_x + context_ptr->blk_geom->origin_x;
                    context_ptr->cu_origin_y = sb_origin_y + context_ptr->blk_geom->origin_y;

                    uint32_t  bwidth = context_ptr->blk_geom->bwidth;
                    uint32_t  bheight = context_ptr->blk_geom->bheight;

                    uint8_t* srcPtr = &(((uint8_t*)context_ptr->cu_ptr->recon_tmp->bufferY)[0]);
                    uint8_t* dstPtr = ref_pic->bufferY + ref_pic->origin_x + context_ptr->cu_origin_x + (ref_pic->origin_y + context_ptr->cu_origin_y)*ref_pic->strideY;

                    uint32_t j;
                    for (j = 0; j < bheight; j++)
                    {
                        memcpy(dstPtr + j * ref_pic->strideY, srcPtr + j * 128, bwidth * sizeof(uint8_t));
                    }

                    if (context_ptr->blk_geom->has_uv_ex)
                    {

                        bwidth = context_ptr->blk_geom->bwidth_uv;
                        bheight = context_ptr->blk_geom->bheight_uv;

                        srcPtr = &(((uint8_t*)context_ptr->cu_ptr->recon_tmp->bufferCb)[0]);

                        dstPtr = ref_pic->bufferCb + ref_pic->origin_x / 2 + ((context_ptr->cu_origin_x >> 3) << 3) / 2 + (ref_pic->origin_y / 2 + ((context_ptr->cu_origin_y >> 3) << 3) / 2)*ref_pic->strideCb;

                        for (j = 0; j < bheight; j++)
                        {
                            memcpy(dstPtr + j * ref_pic->strideCb, srcPtr + j * 64, bwidth * sizeof(uint8_t));
                        }

                        srcPtr = &(((uint8_t*)context_ptr->cu_ptr->recon_tmp->bufferCr)[0]);

                        dstPtr = ref_pic->bufferCr + ref_pic->origin_x / 2 + ((context_ptr->cu_origin_x >> 3) << 3) / 2 + (ref_pic->origin_y / 2 + ((context_ptr->cu_origin_y >> 3) << 3) / 2)*ref_pic->strideCr;


                        for (j = 0; j < bheight; j++)
                        {
                            memcpy(dstPtr + j * ref_pic->strideCr, srcPtr + j * 64, bwidth * sizeof(uint8_t));
                        }

                    }

                }



            }
            blk_it += ns_depth_offset[sequence_control_set_ptr->sb_size == BLOCK_128X128][context_ptr->blk_geom->depth];
        }
        else
        {
            blk_it += d1_depth_offset[sequence_control_set_ptr->sb_size == BLOCK_128X128][context_ptr->blk_geom->depth];
        }

    } // CU Loop



    return;
}
#endif
