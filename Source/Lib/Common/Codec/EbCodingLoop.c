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
#include "EbSvtAv1ErrorCodes.h"
#include "EbTransforms.h"
#include "EbModeDecisionConfiguration.h"
#include "EbIntraPrediction.h"
#include "aom_dsp_rtcd.h"
#include "EbCodingLoop.h"

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


    unsigned char* luma_ptr = reconBuffer->buffer_y +
        ((reconBuffer->stride_y * (reconBuffer->origin_y) + reconBuffer->origin_x) * unitSize);
    unsigned char* cb_ptr = reconBuffer->bufferCb +
        ((reconBuffer->strideCb * (reconBuffer->origin_y >> subHeightCMinus1) + (reconBuffer->origin_x>>subWidthCMinus1)) * unitSize);
    unsigned char* cr_ptr = reconBuffer->bufferCr +
        ((reconBuffer->strideCr * (reconBuffer->origin_y >> subHeightCMinus1) + (reconBuffer->origin_x>>subWidthCMinus1)) * unitSize);

    for (int i=0; i<reconBuffer->height; i++) {
        fwrite(luma_ptr, 1, reconBuffer->width * unitSize, fp);
        luma_ptr += reconBuffer->stride_y * unitSize;
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
        buf=buf_tmp->buffer_y;
        stride=buf_tmp->stride_y;
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
                        printf("%3u ", start_tmp[j]);
                    } else if (bitDepth == 10 || bitDepth == 16) {
                        printf("%3d ", *((int16_t*)start_tmp + j));
                    } else if (bitDepth == 32) {
                        printf("%3d ", *((int32_t*)start_tmp + j));
                    } else {
                        printf("bitDepth is %d\n", bitDepth);
                        assert(0);
                    }
                }
                printf("\n");
                if (i % 4 == 3) {
                    for (int k=0;k<txw;k++) {
                        printf("-");
                    }
                    printf("\n");
                }
                        
                start_tmp += stride*val;
            }
    printf("------------------------\n");
}

static void dump_coeff_block_from_desc(int txw, int txh, EbPictureBufferDesc_t *buf_tmp, int startX, int startY, int componentMask, int offset)
{
    int32_t* buf=NULL;
    int stride=0;
    int bitDepth = buf_tmp->bit_depth;
    EbColorFormat colorFormat = buf_tmp->color_format;    // Chroma format
    uint16_t subWidthCMinus1 = (colorFormat==EB_YUV444?1:2)-1;
    uint16_t subHeightCMinus1 = (colorFormat>=EB_YUV422?1:2)-1;
    if (componentMask ==0) {
        buf=(int32_t *)buf_tmp->buffer_y;
        stride=buf_tmp->stride_y;
        subWidthCMinus1=0;
        subHeightCMinus1=0;
    } else if (componentMask == 1) {
        buf=(int32_t *)buf_tmp->bufferCb;
        stride=buf_tmp->strideCb;
        startX=ROUND_UV_EX(startX, subWidthCMinus1);
        startY=ROUND_UV_EX(startY, subHeightCMinus1);
    } else if (componentMask == 2) {
        buf=(int32_t *)buf_tmp->bufferCr;
        stride=buf_tmp->strideCr;
        startX=ROUND_UV_EX(startX, subWidthCMinus1);
        startY=ROUND_UV_EX(startY, subHeightCMinus1);
    } else {
        assert(0);
    }

    printf("dump coeff block size %dx%d at offset %d, (%d, %d), component is %s\n",
            txw, txh, offset, startX, startY, componentMask==0?"luma":(componentMask==1?"Cb":"Cr"));
            int32_t* start_tmp=buf+offset;
            for (int i=0;i<txh;i++) {
                for (int j=0;j<txw;j++) {
                    if (j%4 == 0) {
                        printf("|");
                    }

                    printf("%3d ", start_tmp[j]);
                }
                printf("\n");
                if (i % 4 == 3) {
                    for (int k=0;k<txw;k++) {
                        printf("-");
                    }
                    printf("\n");
                }
                start_tmp += txw;
            }
    printf("------------------------\n");
}
#endif

extern void av1_predict_intra_block(
    TileInfo                    *tile,

    STAGE                       stage,
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
    EbPictureBufferDesc_t  *recon_buffer,
    int32_t col_off,
    int32_t row_off,
    int32_t plane,
    block_size bsize,
    uint32_t bl_org_x_pict,
    uint32_t bl_org_y_pict,
    uint32_t bl_org_x_mb,
    uint32_t bl_org_y_mb);

void av1_predict_intra_block_16bit(
    TileInfo               *tile,

    EncDecContext_t         *context_ptr,
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
    EbPictureBufferDesc_t  *recon_buffer,
    int32_t col_off,
    int32_t row_off,
    int32_t plane,
    block_size bsize,
    uint32_t bl_org_x_pict,
    uint32_t bl_org_y_pict);


/*******************************************
* set Penalize Skip Flag
*
* Summary: Set the penalize_skipflag to true
* When there is luminance/chrominance change
* or in noisy clip with low motion at meduim
* varince area
*
*******************************************/

#define S32 32*32
#define S16 16*16
#define S8  8*8
#define S4  4*4

typedef void(*EB_AV1_ENCODE_LOOP_FUNC_PTR)(
#if ENCDEC_TX_SEARCH
    PictureControlSet_t    *picture_control_set_ptr,
#endif
    EncDecContext_t       *context_ptr,
    LargestCodingUnit_t   *sb_ptr,
    uint32_t                 origin_x,
    uint32_t                 origin_y,
    uint32_t                 cbQp,
    EbPictureBufferDesc_t *predSamples,             // no basis/offset
    EbPictureBufferDesc_t *coeffSamplesTB,          // lcu based
    EbPictureBufferDesc_t *residual16bit,           // no basis/offset
    EbPictureBufferDesc_t *transform16bit,          // no basis/offset
    EbPictureBufferDesc_t *inverse_quant_buffer,
    int16_t                *transformScratchBuffer,
    EbAsm                 asm_type,
    uint32_t                  *count_non_zero_coeffs,
    uint32_t                 component_mask,
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
    uint32_t                 component_mask,
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
    uint32_t                      luma_mode,
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
        neighbor_array_unit_mode_write(
            mode_type_neighbor_array,
            &modeType,
            pu_origin_x,
            pu_origin_y,
            tx_width,
            tx_height,
            NEIGHBOR_ARRAY_UNIT_FULL_MASK);
    }

    // Intra Mode Update
    neighbor_array_unit_mode_write(
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
    uint8_t                   *skip_flag,
    uint32_t                   origin_x,
    uint32_t                   origin_y,
    uint32_t                   bwidth,
    uint32_t                   bheight)
{
    uint8_t modeType = INTER_MODE;

    // Mode Type Update
    neighbor_array_unit_mode_write(
        mode_type_neighbor_array,
        &modeType,
        origin_x,
        origin_y,
        bwidth,
        bheight,
        NEIGHBOR_ARRAY_UNIT_FULL_MASK);

    // Motion Vector Unit
    neighbor_array_unit_mode_write(
        mv_neighbor_array,
        (uint8_t*)mv_unit,
        origin_x,
        origin_y,
        bwidth,
        bheight,
        NEIGHBOR_ARRAY_UNIT_FULL_MASK);

    // Skip Flag
    neighbor_array_unit_mode_write(
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
    EncDecContext_t         *context_ptr,
    NeighborArrayUnit_t     *lumaReconSampleNeighborArray,
    NeighborArrayUnit_t     *cbReconSampleNeighborArray,
    NeighborArrayUnit_t     *crReconSampleNeighborArray,
    EbPictureBufferDesc_t   *recon_buffer,
    uint32_t                   origin_x,
    uint32_t                   origin_y,
    uint32_t                   width,
    uint32_t                   height,
    uint32_t                   bwidth_uv,
    uint32_t                   bheight_uv,
    uint32_t                   component_mask,
    EbBool                  is16bit)
{
    //uint32_t                 round_origin_x = (origin_x >> 3) << 3;// for Chroma blocks with size of 4
    //uint32_t                 round_origin_y = (origin_y >> 3) << 3;// for Chroma blocks with size of 4
    const EbColorFormat  color_format = context_ptr->color_format;
    const uint16_t ss_x = (color_format == EB_YUV444 ? 1 : 2) - 1;
    const uint16_t ss_y = (color_format >= EB_YUV422 ? 1 : 2) - 1;

    uint32_t  round_origin_x = (origin_x >> (2 + ss_x)) << (2 + ss_x);// for Chroma blocks with size of 4
    uint32_t  round_origin_y = (origin_y >> (2 + ss_y)) << (2 + ss_y);// for Chroma blocks with size of 4

    if (is16bit == EB_TRUE) {
        if (component_mask & PICTURE_BUFFER_DESC_LUMA_MASK)
        {
            // Recon Samples - Luma
            neighbor_array_unit16bit_sample_write(
                lumaReconSampleNeighborArray,
                (uint16_t*)(recon_buffer->buffer_y),
                recon_buffer->stride_y,
                recon_buffer->origin_x + origin_x,
                recon_buffer->origin_y + origin_y,
                origin_x,
                origin_y,
                width,
                height,
                NEIGHBOR_ARRAY_UNIT_FULL_MASK);
        }

        if (component_mask & PICTURE_BUFFER_DESC_CHROMA_MASK)
        {
            // Recon Samples - Cb
            neighbor_array_unit16bit_sample_write(
                cbReconSampleNeighborArray,
                (uint16_t*)(recon_buffer->bufferCb),
                recon_buffer->strideCb,
                (recon_buffer->origin_x + round_origin_x) >> 1,
                (recon_buffer->origin_y + round_origin_y) >> 1,
                round_origin_x >> 1,
                round_origin_y >> 1,
                bwidth_uv,
                bheight_uv,
                NEIGHBOR_ARRAY_UNIT_FULL_MASK);

            // Recon Samples - Cr
            neighbor_array_unit16bit_sample_write(
                crReconSampleNeighborArray,
                (uint16_t*)(recon_buffer->bufferCr),
                recon_buffer->strideCr,
                (recon_buffer->origin_x + round_origin_x) >> 1,
                (recon_buffer->origin_y + round_origin_y) >> 1,
                round_origin_x >> 1,
                round_origin_y >> 1,
                bwidth_uv,
                bheight_uv,
                NEIGHBOR_ARRAY_UNIT_FULL_MASK);
        }

    }
    else {
        if (component_mask & PICTURE_BUFFER_DESC_LUMA_MASK)
        {
            // Recon Samples - Luma
            neighbor_array_unit_sample_write(
                lumaReconSampleNeighborArray,
                recon_buffer->buffer_y,
                recon_buffer->stride_y,
                recon_buffer->origin_x + origin_x,
                recon_buffer->origin_y + origin_y,
                origin_x,
                origin_y,
                width,
                height,
                NEIGHBOR_ARRAY_UNIT_FULL_MASK);
        }

        if (component_mask & PICTURE_BUFFER_DESC_CHROMA_MASK)
        {
            // Recon Samples - Cb
            neighbor_array_unit_sample_write(
                cbReconSampleNeighborArray,
                recon_buffer->bufferCb,
                recon_buffer->strideCb,
                (recon_buffer->origin_x + round_origin_x) >> ss_x,
                (recon_buffer->origin_y + round_origin_y) >> ss_y,
                round_origin_x >> ss_x,
                round_origin_y >> ss_y,
                bwidth_uv,
                bheight_uv,
                NEIGHBOR_ARRAY_UNIT_FULL_MASK);

            // Recon Samples - Cr
            neighbor_array_unit_sample_write(
                crReconSampleNeighborArray,
                recon_buffer->bufferCr,
                recon_buffer->strideCr,
                (recon_buffer->origin_x + round_origin_x) >> ss_x,
                (recon_buffer->origin_y + round_origin_y) >> ss_y,
                round_origin_x >> ss_x,
                round_origin_y >> ss_y,
                bwidth_uv,
                bheight_uv,
                NEIGHBOR_ARRAY_UNIT_FULL_MASK);
        }
    }

    return;
}




/************************************************************
* Update Intra Luma Neighbor Modes
************************************************************/
void GeneratePuIntraLumaNeighborModes(
    CodingUnit_t            *cu_ptr,
    uint32_t                   pu_origin_x,
    uint32_t                   pu_origin_y,
    uint32_t                   sb_sz,
    NeighborArrayUnit_t     *intraLumaNeighborArray,
    NeighborArrayUnit_t     *intraChromaNeighborArray,
    NeighborArrayUnit_t     *mode_type_neighbor_array)
{

    (void)sb_sz;

    uint32_t modeTypeLeftNeighborIndex = get_neighbor_array_unit_left_index(
        mode_type_neighbor_array,
        pu_origin_y);
    uint32_t modeTypeTopNeighborIndex = get_neighbor_array_unit_top_index(
        mode_type_neighbor_array,
        pu_origin_x);
    uint32_t intraLumaModeLeftNeighborIndex = get_neighbor_array_unit_left_index(
        intraLumaNeighborArray,
        pu_origin_y);
    uint32_t intraLumaModeTopNeighborIndex = get_neighbor_array_unit_top_index(
        intraLumaNeighborArray,
        pu_origin_x);

    uint32_t puOriginX_round = (pu_origin_x >> 3) << 3;
    uint32_t puOriginY_round = (pu_origin_y >> 3) << 3;

    uint32_t intraChromaModeLeftNeighborIndex = get_neighbor_array_unit_left_index(
        intraChromaNeighborArray,
        puOriginY_round >> 1);
    uint32_t intraChromaModeTopNeighborIndex = get_neighbor_array_unit_top_index(
        intraChromaNeighborArray,
        puOriginX_round >> 1);

    (&cu_ptr->prediction_unit_array[0])->intra_luma_left_mode = (uint32_t)(
        (mode_type_neighbor_array->leftArray[modeTypeLeftNeighborIndex] != INTRA_MODE) ? DC_PRED/*EB_INTRA_DC*/ :
        (uint32_t)intraLumaNeighborArray->leftArray[intraLumaModeLeftNeighborIndex]);

    (&cu_ptr->prediction_unit_array[0])->intra_luma_top_mode = (uint32_t)(
        (mode_type_neighbor_array->topArray[modeTypeTopNeighborIndex] != INTRA_MODE) ? DC_PRED/*EB_INTRA_DC*/ :
        (uint32_t)intraLumaNeighborArray->topArray[intraLumaModeTopNeighborIndex]);       //   use DC. This seems like we could use a LCU-width

    uint32_t modeTypeLeftNeighborIndex_round = get_neighbor_array_unit_left_index(
        mode_type_neighbor_array,
        puOriginY_round);
    uint32_t modeTypeTopNeighborIndex_round = get_neighbor_array_unit_top_index(
        mode_type_neighbor_array,
        puOriginX_round);

    (&cu_ptr->prediction_unit_array[0])->intra_chroma_left_mode = (uint32_t)(
        (mode_type_neighbor_array->leftArray[modeTypeLeftNeighborIndex_round] != INTRA_MODE) ? UV_DC_PRED :
        (uint32_t)intraChromaNeighborArray->leftArray[intraChromaModeLeftNeighborIndex]);

    (&cu_ptr->prediction_unit_array[0])->intra_chroma_top_mode = (uint32_t)(
        (mode_type_neighbor_array->topArray[modeTypeTopNeighborIndex_round] != INTRA_MODE) ? UV_DC_PRED :
        (uint32_t)intraChromaNeighborArray->topArray[intraChromaModeTopNeighborIndex]);       //   use DC. This seems like we could use a LCU-width


    return;
}


void PfZeroOutUselessQuadrants(
    int16_t* transformCoeffBuffer,
    uint32_t  transformCoeffStride,
    uint32_t  quadrantSize,
    EbAsm  asm_type) {

    pic_zero_out_coef_func_ptr_array[asm_type][quadrantSize >> 3](
        transformCoeffBuffer,
        transformCoeffStride,
        quadrantSize,
        quadrantSize,
        quadrantSize);

    pic_zero_out_coef_func_ptr_array[asm_type][quadrantSize >> 3](
        transformCoeffBuffer,
        transformCoeffStride,
        quadrantSize * transformCoeffStride,
        quadrantSize,
        quadrantSize);

    pic_zero_out_coef_func_ptr_array[asm_type][quadrantSize >> 3](
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


/***************************************************
* Update Recon Samples Neighbor Arrays
***************************************************/
static void EncodePassUpdateIntraReconSampleNeighborArrays(
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
        recon_ptr = reconBuffer->buffer_y;
        recon_stride = reconBuffer->stride_y;
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
        neighbor_array_unit16bit_sample_write(
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
        neighbor_array_unit_sample_write(
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

static void Get1dOrigin(uint32_t offset, EbPictureBufferDesc_t *desc, uint8_t p, EbByte *ptr)
{
    uint8_t subsampling_x = (p == 0) ? 0 : ((desc->color_format == EB_YUV444 ? 1 : 2) - 1);
    uint8_t subsampling_y = (p == 0) ? 0 : ((desc->color_format >= EB_YUV422 ? 1 : 2) - 1);
    uint8_t desc_depth_size = (desc->bit_depth == EB_8BIT) ? 1 :
                               (desc->bit_depth == EB_32BIT) ? 4 : 2;

    uint32_t inputOffset = offset * desc_depth_size;
    EbByte desc_buf = (p == 0) ? desc->buffer_y: 
                      (p == 1) ? desc->bufferCb : desc->bufferCr;
    *ptr = desc_buf + inputOffset;
}

static void Get2dOrigin(uint32_t origin_x, uint32_t origin_y, EbPictureBufferDesc_t* desc,
        uint8_t p, EbByte *ptr, uint32_t* stride, uint32_t* offset)
{
    uint8_t subsampling_x = (p == 0) ? 0 : ((desc->color_format == EB_YUV444 ? 1 : 2) - 1);
    uint8_t subsampling_y = (p == 0) ? 0 : ((desc->color_format >= EB_YUV422 ? 1 : 2) - 1);

    uint32_t desc_origin_x = desc->origin_x >> subsampling_x;
    uint32_t desc_origin_y = desc->origin_y >> subsampling_y;
    uint32_t plane_stride = (p == 0) ? desc->stride_y:
                            (p == 1) ? desc->strideCb: desc->strideCr;

    uint8_t desc_depth_size = (desc->bit_depth == EB_8BIT) ? 1 :
                               (desc->bit_depth == EB_32BIT) ? 4 : 2;

    uint32_t inputOffset = ((origin_y + desc_origin_y) * plane_stride + (origin_x + desc_origin_x)) * desc_depth_size;

    EbByte desc_buf = (p == 0) ? desc->buffer_y: 
                      (p == 1) ? desc->bufferCb : desc->bufferCr;
    if (stride) {
        *stride = plane_stride;
    }

    if (ptr) {
        *ptr = desc_buf + inputOffset;
    }

    if (offset) {
        *offset = inputOffset;
    }
}

static void GeneratePuNeighborModes(
    EncDecContext_t         *context_ptr,
    uint32_t                 pu_origin_x,
    uint32_t                 pu_origin_y,
    NeighborArrayUnit_t     *mode_type_neighbor_array)
{
    uint32_t modeTypeLeftNeighborIndex = get_neighbor_array_unit_left_index(
        mode_type_neighbor_array,
        pu_origin_y);

    uint32_t modeTypeTopNeighborIndex = get_neighbor_array_unit_top_index(
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
    uint32_t intraModeLeftNeighborIndex = get_neighbor_array_unit_left_index(
        intra_neighbor_array,
        pu_origin_y);
        
    uint32_t intraModeTopNeighborIndex = get_neighbor_array_unit_top_index(
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

    //Jing: Need to have more than 1 tu for 422 & 444
    TransformUnit       *txb_ptr = &cu_ptr->transform_unit_array[txb_itr];
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

    if (cu_ptr->prediction_mode_flag == INTRA_MODE && plane && context_ptr->evaluate_cfl_ep) {
        // 3: Loop over alphas and find the best or choose DC
        // Use the 1st spot of the candidate buffer to hold cfl settings: (1) to use same kernel as MD for CFL evaluation: cfl_rd_pick_alpha() (toward unification), (2) to avoid dedicated buffers for CFL evaluation @ EP (toward less memory)
        ModeDecisionCandidateBuffer_t  *candidateBuffer = &(context_ptr->md_context->candidate_buffer_ptr_array[0][0]);

        // Input(s)
        candidateBuffer->candidate_ptr->type = INTRA_MODE;
        candidateBuffer->candidate_ptr->intra_luma_mode = cu_ptr->pred_mode;
        candidateBuffer->candidate_ptr->cfl_alpha_signs = 0;
        candidateBuffer->candidate_ptr->cfl_alpha_idx = 0;
        context_ptr->md_context->blk_geom = context_ptr->blk_geom;

        EbByte src_pred_ptr;
        EbByte dst_pred_ptr;
        uint32_t src_stride;
        uint32_t dst_stride;
        uint32_t src_offset;
        uint32_t dst_offset;

        uint32_t sb_uv_origin_x = sb_origin_x >> subsampling_x;
        uint32_t sb_uv_origin_y = sb_origin_y >> subsampling_y;

        // Copy Cb/Cr pred samples from ep buffer to md buffer
        for (int p=1; p<=2; p++) {
            Get2dOrigin(origin_x, origin_y, predSamples, p, &src_pred_ptr, &src_stride, &src_offset);
            // Jing:
            // For 422 case, copy 422 chroma to 420 MD buffer is OK, since CFL only happens below 32x32
            // So put it at the beginning
            //Get2dOrigin(origin_x - sb_uv_origin_x, origin_y - sb_uv_origin_y,
            //        candidateBuffer->prediction_ptr, p, &dst_pred_ptr, &dst_stride, &dst_offset);
            Get2dOrigin(0, 0, candidateBuffer->prediction_ptr, p, &dst_pred_ptr, &dst_stride, &dst_offset);

            for (int i = 0; i < context_ptr->blk_geom->bheight_uv_ex; i++) {
                memcpy(dst_pred_ptr, src_pred_ptr, context_ptr->blk_geom->bwidth_uv_ex);
                src_pred_ptr += src_stride;
                dst_pred_ptr += dst_stride;
            }
        }

        cfl_rd_pick_alpha(
                picture_control_set_ptr,
                ED_STAGE,
                candidateBuffer,
                sb_ptr,
                context_ptr->md_context,
                input_samples,
                src_offset,
                dst_offset,
                asm_type);

        // Output(s)
        if (candidateBuffer->candidate_ptr->intra_chroma_mode == UV_CFL_PRED) {
            cu_ptr->prediction_unit_array->intra_chroma_mode = UV_CFL_PRED;
            cu_ptr->prediction_unit_array->cfl_alpha_idx = candidateBuffer->candidate_ptr->cfl_alpha_idx;
            cu_ptr->prediction_unit_array->cfl_alpha_signs = candidateBuffer->candidate_ptr->cfl_alpha_signs;
            cu_ptr->prediction_unit_array->is_directional_chroma_mode_flag = EB_FALSE;
        }
    }

    for (int i = 0; i <= plane; i++) {
        uint8_t p = i + plane;
        uint32_t sb_plane_origin_x = (p == 0) ? sb_origin_x : sb_origin_x >> subsampling_x;
        uint32_t sb_plane_origin_y = (p == 0) ? sb_origin_y : sb_origin_y >> subsampling_y;
        Get2dOrigin(origin_x, origin_y, input_samples, p, &input_ptr[p], &input_stride[p], NULL);
        Get2dOrigin(origin_x, origin_y, predSamples, p, &pred_ptr[p], &pred_stride[p], NULL);
        Get2dOrigin(origin_x - sb_plane_origin_x, origin_y - sb_plane_origin_y, residual16bit, p, &res_ptr[p], &res_stride[p], NULL);
        //if (p == 0) {
        //printf("plane is %d, block (%d, %d), size %dx%d, stride is %d/%d/%d, res ptr %p\n",
        //        p, origin_x, origin_y, txw, txh,
        //        input_stride[p], pred_stride[p], res_stride[p], residual16bit);
        //}

        Get1dOrigin(context_ptr->coded_area_sb[plane], transform32bit, p, &trans_ptr[p]);
        Get1dOrigin(context_ptr->coded_area_sb[plane], coeffSamplesTB, p, &coeff_ptr[p]);
        Get1dOrigin(context_ptr->coded_area_sb[plane], inverse_quant_buffer, p, &inverse_ptr[p]);

        EbBool clean_sparse_coeff_flag = EB_FALSE;

        context_ptr->three_quad_energy = 0;

        PLANE_TYPE p_type = (p == 0) ? PLANE_TYPE_Y : PLANE_TYPE_UV;
        COMPONENT_TYPE c_type = (p == 0) ? COMPONENT_LUMA : (p == 1) ? COMPONENT_CHROMA_CB: COMPONENT_CHROMA_CR;
        EB_TRANS_COEFF_SHAPE trans_shape = (p == 0) ? context_ptr->trans_coeff_shape_luma : context_ptr->trans_coeff_shape_chroma;


        if (cu_ptr->prediction_mode_flag == INTRA_MODE && p && cu_ptr->prediction_unit_array->intra_chroma_mode == UV_CFL_PRED) {
            //Jing: CFL case
            CFL_PRED_TYPE pred_type = (p == 1 ? CFL_PRED_U : CFL_PRED_V);
            int32_t alpha_q3 =
                cfl_idx_to_alpha(cu_ptr->prediction_unit_array->cfl_alpha_idx, cu_ptr->prediction_unit_array->cfl_alpha_signs, pred_type);

            cfl_predict_lbd(
                    context_ptr->md_context->pred_buf_q3,
                    pred_ptr[p],
                    pred_stride[p],
                    pred_ptr[p],
                    pred_stride[p],
                    alpha_q3,
                    8,
                    context_ptr->blk_geom->tx_width_uv_ex[context_ptr->txb_itr],
                    context_ptr->blk_geom->tx_height_uv_ex[context_ptr->txb_itr]);
        }

        ResidualKernel(
            input_ptr[p],
            input_stride[p],
            pred_ptr[p],
            pred_stride[p],
            res_ptr[p],
            res_stride[p],
            txw,
            txh);

        if (plane == 0) {
            uint8_t tx_search_skip_fag = picture_control_set_ptr->parent_pcs_ptr->tx_search_level == TX_SEARCH_ENC_DEC ?
                get_skip_tx_search_flag(context_ptr->blk_geom->sq_size, MAX_MODE_COST, 0, 1) : 1;

            if (!tx_search_skip_fag) {
                encode_pass_tx_search(
                        picture_control_set_ptr,
                        context_ptr,
                        sb_ptr,
                        cbQp,
                        coeffSamplesTB,
                        residual16bit,
                        transform32bit,
                        inverse_quant_buffer,
                        transformScratchBuffer,
                        asm_type,
                        count_non_zero_coeffs,
                        PICTURE_BUFFER_DESC_LUMA_MASK, //not used
                        use_delta_qp,
                        dZoffset,
                        eob,
                        candidate_plane);
            }
        }

        av1_estimate_transform(
                res_ptr[p],
                res_stride[p],
                trans_ptr[p],
                NOT_USED_VALUE,
                tx_size,
                &context_ptr->three_quad_energy,
                transformScratchBuffer,
                BIT_INCREMENT_8BIT,
                txb_ptr->transform_type[p_type],
                asm_type,
                p_type,
#if PF_N2_32X32
                DEFAULT_SHAPE);
#else
                trans_shape);
#endif
        //printf("About to do the transform, plane %d, tx_size %d, tx type %d\n", plane, tx_size, txb_ptr->transform_type[p_type]);

        av1_quantize_inv_quantize(
                sb_ptr->picture_control_set_ptr,
                trans_ptr[p],
                NOT_USED_VALUE,
                coeff_ptr[p],
                inverse_ptr[p],
                qp,
                txw, txh, tx_size,
                &eob[p],
                candidate_plane[p],
                asm_type,
                &(count_non_zero_coeffs[p]),
#if !PF_N2_32X32
                0,
#endif
                0,
                c_type,
                BIT_INCREMENT_8BIT,
                txb_ptr->transform_type[p_type],
                clean_sparse_coeff_flag);

        if (p == 0) {
            if (count_non_zero_coeffs[0] == 0) {
                // INTER. Chroma follows Luma in transform type
                if (cu_ptr->prediction_mode_flag == INTER_MODE) {
                    txb_ptr->transform_type[PLANE_TYPE_Y] = DCT_DCT;
                    txb_ptr->transform_type[PLANE_TYPE_UV] = DCT_DCT;
                } else { // INTRA
                    txb_ptr->transform_type[PLANE_TYPE_Y] = DCT_DCT;
                }
            }

            txb_ptr->y_has_coeff = count_non_zero_coeffs[0] ? EB_TRUE : EB_FALSE;
            txb_ptr->trans_coeff_shape_luma = context_ptr->trans_coeff_shape_luma;
        } else if (p == 1) {
            txb_ptr->u_has_coeff = count_non_zero_coeffs[1] ? EB_TRUE : EB_FALSE;
            txb_ptr->trans_coeff_shape_chroma = context_ptr->trans_coeff_shape_chroma;
        } else {
            txb_ptr->v_has_coeff = count_non_zero_coeffs[2] ? EB_TRUE : EB_FALSE;
            txb_ptr->trans_coeff_shape_chroma = context_ptr->trans_coeff_shape_chroma;
        }
        txb_ptr->nz_coef_count[p] = (uint16_t)count_non_zero_coeffs[p];
    }
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
    return;
#if 0
    (void)use_delta_qp;
    (void)dZoffset;
    (void)cbQp;

    CodingUnit_t          *cu_ptr = context_ptr->cu_ptr;
    TransformUnit       *txb_ptr = &cu_ptr->transform_unit_array[context_ptr->txb_itr];
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
    const uint32_t           predLumaOffset = ((predSamples16bit->origin_y + origin_y)        * predSamples16bit->stride_y) + (predSamples16bit->origin_x + origin_x);
    const uint32_t           predCbOffset = (((predSamples16bit->origin_y + round_origin_y) >> 1)  * predSamples16bit->strideCb) + ((predSamples16bit->origin_x + round_origin_x) >> 1);
    const uint32_t           predCrOffset = (((predSamples16bit->origin_y + round_origin_y) >> 1)  * predSamples16bit->strideCr) + ((predSamples16bit->origin_x + round_origin_x) >> 1);
    const uint32_t scratchLumaOffset = context_ptr->blk_geom->origin_x + context_ptr->blk_geom->origin_y * SB_STRIDE_Y;
    const uint32_t scratchCbOffset = ROUND_UV(context_ptr->blk_geom->origin_x) / 2 + ROUND_UV(context_ptr->blk_geom->origin_y) / 2 * SB_STRIDE_UV;
    const uint32_t scratchCrOffset = ROUND_UV(context_ptr->blk_geom->origin_x) / 2 + ROUND_UV(context_ptr->blk_geom->origin_y) / 2 * SB_STRIDE_UV;

    const uint32_t coeff1dOffset = context_ptr->coded_area_sb;
    const uint32_t coeff1dOffsetChroma = context_ptr->coded_area_sb_uv;
    UNUSED(coeff1dOffsetChroma);


    EbBool clean_sparse_coeff_flag = EB_FALSE;

    //Update QP for Quant
    qp += QP_BD_OFFSET;

    {

        //**********************************
        // Luma
        //**********************************
        if (component_mask == PICTURE_BUFFER_DESC_FULL_MASK || component_mask == PICTURE_BUFFER_DESC_LUMA_MASK) {

            residual_kernel16bit(
                ((uint16_t*)inputSamples16bit->buffer_y) + inputLumaOffset,
                inputSamples16bit->stride_y,
                ((uint16_t*)predSamples16bit->buffer_y) + predLumaOffset,
                predSamples16bit->stride_y,
                ((int16_t*)residual16bit->buffer_y) + scratchLumaOffset,
                residual16bit->stride_y,
                context_ptr->blk_geom->tx_width[context_ptr->txb_itr],
                context_ptr->blk_geom->tx_height[context_ptr->txb_itr]);

            uint8_t  tx_search_skip_fag = picture_control_set_ptr->parent_pcs_ptr->tx_search_level == TX_SEARCH_ENC_DEC ? get_skip_tx_search_flag(
                context_ptr->blk_geom->sq_size,
                MAX_MODE_COST,
                0,
                1) : 1;

            if (!tx_search_skip_fag) {

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
            }


            av1_estimate_transform(
                ((int16_t*)residual16bit->buffer_y) + scratchLumaOffset,
                residual16bit->stride_y,
                ((tran_low_t*)transform16bit->buffer_y) + coeff1dOffset,
                NOT_USED_VALUE,
                context_ptr->blk_geom->txsize[context_ptr->txb_itr],
                &context_ptr->three_quad_energy,
                transformScratchBuffer,
                BIT_INCREMENT_10BIT,
                txb_ptr->transform_type[PLANE_TYPE_Y],
                asm_type,
                PLANE_TYPE_Y,
#if PF_N2_32X32
                DEFAULT_SHAPE);
#else
                context_ptr->trans_coeff_shape_luma);
#endif

            av1_quantize_inv_quantize(
                sb_ptr->picture_control_set_ptr,
                ((int32_t*)transform16bit->buffer_y) + coeff1dOffset,
                NOT_USED_VALUE,
                ((int32_t*)coeffSamplesTB->buffer_y) + coeff1dOffset,
                ((int32_t*)inverse_quant_buffer->buffer_y) + coeff1dOffset,
                qp,
                context_ptr->blk_geom->tx_width[context_ptr->txb_itr],
                context_ptr->blk_geom->tx_height[context_ptr->txb_itr],
                context_ptr->blk_geom->txsize[context_ptr->txb_itr],
                &eob[0],
                candidate_plane[0],
                asm_type,
                &(count_non_zero_coeffs[0]),
#if !PF_N2_32X32
                0,
#endif
                0,
                COMPONENT_LUMA,
                BIT_INCREMENT_10BIT,

                txb_ptr->transform_type[PLANE_TYPE_Y],
                clean_sparse_coeff_flag);
            txb_ptr->y_has_coeff = count_non_zero_coeffs[0] ? EB_TRUE : EB_FALSE;
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


        }

        if (component_mask == PICTURE_BUFFER_DESC_FULL_MASK || component_mask == PICTURE_BUFFER_DESC_CHROMA_MASK) {

            if (cu_ptr->prediction_mode_flag == INTRA_MODE && cu_ptr->prediction_unit_array->intra_chroma_mode == UV_CFL_PRED) {
                EbPictureBufferDesc_t *reconSamples = predSamples16bit;
                uint32_t reconLumaOffset = (reconSamples->origin_y + origin_y)            * reconSamples->stride_y + (reconSamples->origin_x + origin_x);
                if (txb_ptr->y_has_coeff == EB_TRUE && cu_ptr->skip_flag == EB_FALSE) {
                    uint16_t     *predBuffer = ((uint16_t*)predSamples16bit->buffer_y) + predLumaOffset;
                    av1_inv_transform_recon(
                        ((int32_t*)inverse_quant_buffer->buffer_y) + coeff1dOffset,
                        CONVERT_TO_BYTEPTR(predBuffer),
                        predSamples->stride_y,
                        context_ptr->blk_geom->txsize[context_ptr->txb_itr],
                        BIT_INCREMENT_10BIT,
                        txb_ptr->transform_type[PLANE_TYPE_Y],
                        PLANE_TYPE_Y,
                        eob[0]);
                }

                // Down sample Luma
                cfl_luma_subsampling_420_hbd_c(
                    ((uint16_t*)reconSamples->buffer_y) + reconLumaOffset,
                    reconSamples->stride_y,
                    context_ptr->md_context->pred_buf_q3,
                    context_ptr->blk_geom->tx_width[context_ptr->txb_itr],
                    context_ptr->blk_geom->tx_height[context_ptr->txb_itr]);

                int32_t round_offset = ((context_ptr->blk_geom->tx_width_uv[context_ptr->txb_itr])*(context_ptr->blk_geom->tx_height_uv[context_ptr->txb_itr])) / 2;


                subtract_average(
                    context_ptr->md_context->pred_buf_q3,
                    context_ptr->blk_geom->tx_width_uv[context_ptr->txb_itr],
                    context_ptr->blk_geom->tx_height_uv[context_ptr->txb_itr],
                    round_offset,
                    LOG2F(context_ptr->blk_geom->tx_width_uv[context_ptr->txb_itr]) + LOG2F(context_ptr->blk_geom->tx_height_uv[context_ptr->txb_itr]));



                int32_t alpha_q3 =
                    cfl_idx_to_alpha(cu_ptr->prediction_unit_array->cfl_alpha_idx, cu_ptr->prediction_unit_array->cfl_alpha_signs, CFL_PRED_U); // once for U, once for V
                // TOCHANGE
                // assert(chromaSize * CFL_BUF_LINE + chromaSize <=                CFL_BUF_SQUARE);

                cfl_predict_hbd(
                    context_ptr->md_context->pred_buf_q3,
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
                    context_ptr->md_context->pred_buf_q3,
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
            residual_kernel16bit(
                ((uint16_t*)inputSamples16bit->bufferCb) + inputCbOffset,
                inputSamples16bit->strideCb,
                ((uint16_t*)predSamples16bit->bufferCb) + predCbOffset,
                predSamples16bit->strideCb,
                ((int16_t*)residual16bit->bufferCb) + scratchCbOffset,
                residual16bit->strideCb,
                context_ptr->blk_geom->tx_width_uv[context_ptr->txb_itr],
                context_ptr->blk_geom->tx_height_uv[context_ptr->txb_itr]);


            residual_kernel16bit(

                ((uint16_t*)inputSamples16bit->bufferCr) + inputCrOffset,
                inputSamples16bit->strideCr,
                ((uint16_t*)predSamples16bit->bufferCr) + predCrOffset,
                predSamples16bit->strideCr,
                ((int16_t*)residual16bit->bufferCr) + scratchCrOffset,
                residual16bit->strideCr,
                context_ptr->blk_geom->tx_width_uv[context_ptr->txb_itr],
                context_ptr->blk_geom->tx_height_uv[context_ptr->txb_itr]);


            av1_estimate_transform(
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
#if PF_N2_32X32
                DEFAULT_SHAPE);
#else
                context_ptr->trans_coeff_shape_chroma);
#endif

            av1_quantize_inv_quantize(
                sb_ptr->picture_control_set_ptr,
                ((int32_t*)transform16bit->bufferCb) + context_ptr->coded_area_sb_uv,
                NOT_USED_VALUE,

                ((int32_t*)coeffSamplesTB->bufferCb) + context_ptr->coded_area_sb_uv,
                ((int32_t*)inverse_quant_buffer->bufferCb) + context_ptr->coded_area_sb_uv,
                qp,
                context_ptr->blk_geom->tx_width_uv[context_ptr->txb_itr],
                context_ptr->blk_geom->tx_height_uv[context_ptr->txb_itr],
                context_ptr->blk_geom->txsize_uv[context_ptr->txb_itr],
                &eob[1],
                candidate_plane[1],
                asm_type,
                &(count_non_zero_coeffs[1]),
#if !PF_N2_32X32
                0,
#endif
                0,
                COMPONENT_CHROMA_CB,
                BIT_INCREMENT_10BIT,
                txb_ptr->transform_type[PLANE_TYPE_UV],
                clean_sparse_coeff_flag);

            txb_ptr->u_has_coeff = count_non_zero_coeffs[1] ? EB_TRUE : EB_FALSE;

            //**********************************
            // Cr
            //**********************************

            av1_estimate_transform(
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
#if PF_N2_32X32
                DEFAULT_SHAPE);
#else
                context_ptr->trans_coeff_shape_chroma);
#endif


            av1_quantize_inv_quantize(
                sb_ptr->picture_control_set_ptr,
                ((int32_t*)transform16bit->bufferCr) + context_ptr->coded_area_sb_uv,
                NOT_USED_VALUE,
                ((int32_t*)coeffSamplesTB->bufferCr) + context_ptr->coded_area_sb_uv,
                ((int32_t*)inverse_quant_buffer->bufferCr) + context_ptr->coded_area_sb_uv,
                qp,
                context_ptr->blk_geom->tx_width_uv[context_ptr->txb_itr],
                context_ptr->blk_geom->tx_height_uv[context_ptr->txb_itr],
                context_ptr->blk_geom->txsize_uv[context_ptr->txb_itr],
                &eob[2],
                candidate_plane[2],
                asm_type,
                &(count_non_zero_coeffs[2]),
#if !PF_N2_32X32
                0,
#endif
                0,
                COMPONENT_CHROMA_CR,
                BIT_INCREMENT_10BIT,
                txb_ptr->transform_type[PLANE_TYPE_UV],
                clean_sparse_coeff_flag);
            txb_ptr->v_has_coeff = count_non_zero_coeffs[2] ? EB_TRUE : EB_FALSE;

        }
    }

#if !PF_N2_32X32
    txb_ptr->trans_coeff_shape_luma = context_ptr->trans_coeff_shape_luma;
    txb_ptr->trans_coeff_shape_chroma = context_ptr->trans_coeff_shape_chroma;
#endif
    txb_ptr->nz_coef_count[0] = (uint16_t)count_non_zero_coeffs[0];
    txb_ptr->nz_coef_count[1] = (uint16_t)count_non_zero_coeffs[1];
    txb_ptr->nz_coef_count[2] = (uint16_t)count_non_zero_coeffs[2];
    return;
#endif
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
    const EbColorFormat             color_format = context_ptr->color_format;
    const uint16_t subsampling_x = (color_format == EB_YUV444 ? 1 : 2) - 1;
    const uint16_t subsampling_y = (color_format >= EB_YUV422 ? 1 : 2) - 1;
    (void)transformScratchBuffer;
    CodingUnit_t        *cu_ptr = context_ptr->cu_ptr;
    TransformUnit       *txb_ptr = &cu_ptr->transform_unit_array[txb_itr];
    PLANE_TYPE p_type = (plane == 0) ? PLANE_TYPE_Y : PLANE_TYPE_UV;

    //2D
    EbByte pred_ptr;
    uint32_t pred_stride;
    
    //1D
    EbByte inverse_ptr;

    uint32_t offset1d = context_ptr->coded_area_sb[p_type];

    EbBool has_coeff = (plane == 0) ? txb_ptr->y_has_coeff :
                       (plane == 1) ? txb_ptr->u_has_coeff :txb_ptr->v_has_coeff;

    Get1dOrigin(offset1d, inverse_quant_buffer, plane, &inverse_ptr);
    Get2dOrigin(origin_x, origin_y, predSamples, plane, &pred_ptr, &pred_stride, NULL);

    //printf("--Generate Recon, plane %d, size %dx%d, offset1d %d, tx type %d\n",
    //        plane, tx_size_wide[tx_size], tx_size_high[tx_size], offset1d, txb_ptr->transform_type[p_type]);
    if (has_coeff && cu_ptr->skip_flag == EB_FALSE) {
        av1_inv_transform_recon8bit(
                inverse_ptr,
                pred_ptr,
                pred_stride,
                tx_size,
                txb_ptr->transform_type[p_type],
                p_type,
                eob[plane]);
    }

    if (plane == 0 && cu_ptr->prediction_mode_flag == INTRA_MODE &&
            (context_ptr->evaluate_cfl_ep || cu_ptr->prediction_unit_array->intra_chroma_mode == UV_CFL_PRED)) {
        // Jing: Prepare luma for CFL

        assert(context_ptr->cu_origin_x == origin_x);
        assert(context_ptr->cu_origin_y == origin_y);

        if (subsampling_x == 1) {
            if (subsampling_y == 1) {
                cfl_luma_subsampling_420_lbd_c(
                        pred_ptr, pred_stride,
                        context_ptr->md_context->pred_buf_q3,
                        context_ptr->blk_geom->tx_width[context_ptr->txb_itr],
                        context_ptr->blk_geom->tx_height[context_ptr->txb_itr]);
            } else {
                cfl_luma_subsampling_422_lbd_c(
                        pred_ptr, pred_stride,
                        context_ptr->md_context->pred_buf_q3,
                        context_ptr->blk_geom->tx_width[context_ptr->txb_itr],
                        context_ptr->blk_geom->tx_height[context_ptr->txb_itr]);
            }
        } else {
            //cfl_luma_subsampling_444_lbd_c(
            //        pred_ptr, pred_stride,
            //        context_ptr->md_context->pred_buf_q3,
            //        context_ptr->blk_geom->tx_width[context_ptr->txb_itr],
            //        context_ptr->blk_geom->tx_height[context_ptr->txb_itr]);
        }

        int32_t round_offset = ((context_ptr->blk_geom->tx_width_uv_ex[context_ptr->txb_itr])*(context_ptr->blk_geom->tx_height_uv_ex[context_ptr->txb_itr])) / 2;

        subtract_average(
                context_ptr->md_context->pred_buf_q3,
                context_ptr->blk_geom->tx_width_uv_ex[context_ptr->txb_itr],
                context_ptr->blk_geom->tx_height_uv_ex[context_ptr->txb_itr],
                round_offset,
                LOG2F(context_ptr->blk_geom->tx_width_uv_ex[context_ptr->txb_itr]) + LOG2F(context_ptr->blk_geom->tx_height_uv_ex[context_ptr->txb_itr])
                );
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
    uint16_t                *eob,
    EbAsm                    asm_type)
{

#if 0
    uint32_t predLumaOffset;
    uint32_t predChromaOffset;

    CodingUnit_t          *cu_ptr = context_ptr->cu_ptr;
    TransformUnit       *txb_ptr = &cu_ptr->transform_unit_array[context_ptr->txb_itr];

    (void)asm_type;
    (void)transformScratchBuffer;
    //**********************************
    // Luma
    //**********************************
    if (component_mask & PICTURE_BUFFER_DESC_LUMA_MASK) {
        if (cu_ptr->prediction_mode_flag != INTRA_MODE || cu_ptr->prediction_unit_array->intra_chroma_mode != UV_CFL_PRED)

        {
            predLumaOffset = (predSamples->origin_y + origin_y)* predSamples->stride_y + (predSamples->origin_x + origin_x);
            if (txb_ptr->y_has_coeff == EB_TRUE && cu_ptr->skip_flag == EB_FALSE) {

                uint16_t     *predBuffer = ((uint16_t*)predSamples->buffer_y) + predLumaOffset;
                av1_inv_transform_recon(
                    ((int32_t*)residual16bit->buffer_y) + context_ptr->coded_area_sb,
                    CONVERT_TO_BYTEPTR(predBuffer),
                    predSamples->stride_y,
                    context_ptr->blk_geom->txsize[context_ptr->txb_itr],
                    BIT_INCREMENT_10BIT,
                    txb_ptr->transform_type[PLANE_TYPE_Y],
                    PLANE_TYPE_Y,
                    eob[0]
                );

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

        if (txb_ptr->u_has_coeff == EB_TRUE && cu_ptr->skip_flag == EB_FALSE) {


            uint16_t     *predBuffer = ((uint16_t*)predSamples->bufferCb) + predChromaOffset;
            av1_inv_transform_recon(
                ((int32_t*)residual16bit->bufferCb) + context_ptr->coded_area_sb_uv,
                CONVERT_TO_BYTEPTR(predBuffer),
                predSamples->strideCb,
                context_ptr->blk_geom->txsize_uv[context_ptr->txb_itr],
                BIT_INCREMENT_10BIT,
                txb_ptr->transform_type[PLANE_TYPE_UV],
                PLANE_TYPE_UV,
                eob[1]);

        }

        //**********************************
        // Cr
        //**********************************
        predChromaOffset = (((predSamples->origin_y + round_origin_y) >> 1)           * predSamples->strideCr) + ((predSamples->origin_x + round_origin_x) >> 1);
        if (txb_ptr->v_has_coeff == EB_TRUE && cu_ptr->skip_flag == EB_FALSE) {

            uint16_t     *predBuffer = ((uint16_t*)predSamples->bufferCr) + predChromaOffset;
            av1_inv_transform_recon(
                ((int32_t*)residual16bit->bufferCr) + context_ptr->coded_area_sb_uv,
                CONVERT_TO_BYTEPTR(predBuffer),
                predSamples->strideCr,
                context_ptr->blk_geom->txsize_uv[context_ptr->txb_itr],
                BIT_INCREMENT_10BIT,
                txb_ptr->transform_type[PLANE_TYPE_UV],
                PLANE_TYPE_UV,
                eob[2]);


        }
    }
#endif

    return;
}
static EB_AV1_ENCODE_LOOP_FUNC_PTR   Av1EncodeLoopFunctionTable[2] =
{
    Av1EncodeLoop,
    Av1EncodeLoop16bit
};

EB_AV1_GENERATE_RECON_FUNC_PTR   Av1EncodeGenerateReconFunctionPtr[2] =
{
    Av1EncodeGenerateRecon,
    Av1EncodeGenerateRecon16bit
};

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
    SequenceControlSet                   *sequence_control_set_ptr,
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
    SequenceControlSet                   *sequence_control_set_ptr,
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


            
            ois_sb_results_t        *ois_sb_results_ptr = picture_control_set_ptr->parent_pcs_ptr->ois_sb_results[sb_index];
            ois_candidate_t *OisCuPtr = ois_sb_results_ptr->sorted_ois_candidate[from_1101_to_85[cu_index]];
            distortion = OisCuPtr[ois_sb_results_ptr->best_distortion_index[from_1101_to_85[cu_index]]].distortion;



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

            if ((picture_control_set_ptr->parent_pcs_ptr->y_mean[sb_index][0] > ANTI_CONTOURING_LUMA_T2) || (picture_control_set_ptr->parent_pcs_ptr->y_mean[sb_index][0] < ANTI_CONTOURING_LUMA_T1)) {

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
    SequenceControlSet                   *sequence_control_set_ptr,
    PictureControlSet_t                    *picture_control_set_ptr,
    LargestCodingUnit_t                    *sb_ptr,
    uint32_t                                  sb_index,
    CodingUnit_t                           *cu_ptr,
    uint32_t                                  cu_depth,
    uint32_t                                  cu_index,
    uint32_t                                  cu_size,
    uint8_t                                   type,
    uint8_t                                   parent32x32_index,
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

    EbBool  use16x16Stat = EB_FALSE;

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



            ois_sb_results_t        *ois_sb_results_ptr = picture_control_set_ptr->parent_pcs_ptr->ois_sb_results[sb_index];
            ois_candidate_t *OisCuPtr = ois_sb_results_ptr->ois_candidate_array[ep_to_pa_block_index[cu_index]];
            distortion = OisCuPtr[ois_sb_results_ptr->best_distortion_index[ep_to_pa_block_index[cu_index]]].distortion;




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

            if (((cu_index > 0) && ((picture_control_set_ptr->parent_pcs_ptr->y_mean[sb_index][parent32x32_index]) > ANTI_CONTOURING_LUMA_T2 || (picture_control_set_ptr->parent_pcs_ptr->y_mean[sb_index][parent32x32_index]) < ANTI_CONTOURING_LUMA_T1)) ||
                ((cu_index == 0) && ((picture_control_set_ptr->parent_pcs_ptr->y_mean[sb_index][0]) > ANTI_CONTOURING_LUMA_T2 || (picture_control_set_ptr->parent_pcs_ptr->y_mean[sb_index][0]) < ANTI_CONTOURING_LUMA_T1))) {

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

    fromPtr = (uint16_t*)context_ptr->input_sample16bit_buffer->buffer_y;
    toPtr = (uint16_t*)picture_control_set_ptr->input_frame16bit->buffer_y + (lcuX + picture_control_set_ptr->input_frame16bit->origin_x) + (lcuY + picture_control_set_ptr->input_frame16bit->origin_y)*picture_control_set_ptr->input_frame16bit->stride_y;

    for (rowIt = 0; rowIt < lcuH; rowIt++)
    {
        memcpy(toPtr + rowIt * picture_control_set_ptr->input_frame16bit->stride_y, fromPtr + rowIt * context_ptr->input_sample16bit_buffer->stride_y, lcuW * 2);
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
    SequenceControlSet      *sequence_control_set_ptr,
    PictureControlSet_t       *picture_control_set_ptr,
    LargestCodingUnit_t       *sb_ptr,
    uint32_t                   tbAddr,
    uint32_t                   sb_origin_x,
    uint32_t                   sb_origin_y,
    uint32_t                   sb_qp,
    EncDecContext_t           *context_ptr)
{
    EbBool                    is16bit = context_ptr->is16bit;
    const EbColorFormat       color_format = context_ptr->color_format;
    EbPictureBufferDesc_t    *recon_buffer = is16bit ? picture_control_set_ptr->recon_picture16bit_ptr : picture_control_set_ptr->recon_picture_ptr;
    EbPictureBufferDesc_t    *coeff_buffer_sb = sb_ptr->quantized_coeff;
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
    uint64_t                  coeff_bits;
    uint64_t                  y_coeff_bits;
    uint64_t                  cb_coeff_bits;
    uint64_t                  cr_coeff_bits;
    uint64_t                  y_full_distortion[DIST_CALC_TOTAL];
    uint64_t                  yTuFullDistortion[DIST_CALC_TOTAL];
    uint32_t                  count_non_zero_coeffs[3];
    MacroblockPlane           cuPlane[3];
    uint16_t                  eobs[MAX_TXB_COUNT][3];
    uint64_t                  tuCoeffBits;
    uint64_t                  y_tu_coeff_bits;
    uint64_t                  cb_tu_coeff_bits;
    uint64_t                  cr_tu_coeff_bits;
    EncodeContext_t          *encode_context_ptr;
    uint32_t                  lcuRowIndex = sb_origin_y / BLOCK_SIZE_64;

    // Dereferencing early
    NeighborArrayUnit_t      *ep_mode_type_neighbor_array = picture_control_set_ptr->ep_mode_type_neighbor_array;
    NeighborArrayUnit_t      *ep_intra_mode_neighbor_array[2];
    ep_intra_mode_neighbor_array[0] = picture_control_set_ptr->ep_intra_luma_mode_neighbor_array;
    ep_intra_mode_neighbor_array[1] = picture_control_set_ptr->ep_intra_chroma_mode_neighbor_array;

    NeighborArrayUnit_t      *ep_mv_neighbor_array = picture_control_set_ptr->ep_mv_neighbor_array;
    NeighborArrayUnit_t      *recon_neighbor_array[3];
    recon_neighbor_array[0] = is16bit ? picture_control_set_ptr->ep_luma_recon_neighbor_array16bit : picture_control_set_ptr->ep_luma_recon_neighbor_array;
    recon_neighbor_array[1] = is16bit ? picture_control_set_ptr->ep_cb_recon_neighbor_array16bit : picture_control_set_ptr->ep_cb_recon_neighbor_array;
    recon_neighbor_array[2] = is16bit ? picture_control_set_ptr->ep_cr_recon_neighbor_array16bit : picture_control_set_ptr->ep_cr_recon_neighbor_array;
    NeighborArrayUnit_t      *ep_skip_flag_neighbor_array = picture_control_set_ptr->ep_skip_flag_neighbor_array;

    EbBool                 constrained_intra_flag = picture_control_set_ptr->constrained_intra_flag;

    EbBool dlfEnableFlag = (EbBool)(picture_control_set_ptr->parent_pcs_ptr->loop_filter_mode &&
        (picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag ||
            sequence_control_set_ptr->static_config.recon_enabled ||
            sequence_control_set_ptr->static_config.stat_report));

    //Jing: TODO remove this 
    dlfEnableFlag = EB_FALSE; 

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


    encode_context_ptr = ((SequenceControlSet*)(picture_control_set_ptr->sequence_control_set_wrapper_ptr->object_ptr))->encode_context_ptr;

    if (picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag == EB_TRUE) {
        //get the 16bit form of the input LCU
        if (is16bit) {
            recon_buffer = ((EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)->reference_picture16bit;
        } else {
            recon_buffer = ((EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)->reference_picture;
        }
    }
    else { // non ref pictures
        recon_buffer = is16bit ? picture_control_set_ptr->recon_picture16bit_ptr : picture_control_set_ptr->recon_picture_ptr;
    }


    EbBool use_delta_qp = (EbBool)sequence_control_set_ptr->static_config.improve_sharpness;
    EbBool oneSegment = (sequence_control_set_ptr->enc_dec_segment_col_count_array[picture_control_set_ptr->temporal_layer_index] == 1) && (sequence_control_set_ptr->enc_dec_segment_row_count_array[picture_control_set_ptr->temporal_layer_index] == 1);
    EbBool useDeltaQpSegments = oneSegment ? 0 : (EbBool)sequence_control_set_ptr->static_config.improve_sharpness;

    // DeriveZeroLumaCbf
    EbBool  highIntraRef = EB_FALSE;
    EbBool  checkZeroLumaCbf = EB_FALSE;

    if (is16bit) {
        uint16_t subsampling_x = (color_format == EB_YUV444 ? 1 : 2) - 1;
        uint16_t subsampling_y = (color_format >= EB_YUV422 ? 1 : 2) - 1;
        //SB128_TODO change 10bit SB creation

        if ((sequence_control_set_ptr->static_config.ten_bit_format == 1) || (sequence_control_set_ptr->static_config.compressed_ten_bit_format == 1))
        {

            const uint32_t inputLumaOffset = ((sb_origin_y + inputPicture->origin_y)         * inputPicture->stride_y) + (sb_origin_x + inputPicture->origin_x);
            const uint32_t inputCbOffset = (((sb_origin_y + inputPicture->origin_y) >> subsampling_y) * inputPicture->strideCb) + ((sb_origin_x + inputPicture->origin_x) >> subsampling_x);
            const uint32_t inputCrOffset = (((sb_origin_y + inputPicture->origin_y) >> subsampling_y) * inputPicture->strideCr) + ((sb_origin_x + inputPicture->origin_x) >> subsampling_x);
            const uint16_t luma2BitWidth = inputPicture->width / 4;
            const uint16_t chroma2BitWidth = inputPicture->width / 8;


            compressed_pack_lcu(
                inputPicture->buffer_y + inputLumaOffset,
                inputPicture->stride_y,
                inputPicture->bufferBitIncY + sb_origin_y * luma2BitWidth + (sb_origin_x / 4)*sb_height,
                sb_width / 4,
                (uint16_t *)context_ptr->input_sample16bit_buffer->buffer_y,
                context_ptr->input_sample16bit_buffer->stride_y,
                sb_width,
                sb_height,
                asm_type);

            compressed_pack_lcu(
                inputPicture->bufferCb + inputCbOffset,
                inputPicture->strideCb,
                inputPicture->bufferBitIncCb + (sb_origin_y >> subsampling_y) * chroma2BitWidth + (sb_origin_x / 8)*(sb_height >> subsampling_y),
                sb_width / 8,
                (uint16_t *)context_ptr->input_sample16bit_buffer->bufferCb,
                context_ptr->input_sample16bit_buffer->strideCb,
                sb_width >> subsampling_x,
                sb_height >> subsampling_y,
                asm_type);

            compressed_pack_lcu(
                inputPicture->bufferCr + inputCrOffset,
                inputPicture->strideCr,
                inputPicture->bufferBitIncCr + (sb_origin_y >> subsampling_y) * chroma2BitWidth + (sb_origin_x / 8)*(sb_height >> subsampling_y),
                sb_width / 8,
                (uint16_t *)context_ptr->input_sample16bit_buffer->bufferCr,
                context_ptr->input_sample16bit_buffer->strideCr,
                sb_width >> subsampling_x,
                sb_height >> subsampling_y,
                asm_type);

        }
        else {

            const uint32_t inputLumaOffset = ((sb_origin_y + inputPicture->origin_y)         * inputPicture->stride_y) + (sb_origin_x + inputPicture->origin_x);
            const uint32_t inputBitIncLumaOffset = ((sb_origin_y + inputPicture->origin_y)         * inputPicture->strideBitIncY) + (sb_origin_x + inputPicture->origin_x);
            const uint32_t inputCbOffset = (((sb_origin_y + inputPicture->origin_y) >> 1)  * inputPicture->strideCb) + ((sb_origin_x + inputPicture->origin_x) >> 1);
            const uint32_t inputBitIncCbOffset = (((sb_origin_y + inputPicture->origin_y) >> 1)  * inputPicture->strideBitIncCb) + ((sb_origin_x + inputPicture->origin_x) >> 1);
            const uint32_t inputCrOffset = (((sb_origin_y + inputPicture->origin_y) >> 1)  * inputPicture->strideCr) + ((sb_origin_x + inputPicture->origin_x) >> 1);
            const uint32_t inputBitIncCrOffset = (((sb_origin_y + inputPicture->origin_y) >> 1)  * inputPicture->strideBitIncCr) + ((sb_origin_x + inputPicture->origin_x) >> 1);

            pack2d_src(
                inputPicture->buffer_y + inputLumaOffset,
                inputPicture->stride_y,
                inputPicture->bufferBitIncY + inputBitIncLumaOffset,
                inputPicture->strideBitIncY,
                (uint16_t *)context_ptr->input_sample16bit_buffer->buffer_y,
                context_ptr->input_sample16bit_buffer->stride_y,
                sb_width,
                sb_height,
                asm_type);


            pack2d_src(
                inputPicture->bufferCb + inputCbOffset,
                inputPicture->strideCr,
                inputPicture->bufferBitIncCb + inputBitIncCbOffset,
                inputPicture->strideBitIncCr,
                (uint16_t *)context_ptr->input_sample16bit_buffer->bufferCb,
                context_ptr->input_sample16bit_buffer->strideCb,
                sb_width >> 1,
                sb_height >> 1,
                asm_type);


            pack2d_src(
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

                EbReferenceObject  *refObjL0 = (EbReferenceObject*)picture_control_set_ptr->ref_pic_ptr_array[REF_LIST_0]->object_ptr;
                EbReferenceObject  *refObjL1 = (EbReferenceObject*)picture_control_set_ptr->ref_pic_ptr_array[REF_LIST_1]->object_ptr;
                uint32_t const TH = (sequence_control_set_ptr->static_config.frame_rate >> 16) < 50 ? 25 : 30;

                if ((refObjL0->tmp_layer_idx == 2 && refObjL0->intra_coded_area > TH) || (refObjL1->tmp_layer_idx == 2 && refObjL1->intra_coded_area > TH))
                    highIntraRef = EB_TRUE;

            }
            if (highIntraRef == EB_FALSE) {

                checkZeroLumaCbf = EB_TRUE;
            }
        }
    }
    context_ptr->intra_coded_area_sb[tbAddr] = 0;
#if !PF_N2_32X32
    context_ptr->trans_coeff_shape_luma = 0;
    context_ptr->trans_coeff_shape_chroma = 0;
#endif
    context_ptr->coded_area_sb[0] = 0;
    context_ptr->coded_area_sb[1] = 0;

#if AV1_LF 
    if (dlfEnableFlag && picture_control_set_ptr->parent_pcs_ptr->loop_filter_mode == 1){        
        if (tbAddr == 0) {
            av1_loop_filter_init(picture_control_set_ptr);

            av1_pick_filter_level(
                0,
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

        const BlockGeom * blk_geom = context_ptr->blk_geom = get_blk_geom_mds(blk_it);
        UNUSED(blk_geom);

        sb_ptr->cu_partition_array[blk_it] = context_ptr->md_context->md_cu_arr_nsq[blk_it].part;


        if (part != PARTITION_SPLIT) {
            int32_t offset_d1 = ns_blk_offset[(int32_t)part]; //cu_ptr->best_d1_blk; // TOCKECK
            int32_t num_d1_block = ns_blk_num[(int32_t)part]; // context_ptr->blk_geom->totns; // TOCKECK

            // for (int32_t d1_itr = blk_it; d1_itr < blk_it + num_d1_block; d1_itr++) {
            for (int32_t d1_itr = (int32_t)blk_it + offset_d1; d1_itr < (int32_t)blk_it + offset_d1 + num_d1_block; d1_itr++) {

                const BlockGeom * blk_geom = context_ptr->blk_geom = get_blk_geom_mds(d1_itr);
                assert(blk_geom->valid_block == 1);

                // PU Stack variables
                PredictionUnit_t        *pu_ptr = (PredictionUnit_t *)EB_NULL; //  done
                EbPictureBufferDesc_t   *residual_buffer = context_ptr->residual_buffer;
                EbPictureBufferDesc_t   *transform_buffer = context_ptr->transform_buffer;

                EbPictureBufferDesc_t   *inverse_quant_buffer = context_ptr->inverse_quant_buffer;

                int16_t                  *transform_inner_array_ptr = context_ptr->transform_inner_array_ptr;

                CodingUnit_t            *cu_ptr = context_ptr->cu_ptr = &context_ptr->md_context->md_cu_arr_nsq[d1_itr];

                context_ptr->cu_origin_x = (uint16_t)(sb_origin_x + blk_geom->origin_x);
                context_ptr->cu_origin_y = (uint16_t)(sb_origin_y + blk_geom->origin_y);
                //printf("Processing CU (%d, %d), block size %d, luma tx size %d\n",
                //        context_ptr->cu_origin_x, context_ptr->cu_origin_y, blk_geom->bsize, blk_geom->txsize[0]);
                cu_ptr->delta_qp = 0;
                cu_ptr->block_has_coeff = 0;


                // if(picture_control_set_ptr->picture_number==4 && context_ptr->cu_origin_x==0 && context_ptr->cu_origin_y==0)
                //     printf("CHEDD");
                uint32_t  coded_area_org = context_ptr->coded_area_sb[0];
                uint32_t  coded_area_org_uv = context_ptr->coded_area_sb[1];

                // Derive disable_cfl_flag as evaluate_cfl_ep = f(disable_cfl_flag)
                EbBool disable_cfl_flag = (context_ptr->blk_geom->sq_size > 32 ||
                        context_ptr->blk_geom->bwidth == 4 ||
                        context_ptr->blk_geom->bheight == 4) ? EB_TRUE : EB_FALSE;
                // Evaluate cfl @ EP if applicable, and not done @ MD 
                context_ptr->evaluate_cfl_ep = (disable_cfl_flag == EB_FALSE && context_ptr->md_context->chroma_level == CHROMA_MODE_1);

                //Jing:TODO remove this
                context_ptr->evaluate_cfl_ep = EB_FALSE;

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
                            context_ptr->cu_stats->parent32x32_index, // TOCHECK not valid
                            context_ptr);
                }

#endif

                if (cu_ptr->prediction_mode_flag == INTRA_MODE) {
                    context_ptr->is_inter = cu_ptr->av1xd->use_intrabc;
                    context_ptr->tot_intra_coded_area += blk_geom->bwidth* blk_geom->bheight;
                    if (picture_control_set_ptr->slice_type != I_SLICE) {
                        context_ptr->intra_coded_area_sb[tbAddr] += blk_geom->bwidth* blk_geom->bheight;
                    }

                    // *Note - Transforms are the same size as predictions
                    // Partition Loop
                    context_ptr->txb_itr = 0;
                    //Jing: one loop for CbCr in EncodeLoop() due to CFL@Ep
                    for (int32_t plane = 0; plane <= blk_geom->has_uv_ex; ++plane) {
                        assert(plane == 0 || plane == 1);
                        TxSize  tx_size = plane ? blk_geom->txsize_uv_ex[context_ptr->txb_itr] : blk_geom->txsize[context_ptr->txb_itr];
                        int32_t plane_width = plane ? blk_geom->bwidth_uv_ex : blk_geom->bwidth;
                        int32_t plane_height = plane ? blk_geom->bheight_uv_ex : blk_geom->bheight;
                        const int32_t txw = tx_size_wide[tx_size];
                        const int32_t txh = tx_size_high[tx_size];

                        //TU loop
                        uint16_t txb_itr[2] = {0};
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

                                // Jing: The predict will do luma/chroma together
                                if (cu_ptr->av1xd->use_intrabc) {
                                    if (plane == 0) {
                                        MvReferenceFrame ref_frame = INTRA_FRAME;
                                        generate_av1_mvp_table(
                                                &sb_ptr->tile_info,
                                                context_ptr->md_context,
                                                cu_ptr,
                                                context_ptr->blk_geom,
                                                context_ptr->cu_origin_x,
                                                context_ptr->cu_origin_y,
                                                &ref_frame,
                                                1,
                                                picture_control_set_ptr);

                                        IntMv nearestmv, nearmv;
                                        av1_find_best_ref_mvs_from_stack(0, 
                                                context_ptr->md_context->md_local_cu_unit[blk_geom->blkidx_mds].ed_ref_mv_stack,
                                                cu_ptr->av1xd, ref_frame, &nearestmv, &nearmv, 0);

                                        if (nearestmv.as_int == INVALID_MV) {
                                            nearestmv.as_int = 0;
                                        }
                                        if (nearmv.as_int == INVALID_MV) {
                                            nearmv.as_int = 0;
                                        }

                                        IntMv dv_ref = nearestmv.as_int == 0 ? nearmv : nearestmv;
                                        if (dv_ref.as_int == 0)
                                            av1_find_ref_dv(&dv_ref, &cu_ptr->av1xd->tile,
                                                    sequence_control_set_ptr->mib_size,
                                                    context_ptr->cu_origin_y >> MI_SIZE_LOG2,
                                                    context_ptr->cu_origin_x >> MI_SIZE_LOG2);
                                        // Ref DV should not have sub-pel.
                                        assert((dv_ref.as_mv.col & 7) == 0);
                                        assert((dv_ref.as_mv.row & 7) == 0);
                                        context_ptr->md_context->md_local_cu_unit[blk_geom->blkidx_mds].ed_ref_mv_stack[INTRA_FRAME][0].this_mv = dv_ref;
                                        cu_ptr->predmv[0] = dv_ref;

                                        //keep final usefull mvp for entropy
                                        memcpy(cu_ptr->av1xd->final_ref_mv_stack,
                                                context_ptr->md_context->md_local_cu_unit[context_ptr->blk_geom->blkidx_mds].ed_ref_mv_stack[cu_ptr->prediction_unit_array[0].ref_frame_type],
                                                sizeof(CandidateMv)*MAX_REF_MV_STACK_SIZE);

                                        pu_ptr = cu_ptr->prediction_unit_array;
                                        // Set MvUnit
                                        context_ptr->mv_unit.predDirection = (uint8_t)pu_ptr->inter_pred_direction_index;
                                        context_ptr->mv_unit.mv[REF_LIST_0].mvUnion = pu_ptr->mv[REF_LIST_0].mvUnion;
                                        context_ptr->mv_unit.mv[REF_LIST_1].mvUnion = pu_ptr->mv[REF_LIST_1].mvUnion;

                                        EbPictureBufferDesc_t * ref_pic_list0 = ((EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)->reference_picture;

                                        if (is16bit) {
                                            ref_pic_list0 = ((EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)->reference_picture16bit;
                                            av1_inter_prediction_hbd(
                                                picture_control_set_ptr,
                                                cu_ptr->prediction_unit_array->ref_frame_type,
                                                cu_ptr,
                                                &context_ptr->mv_unit,
                                                1,//intrabc
                                                context_ptr->cu_origin_x,
                                                context_ptr->cu_origin_y,
                                                blk_geom->bwidth,
                                                blk_geom->bheight,
                                                ref_pic_list0,
                                                0,
                                                recon_buffer,
                                                context_ptr->cu_origin_x,
                                                context_ptr->cu_origin_y,
                                                (uint8_t)sequence_control_set_ptr->static_config.encoder_bit_depth,
                                                asm_type);
                                        } else {
                                            av1_inter_prediction(
                                                picture_control_set_ptr,
                                                ED_STAGE,
                                                cu_ptr->interp_filters,
                                                cu_ptr,
                                                cu_ptr->prediction_unit_array->ref_frame_type,
                                                &context_ptr->mv_unit,
                                                1,// use_intrabc,
                                                context_ptr->cu_origin_x,
                                                context_ptr->cu_origin_y,
                                                blk_geom->bwidth,
                                                blk_geom->bheight,
                                                ref_pic_list0,
                                                0,
                                                recon_buffer,
                                                context_ptr->cu_origin_x,
                                                context_ptr->cu_origin_y,
                                                EB_TRUE,
                                                asm_type);
                                        }
                                    }
                                } else {
                                    if (is16bit) {
                                        assert(0);
#if 0
                                        uint16_t    topNeighArray[64 * 2 + 1];
                                        uint16_t    leftNeighArray[64 * 2 + 1];
                                        PredictionMode mode;

                                        int32_t plane_end = blk_geom->has_uv ? 2 : 0;

                                        for (int32_t plane = 0; plane <= plane_end; ++plane) {
                                            TxSize  tx_size = plane ? blk_geom->txsize_uv[context_ptr->txb_itr] : blk_geom->txsize[context_ptr->txb_itr];
                                            if (plane == 0) {
                                                if (context_ptr->cu_origin_y != 0)
                                                    memcpy(topNeighArray + 1, (uint16_t*)(ep_luma_recon_neighbor_array->topArray) + context_ptr->cu_origin_x, blk_geom->bwidth * 2 * sizeof(uint16_t));
                                                if (context_ptr->cu_origin_x != 0)
                                                    memcpy(leftNeighArray + 1, (uint16_t*)(ep_luma_recon_neighbor_array->leftArray) + context_ptr->cu_origin_y, blk_geom->bheight * 2 * sizeof(uint16_t));
                                                if (context_ptr->cu_origin_y != 0 && context_ptr->cu_origin_x != 0)
                                                    topNeighArray[0] = leftNeighArray[0] = ((uint16_t*)(ep_luma_recon_neighbor_array->topLeftArray) + MAX_PICTURE_HEIGHT_SIZE + context_ptr->cu_origin_x - context_ptr->cu_origin_y)[0];
                                            }

                                            else if (plane == 1) {
                                                if (cu_originy_uv != 0)
                                                    memcpy(topNeighArray + 1, (uint16_t*)(ep_cb_recon_neighbor_array->topArray) + cu_originx_uv, blk_geom->bwidth_uv * 2 * sizeof(uint16_t));
                                                if (cu_originx_uv != 0)
                                                    memcpy(leftNeighArray + 1, (uint16_t*)(ep_cb_recon_neighbor_array->leftArray) + cu_originy_uv, blk_geom->bheight_uv * 2 * sizeof(uint16_t));
                                                if (cu_originy_uv != 0 && cu_originx_uv != 0)
                                                    topNeighArray[0] = leftNeighArray[0] = ((uint16_t*)(ep_cb_recon_neighbor_array->topLeftArray) + MAX_PICTURE_HEIGHT_SIZE / 2 + cu_originx_uv - cu_originy_uv)[0];
                                            }
                                            else {
                                                if (cu_originy_uv != 0)
                                                    memcpy(topNeighArray + 1, (uint16_t*)(ep_cr_recon_neighbor_array->topArray) + cu_originx_uv, blk_geom->bwidth_uv * 2 * sizeof(uint16_t));
                                                if (cu_originx_uv != 0)
                                                    memcpy(leftNeighArray + 1, (uint16_t*)(ep_cr_recon_neighbor_array->leftArray) + cu_originy_uv, blk_geom->bheight_uv * 2 * sizeof(uint16_t));
                                                if (cu_originy_uv != 0 && cu_originx_uv != 0)
                                                    topNeighArray[0] = leftNeighArray[0] = ((uint16_t*)(ep_cr_recon_neighbor_array->topLeftArray) + MAX_PICTURE_HEIGHT_SIZE / 2 + cu_originx_uv - cu_originy_uv)[0];

                                            }


                                            if (plane)
                                                mode = (pu_ptr->intra_chroma_mode == UV_CFL_PRED) ? (PredictionMode)UV_DC_PRED : (PredictionMode)pu_ptr->intra_chroma_mode;
                                            else
                                                mode = cu_ptr->pred_mode; //PredictionMode mode,

                                            av1_predict_intra_block_16bit(
                                                    &sb_ptr->tile_info,
                                                    context_ptr,
                                                    picture_control_set_ptr->parent_pcs_ptr->av1_cm,                  //const Av1Common *cm,
                                                    plane ? blk_geom->bwidth_uv : blk_geom->bwidth,                  //int32_t wpx,
                                                    plane ? blk_geom->bheight_uv : blk_geom->bheight,                  //int32_t hpx,
                                                    tx_size,
                                                    mode,                                                       //PredictionMode mode,
                                                    plane ? 0 : pu_ptr->angle_delta[PLANE_TYPE_Y],                //int32_t angle_delta,
                                                    0,                                                          //int32_t use_palette,
                                                    FILTER_INTRA_MODES,                                         //CHKN FILTER_INTRA_MODE filter_intra_mode,
                                                    topNeighArray + 1,
                                                    leftNeighArray + 1,
                                                    recon_buffer,                                                //uint8_t *dst,
                                                    //int32_t dst_stride,
                                                    0,                                                          //int32_t col_off,
                                                    0,                                                          //int32_t row_off,
                                                    plane,                                                      //int32_t plane,
                                                    blk_geom->bsize,                  //uint32_t puSize,
                                                    context_ptr->cu_origin_x,  //uint32_t cuOrgX,
                                                    context_ptr->cu_origin_y);   //uint32_t cuOrgY


                                        }
#endif
                                    } else {
                                        uint8_t topNeighArray[2][64 * 2 + 1];
                                        uint8_t leftNeighArray[2][64 * 2 + 1];
                                        PredictionMode mode;
                                        if (plane) {
                                            mode = (pu_ptr->intra_chroma_mode == UV_CFL_PRED) ? (PredictionMode)UV_DC_PRED : (PredictionMode)pu_ptr->intra_chroma_mode;
                                        } else {
                                            mode = cu_ptr->pred_mode; //PredictionMode mode,
                                        }

                                        // Generate prediction
                                        for (int i=0; i<=plane; i++) {
                                            if (pu_block_origin_y != 0) {
                                                memcpy(topNeighArray[i] + 1,
                                                        recon_neighbor_array[i + plane]->topArray + pu_block_origin_x,
                                                        txw * 2);
                                            }
                                            if (pu_block_origin_x != 0) {
                                                memcpy(leftNeighArray[i] + 1,
                                                        recon_neighbor_array[i + plane]->leftArray + pu_block_origin_y,
                                                        txh * 2);
                                            }
                                            if (pu_block_origin_y != 0 && pu_block_origin_x != 0) {
                                                topNeighArray[i][0] = leftNeighArray[i][0] = ((recon_neighbor_array[i + plane]->topLeftArray) + (MAX_PICTURE_HEIGHT_SIZE >> subsampling_y) + pu_block_origin_x - pu_block_origin_y)[0];
                                            }

                                            // Hsan: if CHROMA_MODE_1, then CFL will be evaluated @ EP as no CHROMA @ MD 
                                            // If that's the case then you should ensure than the 1st chroma prediction uses UV_DC_PRED (that's the default configuration for CHROMA_MODE_1 if CFL applicable (set @ fast loop candidates injection) then MD assumes chroma mode always UV_DC_PRED)
                                            av1_predict_intra_block(
                                                &sb_ptr->tile_info,
                                                ED_STAGE,
                                                context_ptr->blk_geom,
                                                picture_control_set_ptr->parent_pcs_ptr->av1_cm,                  //const Av1Common *cm,
                                                plane ? blk_geom->bwidth_uv_ex : blk_geom->bwidth,                   //int32_t wpx,
                                                plane ? blk_geom->bheight_uv_ex : blk_geom->bheight,                  //int32_t hpx,
                                                tx_size,
                                                mode,                                                       //PredictionMode mode,
                                                plane ? 0 : pu_ptr->angle_delta[PLANE_TYPE_Y],                //int32_t angle_delta,
                                                0,                                                          //int32_t use_palette,
                                                FILTER_INTRA_MODES,                                         //CHKN FILTER_INTRA_MODE filter_intra_mode,
                                                topNeighArray[i] + 1,
                                                leftNeighArray[i] + 1,
                                                recon_buffer,                     //uint8_t *dst,
                                                //int32_t dst_stride,
                                                col >> MI_SIZE_LOG2,              //int32_t col_off,
                                                row >> MI_SIZE_LOG2,              //int32_t row_off,
                                                i + plane,                        //int32_t plane,
                                                blk_geom->bsize,                  //uint32_t puSize,
                                                context_ptr->cu_origin_x,
                                                context_ptr->cu_origin_y,
                                                0,
                                                0);
                                        }
                                    }//8bit
                                }//intrabc

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
									cu_ptr->qp,
									recon_buffer,
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

								for (int i=0; i<=plane; i++) {
									int p = i + plane;
#ifdef DEBUG_REF_INFO
									{
										int cu_originX = context_ptr->cu_origin_x;
										int cu_originY = context_ptr->cu_origin_y;
										int plane_originX = pu_block_origin_x;
										int plane_originY = pu_block_origin_y;
										{
											printf("\nAbout to dump pred for CU (%d, %d) at plane %d, size %dx%d, pu offset (%d, %d)\n",
													cu_originX, cu_originY, p, txw, txh, col, row);
											dump_block_from_desc(txw, txh, recon_buffer, cu_originX, cu_originY, p);
										}
									}
#endif
									Av1EncodeGenerateRecon(
											context_ptr,
											pu_block_origin_x,
											pu_block_origin_y,
											tx_size,
											recon_buffer,
											inverse_quant_buffer,
											transform_inner_array_ptr,
											p,
											txb_itr[plane],
											eobs[txb_itr[plane]],
											asm_type);
#ifdef DEBUG_REF_INFO
									{
										int cu_originX = context_ptr->cu_origin_x;
										int cu_originY = context_ptr->cu_origin_y;
										int plane_originX = pu_block_origin_x;
										int plane_originY = pu_block_origin_y;
										{
											printf("\nAbout to dump recon for CU (%d, %d) at plane %d, size %dx%d, pu offset (%d, %d)\n",
													cu_originX, cu_originY, p, txw, txh, col, row);
											dump_block_from_desc(txw, txh, recon_buffer, cu_originX, cu_originY, p);

											printf("\nAbout to dump inverse quant for CU (%d, %d) at plane %d offset (%d, %d), tx size %d\n",
													cu_originX, cu_originY, p, plane_originX, plane_originY, tx_size);
											dump_coeff_block_from_desc(txw, txh, inverse_quant_buffer, cu_originX, cu_originY, p, context_ptr->coded_area_sb[plane]);
											printf("\nAbout to dump coeff for CU (%d, %d) at plane %d offset (%d, %d), tx size %d\n",
													cu_originX, cu_originY, p, plane_originX, plane_originY, tx_size);
											dump_coeff_block_from_desc(txw, txh, coeff_buffer_sb, cu_originX, cu_originY, p, context_ptr->coded_area_sb[plane]);
										}
									}
#endif

									// Update Recon Samples-INTRA-
									EncodePassUpdateIntraReconSampleNeighborArrays(
											recon_neighbor_array[i + plane],
											recon_buffer,
											i + plane,
											pu_block_origin_x,
											pu_block_origin_y,
											txw,
											txh,
											is16bit);
								}

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

								if (plane == 0) {
									cu_ptr->block_has_coeff = cu_ptr->block_has_coeff |
										cu_ptr->transform_unit_array[txb_itr[plane]].y_has_coeff;
								} else {
									cu_ptr->block_has_coeff = cu_ptr->block_has_coeff |
										cu_ptr->transform_unit_array[txb_itr[plane]].u_has_coeff |
										cu_ptr->transform_unit_array[txb_itr[plane]].v_has_coeff;
								}
								context_ptr->coded_area_sb[plane] += txw * txh;
								txb_itr[plane] += 1;
							}//tx col
                        }//tx row
                    }//plane loop
                } else if (cu_ptr->prediction_mode_flag == INTER_MODE) {
                } else {
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

                if (dlfEnableFlag) {
                    assert(0);
                    if (blk_geom->has_uv) {
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
#if AV1_LF
    // First Pass Deblocking
    if (dlfEnableFlag && picture_control_set_ptr->parent_pcs_ptr->loop_filter_mode == 1) {
        if (picture_control_set_ptr->parent_pcs_ptr->lf.filter_level[0] || picture_control_set_ptr->parent_pcs_ptr->lf.filter_level[1]) {
            uint8_t LastCol = ((sb_origin_x)+sb_width == sequence_control_set_ptr->luma_width) ? 1 : 0;
            loop_filter_sb(
                recon_buffer,
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
    SequenceControlSet    *sequence_control_set_ptr,
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
        const BlockGeom * blk_geom = context_ptr->blk_geom = get_blk_geom_mds(blk_it);


        sb_ptr->cu_partition_array[blk_it] = context_ptr->md_context->md_cu_arr_nsq[blk_it].part;

        if (part != PARTITION_SPLIT) {



            int32_t offset_d1 = ns_blk_offset[(int32_t)part]; //cu_ptr->best_d1_blk; // TOCKECK
            int32_t num_d1_block = ns_blk_num[(int32_t)part]; // context_ptr->blk_geom->totns; // TOCKECK

            for (int32_t d1_itr = blk_it + offset_d1; d1_itr < blk_it + offset_d1 + num_d1_block; d1_itr++) {

                const BlockGeom * blk_geom = context_ptr->blk_geom = get_blk_geom_mds(d1_itr);
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

                    int32_t* src_ptr = &(((int32_t*)context_ptr->cu_ptr->coeff_tmp->buffer_y)[txb_1d_offset]);
                    int32_t* dst_ptr = &(((int32_t*)sb_ptr->quantized_coeff->buffer_y)[context_ptr->coded_area_sb]);

                    uint32_t j;
                    for (j = 0; j < bheight; j++)
                    {
                        memcpy(dst_ptr + j * bwidth, src_ptr + j * bwidth, bwidth * sizeof(int32_t));
                    }

                    if (context_ptr->blk_geom->has_uv)
                    {
                        // Cb
                        bwidth = context_ptr->blk_geom->tx_width_uv[txb_itr];
                        bheight = context_ptr->blk_geom->tx_height_uv[txb_itr];

                        src_ptr = &(((int32_t*)context_ptr->cu_ptr->coeff_tmp->bufferCb)[txb_1d_offset_uv]);
                        dst_ptr = &(((int32_t*)sb_ptr->quantized_coeff->bufferCb)[context_ptr->coded_area_sb_uv]);

                        for (j = 0; j < bheight; j++)
                        {
                            memcpy(dst_ptr + j * bwidth, src_ptr + j * bwidth, bwidth * sizeof(int32_t));
                        }

                        //Cr
                        src_ptr = &(((int32_t*)context_ptr->cu_ptr->coeff_tmp->bufferCr)[txb_1d_offset_uv]);
                        dst_ptr = &(((int32_t*)sb_ptr->quantized_coeff->bufferCr)[context_ptr->coded_area_sb_uv]);

                        for (j = 0; j < bheight; j++)
                        {
                            memcpy(dst_ptr + j * bwidth, src_ptr + j * bwidth, bwidth * sizeof(int32_t));
                        }

                    }

                    context_ptr->coded_area_sb += context_ptr->blk_geom->tx_width[txb_itr] * context_ptr->blk_geom->tx_height[txb_itr];
                    if (context_ptr->blk_geom->has_uv)
                        context_ptr->coded_area_sb_uv += context_ptr->blk_geom->tx_width_uv[txb_itr] * context_ptr->blk_geom->tx_height_uv[txb_itr];

                    txb_1d_offset += context_ptr->blk_geom->tx_width[txb_itr] * context_ptr->blk_geom->tx_height[txb_itr];
                    if (context_ptr->blk_geom->has_uv)
                        txb_1d_offset_uv += context_ptr->blk_geom->tx_width_uv[txb_itr] * context_ptr->blk_geom->tx_height_uv[txb_itr];

                    txb_itr++;

                } while (txb_itr < context_ptr->blk_geom->txb_count);






                //copy recon
                {
                    EbPictureBufferDesc_t          *ref_pic;
                    if (picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag)
                    {
                        EbReferenceObject* refObj = (EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr;
                        ref_pic = refObj->reference_picture;
                    }
                    else
                    {
                        ref_pic = picture_control_set_ptr->recon_picture_ptr;
                    }

                    context_ptr->cu_origin_x = sb_origin_x + context_ptr->blk_geom->origin_x;
                    context_ptr->cu_origin_y = sb_origin_y + context_ptr->blk_geom->origin_y;

                    uint32_t  bwidth = context_ptr->blk_geom->bwidth;
                    uint32_t  bheight = context_ptr->blk_geom->bheight;

                    uint8_t* src_ptr = &(((uint8_t*)context_ptr->cu_ptr->recon_tmp->buffer_y)[0]);
                    uint8_t* dst_ptr = ref_pic->buffer_y + ref_pic->origin_x + context_ptr->cu_origin_x + (ref_pic->origin_y + context_ptr->cu_origin_y)*ref_pic->stride_y;

                    uint32_t j;
                    for (j = 0; j < bheight; j++)
                    {
                        memcpy(dst_ptr + j * ref_pic->stride_y, src_ptr + j * 128, bwidth * sizeof(uint8_t));
                    }

                    if (context_ptr->blk_geom->has_uv)
                    {

                        bwidth = context_ptr->blk_geom->bwidth_uv;
                        bheight = context_ptr->blk_geom->bheight_uv;

                        src_ptr = &(((uint8_t*)context_ptr->cu_ptr->recon_tmp->bufferCb)[0]);

                        dst_ptr = ref_pic->bufferCb + ref_pic->origin_x / 2 + ((context_ptr->cu_origin_x >> 3) << 3) / 2 + (ref_pic->origin_y / 2 + ((context_ptr->cu_origin_y >> 3) << 3) / 2)*ref_pic->strideCb;

                        for (j = 0; j < bheight; j++)
                        {
                            memcpy(dst_ptr + j * ref_pic->strideCb, src_ptr + j * 64, bwidth * sizeof(uint8_t));
                        }

                        src_ptr = &(((uint8_t*)context_ptr->cu_ptr->recon_tmp->bufferCr)[0]);

                        dst_ptr = ref_pic->bufferCr + ref_pic->origin_x / 2 + ((context_ptr->cu_origin_x >> 3) << 3) / 2 + (ref_pic->origin_y / 2 + ((context_ptr->cu_origin_y >> 3) << 3) / 2)*ref_pic->strideCr;


                        for (j = 0; j < bheight; j++)
                        {
                            memcpy(dst_ptr + j * ref_pic->strideCr, src_ptr + j * 64, bwidth * sizeof(uint8_t));
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
