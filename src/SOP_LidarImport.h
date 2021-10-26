/*
 * Copyright (c) 2021
 *        Side Effects Software Inc.  All rights reserved.
 *
 * Redistribution and use of Houdini Development Kit samples in source and
 * binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. The name of Side Effects Software may not be used to endorse or
 *    promote products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY SIDE EFFECTS SOFTWARE `AS IS' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 * NO EVENT SHALL SIDE EFFECTS SOFTWARE BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *----------------------------------------------------------------------------
 */

#ifndef __SOP_LidarImport_h__
#define __SOP_LidarImport_h__

#include <E57/E57Simple.h>

#include <SOP/SOP_Node.h>
#include <UT/UT_StringHolder.h>
#include <UT/UT_Interrupt.h>

class SOP_LidarImport : public SOP_Node 
{
public:
    static OP_Node              *myConstructor(
                                        OP_Network *net, 
                                        const char *name,
                                        OP_Operator *op);

    static PRM_Template          myTemplateList[];
    static void                  installSOP(OP_OperatorTable *table);

    class E57Reader;
    class LASReader;

protected:
	                         SOP_LidarImport(
                                        OP_Network *net,
                                        const char *name, 
                                        OP_Operator *op);
                                ~SOP_LidarImport() override;


    bool                         updateParmsFlags() override;
    OP_ERROR                     cookMySop(OP_Context &context) override;

private:
    void		         readLASFile(
                                        const char *filename,
                                        UT_AutoInterrupt &boss,
                                        bool clear_cache_required,
                                        bool color_changed,
                                        bool intensity_changed,
                                        bool ret_data_changed,
                                        bool timestamp_changed,
                                        exint file_max_pts);

    bool		         readE57Scan(
                                        int scan_index,
                                        int64 &pt_idx,
                                        E57Reader &reader,
                                        UT_AutoInterrupt &boss,
                                        UT_StringArray &missing_attribs,
                                        bool clear_cache_required,
                                        bool color_changed,
                                        bool intensity_changed,
                                        bool row_col_changed,
                                        bool ret_data_changed,
                                        bool timestamp_changed,
                                        bool normals_changed,
                                        bool transforms_changed,
                                        UT_StringHolder current_prefix,
                                        exint scan_max_pts);

    void			 updateScanGroupMap(
                                        int scan_index, 
					E57Reader &reader);

    bool			 updateColourFromImage(
                                        int image_index,
					E57Reader &reader,
					UT_AutoInterrupt &boss);

    void			 clearSopNodeCache();

    UT_StringHolder		 myCachedFileName;
    UT_StringHolder		 myCachedGroupPrefix;
    int				 myCachedUseColor;
    bool			 myCachedUseIntensity;
    bool			 myCachedUseRowCol;
    bool			 myCachedUseReturnData;
    bool			 myCachedUseTimestamp;
    bool			 myCachedUseNormals;
    bool			 myCachedUseTransforms;
    bool			 myCachedUseNames;

    int				 myCachedFilterType;
    int				 myCachedRangeGood;
    int				 myCachedRangeSize;
    exint			 myCachedMaxPoints;

    bool			 myCachedDeleteInvalid;

    UT_StringMap<UT_IntArray>    myScanGroupMap;

    enum ColorDataSrc
    {
	NONE = 0,
	PTCLOUD,
	IMAGES,
    };

    enum FilterType
    {
	NOFILTER = 0,
	RANGE,
	MAX,
    };
};

#endif

