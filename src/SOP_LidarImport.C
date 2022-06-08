/*
 * Copyright (c) 2022
 *	Side Effects Software Inc.  All rights reserved.
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

#include <E57/E57Foundation.h>
#include <E57/E57Simple.h>

#include <laszip/laszip_api.h>

#include <GU/GU_Detail.h>
#include <GA/GA_AttributeTransformer.h>
#include <GA/GA_Handle.h>
#include <GA/GA_PageHandle.h>
#include <GA/GA_SplittableRange.h>
#include <OP/OP_Operator.h>
#include <OP/OP_OperatorTable.h>
#include <PRM/PRM_Include.h>
#include <PRM/PRM_SpareData.h>
#include <PRM/PRM_TemplateBuilder.h>
#include <CH/CH_LocalVariable.h>
#include <CH/CH_Manager.h>
#include <PXL/PXL_Raster.h>
#include <IMG/IMG_File.h>
#include <IMG/IMG_Format.h>
#include <UT/UT_Assert.h>
#include <UT/UT_DSOVersion.h>
#include <UT/UT_Interrupt.h>
#include <UT/UT_IStream.h>
#include <UT/UT_Matrix4.h>
#include <UT/UT_Quaternion.h>
#include <UT/UT_String.h>
#include <UT/UT_UndoManager.h>
#include <UT/UT_UniquePtr.h>
#include <UT/UT_VectorTypes.h>

#include <vector>
#include <functional>
#include <exception>

#include "SOP_LidarImport.h"
#include "SOP_LidarImport.proto.h"

//#define E57_SOP_VERBOSE

static const char *theDsFile = R"THEDSFILE(
{
    name    parameters
    parm {
	    name    "filename"
	    label   "File"
    proto_includeraw
	type    file
	default { "" }
	export  all
	parmtag { "filechooser_pattern" "*.e57 *.las *.laz" }
	parmtag { "filechooser_mode" "read" }
    }
    parm {
        name    "precision"
        label   "Precision"
        type    string
        default { "32" }
        menu {
            "32"        "32-bit"
            "64"        "64-bit"
        }
    }
    parm {
	name	"loadtype"
	label	"Load"
	type	ordinal
	default	{ "points" }
	menu	{
	    "points"	"Point Cloud"
            "infobbox"  "Info Bounding Box"
            "info"      "Info"
	}
    }
        groupsimple {
	name	    "filtering"
	label	    "Filtering"
	grouptag    { "group_type" "simple" }
    disablewhen "{ loadtype != points }"

    parm {
	    name    "filter_type"
	    label   "Filter Type"
	    type    ordinal
	    default { "0" }
	    menu {
		    "no_filter" "None"
		    "range_filter" "Range"
		    "max_filter" "Maximum"
	    }
    }
    parm {
	    name    "select_range"
	    label   "Select _ of _"
	    type    integer
	    size    2
	    default { "1" "1" }
	    range   { 0 10 }
    disablewhen "{ filter_type != range_filter }"
    }
    parm {
	    name    "max_points"
	    label   "Max Number of Points"
	    type    integer
	    default { "0" }
    disablewhen "{ filter_type != max_filter }"
    }
    parm {
	    name    "delete_invalid"
	    label   "Delete Invalid Points"
	    type    toggle
	    default { "off" }
    }
    } // Filtering
        groupsimple {
	name	    "attribs"
	label	    "Attributes"
	grouptag    { "group_type" "simple" }
    disablewhen "{ loadtype != points }"
    
    parm {
	    name    "color"
	    label   "Color"
	    type    ordinal
	    default { "0" }
	    menu {
		    "none" "None"
		    "from_ptcloud" "From Point Cloud"
		    "from_images" "From Images"
	    }
    }
    parm {
	    name    "intensity"
	    label   "Intensity"
	    type    toggle
	    default { "off" }
    }
    parm {
	    name    "ret_data"
	    label   "Return Data"
	    type    toggle
	    default { "off" }
    }
    parm {
	    name    "timestamp"
	    label   "Timestamp"
	    type    toggle
	    default { "off" }
    }

            groupcollapsible {
	    name	    "e57"
	    label	    "E57"
	    grouptag    { "group_type" "collapsible" }
        disablewhen "{ loadtype != points }"

        parm {
	        name    "normals"
	        label   "Surface Normals"
	        type    toggle
	        default { "off" }
        }
        parm {
	        name    "row_col"
	        label   "Row and Column"
	        type    toggle
	        default { "off" }
        }
        parm {  
                name    "scanindex"
                label   "Scan Index"
                type    toggle
                default { "off" }
        }
        parm {
	        name    "ptnames"
	        label   "Scan Name"
	        type    toggle
	        default { "off" }
        }
                groupsimple {
	        name	    "info"
	        label	    "Scan Info (Legacy)"
	        grouptag    { "group_type" "simple" }
            disablewhen "{ loadtype != points }"
    
            parm {
	            name    "group_prefix"
	            label   "Group Prefix"
	            type    string
	            default { "lidar_group" }
            }
            parm {
                    name    "scangroups"
                    label   "Group Individual Scans"
                    type    toggle
                    default { "off" }
            }
            parm {
	            name    "rigidtransforms"
	            label   "Rigid Transforms"
	            type    toggle
	            default { "off" }
            }
            parm {
	            name    "scannames"
	            label   "Scan Name List"
	            type    toggle
	            default { "off" }
            }
            } // Scan Info
        } // E57
            groupcollapsible {
	    name	    "las"
	    label	    "LAS and LAZ"
	    grouptag    { "group_type" "collapsible" }
        disablewhen "{ loadtype != points }"

        parm {
                name    "classindex"
                label   "Classification Index"
                type    toggle
                default { "off" }
        }
        parm {
                name    "classname"
                label   "Classification Name"
                type    toggle
                default { "off" }
        }
        parm {
                name    "classflags"
                label   "Classification Flags"
                type    toggle
                default { "off" }
        }
        parm {
                name    "scannerchannel"
                label   "Scanner Channel"
                type    toggle
                default { "off" }
        }
        parm {
                name    "scanflags"
                label   "Scan Flags"
                type    toggle
                default { "off" }
        }
        parm {
                name    "userdata"
                label   "User Data"
                type    toggle
                default { "off" }
        }
        parm {
                name    "scanangle"
                label   "Scan Angle"
                type    toggle
                default { "off" }
        }
        parm {
                name    "pointsourceid"
                label   "Point Source ID"
                type    toggle
                default { "off" }
        }
        parm {
                name    "nearinfrared"
                label   "Near Infrared"
                type    toggle
                default { "off" }
        }
        } //LAS and LAZ
    } // Attributes
groupsimple {
	name	    "transform"
	label	    "Transform"
	grouptag    { "group_type" "simple" }

    parm {
	name	"xOrd"
	label	"Transform Order"
	type	ordinal
	joinnext
	default	{ "srt" }
	menu	{
	    "srt"	"Scale Rot Trans"
	    "str"	"Scale Trans Rot"
	    "rst"	"Rot Scale Trans"
	    "rts"	"Rot Trans Scale"
	    "tsr"	"Trans Scale Rot"
	    "trs"	"Trans Rot Scale"
	}
    }
    parm {
	name	"rOrd"
	label	"Rotate Order"
	type	ordinal
	nolabel
	default	{ "xyz" }
	menu	{
	    "xyz"	"Rx Ry Rz"
	    "xzy"	"Rx Rz Ry"
	    "yxz"	"Ry Rx Rz"
	    "yzx"	"Ry Rz Rx"
	    "zxy"	"Rz Rx Ry"
	    "zyx"	"Rz Ry Rx"
	}
    }
    parm {
	name	"t"
	label	"Translate"
	type	vector
	size	3
	default	{ "0" "0" "0" }
	range	{ -1 1 }
	parmtag	{ "autoscope" "1111111111111111111111111111111" }
    }
    parm {
	name	"r"
	label	"Rotate"
	type	vector
	size	3
	default	{ "0" "0" "0" }
	range	{ 0 360 }
	parmtag	{ "autoscope" "1111111111111111111111111111111" }
    }
    parm {
	name	"s"
	label	"Scale"
	type	vector
	size	3
	default	{ "1" "1" "1" }
	range	{ -1 1 }
	parmtag	{ "autoscope" "1111111111111111111111111111111" }
    }
    parm {
	name	"shear"
	label	"Shear"
	type	float
	size	3
	default	{ "0" "0" "0" }
	range	{ 0 10 }
    }
    parm {
	name	"scale"
	label	"Uniform Scale"
	type	float
	default	{ "1" }
	range	{ 0 10 }
    }
    parm {
        name    "sepparm"
        label   "Spacer"
        type    separator
        default { "" }
        parmtag { "sidefx::layout_height" "medium" }
        parmtag { "sidefx::look" "blank" }
    }
    parm {
	    name    "centroid"
	    label   "Centroid"
	    type    ordinal
            nolabel
            joinnext
	    default { "0" }
	    menu {
		    "metadata" "Using Metadata Centroid"
		    "calculated" "Using Calculated Centroid"
	    }
    }
    parm {
	name	"movecentroid"
	label	"Move Centroid to Origin"
	type	button
        nolabel
        joinnext
	default	{ "0" }
	range	{ 0 1 }
    }
    parm {
    	name	"movepivot"
    	label	"Move Pivot to Centroid"
    	type	button
        nolabel
    	default	{ "0" }
    	range	{ 0 1 }
    }
        groupcollapsible {
            name	    "parmgroup_pivotxform"
            label	    "Pivot Transform"
            grouptag    { "group_type" "collapsible" }

	    parm {
	        name	"p"
	        label	"Pivot Translate"
	        type	vector
	        size	3
	        default	{ "0" "0" "0" }
	        range	{ -1 1 }
	    }
	    parm {
	        name	"pr"
	        label	"Pivot Rotate"
	        type	vector
	        size	3
	        default	{ "0" "0" "0" }
	        range	{ 0 360 }
	    }
        } // Pivot Transform
        groupcollapsible {
            name	    "parmgroup_prexform"
            label	    "Pre-Transform"
            grouptag    { "group_type" "collapsible" }

	    parm {
	        name	"prexform_xOrd"
	        label	"Pre-Transform Order"
	        type	ordinal
	        default	{ "srt" }
	        menu	{
		    "srt"	"Scale Rot Trans"
		    "str"	"Scale Trans Rot"
		    "rst"	"Rot Scale Trans"
		    "rts"	"Rot Trans Scale"
		    "tsr"	"Trans Scale Rot"
		    "trs"	"Trans Rot Scale"
	        }
	        joinnext
	    }
	    parm {
	        name	"prexform_rOrd"
	        label	"Pre-Rotate Order"
	        type	ordinal
	        nolabel
	        default	{ "xyz" }
	        menu	{
		    "xyz"	"Rx Ry Rz"
		    "xzy"	"Rx Rz Ry"
		    "yxz"	"Ry Rx Rz"
		    "yzx"	"Ry Rz Rx"
		    "zxy"	"Rz Rx Ry"
		    "zyx"	"Rz Ry Rx"
	        }
	    }
	    parm {
	        name	"prexform_t"
	        label	"Pre-Translate"
	        type	vector
	        size	3
	        default	{ 0 0 0 }
	    }
	    parm {
	        name	"prexform_r"
	        label	"Pre-Rotate"
	        type	vector
	        size	3
	        default	{ 0 0 0 }
	    }
	    parm {
	        name	"prexform_s"
	        label	"Pre-Scale"
	        type	vector
	        size	3
	        default	{ 1 1 1 }
	    }
	    parm {
	        name	"prexform_shear"
	        label	"Pre-Shear"
	        type	float
	        size	3
	        default	{ "0" "0" "0" }
	        range	{ 0 10 }
	    }
        } // Pre-Transform
    } // Transform
}
)THEDSFILE";

SOP_LidarImport::SOP_LidarImport(OP_Network *net, const char *name, OP_Operator *op)
    : SOP_Node(net, name, op)
{
}

SOP_LidarImport::~SOP_LidarImport() {}

// N-scangroups was a default behavior of the SOP. Now it is a deprecated
// explicit toggle that is set to 'on' for older .hip files.
void
SOP_LidarImport::syncNodeVersion(
    const char *old_version, const char *cur_version, bool *node_deleted)
{
    if (UT_String::compareVersionString(old_version, "19.5.234") < 0)
    {
        setInt("scangroups", 0, 0.0, 1);
    }
    SOP_Node::syncNodeVersion(old_version, cur_version, node_deleted);
}

OP_ERROR
SOP_LidarImport::cookMySop(OP_Context &context)
{
    return cookMyselfAsVerb(context);
}

class SOP_LidarImportCache : public SOP_NodeCache
{
public:
    SOP_LidarImportCache()
        : SOP_NodeCache()
    {
        myFileMetadataBox.initBounds();
        myPositionBox.initBounds();
    }

    ~SOP_LidarImportCache() override {}

    const SOP_LidarImportParms &getCachedParms() const { return myParms; }
    UT_StringMap<UT_IntArray> &getScanGroupMap() { return myScanGroupMap; }

    GA_Attribute *getDetachedPosition() { return myPosition.get(); }
    GA_Attribute *getDetachedNormals() { return myNormals.get(); }
    const GA_Attribute *getDetachedPosition() const { return myPosition.get(); }
    const GA_Attribute *getDetachedNormals() const { return myNormals.get(); }

    const UT_BoundingBoxD &getFileBBox() const { return myFileMetadataBox; }
    const UT_BoundingBoxD &getPointBBox() const { return myPositionBox; }
    void setFileBBox(const UT_BoundingBoxD &bbox) { myFileMetadataBox = bbox; }
    void setPointBBox(const UT_BoundingBoxD &bbox) { myPositionBox = bbox; }

    void updatePointBBox();
    bool isPointBBoxValid() const { return myPositionBox.isValid(); }

    // Binds the detached position attribute.
    // This should be called after the detail's point capacity has been set.
    void bindDetachedPosition(const GA_Detail &gdp)
    {
        myPosition = gdp.createDetachedTupleAttribute(
                GA_ATTRIB_POINT, GA_STORE_REAL64, 3);
        myPosition->setTypeInfo(GA_TYPE_POINT);
    }

    // Binds a detached point normal attribute.
    // This should be called after the detail's point capacity has been set.
    void bindDetachedNormals(GEO_Detail &gdp)
    {
        myNormals = gdp.createDetachedTupleAttribute(
                GA_ATTRIB_POINT, GA_STORE_REAL32, 3);
        myNormals->setTypeInfo(GA_TYPE_NORMAL);
    }

    void destroyDetachedNormals() { myNormals.reset(); }

    void updateCachedParms(const SOP_LidarImportParms &sopparms)
    {
        myParms = sopparms;
    }

    void clearCache()
    {
        myParms = SOP_LidarImportParms();
        myScanGroupMap.clear();
        myPosition.reset();
        myNormals.reset();
        myFileMetadataBox.initBounds();
        myPositionBox.initBounds();
    }

private:
    class PointBoundsComputer;

    SOP_LidarImportParms myParms;
    UT_StringMap<UT_IntArray> myScanGroupMap;
    GA_ATINumericUPtr myPosition;
    GA_ATINumericUPtr myNormals;

    // The bounding box of the file, provided by its metadata.
    // These bounds are the original positions (i.e. untransformed).
    UT_BoundingBoxD myFileMetadataBox;

    // The bounding box of detached position attribute.
    // These bounds correspond to the original positions (i.e. untransformed).
    UT_BoundingBoxD myPositionBox;
};

class SOP_LidarImportVerb : public SOP_NodeVerb
{
public:
    SOP_LidarImportVerb() {}
    ~SOP_LidarImportVerb() override {}

    SOP_NodeParms   *allocParms() const override
                        { return new SOP_LidarImportParms(); }
    SOP_NodeCache   *allocCache() const override
                        { return new SOP_LidarImportCache(); }
    UT_StringHolder  name() const override
                        { return "lidarimport"_sh; }

    CookMode         cookMode(const SOP_NodeParms *parms) const override { return COOK_GENERIC; }
    void	     cook(const CookParms &cookparms) const override;
};


static SOP_NodeVerb::Register<SOP_LidarImportVerb> theSOPLidarImportVerb;

const SOP_NodeVerb *SOP_LidarImport::cookVerb() const
{
    return theSOPLidarImportVerb.get();
}

PRM_Template*
SOP_LidarImport::buildTemplates()
{
    static PRM_TemplateBuilder templ("SOP_LidarImport.C"_sh, theDsFile);
    if (templ.justBuilt())
    {
        templ.setNoCook("centroid", true);
        templ.setCallback("movecentroid", &moveCentroidToOriginCB);
        templ.setCallback("movepivot", &movePivotToCentroidCB);
    }
    return templ.templates();
}

OP_Node *
SOP_LidarImport::myConstructor(OP_Network *net, 
				 const char *name, 
				 OP_Operator *op)
{
    return new SOP_LidarImport(net, name, op);
}

void
SOP_LidarImport::installSOP(OP_OperatorTable *table)
{
    OP_Operator *lidar_import_sop =
        new OP_Operator(
            "lidarimport",
            "Lidar Import",
            SOP_LidarImport::myConstructor,
            SOP_LidarImport::buildTemplates(),
            0, 0, 0,
            OP_FLAG_GENERATOR
        );

    table->addOperator(lidar_import_sop);
}

void
newSopOperator(OP_OperatorTable *table)
{
    SOP_LidarImport::installSOP(table);
}

class SOP_LidarImportCache::PointBoundsComputer
{
public:
    PointBoundsComputer(GA_Attribute &attrib) : myAttrib(&attrib)
    {
        myBox.initBounds();
    }

    PointBoundsComputer(const PointBoundsComputer &that, UT_Split)
        : myAttrib(that.myAttrib)
    {
        myBox.initBounds();
    }

    void operator()(const GA_SplittableRange &r)
    {
        GA_ROPageHandleV3D handle(myAttrib);

        GA_Offset start;
        GA_Offset end;
        for (GA_Iterator it = r.begin(); it.blockAdvance(start, end);)
        {
            handle.setPage(start);
            GA_PageOff rstart = GAgetPageOff(start);
            GA_PageOff rend = GAgetPageOff(end - 1) + 1;
            for (GA_PageOff off = rstart; off < rend; ++off)
            {
                myBox.enlargeBounds(handle.valueRelative(off));
            }
        }
    }

    void join(const PointBoundsComputer &other)
    {
        myBox.enlargeBounds(other.myBox);
    }

    const UT_BoundingBoxD &getBox() const { return myBox; }

private:
    UT_BoundingBoxD myBox;
    const GA_Attribute *myAttrib;
};

// Updates the detached position bounding box.
void
SOP_LidarImportCache::updatePointBBox()
{
    // No current position
    if (!myPosition)
        return;

    PointBoundsComputer first(*myPosition);
    UTparallelReduce(
            GA_SplittableRange(myPosition->getDetail().getPointRange()), first);
    myPositionBox = first.getBox();
}

//******************************************************************************
//*			        LAS/LAZ READER                                 *
//******************************************************************************
namespace
{
// LASzip read wrapper, for reading .las and .laz (compressed .las) files.
class LASReader
{
public:
    LASReader(const char *filename);
    ~LASReader();

    bool isValid() const { return myReader; }
    exint getPointCount() const { return myPointCount; }

    // Read next point data from the stream
    SYS_FORCE_INLINE bool readNextPoint()
    {
        if (laszip_read_point(myReader))
        {
            UTdebugPrint("LASzip failed: laszip_read_point");
            return false;
        }
        return true;
    }

    // Seek to a point in the stream
    SYS_FORCE_INLINE bool seekPoint(uint64 pos)
    {
        if (laszip_seek_point(myReader, pos))
        {
            UTdebugPrint("LASzip failed: laszip_seek_point");
            return false;
        }
        return true;
    }

    // See if standard data exists for the file.
    // Note: Unused LAS fields are filled with default values (usually 0).
    //       We may want to check for unused (default) values after reading,
    //       and give a warning.
    SYS_FORCE_INLINE bool hasGPSTime() const { return myHasGPSTime; }
    SYS_FORCE_INLINE bool hasRGB() const { return myHasRGB; }
    SYS_FORCE_INLINE bool hasNIR() const { return myHasNIR; }
    SYS_FORCE_INLINE bool hasClassFlagOverlap() const
    { return myHasClassFlagOverlap; }
    SYS_FORCE_INLINE bool hasScannerChannel() const
    { return myHasScannerChannel; }

    SYS_FORCE_INLINE UT_Vector3D getScale() const { return myScale; }
    SYS_FORCE_INLINE UT_Vector3D getOffset() const { return myOffset; }
    SYS_FORCE_INLINE UT_BoundingBoxD getBoundingBox() const
    { return myFileBoundingBox; }

    // myScale and myOffset must be applied to get the true xyz coords.
    SYS_FORCE_INLINE void getXYZ(UT_Vector3D &xyz) const;
    SYS_FORCE_INLINE void getIntensity(fpreal32 &in) const;
    SYS_FORCE_INLINE void getUserData(uint8 &in) const;
    SYS_FORCE_INLINE void getPointSourceID(int32 &in) const;
    SYS_FORCE_INLINE void getGPSTime(fpreal64 &in) const;
    SYS_FORCE_INLINE void getRGB(UT_Vector3F &rgb) const;
    SYS_FORCE_INLINE void getNIR(fpreal32 &in) const;

    // These methods differ between legacy formats (0-5) and LAS 1.4 (6-10).
    // Flags are read into uint8 attributes, instead of groups.
    SYS_FORCE_INLINE void getReturnNumber(uint8 &in) const;
    SYS_FORCE_INLINE void getReturnCount(uint8 &in) const;
    SYS_FORCE_INLINE void getClassFlagSynthetic(uint8 &in) const;
    SYS_FORCE_INLINE void getClassFlagKeyPoint(uint8 &in) const;
    SYS_FORCE_INLINE void getClassFlagWithheld(uint8 &in) const;
    SYS_FORCE_INLINE void getClassFlagOverlap(uint8 &in) const;
    SYS_FORCE_INLINE void getScannerChannel(uint8 &in) const;
    SYS_FORCE_INLINE void getScanDirectionFlag(uint8 &in) const;
    SYS_FORCE_INLINE void getEdgeFlightLineFlag(uint8 &in) const;
    SYS_FORCE_INLINE void getClassIndex(uint8 &in) const;
    SYS_FORCE_INLINE void getScanAngle(fpreal32 &in) const;

private:
    laszip_POINTER myReader;
    laszip_BOOL myIsCompressed;
    laszip_header *myHeader;
    laszip_point *myPoint;

    // LAS file metadata
    uint64 myPointCount;
    uint8 myPointFormat;
    UT_Vector3D myScale;
    UT_Vector3D myOffset;
    UT_BoundingBoxD myFileBoundingBox;

    // Available attributes that differ between point formats
    bool myHasGPSTime = false;
    bool myHasRGB = false;
    bool myHasNIR = false;
    bool myHasClassFlagOverlap = false;
    bool myHasScannerChannel = false;

    void closeReader();
    void destroyReader();
    void initializeAvailableAttribs();
};

LASReader::LASReader(const char *filename)
{
    if (laszip_create(&myReader))
    {
        UTdebugPrint("LASzip failed: laszip_create");
        myReader = nullptr;
        return;
    }

    if (laszip_open_reader(myReader, filename, &myIsCompressed))
    {
        UTdebugPrint("LASzip failed: laszip_open_reader_stream");
        destroyReader();
        return;
    }

    if (laszip_get_header_pointer(myReader, &myHeader))
    {
        UTdebugPrint("LASzip failed: laszip_get_header_pointer");
        closeReader();
        destroyReader();
        return;
    }

    if (laszip_get_point_pointer(myReader, &myPoint))
    {
        UTdebugPrint("LASzip failed: laszip_get_point_pointer");
        closeReader();
        destroyReader();
        return;
    }
   
    laszip_I64 npoints
            = (myHeader->number_of_point_records ?
                       myHeader->number_of_point_records :
                       myHeader->extended_number_of_point_records);
    myPointCount = static_cast<exint>(npoints);
    myPointFormat = static_cast<uint8>(myHeader->point_data_format);

    myScale.assign(
            myHeader->x_scale_factor, myHeader->y_scale_factor,
            myHeader->z_scale_factor);

    myOffset.assign(myHeader->x_offset, myHeader->y_offset, myHeader->z_offset);

    myFileBoundingBox.setBounds(
            myHeader->min_x, myHeader->min_y, myHeader->min_z, myHeader->max_x,
            myHeader->max_y, myHeader->max_z);

    // Ensure that the header was read correctly
    myPointFormat = myHeader->point_data_format;
    if (myPointFormat > 10)
    {
        closeReader();
        destroyReader();
    }

    initializeAvailableAttribs();
}

LASReader::~LASReader()
{
    closeReader();
    destroyReader();
}

void
LASReader::closeReader()
{
    if (myReader && laszip_close_reader(myReader))
        UTdebugPrint("LASzip failed: laszip_close_reader");
}

void
LASReader::destroyReader()
{
    if (myReader && laszip_destroy(myReader))
        UTdebugPrint("LASzip failed: laszip_destroy");
    myReader = nullptr;
}

void
LASReader::initializeAvailableAttribs()
{
    // LAS 1.4 point formats:
    if (myPointFormat > 5)
    {
        myHasClassFlagOverlap = true;
        myHasScannerChannel = true;
    }

    if (myPointFormat != 0 && myPointFormat != 2)
    {
        myHasGPSTime = true;
    }

    if (myPointFormat == 2 || myPointFormat == 3 || myPointFormat == 5
        || myPointFormat == 7 || myPointFormat == 8 || myPointFormat == 10)
    {
        myHasRGB = true;
    }

    if (myPointFormat == 8 || myPointFormat == 10)
    {
        myHasNIR = true;
    }
}

void
LASReader::getXYZ(UT_Vector3D &xyz) const
{
    xyz = UT_Vector3D(myPoint->X, myPoint->Y, myPoint->Z) * myScale + myOffset;
}

void
LASReader::getIntensity(fpreal32 &in) const
{
    in = static_cast<fpreal32>(myPoint->intensity) / UINT16_MAX;
}

void
LASReader::getUserData(uint8 &in) const
{
    in = myPoint->user_data;
}

void
LASReader::getPointSourceID(int32 &in) const
{
    in = (int32)myPoint->point_source_ID;
}

void
LASReader::getGPSTime(fpreal64 &in) const
{
    in = myPoint->gps_time;
}

void
LASReader::getRGB(UT_Vector3F &rgb) const
{
    rgb = UT_Vector3F(myPoint->rgb[0], myPoint->rgb[1], myPoint->rgb[2])
          / UINT16_MAX;
}

void
LASReader::getNIR(fpreal32 &in) const
{
    in = static_cast<fpreal32>(myPoint->rgb[3]) / UINT16_MAX;
}

void
LASReader::getReturnNumber(uint8 &in) const
{
    if (myPointFormat > 5)
        in = myPoint->extended_return_number;
    else
        in = myPoint->return_number;
}

void
LASReader::getReturnCount(uint8 &in) const
{
    if (myPointFormat > 5)
        in = myPoint->extended_number_of_returns;
    else
        in = myPoint->number_of_returns;
}

void
LASReader::getClassFlagSynthetic(uint8 &in) const
{
    if (myPointFormat > 5)
        in = myPoint->extended_classification_flags & 0x1;
    else
        in = myPoint->synthetic_flag;
}

void
LASReader::getClassFlagKeyPoint(uint8 &in) const
{
    if (myPointFormat > 5)
        in = (myPoint->extended_classification_flags & 0x2) >> 1;
    else
        in = myPoint->keypoint_flag;
}

void
LASReader::getClassFlagWithheld(uint8 &in) const
{
    if (myPointFormat > 5)
        in = (myPoint->extended_classification_flags & 0x4) >> 2;
    else
        in = myPoint->withheld_flag;
}

void
LASReader::getClassFlagOverlap(uint8 &in) const
{
    in = (myPoint->extended_classification_flags & 0x8) >> 3;
}

void
LASReader::getScannerChannel(uint8 &in) const
{
    in = myPoint->extended_scanner_channel;
}

void
LASReader::getScanDirectionFlag(uint8 &in) const
{
    in = myPoint->scan_direction_flag;
}

void
LASReader::getEdgeFlightLineFlag(uint8 &in) const
{
    in = myPoint->edge_of_flight_line;
}

void
LASReader::getClassIndex(uint8 &in) const
{
    if (myPointFormat > 5)
        in = myPoint->extended_classification;
    else
        in = myPoint->classification;
}

// This angle is in degrees.
void
LASReader::getScanAngle(fpreal32 &in) const
{
    if (myPointFormat > 5)
        in = (fpreal32)myPoint->extended_scan_angle * 0.006f;
    else
        in = (fpreal32)myPoint->scan_angle_rank;
}
} // namespace
//******************************************************************************
//*			          E57 READER                                   *
//******************************************************************************
namespace
{
struct sop_AvailableE57Attribs
{
    bool myHasCartesian;
    bool myHasCartesianInvalid;
    bool myHasSpherical;
    bool myHasSphericalInvalid;
    bool myHasColor;
    bool myHasColorInvalid;
    bool myHasIntensity;
    bool myHasIntensityInvalid;
    bool myHasRowCol;
    bool myHasRetData;
    bool myHasTimestamp;
    bool myHasTimestampInvalid;
    bool myHasNormals;
};
struct sop_ScanInfo
{
    e57::IntensityLimits	myIntensityLimits;
    e57::ColorLimits		myColorLimits;
    UT_Matrix4D			myRBXForm;
    UT_StringHolder		myGuid;
    exint			myNumPoints;
    UT_StringHolder             myName;
    UT_BoundingBoxD             myMetadataBox;

    sop_AvailableE57Attribs	myAvailableAttribs;
};

struct sop_ImageInfo
{
    typedef e57::PinholeRepresentation PinholeRep;
    typedef e57::SphericalRepresentation SphericalRep;
    typedef e57::CylindricalRepresentation CylindricalRep;

    union
    {
	PinholeRep		myPinholeRep;
	SphericalRep		mySphericalRep;
	CylindricalRep		myCylindricalRep;
    };
    UT_Matrix4D			myRBXForm;
    UT_StringHolder		myAssociatedScanGuid;
    e57::Image2DType		myFileType;
    e57::Image2DProjection	myProjectionType;
    exint			myWidth;
    exint			myHeight;
    size_t			mySize;
};

// The E57 CompressedVectorReader reads pages of data from the file and copies
// them into buffers. A sop_E57BufferInfo manages one of these buffers, and
// copies its data into its corresponding attribute's page buffer. It can also
// perform additional post-copy work on the attribute page data if required.
class sop_E57BufferInfo
{
public:
    virtual ~sop_E57BufferInfo() {}

    virtual e57::SourceDestBuffer createBuffer(e57::ImageFile handle, int comp) = 0;
    virtual void writeBlockToAttribute(int offset, GA_Offset start, GA_Offset end) = 0;
    virtual int getComps() const = 0;
};

template<typename T>
class sop_E57BufferInfoV3T : public sop_E57BufferInfo
{
    typedef std::function<void(T*, GA_Offset, GA_Offset)> CallbackType;
    static void theEmptyCallback(T*, GA_Offset, GA_Offset) {}
public:
    sop_E57BufferInfoV3T(const char **e57_names,
	    GA_Attribute *attrib,
	    GU_Detail *gdp,
	    CallbackType callback = theEmptyCallback)
	: sop_E57BufferInfo()
	, myE57AttribNames(e57_names)
	, myAttrib(attrib)
	, myGdp(gdp)
	, myPostCopyCallback(callback)
    {
	UT_ASSERT(myE57AttribNames.entries() == 3);
	myPageHandle.bind(myAttrib);
    }

    ~sop_E57BufferInfoV3T() override
    {
	if (myAttrib->getScope() == GA_SCOPE_PRIVATE)
	    myGdp->destroyPointAttrib(myAttrib->getFullName());
    }

    e57::SourceDestBuffer
    createBuffer(e57::ImageFile handle, int comp) override
    {
	return e57::SourceDestBuffer(handle, myE57AttribNames(comp).c_str(), 
	    myBuffer + comp, GA_PAGE_SIZE, true, true, 3 * sizeof(T));
    }

    void
    writeBlockToAttribute(int offset, GA_Offset start, GA_Offset end) override
    {
	myPageHandle.setPage(start);
	::memcpy((T *)&myPageHandle.value(start), myBuffer + offset*3, 
		sizeof(T)*(end-start)*3);
	myPostCopyCallback((T *)&myPageHandle.value(start), start, end);
    }

    int getComps() const override { return 3; }

private:
    typedef typename GA_PageHandleV< UT_Vector3T<T> >::RWType GA_RWPageHandle;

    const UT_StringArray     myE57AttribNames;
    GA_Attribute	    *myAttrib;
    GA_RWPageHandle	     myPageHandle;
    GU_Detail		    *myGdp;
    T			     myBuffer[GA_PAGE_SIZE*3];
    const CallbackType	     myPostCopyCallback;
};

template<typename T>
class sop_E57BufferInfoT : public sop_E57BufferInfo
{
    typedef std::function<void(T*, GA_Offset, GA_Offset)> CallbackType;
    static void theEmptyCallback(T*, GA_Offset, GA_Offset) {}
public:
    sop_E57BufferInfoT(const char *e57_name,
	    GA_Attribute *attrib,
	    GU_Detail *gdp,
	    CallbackType callback = theEmptyCallback)
	: sop_E57BufferInfo()
	, myE57AttribName(e57_name)
	, myAttrib(attrib)
	, myGdp(gdp)
	, myPostCopyCallback(callback)
    {
	myPageHandle.bind(myAttrib);
    }

    ~sop_E57BufferInfoT() override
    {
	if (myAttrib->getScope() == GA_SCOPE_PRIVATE)
	    myGdp->destroyPointAttrib(myAttrib->getFullName());
    }

    e57::SourceDestBuffer
    createBuffer(e57::ImageFile handle, int) override
    {
	return e57::SourceDestBuffer(handle, myE57AttribName.c_str(), 
	    myBuffer, GA_PAGE_SIZE, true, true);
    }

    void
    writeBlockToAttribute(int offset, GA_Offset start, GA_Offset end) override
    {
	myPageHandle.setPage(start);
	::memcpy(&myPageHandle.value(start), myBuffer + offset, sizeof(T)*(end-start));
	myPostCopyCallback(&myPageHandle.value(start), start, end);
    }

    int getComps() const override { return 1; }

private:
    typedef typename GA_PageHandleScalar<T>::RWType GA_RWPageHandle;

    const UT_StringHolder    myE57AttribName;
    GA_Attribute	    *myAttrib;
    GA_RWPageHandle	     myPageHandle;
    GU_Detail		    *myGdp;
    T			     myBuffer[GA_PAGE_SIZE];
    const CallbackType	     myPostCopyCallback;
};

// This class owns several sop_E57BufferInfos that are created on request.
// It controls when they receive data from the E57 reader, as well as which 
// portion of their buffers must be copied to the corresponding attributes.
class sop_E57PointReader
{
    typedef UT_Array<sop_E57BufferInfo*> BufferInfoArray;
    typedef e57::CompressedVectorReader CVReader;
public:
    class Builder
    {
    public:
	Builder(e57::ImageFile handle, int idx, sop_ScanInfo info, GU_Detail *gdp, SOP_LidarImportCache* cache,
		int range_good, int range_size);
	virtual ~Builder();

	void cartesianPosition();
	void cartesianPositionInvalid();
	void sphericalPosition();
	void sphericalPositionInvalid();
	void color();
	void colorInvalid();
	void intensity();
	void intensityInvalid();
	void rowIndex();
	void columnIndex();
	void returnIndex();
	void returnCount();
	void timestamp();
	void timestampInvalid();
	void normals();

	sop_E57PointReader build();
    private:
	e57::ImageFile myFileHandle;
	BufferInfoArray myBufferInfos;
	const sop_ScanInfo myScanInfo;
	const int myScanIndex;
	GU_Detail *myGdp;
        SOP_LidarImportCache *myCache;
	const int myRangeGood;
	const int myRangeSize;

	bool myHasBuiltReader;
    };

    sop_E57PointReader(CVReader reader, BufferInfoArray &buf_infos,
		       e57::ImageFile handle, int range_good, int range_size);
    virtual ~sop_E57PointReader();

    bool readBlock(GA_Offset start, GA_Offset end);
private:
    CVReader myReader;
    BufferInfoArray myBufferInfos;
    e57::ImageFile myFileHandle;
    int myBlockPos;
    int myBlockSize;
    exint myPointRecordPos;
    int myRangePos;
    const int myRangeGood;
    const int myRangeSize;
};

sop_E57PointReader::Builder::Builder(
        e57::ImageFile handle,
        int idx,
        sop_ScanInfo info,
        GU_Detail *gdp,
        SOP_LidarImportCache *cache,
        int range_good,
        int range_size)
    : myFileHandle(handle)
    , myScanInfo(info)
    , myScanIndex(idx)
    , myGdp(gdp)
    , myCache(cache)
    , myRangeGood(range_good)
    , myRangeSize(range_size)
    , myHasBuiltReader(false)
{}

sop_E57PointReader::Builder::~Builder()
{
    if (!myHasBuiltReader)
	for (int i = 0, ni = myBufferInfos.entries(); i < ni; ++i)
	    delete myBufferInfos(i);
}

void
sop_E57PointReader::Builder::cartesianPosition()
{
    const char *e57_names[] = { "cartesianX", "cartesianY", "cartesianZ", 
	nullptr };

    myBufferInfos.append(new sop_E57BufferInfoV3T<fpreal>(e57_names, myCache->getDetachedPosition(),
		myGdp));
}

void
sop_E57PointReader::Builder::cartesianPositionInvalid()
{
    GA_Attribute *attrib = myGdp->addIntTuple(GA_ATTRIB_POINT, GA_SCOPE_PRIVATE,
	    "__invalid_pos__", 1);

    GU_Detail *gdp = myGdp;
    auto callback = [gdp](int32 *b, GA_Offset start, GA_Offset end)
    {
	GA_PointGroup *invalid_pos_group = gdp->findPointGroup("e57_invalid_pos");
	if (!invalid_pos_group)
	    invalid_pos_group = gdp->newPointGroup("e57_invalid_pos");

	for (int i = 0, ni = end - start; i < ni; ++i)
	    if (b[i])
		invalid_pos_group->addOffset(GA_Offset(start + i));
    };

    myBufferInfos.append(new sop_E57BufferInfoT<int32>("cartesianInvalidState",
	       	attrib, myGdp, callback));
}

void
sop_E57PointReader::Builder::sphericalPosition()
{
    const char *e57_names[] = 
	{ "sphericalRange", "sphericalAzimuth", "sphericalElevation", nullptr };

    auto callback = [](fpreal *b, GA_Offset start, GA_Offset end)
    {
	for (int i = 0, ni = end - start; i < ni; ++i)
	{
	    fpreal range = b[i*3];
	    fpreal azimuth = b[i*3+1];
	    fpreal elevation = b[i*3+2];

	    b[i*3] = range * SYScos(elevation) * SYScos(azimuth);
	    b[i*3+1] = range * SYScos(elevation) * SYSsin(azimuth);
	    b[i*3+2] = range * SYSsin(elevation);
	}
    };

    myBufferInfos.append(new sop_E57BufferInfoV3T<fpreal>(
            e57_names, myCache->getDetachedPosition(), myGdp, callback));
}

void
sop_E57PointReader::Builder::sphericalPositionInvalid()
{
    GA_Attribute *attrib = myGdp->addIntTuple(GA_ATTRIB_POINT, GA_SCOPE_PRIVATE,
	    "__invalid_pos__", 1);

    GU_Detail *gdp = myGdp;
    auto callback = [gdp](int32 *b, GA_Offset start, GA_Offset end)
    {
	GA_PointGroup *invalid_pos_group = gdp->findPointGroup("e57_invalid_pos");
	if (!invalid_pos_group)
	    invalid_pos_group = gdp->newPointGroup("e57_invalid_pos");

	for (int i = 0, ni = end - start; i < ni; ++i)
	    if (b[i])
		invalid_pos_group->addOffset(GA_Offset(start + i));
    };

    myBufferInfos.append(new sop_E57BufferInfoT<int32>("sphericalInvalidState", 
		attrib, myGdp, callback));
}

void
sop_E57PointReader::Builder::color()
{
    GA_Attribute *attrib = myGdp->addDiffuseAttribute(GA_ATTRIB_POINT);
    const char *e57_names[] = { "colorRed", "colorGreen", "colorBlue", nullptr};

    const e57::ColorLimits &lim = myScanInfo.myColorLimits;
    auto callback = [lim](fpreal32 *b, GA_Offset start, GA_Offset end)
    {
	for (int i = 0, ni = end - start; i < ni; ++i)
	{
	    b[i*3] = (b[i*3] - lim.colorRedMinimum) 
		/ (lim.colorRedMaximum - lim.colorRedMinimum); 
	    b[i*3+1] = (b[i*3+1] - lim.colorGreenMinimum) 
		/ (lim.colorGreenMaximum - lim.colorGreenMinimum); 
	    b[i*3+2] = (b[i*3+2] - lim.colorBlueMinimum) 
		/ (lim.colorBlueMaximum - lim.colorBlueMinimum); 
	}
    };

    myBufferInfos.append(new sop_E57BufferInfoV3T<fpreal32>(e57_names, attrib,
		myGdp, callback));
}

void
sop_E57PointReader::Builder::colorInvalid()
{
    GA_Attribute *attrib = myGdp->addIntTuple(GA_ATTRIB_POINT, GA_SCOPE_PRIVATE,
	    "__invalid_color__", 1);

    GU_Detail *gdp = myGdp;
    auto callback = [gdp](int32 *b, GA_Offset start, GA_Offset end)
    {
	GA_PointGroup *invalid_color_group = gdp->findPointGroup("e57_invalid_color");
	if (!invalid_color_group)
	    invalid_color_group = gdp->newPointGroup("e57_invalid_color");

	for (int i = 0, ni = end - start; i < ni; ++i)
	    if (b[i])
		invalid_color_group->addOffset(GA_Offset(start + i));
    };

    myBufferInfos.append(new sop_E57BufferInfoT<int32>("isColorInvalid", 
		attrib, myGdp, callback));
}

void
sop_E57PointReader::Builder::intensity()
{
    GA_Attribute *attrib = myGdp->addFloatTuple(GA_ATTRIB_POINT, "intensity", 1);

    const e57::IntensityLimits &lim = myScanInfo.myIntensityLimits;
    auto callback = [lim](fpreal32 *b, GA_Offset start, GA_Offset end)
    {
	for (int i = 0, ni = end - start; i < ni; ++i)
	{
	    b[i] = (b[i] - lim.intensityMinimum) 
		/ (lim.intensityMaximum - lim.intensityMinimum); 
	}
    };

    myBufferInfos.append(new sop_E57BufferInfoT<fpreal32>("intensity", 
		attrib, myGdp, callback));
}

void
sop_E57PointReader::Builder::intensityInvalid()
{
    GA_Attribute *attrib = myGdp->addIntTuple(GA_ATTRIB_POINT, GA_SCOPE_PRIVATE,
	    "__invalid_intensity__", 1);

    GU_Detail *gdp = myGdp;
    auto callback = [gdp](int32 *b, GA_Offset start, GA_Offset end)
    {
	GA_PointGroup *invalid_intensity_group = 
		gdp->findPointGroup("e57_invalid_intensity");
	if (!invalid_intensity_group)
	    invalid_intensity_group = gdp->newPointGroup("e57_invalid_intensity");

	for (int i = 0, ni = end - start; i < ni; ++i)
	    if (b[i])
		invalid_intensity_group->addOffset(GA_Offset(start + i));
    };

    myBufferInfos.append(new sop_E57BufferInfoT<int32>("isIntensityInvalid", 
		attrib, myGdp, callback));
}

void
sop_E57PointReader::Builder::rowIndex()
{
    GA_Attribute *attrib = myGdp->addIntTuple(GA_ATTRIB_POINT, "row_index", 1);

    myBufferInfos.append(new sop_E57BufferInfoT<int32>("rowIndex", attrib, 
		myGdp));
}

void
sop_E57PointReader::Builder::columnIndex()
{
    GA_Attribute *attrib = myGdp->addIntTuple(GA_ATTRIB_POINT, "column_index", 1);

    myBufferInfos.append(new sop_E57BufferInfoT<int32>("columnIndex", attrib, 
		myGdp));
}

void
sop_E57PointReader::Builder::returnIndex()
{
    GA_Attribute *attrib = myGdp->addIntTuple(GA_ATTRIB_POINT, "return_index", 1);

    myBufferInfos.append(new sop_E57BufferInfoT<int32>("returnIndex", attrib, 
		myGdp));
}

void
sop_E57PointReader::Builder::returnCount()
{
    GA_Attribute *attrib = myGdp->addIntTuple(GA_ATTRIB_POINT, "return_count", 1);

    myBufferInfos.append(new sop_E57BufferInfoT<int32>("returnCount", attrib, 
		myGdp));
}

void
sop_E57PointReader::Builder::timestamp()
{
    GA_Attribute *attrib = myGdp->addFloatTuple(
            GA_ATTRIB_POINT, "timestamp", 1, GA_Defaults(), nullptr, nullptr,
            GA_STORE_REAL64);

    myBufferInfos.append(new sop_E57BufferInfoT<fpreal64>("timeStamp", attrib, 
		myGdp));
}

void
sop_E57PointReader::Builder::timestampInvalid()
{
    GA_Attribute *attrib = myGdp->addIntTuple(GA_ATTRIB_POINT, GA_SCOPE_PRIVATE,
	    "__invalid_timestamp__", 1);

    GU_Detail *gdp = myGdp;
    auto callback = [gdp](int32 *b, GA_Offset start, GA_Offset end)
    {
	GA_PointGroup *invalid_timestamp_group = 
		gdp->findPointGroup("e57_invalid_timestamp");
	if (!invalid_timestamp_group)
	    invalid_timestamp_group = gdp->newPointGroup("e57_invalid_timestamp");

	for (int i = 0, ni = end - start; i < ni; ++i)
	    if (b[i])
		invalid_timestamp_group->addOffset(GA_Offset(start + i));
    };

    myBufferInfos.append(new sop_E57BufferInfoT<int32>("isTimeStampInvalid", 
		attrib, myGdp, callback));
}

void
sop_E57PointReader::Builder::normals()
{
    myGdp->addNormalAttribute(GA_ATTRIB_POINT);
    const char *e57_names[] = { "nor:normalX", "nor:normalY", "nor:normalZ", 
	nullptr };

    myBufferInfos.append(new sop_E57BufferInfoV3T<fpreal32>(
            e57_names, myCache->getDetachedNormals(), myGdp));
}

sop_E57PointReader
sop_E57PointReader::Builder::build()
{
    e57::VectorNode scan_root_node(myFileHandle.root().get("/data3D"));
    e57::StructureNode scan_node(scan_root_node.get(myScanIndex));
    e57::CompressedVectorNode pts_node(scan_node.get("points"));

    std::vector<e57::SourceDestBuffer> bufs;
    for (int i = 0, ni = myBufferInfos.entries(); i < ni; ++i)
	for (int j = 0, nj = myBufferInfos(i)->getComps(); j < nj; ++j)
	    bufs.push_back(myBufferInfos(i)->createBuffer(myFileHandle, j));

    e57::CompressedVectorReader cv_reader(pts_node.reader(bufs));

    myHasBuiltReader = true;
    
    return sop_E57PointReader(cv_reader, myBufferInfos, myFileHandle, 
			      myRangeGood, myRangeSize);
}

sop_E57PointReader::sop_E57PointReader(CVReader reader, 
	BufferInfoArray &buf_infos, e57::ImageFile handle,
	int range_good, int range_size)
    : myReader(reader)
    , myBufferInfos(buf_infos)
    , myFileHandle(handle)
    , myBlockPos(0)
    , myBlockSize(0)
    , myPointRecordPos(0)
    , myRangePos(0)
    , myRangeGood(range_good)
    , myRangeSize(range_size)
{}

sop_E57PointReader::~sop_E57PointReader()
{
    for (int i = 0, ni = myBufferInfos.entries(); i < ni; ++i)
	delete myBufferInfos(i);
}

bool
sop_E57PointReader::readBlock(GA_Offset start, GA_Offset end)
{
    // Unfortunately, e57::CompressedVectorReader::seek() isn't implemented yet,
    // so we read the entire file and ignore the points we don't want.
    while (start < end)
    {
	UT_ASSERT(myBlockPos <= myBlockSize && myBlockPos >= 0);

	if (myBlockPos < myBlockSize)
	{
	    // We have uncopied data in the intermediary buffers that's in the "good"
	    // part of the range interval, so we just need to write it to the attribute
	    // page buffers. We may need to stop before the end of the range, if we
	    // reach either the end of the intermediary buffers or the end of the 
	    // attribute page buffers first.
	    if (myRangePos < myRangeGood)
	    {
		int rem_block_pts = myBlockSize - myBlockPos;
		int rem_good_range_pts = myRangeGood - myRangePos;
		int unfilled_page_pts = end - start;
		int copy_chunk_size = 
		    SYSmin(rem_block_pts, rem_good_range_pts, unfilled_page_pts);

		for (int i = 0, ni = myBufferInfos.entries(); i < ni; ++i)
		    myBufferInfos(i)->writeBlockToAttribute(
			    myBlockPos, start, GA_Offset(start + copy_chunk_size));

		start += copy_chunk_size;

		myBlockPos += copy_chunk_size;
		myPointRecordPos += copy_chunk_size;
		myRangePos += copy_chunk_size;
	    }
	    // We have uncopied data, but it's in the "bad" part of the range
	    // interval. We either move to the beginning of the next range interval,
	    // or to 1 offset after the end of the intermediary buffers (if there
	    // are no uncopied range intervals left to copy) - this will facilitate
	    // refilling the intermediary buffers on the next loop iteration.
	    else
	    {
		int rem_block_pts = myBlockSize - myBlockPos;
		int rem_bad_range_pts = myRangeSize - myRangePos;
		int skip_chunk_size = SYSmin(rem_block_pts, rem_bad_range_pts);

		myBlockPos += skip_chunk_size;
		myPointRecordPos += skip_chunk_size;
		myRangePos += skip_chunk_size;
		myRangePos %= myRangeSize;
	    }
	}
	// We have no more uncopied data left in the intermediary buffers, so we
	// read a page from the file to refill them.
	else
	{
	    myBlockSize = myReader.read();
	    if (myBlockSize == 0)
		return false;

	    myBlockPos = 0;
	}
    }

    return true;
}

exint
getNodeValueI(e57::Node node)
{
    UT_ASSERT(node.type() == e57::E57_INTEGER);
    return e57::IntegerNode(node).value();
}

exint
getNodeMinimumI(e57::Node node)
{
    UT_ASSERT(node.type() == e57::E57_INTEGER);
    return e57::IntegerNode(node).minimum();
}

exint
getNodeMaximumI(e57::Node node)
{
    UT_ASSERT(node.type() == e57::E57_INTEGER);
    return e57::IntegerNode(node).maximum();
}

fpreal64
getNodeValueF(e57::Node node)
{
    UT_ASSERT(node.type() == e57::E57_FLOAT 
	    || node.type() == e57::E57_SCALED_INTEGER
	    || node.type() == e57::E57_INTEGER);
    if (node.type() == e57::E57_FLOAT)
	return e57::FloatNode(node).value();
    else if (node.type() == e57::E57_SCALED_INTEGER)
	return e57::ScaledIntegerNode(node).scaledValue();
    else
	return e57::IntegerNode(node).value();
}

fpreal64
getNodeMinimumF(e57::Node node)
{
    UT_ASSERT(node.type() == e57::E57_FLOAT 
	    || node.type() == e57::E57_SCALED_INTEGER
	    || node.type() == e57::E57_INTEGER);
    if (node.type() == e57::E57_FLOAT)
	return e57::FloatNode(node).minimum();
    else if (node.type() == e57::E57_SCALED_INTEGER)
	return e57::ScaledIntegerNode(node).scaledMinimum();
    else
	return e57::IntegerNode(node).minimum();
}

fpreal64
getNodeMaximumF(e57::Node node)
{
    UT_ASSERT(node.type() == e57::E57_FLOAT 
	    || node.type() == e57::E57_SCALED_INTEGER
	    || node.type() == e57::E57_INTEGER);
    if (node.type() == e57::E57_FLOAT)
	return e57::FloatNode(node).maximum();
    else if (node.type() == e57::E57_SCALED_INTEGER)
	return e57::ScaledIntegerNode(node).scaledMaximum();
    else
	return e57::IntegerNode(node).maximum();
}

class E57Reader
{
public:

    E57Reader(const char *filename);
    virtual ~E57Reader() {}

    int getNumScans() const { return myScans.entries(); }
    int getNumImages() const { return myImages.entries(); }
    exint getPointsInFile() const { return myPointsInFile; }
    exint getPointsInScan(int idx) const { return myScans(idx).myNumPoints; }
    const UT_BoundingBoxD &getScanBoundingBox(int idx) const { return myScans(idx).myMetadataBox; }
    const UT_BoundingBoxD &getFileBoundingBox() const { return myFileBoundingBox; }
    UT_Matrix4D getScanRBXForm(int idx) const { return myScans(idx).myRBXForm; }
    UT_Matrix4D getImageRBXForm(int idx) const { return myImages(idx).myRBXForm; }

    const UT_StringHolder &getScanName(int idx) const
    { return myScans(idx).myName; }

    sop_E57PointReader::Builder createPointReaderBuilder(
            int idx,
            GU_Detail *gdp,
            SOP_LidarImportCache *cache,
            int range_good,
            int range_size);

    UT_UniquePtr<unsigned char[]> getImageBuffer(int idx,
                                                 bool read_ref_images) const;

    void getImageDimensions(int idx, exint &width, exint &height) const;
    size_t getImageSize(int idx) const;
    bool hasValidProjectionType(int idx) const;

    const UT_StringHolder &getScanGroupGuid(int idx) const;
    const UT_StringHolder &getImageAssociatedScanGuid(int idx) const;

    void computePointProjection(int idx, const UT_Vector4D pos, 
	    exint &img_x, exint &img_y) const;

    bool hasCartesian(int idx) const
    { return myScans(idx).myAvailableAttribs.myHasCartesian; }
    bool hasCartesianInvalid(int idx) const
    { return myScans(idx).myAvailableAttribs.myHasCartesianInvalid; }
    bool hasSpherical(int idx) const
    { return myScans(idx).myAvailableAttribs.myHasSpherical; }
    bool hasSphericalInvalid(int idx) const
    { return myScans(idx).myAvailableAttribs.myHasSphericalInvalid; }
    bool hasColor(int idx) const
    { return myScans(idx).myAvailableAttribs.myHasColor; }
    bool hasColorInvalid(int idx) const
    { return myScans(idx).myAvailableAttribs.myHasColorInvalid; }
    bool hasIntensity(int idx) const
    { return myScans(idx).myAvailableAttribs.myHasIntensity; }
    bool hasIntensityInvalid(int idx) const
    { return myScans(idx).myAvailableAttribs.myHasIntensityInvalid; }
    bool hasRowCol(int idx) const
    { return myScans(idx).myAvailableAttribs.myHasRowCol; }
    bool hasRetData(int idx) const
    { return myScans(idx).myAvailableAttribs.myHasRetData; }
    bool hasTimestamp(int idx) const
    { return myScans(idx).myAvailableAttribs.myHasTimestamp; }
    bool hasTimestampInvalid(int idx) const
    { return myScans(idx).myAvailableAttribs.myHasTimestampInvalid; }
    bool hasNormals(int idx) const
    { return myScans(idx).myAvailableAttribs.myHasNormals; }
private:
    e57::ImageFile myFileHandle;

    UT_Array<sop_ScanInfo> myScans;
    UT_Array<sop_ImageInfo> myImages;

    exint myPointsInFile;
    UT_BoundingBoxD myFileBoundingBox;
};

E57Reader::E57Reader(const char *filename)
    : myFileHandle(filename, "r")
{
    // NOTE: some of this code may throw an exception; it should be caught in cookMySop
    e57::StructureNode root_node = myFileHandle.root();
    e57::VectorNode scan_root_node(root_node.get("/data3D"));

    auto set_rbxform = [](e57::StructureNode &parent, UT_Matrix4D &rbxform)
    {
	if (parent.isDefined("pose"))
	{
	    e57::StructureNode rbxform_node(parent.get("pose"));

	    fpreal64 rw, rx, ry, rz;
	    if (rbxform_node.isDefined("rotation"))
	    {
		e57::StructureNode rot_node(rbxform_node.get("rotation"));
		rw = getNodeValueF(rot_node.get("w"));
		rx = getNodeValueF(rot_node.get("x"));
		ry = getNodeValueF(rot_node.get("y"));
		rz = getNodeValueF(rot_node.get("z"));
	    }
	    else
	    {
		rx = ry = rz = 0;
		rw = 1.0;
	    }

	    fpreal64 tx, ty, tz;
	    if (rbxform_node.isDefined("translation"))
	    {
		e57::StructureNode trans_node(rbxform_node.get("translation"));
		tx = getNodeValueF(trans_node.get("x"));
		ty = getNodeValueF(trans_node.get("y"));
		tz = getNodeValueF(trans_node.get("z"));
	    }
	    else
	    {
		tx = ty = tz = 0;
	    }

	    UT_QuaternionD rotation(rx, ry, rz, rw);
	    UT_Vector3D translation(tx, ty, tz);

	    rotation.getTransformMatrix(rbxform);
	    rbxform.setTranslates(translation);
	}
	else
	{
	    rbxform.identity();
	}
    };

    auto set_boundingbox = [](e57::StructureNode &parent,
                              UT_BoundingBoxD &bbox_out) {
        // Note: the bounding box is in local coordinates. It should be
        // transformed into the file-level coordinate system with myRBXForm.
        fpreal x_min, y_min, z_min, x_max, y_max, z_max;
        if (parent.isDefined("cartesianBounds"))
        {
            e57::StructureNode bbox(parent.get("cartesianBounds"));
            x_min = getNodeValueF(bbox.get("xMinimum"));
            x_max = getNodeValueF(bbox.get("xMaximum"));
            y_min = getNodeValueF(bbox.get("yMinimum"));
            y_max = getNodeValueF(bbox.get("yMaximum"));
            z_min = getNodeValueF(bbox.get("zMinimum"));
            z_max = getNodeValueF(bbox.get("zMaximum"));

            bbox_out.setBounds(x_min, y_min, z_min, x_max, y_max, z_max);
        }
        else if (parent.isDefined("sphericalBounds"))
        {
            e57::StructureNode sbox(parent.get("sphericalBounds"));

            fpreal range_min, range_max, elevation_min, elevation_max,
                    azimuth_start, azimuth_end;

            range_min = getNodeValueF(sbox.get("rangeMinimum"));
            range_max = getNodeValueF(sbox.get("rangeMaximum"));
            elevation_min = getNodeValueF(sbox.get("elevationMinimum"));
            elevation_max = getNodeValueF(sbox.get("elevationMaximum"));
            azimuth_start = getNodeValueF(sbox.get("azimuthStart"));
            azimuth_end = getNodeValueF(sbox.get("azimuthEnd"));

            // The standard says that azimuth end > start, but for safety:
            fpreal azimuth_min = SYSmin(azimuth_start, azimuth_end);
            fpreal azimuth_max = SYSmax(azimuth_start, azimuth_end);

            fpreal cos_elevation_min = SYScos(elevation_min);
            fpreal cos_elevation_max = SYScos(elevation_max);

            x_min = range_min * cos_elevation_min * SYScos(azimuth_min);
            x_max = range_max * cos_elevation_max * SYScos(azimuth_max);
            y_min = range_min * cos_elevation_min * SYSsin(azimuth_min);
            y_max = range_max * cos_elevation_max * SYSsin(azimuth_max);
            z_min = range_min * SYSsin(elevation_min);
            z_max = range_max * SYSsin(elevation_max);

            bbox_out.setBounds(x_min, y_min, z_min, x_max, y_max, z_max);
        }
    };

    int num_scans = scan_root_node.childCount();
    myScans.setCapacity(num_scans);
    myPointsInFile = 0;

    for (int i = 0; i < num_scans; i++)
    {
	e57::StructureNode scan_node(scan_root_node.get(i));
	sop_ScanInfo scan;

	// set num points
	e57::CompressedVectorNode pts_node(scan_node.get("points"));
	scan.myNumPoints = pts_node.childCount();
	myPointsInFile += scan.myNumPoints;

	// set available attribute flags
	e57::StructureNode proto_node(pts_node.prototype());
	sop_AvailableE57Attribs available_attribs;

	available_attribs.myHasCartesian = proto_node.isDefined("cartesianX")
	    && proto_node.isDefined("cartesianY")
	    && proto_node.isDefined("cartesianZ");
	available_attribs.myHasCartesianInvalid = 
	    proto_node.isDefined("cartesianInvalidState");
	available_attribs.myHasSpherical = proto_node.isDefined("sphericalRange")
	    && proto_node.isDefined("sphericalAzimuth")
	    && proto_node.isDefined("sphericalElevation");
	available_attribs.myHasSphericalInvalid =
	    proto_node.isDefined("sphericalInvalidState");
	available_attribs.myHasColor = proto_node.isDefined("colorRed")
	    && proto_node.isDefined("colorGreen")
	    && proto_node.isDefined("colorBlue");
	available_attribs.myHasColorInvalid = 
	    proto_node.isDefined("isColorInvalid");
	available_attribs.myHasIntensity = proto_node.isDefined("intensity");
	available_attribs.myHasIntensityInvalid = 
	    proto_node.isDefined("isIntensityInvalid");
	available_attribs.myHasRowCol = proto_node.isDefined("rowIndex")
	    && proto_node.isDefined("columnIndex");
	available_attribs.myHasRetData = proto_node.isDefined("returnIndex")
	    && proto_node.isDefined("returnCount");
	available_attribs.myHasTimestamp = proto_node.isDefined("timestamp");
	available_attribs.myHasTimestampInvalid = 
	    proto_node.isDefined("isTimestampInvalid");

	try
	{
	    available_attribs.myHasNormals = proto_node.isDefined("nor:normalX")
		&& proto_node.isDefined("nor:normalY")
		&& proto_node.isDefined("nor:normalZ");
	}
	catch(e57::E57Exception &)
	{
	    available_attribs.myHasNormals = false;
	}

        scan.myAvailableAttribs = available_attribs;

	// set intensity limits
	if (scan_node.isDefined("intensityLimits"))
	{
	    e57::StructureNode intensity_lim_node(scan_node.get("intensityLimits"));
	    e57::IntensityLimits intensity_lim;

	    intensity_lim.intensityMinimum = 
		getNodeValueF(intensity_lim_node.get("intensityMinimum"));
	    intensity_lim.intensityMaximum = 
		getNodeValueF(intensity_lim_node.get("intensityMaximum"));

	    scan.myIntensityLimits = intensity_lim;
	}
	else if (available_attribs.myHasIntensity)
	{
	    e57::IntensityLimits intensity_lim;

	    intensity_lim.intensityMinimum = 
		getNodeMinimumF(proto_node.get("intensity"));
	    intensity_lim.intensityMaximum = 
		getNodeMaximumF(proto_node.get("intensity"));

	    scan.myIntensityLimits = intensity_lim;
	}

	// set color limits
	if (scan_node.isDefined("colorLimits"))
	{
	    e57::StructureNode color_lim_node(scan_node.get("colorLimits"));
	    e57::ColorLimits color_lim;

	    color_lim.colorRedMinimum = 
		getNodeValueF(color_lim_node.get("colorRedMinimum"));
	    color_lim.colorRedMaximum = 
		getNodeValueF(color_lim_node.get("colorRedMaximum"));
	    color_lim.colorGreenMinimum = 
		getNodeValueF(color_lim_node.get("colorGreenMinimum"));
	    color_lim.colorGreenMaximum = 
		getNodeValueF(color_lim_node.get("colorGreenMaximum"));
	    color_lim.colorBlueMinimum = 
		getNodeValueF(color_lim_node.get("colorBlueMinimum"));
	    color_lim.colorBlueMaximum = 
		getNodeValueF(color_lim_node.get("colorBlueMaximum"));

	    scan.myColorLimits = color_lim;
	}
	else if (available_attribs.myHasColor)
	{
	    e57::ColorLimits color_lim;

	    color_lim.colorRedMinimum = 
		getNodeMinimumF(proto_node.get("colorRed"));
	    color_lim.colorRedMaximum = 
		getNodeMaximumF(proto_node.get("colorRed"));
	    color_lim.colorGreenMinimum = 
		getNodeMinimumF(proto_node.get("colorGreen"));
	    color_lim.colorGreenMaximum = 
		getNodeMaximumF(proto_node.get("colorGreen"));
	    color_lim.colorBlueMinimum = 
		getNodeMinimumF(proto_node.get("colorBlue"));
	    color_lim.colorBlueMaximum = 
		getNodeMaximumF(proto_node.get("colorBlue"));

	    scan.myColorLimits = color_lim;
	}

	// set rigid body transform
	set_rbxform(scan_node, scan.myRBXForm);

	// set guid
	UT_String guid_name_wrapper;
	try
	{
	    e57::StringNode scanpos_node(scan_node.get("rlms:scanposGuid"));
	    guid_name_wrapper = UT_String(scanpos_node.value().c_str(), true);
	}
	catch (e57::E57Exception &)
	{
	    e57::StringNode guid_node(scan_node.get("guid"));
	    guid_name_wrapper = UT_String(guid_node.value().c_str(), true);
	}
	guid_name_wrapper.forceValidVariableName();

	scan.myGuid = UT_StringHolder(guid_name_wrapper.c_str());

        // Set name if available, else leave as empty string
	UT_StringHolder scan_name;
	try
	{
	    e57::StringNode name_node(scan_node.get("name"));
	    scan.myName = name_node.value();
	}
	catch (e57::E57Exception &)
	{
	    scan.myName.clear();
	}

        // Get the scan bounding box in local coordinates, and move it to the
	// file coordinate system.
        UT_BoundingBoxD bbox;
        set_boundingbox(scan_node, bbox);
        if (!scan.myRBXForm.isIdentity())
        {
            bbox.transform(scan.myRBXForm);
        }
        scan.myMetadataBox = bbox;

	myScans.append(scan);
    }

    // Get the total bounds of the file
    myFileBoundingBox.initBounds();
    for (int i = 0; i < num_scans; i++)
    {
        myFileBoundingBox.enlargeBounds(myScans(i).myMetadataBox);
    }

    if (!root_node.isDefined("/images2D"))
	return;

    e57::VectorNode image_root_node(root_node.get("/images2D"));

    int num_images = image_root_node.childCount();
    myImages.setCapacity(num_images);

    auto set_image_sizes = [](e57::StructureNode proj_model, sop_ImageInfo &image)
    {
	image.myWidth = getNodeValueI(proj_model.get("imageWidth"));
	image.myHeight = getNodeValueI(proj_model.get("imageHeight"));

	// depending on image type, set size and blob
	if (proj_model.isDefined("pngImage"))
	{
	    e57::BlobNode png_node(proj_model.get("pngImage"));

	    image.myFileType = e57::Image2DType::E57_PNG_IMAGE;
	    image.mySize = png_node.byteCount();
	}
	else if (proj_model.isDefined("jpegImage"))
	{
	    e57::BlobNode jpeg_node(proj_model.get("jpegImage"));

	    image.myFileType = e57::Image2DType::E57_JPEG_IMAGE;
	    image.mySize = jpeg_node.byteCount();
	}
	else
	{
	    image.myFileType = e57::Image2DType::E57_NO_IMAGE;
	    image.mySize = 0;
	}
    };

    for (int i = 0; i < num_images; i++)
    {
	e57::StructureNode image_node(image_root_node.get(i));
	sop_ImageInfo image;

	if (image_node.isDefined("pinholeRepresentation"))
	{
	    // set projection model type
	    image.myProjectionType = e57::Image2DProjection::E57_PINHOLE;

	    e57::StructureNode pinhole_model(
		    image_node.get("pinholeRepresentation"));

	    // get projection model values
	    e57::PinholeRepresentation pinhole_rep;
	    pinhole_rep.focalLength =
		getNodeValueF(pinhole_model.get("focalLength"));
	    pinhole_rep.pixelWidth =
		getNodeValueF(pinhole_model.get("pixelWidth"));
	    pinhole_rep.pixelHeight =
		getNodeValueF(pinhole_model.get("pixelHeight"));
	    pinhole_rep.principalPointX =
		getNodeValueF(pinhole_model.get("principalPointX"));
	    pinhole_rep.principalPointY =
		getNodeValueF(pinhole_model.get("principalPointY"));

	    image.myPinholeRep = pinhole_rep;

	    // set image sizes (dimensions, byte size, file type)
	    set_image_sizes(pinhole_model, image);
	}
	else if (image_node.isDefined("sphericalRepresentation"))
	{
	    // set projection model type
	    image.myProjectionType = e57::Image2DProjection::E57_SPHERICAL;

	    e57::StructureNode spherical_model(
		    image_node.get("sphericalRepresentation"));

	    // set projection model values
	    e57::SphericalRepresentation sphere_rep;
	    sphere_rep.pixelWidth =
		getNodeValueF(spherical_model.get("pixelWidth"));
	    sphere_rep.pixelHeight =
		getNodeValueF(spherical_model.get("pixelHeight"));

	    image.mySphericalRep = sphere_rep;

	    // set image sizes (dimensions, byte size, file type)
	    set_image_sizes(spherical_model, image);
	}
	else if (image_node.isDefined("cylindricalRepresentation"))
	{
	    // set projection model type
	    image.myProjectionType = e57::Image2DProjection::E57_CYLINDRICAL;

	    e57::StructureNode cylindrical_model(
		    image_node.get("cylindricalRepresentation"));

	    // set projection model values
	    e57::CylindricalRepresentation cyl_rep;
	    cyl_rep.pixelWidth =
		getNodeValueF(cylindrical_model.get("pixelWidth"));
	    cyl_rep.pixelHeight =
		getNodeValueF(cylindrical_model.get("pixelHeight"));
	    cyl_rep.radius =
		getNodeValueF(cylindrical_model.get("radius"));
	    cyl_rep.principalPointY =
		getNodeValueF(cylindrical_model.get("principalPointY"));

	    image.myCylindricalRep = cyl_rep;

	    // set image sizes (dimensions, byte size, file type)
	    set_image_sizes(cylindrical_model, image);
	}
	else if (image_node.isDefined("visualReferenceRepresentation"))
	{
	    // set projection model type
	    image.myProjectionType = e57::Image2DProjection::E57_VISUAL;

	    e57::StructureNode ref_model(
		    image_node.get("visualReferenceRepresentation"));

	    // The projection type in this case is not part of the union,
	    // so don't set the union

	    // set image sizes (dimensions, byte size, file type)
	    set_image_sizes(ref_model, image);
	}
	else
	{
	    // set projection model type
	    image.myProjectionType = e57::Image2DProjection::E57_NO_PROJECTION;

	    // Since there's no projection model, there's no image either
	    image.myWidth = 0;
	    image.myHeight = 0;
	    image.mySize = 0;
	    image.myFileType = e57::Image2DType::E57_NO_IMAGE;
	}

	// set rigid body transform
	set_rbxform(image_node, image.myRBXForm);

	// set associated scan guid
	if (image.myProjectionType > e57::Image2DProjection::E57_VISUAL)
	{
	    UT_String guid_name_wrapper;
	    try
	    {
		e57::StringNode scanpos_node(image_node.get("rlms:scanposGuid"));
		guid_name_wrapper = UT_String(scanpos_node.value().c_str(), true);
	    }
	    catch (e57::E57Exception &)
	    {
		e57::StringNode scanpos_node(image_node.get("associatedData3DGuid"));
		guid_name_wrapper = UT_String(scanpos_node.value().c_str(), true);
	    }
	    guid_name_wrapper.forceValidVariableName();

	    image.myAssociatedScanGuid = UT_StringHolder(guid_name_wrapper.c_str());
	}

	myImages.append(image);
    }
}

sop_E57PointReader::Builder
E57Reader::createPointReaderBuilder(int idx, GU_Detail *gdp, SOP_LidarImportCache *cache,
						  int range_good, int range_size)
{
    return sop_E57PointReader::Builder(myFileHandle, idx, myScans(idx), gdp, cache,
				       range_good, range_size);
}

UT_UniquePtr<unsigned char[]>
E57Reader::getImageBuffer(int idx, bool read_ref_images) const
{
    const sop_ImageInfo &image = myImages(idx);

    if (image.myProjectionType == e57::Image2DProjection::E57_NO_PROJECTION
	    || image.myFileType == e57::Image2DType::E57_NO_IMAGE)
	return nullptr;

    if (image.myProjectionType <= e57::Image2DProjection::E57_VISUAL 
	    && !read_ref_images)
	return nullptr;

    auto image_buffer = UTmakeUnique<unsigned char[]>(image.mySize);

    auto read_image_blob = [](e57::StructureNode proj_model, size_t size,
			      unsigned char *buf)
    {
	UT_ASSERT(proj_model.isDefined("pngImage")
		|| proj_model.isDefined("jpegImage"));
	if (proj_model.isDefined("pngImage"))
	{
	    e57::BlobNode png_node(proj_model.get("pngImage"));
	    png_node.read(buf, 0, size);
	}
	else
	{
	    e57::BlobNode jpeg_node(proj_model.get("jpegImage"));
	    jpeg_node.read(buf, 0, size);
	}
    };

    e57::VectorNode image_root_node(myFileHandle.root().get("/images2D"));
    e57::StructureNode image_node(image_root_node.get(idx));
    if (image.myProjectionType == e57::Image2DProjection::E57_PINHOLE)
    {
	e57::StructureNode pinhole_model(image_node.get("pinholeRepresentation"));
	read_image_blob(pinhole_model, image.mySize, image_buffer.get());
    }
    else if (image.myProjectionType == e57::Image2DProjection::E57_SPHERICAL)
    {
	e57::StructureNode spherical_model(
		image_node.get("sphericalRepresentation"));
	read_image_blob(spherical_model, image.mySize, image_buffer.get());
    }
    else
    {
	e57::StructureNode cylindrical_model(
		image_node.get("cylindricalRepresentation"));
	read_image_blob(cylindrical_model, image.mySize, image_buffer.get());
    }

    return image_buffer;
}

void
E57Reader::getImageDimensions(int idx, exint &width, exint &height)
    const
{
    width = myImages(idx).myWidth;
    height = myImages(idx).myHeight;
}

size_t
E57Reader::getImageSize(int idx) const
{
    return myImages(idx).mySize;
}

bool
E57Reader::hasValidProjectionType(int idx) const
{
    return myImages(idx).myProjectionType > e57::Image2DProjection::E57_VISUAL;
}

const UT_StringHolder &
E57Reader::getScanGroupGuid(int idx) const
{
    return myScans(idx).myGuid;
}

const UT_StringHolder &
E57Reader::getImageAssociatedScanGuid(int idx) const
{
    return myImages(idx).myAssociatedScanGuid;
}

void
E57Reader::computePointProjection(int idx, const UT_Vector4D pos, 
					      exint &img_x, exint &img_y) const
{
    const sop_ImageInfo &image = myImages(idx);

    fpreal i_x = 0;
    fpreal i_y = 0;
    fpreal range = SYShypot(pos.x(), pos.y());

    fpreal azimuth = SYSatan(pos.y(), pos.x());
    // Convert azimuth from counter-clockwise angle from 
    // x-axis to counter-clockwise angle from y-axis
    if (SYSisGreater(azimuth, -M_PI/2.0))
	azimuth -= M_PI/2.0;
    else
	azimuth += 3.0*M_PI/2.0;

    if (image.myProjectionType == e57::Image2DProjection::E57_PINHOLE)
    {
	e57::PinholeRepresentation pinhole_rep = 
	    image.myPinholeRep;

	i_x = pinhole_rep.principalPointX 
	    - (pos.x()/pos.z() * pinhole_rep.focalLength
		    / pinhole_rep.pixelWidth);
	i_y = pinhole_rep.principalPointY 
	    - (pos.y()/pos.z() * pinhole_rep.focalLength
		    / pinhole_rep.pixelHeight);
    }
    else if (image.myProjectionType == e57::Image2DProjection::E57_SPHERICAL)
    {
	range = SYShypot(range, pos.z());
	fpreal elevation = SYSasin(pos.z() / range);

	e57::SphericalRepresentation sphere_rep = 
	    image.mySphericalRep;

	i_x = image.myWidth / 2.0 
	    - azimuth / sphere_rep.pixelWidth;
	i_y = image.myHeight / 2.0 
	    - elevation / sphere_rep.pixelHeight;
    }
    else if (image.myProjectionType == e57::Image2DProjection::E57_CYLINDRICAL)
    {
	e57::CylindricalRepresentation cyl_rep = 
	    image.myCylindricalRep;

	i_x = image.myWidth / 2.0 
	    - azimuth / cyl_rep.pixelWidth;
	i_y = cyl_rep.principalPointY 
	    - (pos.z() * (fpreal(cyl_rep.radius)/cyl_rep.pixelHeight) / range);
    }

    // Truncate the image coords to get the pixel location.
    // Also move the origin from the top-left corner to the 
    // bottom-left corner.
    img_x = SYSfloor(i_x);
    img_y = image.myHeight - SYSceil(i_y);
}

class E57ColorFromImage
{
public:
    E57ColorFromImage(
            const SOP_LidarImportCache &cache,
            GA_Attribute &c_attr,
            GA_Attribute &nc_attr,
            const PXL_Raster &raster,
            const E57Reader &reader,
            int img_idx,
            const UT_Matrix4D rbxform_inv)
        : myColorAttrib(c_attr)
        , myNumColorsAttrib(nc_attr)
        , myRaster(raster)
        , myReader(reader)
        , myImageIndex(img_idx)
        , myRBXFormInv(rbxform_inv)
    {
        myHandlePV3.bind(cache.getDetachedPosition());
    }

    void operator()(const GA_SplittableRange &r) const
    {
        UT_ASSERT(
                myReader.hasValidProjectionType(myImageIndex)
                && "Invalid projection type");
        UT_Interrupt *boss = UTgetInterrupt();

        GA_RWPageHandleV3 color_ph(&myColorAttrib);
        GA_RWPageHandleI num_colors_ph(&myNumColorsAttrib);

        GA_Offset start, end;
        for (GA_Iterator it(r); it.blockAdvance(start, end);)
        {
            if (boss->opInterrupt())
                return;

            color_ph.setPage(start);
            num_colors_ph.setPage(start);

            for (GA_Offset ptoff = start; ptoff < end; ++ptoff)
            {
                UT_Vector4D pos(myHandlePV3.get(ptoff));
                pos *= myRBXFormInv;

                exint img_x, img_y;
                myReader.computePointProjection(
                        myImageIndex, pos, img_x, img_y);

                exint img_width, img_height;
                myReader.getImageDimensions(
                        myImageIndex, img_width, img_height);

                if (img_x >= 0 && img_x < img_width && img_y >= 0
                    && img_y < img_height)
                {
                    UT_Vector3 old_color = color_ph.value(ptoff);
                    int32 old_num_colors = num_colors_ph.value(ptoff);
                    fpreal r = (*(unsigned char *)myRaster.getPixel(
                                       img_x, img_y, 0))
                               / 255.0;
                    fpreal g = (*(unsigned char *)myRaster.getPixel(
                                       img_x, img_y, 1))
                               / 255.0;
                    fpreal b = (*(unsigned char *)myRaster.getPixel(
                                       img_x, img_y, 2))
                               / 255.0;

                    r = ((old_color.r() * old_num_colors) + r)
                        / (old_num_colors + 1);
                    g = ((old_color.g() * old_num_colors) + g)
                        / (old_num_colors + 1);
                    b = ((old_color.b() * old_num_colors) + b)
                        / (old_num_colors + 1);

                    color_ph.value(ptoff) = UT_Vector3(r, g, b);
                    num_colors_ph.value(ptoff)++;
                }
            }
        }
    }

private:
    GA_Attribute &myColorAttrib;
    GA_Attribute &myNumColorsAttrib;
    const PXL_Raster &myRaster;
    const E57Reader &myReader;
    const int myImageIndex;
    const UT_Matrix4D myRBXFormInv;
    GA_ROHandleV3D myHandlePV3;
};
} // namespace

//******************************************************************************
//*			        Lidar Importer                                 *
//******************************************************************************
namespace 
{
// Takes in SOP parameters and adds/removes the minimum neccessary data.
class LidarImporter
{
public:
    LidarImporter(const SOP_NodeVerb::CookParms &cookparms);
    bool import();
     
private:
    // Instructs the point cloud attribute reading, and  then copies and
    // transforms P and N from the cache into the detail.
    bool importAndTransformCloud();

    // Instructs point cloud metadata reading, and then transforms the info.
    bool importAndTransformInfo();

    // Read helpers
    bool readPoints();
    bool readInfo();
    void buildInfoCube(const UT_BoundingBoxD &bbox);

    // LAS read
    void warnLASMissingAttribs(const LASReader &reader);
    bool copyClassIndexToString(GA_Attribute *index, GA_Attribute *string);
    bool readLASFile();

    // E57 read
    void warnE57MissingAttribs(const E57Reader &reader);
    bool readE57Scan(E57Reader &reader, int scan_index, int64 &pt_idx);
    bool readE57File();
    bool updateE57ColorFromImages(E57Reader &reader);

    template <typename Body>
    void forAllPoints(const Body &body);

    // Copy and transform detached positions and normals
    void copyAndTransform(const UT_Matrix4D &xform, GA_TypeInfo type);

    // Convenience functions to see if read conditions have changed
    bool hasGroupPrefixChanged() const
    { return myParms.getGroup_prefix() != myCachedParms.getGroup_prefix(); }
    bool hasPrecisionChanged() const
    { return myParms.getPrecision() != myCachedParms.getPrecision(); }
    bool hasColorChanged() const
    { return myParms.getColor() != myCachedParms.getColor(); }
    bool hasIntensityChanged() const
    { return myParms.getIntensity() != myCachedParms.getIntensity();}
    bool hasRowColChanged() const
    { return myParms.getRow_col() != myCachedParms.getRow_col(); }
    bool hasRetDataChanged() const
    { return myParms.getRet_data() != myCachedParms.getRet_data(); }
    bool hasClassIndexChanged() const
    { return myParms.getClassindex() != myCachedParms.getClassindex(); }
    bool hasClassNameChanged() const
    { return myParms.getClassname() != myCachedParms.getClassname(); }
    bool hasClassFlagsChanged() const
    { return myParms.getClassflags() != myCachedParms.getClassflags(); }
    bool hasScannerChannelChanged() const
    { return myParms.getScannerchannel() != myCachedParms.getScannerchannel(); }
    bool hasScanFlagsChanged() const
    { return myParms.getScanflags() != myCachedParms.getScanflags(); }
    bool hasUserDataChanged() const
    { return myParms.getUserdata() != myCachedParms.getUserdata(); }
    bool hasScanAngleChanged() const
    { return myParms.getScanangle() != myCachedParms.getScanangle(); }
    bool hasPointSourceIDChanged() const
    { return myParms.getPointsourceid() != myCachedParms.getPointsourceid(); }
    bool hasNearInfraredChanged() const
    { return myParms.getNearinfrared() != myCachedParms.getNearinfrared(); }
    bool hasTimestampChanged() const
    { return myParms.getTimestamp() != myCachedParms.getTimestamp(); }
    bool hasNormalsChanged() const
    { return myParms.getNormals() != myCachedParms.getNormals(); }
    bool hasRigidTransformsChanged() const
    { return myParms.getRigidtransforms() != myCachedParms.getRigidtransforms(); }
    bool hasScanNamesChanged() const
    { return myParms.getScannames() != myCachedParms.getScannames(); }
    bool hasScanIndexChanged() const
    { return myParms.getScanindex() != myCachedParms.getScanindex(); }
    bool hasScanGroupsChanged() const
    { return myParms.getScangroups() != myCachedParms.getScangroups(); }
    bool hasPtNamesChanged() const
    { return myParms.getPtnames() != myCachedParms.getPtnames(); }

    bool isFilterParmsValid(const SOP_LidarImportParms &parms) const;
    
    bool hasTransformChanged() const;

    bool isClearDetailRequired() const;
    bool isReadPointsRequired() const;
    bool isReadInfoRequired() const;

    // Utilities
    UT_Matrix4D buildXform() const;

    static bool isRangeValid(exint range_good, exint range_size);
    void computeRange(
            int &range_good,
            int &range_size,
            exint pts_in_file,
            exint pts_in_scan) const;
    void computeRange(int &range_good, int &range_size, exint pts_in_file) const;

private:
    const SOP_NodeVerb::CookParms &myCookparms;

    // Convenient references to variables owned by cookparms
    GU_Detail *myGdp;
    SOP_LidarImportCache *myCache;
    const SOP_LidarImportParms &myParms;
    const SOP_LidarImportParms &myCachedParms;

    UT_AutoInterrupt myBoss;
};

LidarImporter::LidarImporter(const SOP_NodeVerb::CookParms &cookparms)
    : myCookparms(cookparms)
    , myGdp(myCookparms.gdh().gdpNC())
    , myCache(static_cast<SOP_LidarImportCache *>(myCookparms.cache()))
    , myParms(myCookparms.parms<SOP_LidarImportParms>())
    , myCachedParms(myCache->getCachedParms())
    , myBoss("Reading lidar file")
{
}

bool
LidarImporter::isFilterParmsValid(const SOP_LidarImportParms &parms) const
{
    using namespace SOP_LidarImportEnums;

    switch (parms.getFilter_type())
    {
    case Filter_type::NO_FILTER:
    {
        return true;
    }
    case Filter_type::RANGE_FILTER:
    {
        return isRangeValid(
                parms.getSelect_range()(0), parms.getSelect_range()(1));
    }
    case Filter_type::MAX_FILTER:
    {
        return parms.getMax_points() > 0;
    }
    default:
        return false;
    }
}

bool
LidarImporter::hasTransformChanged() const
{
    return myParms.getXord() != myCachedParms.getXord()
           || myParms.getRord() != myCachedParms.getRord()
           || myParms.getT() != myCachedParms.getT()
           || myParms.getR() != myCachedParms.getR()
           || myParms.getS() != myCachedParms.getS()
           || myParms.getShear() != myCachedParms.getShear()
           || myParms.getScale() != myCachedParms.getScale()
           || myParms.getP() != myCachedParms.getP()
           || myParms.getPr() != myCachedParms.getPr()
           || myParms.getPrexform_xord() != myCachedParms.getPrexform_xord()
           || myParms.getPrexform_rord() != myCachedParms.getPrexform_rord()
           || myParms.getPrexform_t() != myCachedParms.getPrexform_t()
           || myParms.getPrexform_r() != myCachedParms.getPrexform_r()
           || myParms.getPrexform_s() != myCachedParms.getPrexform_s()
           || myParms.getPrexform_shear() != myCachedParms.getPrexform_shear();
}

// Returns true if the detail needs to be destroyed and re-generated.
bool
LidarImporter::isClearDetailRequired() const
{
    using namespace SOP_LidarImportEnums;

    // Filename changed:
    if (myParms.getFilename() != myCachedParms.getFilename())
        return true;

    // Loadtype changed:
    if (myParms.getLoadtype() != myCachedParms.getLoadtype())
        return true; 

    // Filter changed:
    bool filter_valid = isFilterParmsValid(myParms);
    bool prev_filter_valid = isFilterParmsValid(myCachedParms);

    if (!prev_filter_valid && filter_valid
        && !(myParms.getFilter_type() == Filter_type::NO_FILTER))
    {
        return true;
    }
    if (prev_filter_valid && !filter_valid
        && !(myCachedParms.getFilter_type() == Filter_type::NO_FILTER))
    {
        return true;
    }
    if (prev_filter_valid && filter_valid)
    {
        if (myParms.getFilter_type() != myCachedParms.getFilter_type())
            return true;
        
        switch (myParms.getFilter_type())
        {
        case Filter_type::NO_FILTER:
            break;
        case Filter_type::RANGE_FILTER:
        {
            if (myParms.getSelect_range() != myCachedParms.getSelect_range())
                return true;
            break;
        }
        case Filter_type::MAX_FILTER:
        {
            if (myParms.getMax_points() != myCachedParms.getMax_points())
                return true;
            break;
        }
        default:
            break;
        }
    }

    // E57 invalid group handling changed:
    UT_String filename;
    filename = myParms.getFilename();

    if (filename.matchFileExtension(".e57")
        && (myParms.getDelete_invalid()
            || myParms.getDelete_invalid()
                        != myCachedParms.getDelete_invalid()))
    {
        return true;
    }

    return false;
}

// Returns true if the file needs to be accessed during the cook.
bool
LidarImporter::isReadPointsRequired() const
{
    // Filename and filter parms:
    if (isClearDetailRequired())
        return true;

    // Attributes (all formats):
    if (hasColorChanged() || hasIntensityChanged() || hasRetDataChanged()
        || hasTimestampChanged())
        return true;

    // E57 specific attributes:
    UT_String filename;
    filename = myParms.getFilename();

    if ((filename.matchFileExtension(".las")
         || filename.matchFileExtension(".laz"))
        && (hasClassIndexChanged() || hasClassNameChanged()
            || hasClassFlagsChanged() || hasScannerChannelChanged()
            || hasScanFlagsChanged() || hasUserDataChanged()
            || hasScanAngleChanged() || hasPointSourceIDChanged()
            || hasNearInfraredChanged()))
    {
        return true;
    }

    if (filename.matchFileExtension(".e57")
        && (hasRowColChanged() || hasNormalsChanged()
            || hasRigidTransformsChanged() || hasScanNamesChanged()
            || hasScanIndexChanged() || hasScanGroupsChanged()
            || hasGroupPrefixChanged() || hasPtNamesChanged()))
    {
        return true;
    }

    return false;
}

bool
LidarImporter::isReadInfoRequired() const
{
    // Filename changed:
    if (myParms.getFilename() != myCachedParms.getFilename())
        return true;

    // Loadtype changed:
    if (myParms.getLoadtype() != myCachedParms.getLoadtype())
        return true; 

    // Precision changed:
    if (hasPrecisionChanged())
        return true;

    return false;
}

// Builds the composite transformation from all xform parameters.
UT_Matrix4D
LidarImporter::buildXform() const
{
        // Build xform
    const UT_Vector3D prexform_t = myParms.getPrexform_t();
    const UT_Vector3D prexform_r = myParms.getPrexform_r();
    const UT_Vector3D prexform_s = myParms.getPrexform_s();
    const UT_Vector3D prexform_shear = myParms.getPrexform_shear();

    UT_Matrix4D pre_xform;
    SOP_Node::buildXform(
            (int)myParms.getPrexform_xord(),
            (int)myParms.getPrexform_rord(),
            prexform_t.x(), prexform_t.y(), prexform_t.z(),
            prexform_r.x(), prexform_r.y(), prexform_r.z(),
            prexform_s.x(), prexform_s.y(), prexform_s.z(),
            prexform_shear(0), prexform_shear(1), prexform_shear(2),
            0.0, 0.0, 0.0,
            pre_xform);

    const UT_Vector3D t = myParms.getT();
    const UT_Vector3D r = myParms.getR();
    const UT_Vector3D s = myParms.getS();
    const UT_Vector3D shear = myParms.getShear();
    const fpreal64 scale = myParms.getScale();

    UT_Matrix4D xform;
    OP_Node::buildXform(
            (int)myParms.getXord(),
            (int)myParms.getRord(),
            t.x(), t.y(), t.z(), 
            r.x(), r.y(), r.z(),
            s.x() * scale, s.y() * scale, s.z() * scale,
            shear(0), shear(1), shear(2),
            UT_Matrix4D::PivotSpace(myParms.getP(), myParms.getPr()),
            xform);

    xform *= pre_xform;
    return xform;
}


bool
LidarImporter::isRangeValid(exint range_good, exint range_size)
{ return (range_good < range_size && range_good > 0 && range_size > 0); }

// Returns (range_good, range_size), using the filter settings and given parms.
// If the parameters are invalid for the particular filter setting, then both
// range_good and range_size are set to GA_PAGE_SIZE.
void
LidarImporter::computeRange(
        int &range_good,
        int &range_size,
        exint pts_in_file,
        exint pts_in_scan) const
{
    using namespace SOP_LidarImportEnums;
    auto type = myParms.getFilter_type();
    exint scan_max_pts = myParms.getMax_points()
                         * (fpreal64(pts_in_scan) / pts_in_file);

    if (type == Filter_type::RANGE_FILTER
        && isRangeValid(
                myParms.getSelect_range()[0],
                myParms.getSelect_range()[1]))
    {
        range_good = myParms.getSelect_range()[0];
        range_size = myParms.getSelect_range()[1];
    }
    else if (type == Filter_type::MAX_FILTER
            && isRangeValid(scan_max_pts, pts_in_file))
    {
        range_good = (int)scan_max_pts;
        range_size = (int)pts_in_scan;
        while (range_good > 10)
        {
            range_good = SYSfloor(range_good / 10.0);
            range_size = SYSceil(range_size / 10.0);
        }
    }
    else
    {
        range_good = GA_PAGE_SIZE;
        range_size = GA_PAGE_SIZE;
    }
}

// Overload for single-scan datasets.
void
LidarImporter::computeRange(int &range_good, int &range_size, exint pts_in_file)
        const
{
    computeRange(range_good, range_size, pts_in_file, pts_in_file);
}

//******************************************************************************
//*			     Lidar Importer: LAS/LAZ                           *
//******************************************************************************

void
LidarImporter::warnLASMissingAttribs(const LASReader& reader)
{
    using namespace SOP_LidarImportEnums;

    if (myParms.getColor() == Color::FROM_IMAGES)
        myCookparms.sopAddWarning(SOP_WARN_PARMS_NOT_APPLICABLE, "\"Color From Images\"");

    UT_StringArray missingattribs;
    if (myParms.getColor() == Color::FROM_PTCLOUD && !reader.hasRGB())
        missingattribs.append(UT_StringHolder("Color"));

    if (myParms.getTimestamp() && !reader.hasGPSTime())
        missingattribs.append(UT_StringHolder("Timestamp"));

    if (myParms.getClassflags() && !reader.hasClassFlagOverlap())
        missingattribs.append(UT_StringHolder("Classification Flag: overlap"));

    if (myParms.getScannerchannel() && !reader.hasScannerChannel())
        missingattribs.append(UT_StringHolder("Scanner Channel"));

    if (myParms.getNearinfrared() && !reader.hasNIR())
        missingattribs.append(UT_StringHolder("Near Infrared"));

    UT_WorkBuffer msg;
    for (int i = 0; i < missingattribs.entries(); ++i)
    {
        if (i)
            msg.append(", ");
        msg.append('"');
        msg.append(missingattribs(i));
        msg.append('"');
    }
    if (missingattribs.entries() > 0)
        myCookparms.sopAddWarning(SOP_WARN_ATTRIBS_NOT_FOUND, msg.buffer());
}

// Copies the 8-bit classification index into its equivalent string, using 
// the LAS standard names. Undefined classes are named "Class_<classidx>".
//
// Note: LAS has a standard for embedding custom class definitions, but it is
// so rarely used, that it does not make sense to support it. For now, update
// 'themap' to any standard class definitions in future revisions
// (i.e. the upcoming LAS 1.4 R16).
//
// If the future LAS 1.5 redesigns custom class definitions, and other software
// start to support/preserve them (such as CloudCompare), then it would be good
// for Houdini to add support as well.
bool
LidarImporter::copyClassIndexToString(GA_Attribute *indexattrib, GA_Attribute *stringattrib)
{  
    static const char *themap[] = {
        "Created_Never_Classified", // 0
        "Unclassified",             // 1
        "Ground",                   // 2
        "Low_Vegetation",           // 3
        "Medium_Vegetation",        // 4
        "High_Vegetation",          // 5
        "Building",                 // 6
        "Low_Point",                // 7
        "Model_Keypoint",           // 8
        "Water",                    // 9
        "Rail",                     // 10
        "Road_Surface",             // 11
        "Overlap_Points",           // 12
        "Wire_Guard",               // 13
        "Wire_Conductor",           // 14
        "Transmission_Tower",       // 15
        "Wire_Structure_Connector", // 16
        "Bridge_Deck",              // 17
        "High_Noise",               // 18
        "Overhead_Structure",       // 19
        "Ignored_Ground ",          // 20
        "Snow",                     // 21
        "Temporal_Exclusion",       // 22
    };

    if (!indexattrib || !stringattrib)
        return false;
    else
    {
        GA_RWBatchHandleS batch(stringattrib);
        GA_PageHandleScalar<uint8>::ROType indexPH(indexattrib);
        if (batch.isInvalid() || indexPH.isInvalid())
            return false;
    }

    auto stuple = stringattrib->getAIFSharedStringTuple();
    if (!stuple)
        return false;

    UT_IntArray handlemap(UINT8_MAX + 1, UINT8_MAX + 1);
    GA_AIFSharedStringTuple::StringBuffer stringbuf(stringattrib, stuple);

    int i;
    for (i = 0; i < (sizeof(themap) / sizeof(const char *)); ++i)
        handlemap[i] = stringbuf.append(themap[i]);
    UT_WorkBuffer buf;
    for (; i <= UINT8_MAX; ++i)
    {
        buf.sprintf("Class_%d", i);
        handlemap[i] = stringbuf.append(buf.buffer());
    }

    UTparallelFor(
            GA_SplittableRange(myGdp->getPointRange()),
            [&](const GA_SplittableRange &r)
            {
                GA_RWBatchHandleS batch(stringattrib);
                GA_PageHandleScalar<uint8>::ROType indexPH(indexattrib);

                GA_Offset start, end;
                for (GA_Iterator it(r); it.blockAdvance(start, end);)
                {
                    indexPH.setPage(start);
                    for (GA_Offset pt = start; pt != end; ++pt)
                        batch.set(pt, handlemap(indexPH.get(pt)));
                }
            });
    return true;
}

// Read a .las or .laz file for a SOP cook. Returns true if successful.
bool
LidarImporter::readLASFile()
{
    using namespace SOP_LidarImportEnums;


    LASReader reader(myParms.getFilename().c_str());
    if (!reader.isValid())
        return false;

    warnLASMissingAttribs(reader);
    if (!isReadPointsRequired())
        return true;

    // Scrape metadata
    UT_BoundingBoxD box = reader.getBoundingBox();
    myCache->setFileBBox(box);

    // Configure the number of points to be read from filter parms
    exint pts_in_file = reader.getPointCount();
    int range_good, range_size;
    computeRange(range_good, range_size, pts_in_file);

    exint num_pts = pts_in_file;
    if (isRangeValid(range_good, range_size))
    {
        num_pts = pts_in_file / range_size;
        num_pts *= range_good;

        exint remaining_pts = pts_in_file % range_size;
        num_pts += SYSmin(range_good, remaining_pts);
    }

    // Reserve geometry if position needs to be read.
    bool read_required = false;
    if (isClearDetailRequired())
    {
        myGdp->appendPointBlock(num_pts);
        myCache->bindDetachedPosition(*myGdp);
        UT_ASSERT(myGdp->getNumPoints() == num_pts);
        read_required = true;
    }

    // Add / remove requested attributes
    if (hasColorChanged())
    {
        if (myParms.getColor() == Color::FROM_PTCLOUD && reader.hasRGB())
        {
            myGdp->addDiffuseAttribute(GA_ATTRIB_POINT);
            read_required = true;
        }
        else
            myGdp->destroyDiffuseAttribute(GA_ATTRIB_POINT);
    }
    if (hasIntensityChanged())
    {
        if (myParms.getIntensity())
        {
            myGdp->addFloatTuple(GA_ATTRIB_POINT, "intensity", 1);
            read_required = true;
        }
        else
            myGdp->destroyPointAttrib("intensity");
    }
    if (hasRetDataChanged())
    {
        if (myParms.getRet_data())
        {
            myGdp->addIntTuple(GA_ATTRIB_POINT, "return_index", 1);
            myGdp->addIntTuple(GA_ATTRIB_POINT, "return_count", 1);
            read_required = true;
        }
        else
        {
            myGdp->destroyPointAttrib("return_index");
            myGdp->destroyPointAttrib("return_count");
        }
    }
    // We copy over the class index data to generate the class names, and thus
    // delete it if its no longer needed at the end of reading.
    bool delete_class_index = false;
    if (hasClassIndexChanged())
    {
        if (myParms.getClassindex())
        {
            myGdp->addIntTuple(
                    GA_ATTRIB_POINT, "class_index", 1, GA_Defaults(), nullptr,
                    nullptr, GA_STORE_UINT8);
            read_required = true;
        }
        else if (!(hasClassNameChanged() && myParms.getClassname()))
            delete_class_index = true;
    }
    if (hasClassNameChanged())
    {
        if (myParms.getClassname())
        {
            myGdp->addStringTuple(GA_ATTRIB_POINT, "class_name", 1);

            // For reading in parallel, create class_index if it doesn't exist.
            if (!myGdp->findPointAttribute("class_index"))
            {
                myGdp->addIntTuple(
                        GA_ATTRIB_POINT, "class_index", 1, GA_Defaults(),
                        nullptr, nullptr, GA_STORE_UINT8);
                delete_class_index = true;
                read_required = true;
            }
        }
        else
            myGdp->destroyPointAttrib("class_name");
    }
    if (hasClassFlagsChanged())
    {
        if (myParms.getClassflags())
        {
            myGdp->addTuple(
                    GA_STORE_UINT8, GA_ATTRIB_POINT, "classflag_synthetic", 1);
            myGdp->addTuple(
                    GA_STORE_UINT8, GA_ATTRIB_POINT, "classflag_keypoint", 1);
            myGdp->addTuple(
                    GA_STORE_UINT8, GA_ATTRIB_POINT, "classflag_withheld", 1);
			
	    if (reader.hasClassFlagOverlap())
                myGdp->addTuple(
                        GA_STORE_UINT8, GA_ATTRIB_POINT, "classflag_overlap", 1);

            read_required = true;
        }
        else
        {
            myGdp->destroyPointAttrib("classflag_synthetic");
            myGdp->destroyPointAttrib("classflag_keypoint");
            myGdp->destroyPointAttrib("classflag_withheld");
            myGdp->destroyPointAttrib("classflag_overlap");
        }
    }
    if (hasScannerChannelChanged())
    {
        if (myParms.getScannerchannel() && reader.hasScannerChannel())
        {
            myGdp->addTuple(GA_STORE_UINT8, GA_ATTRIB_POINT, "scanner_channel", 1);
            read_required = true;
        }
        else
            myGdp->destroyPointAttrib("scanner_channel");
    }
    if (hasScanFlagsChanged())
    {
        if (myParms.getScanflags())
        {
            myGdp->addTuple(
                    GA_STORE_UINT8, GA_ATTRIB_POINT, "scanflag_direction", 1);
            myGdp->addTuple(
                    GA_STORE_UINT8, GA_ATTRIB_POINT, "scanflag_edge", 1);
            read_required = true;
        }
        else
        {
            myGdp->destroyPointAttrib("scanflag_direction");
            myGdp->destroyPointAttrib("scanflag_edge");
        }
    }
    if (hasUserDataChanged())
    {
        if (myParms.getUserdata())
        {
            myGdp->addTuple(GA_STORE_UINT8, GA_ATTRIB_POINT, "user_data", 1);
            read_required = true;
        }
        else
            myGdp->destroyPointAttrib("user_data");
    }
    if (hasScanAngleChanged())
    {
        if (myParms.getScanangle())
        {
            myGdp->addFloatTuple(GA_ATTRIB_POINT, "scan_angle", 1);
            read_required = true;
        }
        else
            myGdp->destroyPointAttrib("scan_angle");
    }
    if (hasPointSourceIDChanged())
    {
        if (myParms.getPointsourceid())
        {
            myGdp->addIntTuple(GA_ATTRIB_POINT, "source_id", 1);
            read_required = true;
        }
        else
            myGdp->destroyPointAttrib("source_id");
    }
    if (hasTimestampChanged())
    {
        if (myParms.getTimestamp() && reader.hasGPSTime())
        {
            myGdp->addFloatTuple(
                    GA_ATTRIB_POINT, "timestamp", 1, GA_Defaults(), nullptr,
                    nullptr, GA_STORE_REAL64);
            read_required = true;
        }
        else
            myGdp->destroyPointAttrib("timestamp");
    }
    if (hasNearInfraredChanged())
    {
        if (myParms.getNearinfrared() && reader.hasNIR())
        {
            myGdp->addFloatTuple(GA_ATTRIB_POINT, "near_infrared", 1);
            read_required = true;
        }
        else
            myGdp->destroyPointAttrib("near_infrared");
    }

    // Stop if no new attributes need to be read.
    if (!read_required || !num_pts)
    {
        bool result = true;
        if (hasClassNameChanged() && myParms.getClassname())
        {
            result &= copyClassIndexToString(
                    myGdp->findPointAttribute("class_index"),
                    myGdp->findPointAttribute("class_name"));
        }
        if (delete_class_index)
            myGdp->destroyPointAttrib("class_index");
        return result;
    }

    // Generate point readers
    GA_Offset start_offset = myGdp->pointOffset(0);
    exint first_pagenum = GAgetPageNum(start_offset);
    exint num_pages = GAgetPageNum(start_offset + num_pts - 1) + 1
                      - first_pagenum;
    int num_readers = SYSmin(num_pages, UT_Thread::getNumProcessors());

    UT_Array<LASReader *> ptreaders(num_readers);
    ptreaders.append(&reader);
    for (int i = 1; i < num_readers; ++i)
    {
	LASReader *r = new LASReader(myParms.getFilename().c_str());
	if (!r->isValid())
	{
	    delete r;
	    break;
	}

        ptreaders.append(r);
    }

    GA_Attribute *position(myCache->getDetachedPosition());
    GA_Attribute *color(myGdp->findDiffuseAttribute(GA_ATTRIB_POINT));
    GA_Attribute *intensity(myGdp->findPointAttribute("intensity"));
    GA_Attribute *returnIndex(myGdp->findPointAttribute("return_index"));
    GA_Attribute *returnCount(myGdp->findPointAttribute("return_count"));
    GA_Attribute *classIndex(myGdp->findPointAttribute("class_index"));
    GA_Attribute *cflagSynthetic(myGdp->findPointAttribute("classflag_synthetic"));
    GA_Attribute *cflagKeypoint(myGdp->findPointAttribute("classflag_keypoint"));
    GA_Attribute *cflagWithheld(myGdp->findPointAttribute("classflag_withheld"));
    GA_Attribute *cflagOverlap(myGdp->findPointAttribute("classflag_overlap"));
    GA_Attribute *scannerchannel(myGdp->findPointAttribute("scanner_channel"));
    GA_Attribute *sflagDirection(myGdp->findPointAttribute("scanflag_direction"));
    GA_Attribute *sflagEdge(myGdp->findPointAttribute("scanflag_edge"));
    GA_Attribute *userData(myGdp->findPointAttribute("user_data"));
    GA_Attribute *scanAngle(myGdp->findPointAttribute("scan_angle"));
    GA_Attribute *sourceID(myGdp->findPointAttribute("source_id"));
    GA_Attribute *timestamp(myGdp->findPointAttribute("timestamp"));
    GA_Attribute *nearInfrared(myGdp->findPointAttribute("near_infrared"));

    // Read the requested attributes
    exint seek_size = static_cast<exint>(range_size - range_good);
    UTparallelForEachNumber(
            ptreaders.size(),
            [&](const UT_BlockedRange<exint> &r)
            {
                exint idx = r.begin();

                GA_RWPageHandleV3D positionPH(position);
                GA_RWPageHandleV3 colorPH(color);
                GA_RWPageHandleF intensityPH(intensity);
                GA_PageHandleScalar<uint8>::RWType returnIndexPH(returnIndex);
                GA_PageHandleScalar<uint8>::RWType returnCountPH(returnCount);
                GA_PageHandleScalar<uint8>::RWType classIndexPH(classIndex);
                GA_PageHandleScalar<uint8>::RWType cflagSyntheticPH(cflagSynthetic);
                GA_PageHandleScalar<uint8>::RWType cflagKeypointPH(cflagKeypoint);
                GA_PageHandleScalar<uint8>::RWType cflagWithheldPH(cflagWithheld);
                GA_PageHandleScalar<uint8>::RWType cflagOverlapPH(cflagOverlap);
                GA_PageHandleScalar<uint8>::RWType scannerchannelPH(scannerchannel);
                GA_PageHandleScalar<uint8>::RWType sflagDirectionPH(sflagDirection);
                GA_PageHandleScalar<uint8>::RWType sflagEdgePH(sflagEdge);
                GA_PageHandleScalar<uint8>::RWType userDataPH(userData);
                GA_RWPageHandleF scanAnglePH(scanAngle);
                GA_RWPageHandleI sourceIDPH(sourceID);
                GA_RWPageHandleD timestampPH(timestamp);
                GA_RWPageHandleF nearInfraredPH(nearInfrared);

                bool cflags_valid = cflagSyntheticPH.isValid()
                                  && cflagKeypointPH.isValid()
                                  && cflagWithheldPH.isValid()
                                  && cflagOverlapPH.isValid();

                bool sflags_valid = sflagDirectionPH.isValid()
                                    && sflagEdgePH.isValid();

                exint start_page = (idx * num_pages) / ptreaders.size()
                                   + first_pagenum;
                exint end_page = ((idx + 1) * num_pages) / ptreaders.size()
                                 + first_pagenum;

                GA_Offset start = GA_Offset(SYSmax(
                        start_page * GA_PAGE_SIZE, (exint)start_offset));
                GA_Offset end = GA_Offset(SYSmin(
                        end_page * GA_PAGE_SIZE, (exint)start_offset + num_pts));
                GA_Range pointrange(myGdp->getPointMap(), start, end);

                LASReader *pread = ptreaders(idx);
                for (GA_Iterator it(pointrange); it.blockAdvance(start, end);)
                {
                    if (myBoss.wasInterrupted())
                        break;

                    if (positionPH.isValid())
                        positionPH.setPage(start);
                    if (colorPH.isValid())
                        colorPH.setPage(start);
                    if (intensityPH.isValid())
                        intensityPH.setPage(start);
                    if (returnIndexPH.isValid() && returnCountPH.isValid())
                    {
                        returnIndexPH.setPage(start);
                        returnCountPH.setPage(start);
                    }
                    if (classIndexPH.isValid())
                        classIndexPH.setPage(start);
                    if (cflags_valid)
                    {
                        cflagSyntheticPH.setPage(start);
                        cflagKeypointPH.setPage(start);
                        cflagWithheldPH.setPage(start);
                        cflagOverlapPH.setPage(start);
                    }
                    if (scannerchannelPH.isValid())
                        scannerchannelPH.setPage(start);
                    if (sflags_valid)
                    {
                        sflagDirectionPH.setPage(start);
                        sflagEdgePH.setPage(start);
                    }
                    if (userDataPH.isValid())
                        userDataPH.setPage(start);
                    if (scanAnglePH.isValid())
                        scanAnglePH.setPage(start);
                    if (sourceIDPH.isValid())
                        sourceIDPH.setPage(start);
                    if (timestampPH.isValid())
                        timestampPH.setPage(start);
                    if (nearInfraredPH.isValid())
                        nearInfraredPH.setPage(start);

                    // Determine the point number that we should be reading
                    int range = 0;
                    exint file_idx = myGdp->pointIndex(start);
                    if (seek_size > 0)
                    {
                        exint num_ranges = file_idx / range_good;
                        range = file_idx % range_good;

                        file_idx = num_ranges * range_size + range;
                    }
                    pread->seekPoint(file_idx);

                    for (GA_Offset ptoff = start; ptoff != end;
                         ++ptoff, ++range, ++file_idx)
                    {
                        if (seek_size && (range == range_good))
                        {
                            range = 0;
                            file_idx += seek_size;
                            pread->seekPoint(file_idx);
                        }

                        pread->readNextPoint();

                        if (positionPH.isValid())
                            pread->getXYZ(positionPH.value(ptoff));
                        if (colorPH.isValid())
                            pread->getRGB(colorPH.value(ptoff));
                        if (intensityPH.isValid())
                            pread->getIntensity(intensityPH.value(ptoff));
                        if (returnIndexPH.isValid() && returnCountPH.isValid())
                        {
                            pread->getReturnNumber(returnIndexPH.value(ptoff));
                            pread->getReturnCount(returnCountPH.value(ptoff));
                        }
                        if (classIndexPH.isValid())
                            pread->getClassIndex(classIndexPH.value(ptoff));
                        if (cflags_valid)
                        {
                            pread->getClassFlagSynthetic(cflagSyntheticPH.value(ptoff));
                            pread->getClassFlagKeyPoint(cflagKeypointPH.value(ptoff));
                            pread->getClassFlagWithheld(cflagWithheldPH.value(ptoff));
                            pread->getClassFlagOverlap(cflagOverlapPH.value(ptoff));
                        }
                        if (scannerchannelPH.isValid())
                            pread->getScannerChannel(scannerchannelPH.value(ptoff));
                        if (sflags_valid)
                        {
                            pread->getScanDirectionFlag(
                                    sflagDirectionPH.value(ptoff));
                            pread->getEdgeFlightLineFlag(
                                    sflagEdgePH.value(ptoff));
                        }
                        if (userDataPH.isValid())
                            pread->getUserData(userDataPH.value(ptoff));
                        if (scanAnglePH.isValid())
                            pread->getScanAngle(scanAnglePH.value(ptoff));
                        if (sourceIDPH.isValid())
                            pread->getPointSourceID(sourceIDPH.value(ptoff));
                        if (timestampPH.isValid())
                            pread->getGPSTime(timestampPH.value(ptoff));
                        if (nearInfraredPH.isValid())
                            pread->getNIR(nearInfraredPH.value(ptoff));
                    }
                }
            });
    for (int i = 1; i < ptreaders.size(); ++i)
        delete ptreaders(i);

    if (myBoss.wasInterrupted())
        return false;

    bool result = true;
    if (hasClassNameChanged() && myParms.getClassname())
    {
        result &= copyClassIndexToString(
                myGdp->findPointAttribute("class_index"),
                myGdp->findPointAttribute("class_name"));
    }
    if (delete_class_index)
        myGdp->destroyPointAttrib("class_index");
    return result;
}

//******************************************************************************
//*			       Lidar Importer: E57                             *
//******************************************************************************

void
LidarImporter::warnE57MissingAttribs(const E57Reader& reader)
{
    using namespace SOP_LidarImportEnums;

    UT_StringMap<UT_IntArray> scanmap;
    for (int i = 0; i < reader.getNumScans(); ++i)
    {
        if (myParms.getColor() == Color::FROM_PTCLOUD && !reader.hasColor(i))
            scanmap["Color"].append(i);
        else if (myParms.getColor() == Color::FROM_IMAGES)
        {
            bool has_images = false;
            UT_StringRef guid = reader.getScanGroupGuid(i);
            for (int j = 0; j < reader.getNumImages(); ++j)
            {
                if (reader.getImageAssociatedScanGuid(j) == guid)
                    has_images = true;
            }
            if (!has_images)
                scanmap["Color"].append(i);
        }

        if (myParms.getIntensity() && !reader.hasIntensity(i))
            scanmap["Intensity"].append(i);

        if (myParms.getRet_data() && !reader.hasRetData(i))
            scanmap["Return Data"].append(i);

        if (myParms.getTimestamp() && !reader.hasTimestamp(i))
            scanmap["Timestamp"].append(i);

        if (myParms.getNormals() && !reader.hasNormals(i))
            scanmap["Surface Normals"].append(i);

        if (myParms.getRow_col() && !reader.hasRowCol(i))
            scanmap["Row and Column"].append(i);

        if (reader.getScanName(i) == "")
            scanmap["Scan Name"].append(i);
    }

    UT_WorkBuffer msg;
    for (auto it = scanmap.begin(); it != scanmap.end(); ++it)
    {
        if (it->second.size() == 0)
            continue;

        if (msg.length())
            msg.append("; ");
        msg.appendFormat("\"{}\" in scan", it->first);
        if (it->second.size() > 1)
            msg.append('s');

        for (int i = 0; i < it->second.size(); ++i)
        {
            if (i)
                msg.append(",");
            msg.appendFormat(" {}", it->second(i));
        }
    }
    if (scanmap.size() > 0)
        myCookparms.sopAddWarning(SOP_WARN_ATTRIBS_NOT_FOUND, msg.buffer());
}

bool
LidarImporter::readE57Scan(
	    E57Reader &reader,
	    int scan_index,
	    int64 &pt_idx)
{
    using namespace SOP_LidarImportEnums;
    exint pts_in_scan = reader.getPointsInScan(scan_index);

    int range_good, range_size;
    computeRange(
            range_good, range_size, reader.getPointsInFile(),
            reader.getPointsInScan(scan_index));

    sop_E57PointReader::Builder pt_reader_builder(
            reader.createPointReaderBuilder(
                    scan_index, myGdp, myCache, range_good, range_size));
    
    // Specify the buffer objects that will be needed
    bool read_file_required = false;
    bool reimport_required = isClearDetailRequired();
    if (reimport_required)
    {
        if (reader.hasCartesian(scan_index) || reader.hasSpherical(scan_index))
        {
            if (reader.hasCartesian(scan_index))
            {
                pt_reader_builder.cartesianPosition();

                if (reader.hasCartesianInvalid(scan_index))
                    pt_reader_builder.cartesianPositionInvalid();
            }
            else
            {
                pt_reader_builder.sphericalPosition();

                if (reader.hasSphericalInvalid(scan_index))
                    pt_reader_builder.sphericalPositionInvalid();
            }
            read_file_required = true;
        }
        else
        {
            myCookparms.sopAddError(SOP_ERR_MISSING_POSITION);
            return false;
        }
    }

    if (hasColorChanged())
    {
        myGdp->destroyDiffuseAttribute(GA_ATTRIB_POINT);
        if (myParms.getColor() == Color::FROM_PTCLOUD
            && reader.hasColor(scan_index))
        {
            pt_reader_builder.color();
            read_file_required = true;

            if (reader.hasColorInvalid(scan_index))
                pt_reader_builder.colorInvalid();
        }
        else if (myParms.getColor() != Color::FROM_PTCLOUD)
        {
            GA_PointGroup *invalid_group
                    = myGdp->findPointGroup("e57_invalid_color");
            if (invalid_group)
                myGdp->destroyPointGroup(invalid_group);
        }
    }

    if (hasIntensityChanged() && myParms.getIntensity()
        && reader.hasIntensity(scan_index))
    {
        pt_reader_builder.intensity();
        read_file_required = true;

        if (reader.hasIntensityInvalid(scan_index))
            pt_reader_builder.intensityInvalid();
    }
    else if (hasIntensityChanged() && !myParms.getIntensity())
    {
        myGdp->destroyPointAttrib("intensity");

        GA_PointGroup *invalid_group
                = myGdp->findPointGroup("e57_invalid_intensity");
        if (invalid_group)
            myGdp->destroyPointGroup(invalid_group);
    }

    if (hasRowColChanged() && myParms.getRow_col()
        && reader.hasRowCol(scan_index))
    {
        pt_reader_builder.rowIndex();
        pt_reader_builder.columnIndex();
        read_file_required = true;
    }
    else if (hasRowColChanged() && !myParms.getRow_col())
    {
        myGdp->destroyPointAttrib("row_index");
        myGdp->destroyPointAttrib("column_index");
    }

    if (hasRetDataChanged() && myParms.getRet_data()
        && reader.hasRetData(scan_index))
    {
        pt_reader_builder.returnIndex();
        pt_reader_builder.returnCount();
        read_file_required = true;
    }
    else if (hasRetDataChanged() && !myParms.getRet_data())
    {
        myGdp->destroyPointAttrib("return_index");
        myGdp->destroyPointAttrib("return_count");
    }

    if (hasTimestampChanged() && myParms.getTimestamp()
        && reader.hasTimestamp(scan_index))
    {
        pt_reader_builder.timestamp();
        read_file_required = true;

        if (reader.hasTimestampInvalid(scan_index))
            pt_reader_builder.timestampInvalid();
    }
    else if (hasTimestampChanged() && !myParms.getTimestamp())
    {
        myGdp->destroyPointAttrib("timestamp");

        GA_PointGroup *invalid_group
                = myGdp->findPointGroup("e57_invalid_timestamp");
        if (invalid_group)
            myGdp->destroyPointGroup(invalid_group);
    }

    if (hasNormalsChanged() && myParms.getNormals()
        && reader.hasNormals(scan_index))
    {
        pt_reader_builder.normals();
        read_file_required = true;
    }
    else if (hasNormalsChanged() && !myParms.getNormals())
    {
        myGdp->destroyNormalAttribute(GA_ATTRIB_POINT);
        myCache->destroyDetachedNormals();
    }

    if (reimport_required
	|| hasGroupPrefixChanged()
        || hasRigidTransformsChanged())
    {
        UT_WorkBuffer name(myCachedParms.getGroup_prefix().c_str());
        name.appendFormat("transform{}", scan_index + 1);
        myGdp->destroyAttribute(GA_ATTRIB_GLOBAL, name);

        if (myParms.getRigidtransforms())
        {
            name = myParms.getGroup_prefix();
            name.appendFormat("transform{}", scan_index + 1);

            GA_RWHandleM4D xform_h(myGdp->addFloatTuple(
                    GA_ATTRIB_GLOBAL, name, 16,
                    GA_Defaults(GA_Defaults::matrix4()), 0, 0,
                    GA_STORE_REAL64));
            // Take transform from reader directly to avoid precision lost
            xform_h.set(GA_Offset(0), reader.getScanRBXForm(scan_index));
        }
    }

    // Get number of points and range for this scan
    exint num_pts = pts_in_scan;
    if (isRangeValid(range_good, range_size))
    {
        num_pts = pts_in_scan / range_size;
        num_pts *= range_good;

        exint remaining_pts = pts_in_scan % range_size;
        num_pts += SYSmin(range_good, remaining_pts);
    }
    GA_Range pt_range(
        myGdp->getPointMap(), GA_Offset(pt_idx), GA_Offset(pt_idx + num_pts));

    // Set the scan index attribute for the point range
    if (hasScanIndexChanged())
    {
        if (myParms.getScanindex())
        {
            GA_Attribute *attrib = myGdp->addIntTuple(GA_ATTRIB_POINT, "scanindex", 1);
            const GA_AIFTuple *tuple = attrib->getAIFTuple();

            if (tuple)
            {
                tuple->set(attrib, pt_range, scan_index);
            }
        }
        else
        {
            myGdp->destroyPointAttrib("scanindex");
        }
    }

    // Set the scan name attribute for the point range
    if (hasPtNamesChanged())
    {
        if (myParms.getPtnames())
        {
            GA_Attribute *attrib = myGdp->addStringTuple(
                    GA_ATTRIB_POINT, "scanname", 1);
            auto *stuple = attrib->getAIFSharedStringTuple();

            if (stuple)
            {
                stuple->setString(attrib, pt_range, reader.getScanName(scan_index), 0);
            }
        }
        else
        {
            myGdp->destroyPointAttrib("scanname");
        }
    }

    // Depreacted N-group representation of scans:
    UT_WorkBuffer group_name(myParms.getGroup_prefix().c_str());
    UT_WorkBuffer old_group_name(myCachedParms.getGroup_prefix().c_str());
    group_name.appendFormat("{}", scan_index + 1);
    old_group_name.appendFormat("{}", scan_index + 1);

    if (hasScanGroupsChanged())
    {
        if (myParms.getScangroups())
        {
            GA_PointGroup *scan_group = myGdp->newPointGroup(group_name.buffer());
            scan_group->setElement(pt_range, true);
        }
        else
        {
            myGdp->destroyPointGroup(old_group_name.buffer());
        }
    }
    else if (myParms.getScangroups() && hasGroupPrefixChanged())
    {
        myGdp->destroyPointGroup(old_group_name.buffer());
        myGdp->newPointGroup(group_name.buffer());
    }

    if (!read_file_required)
    {
        pt_idx += num_pts;
        return true;
    }

    // Construct the buffers for each attribute.
    sop_E57PointReader pt_reader(pt_reader_builder.build());

    // readBlock pushes data into the specified 1k buffers. Then, the data is
    // copied to the respective pages, where the buffer's callback is invoked.
    GA_Offset start, end;
    for (GA_Iterator it(pt_range);
         it.blockAdvance(start, end) && pt_reader.readBlock(start, end);)
    {
        if (myBoss.wasInterrupted())
            return false;
    }

    // Apply the scan's rigid transformation to the detached points.
    UT_Matrix4D rigid_body_transform(reader.getScanRBXForm(scan_index));
    if (reimport_required && !rigid_body_transform.isIdentity())
    {
        GA_AttributeTransformer transformer(*myGdp, GA_ATTRIB_POINT);
        GA_AttributeTransformer::Transform<fpreal> rbx(rigid_body_transform);
        transformer.addAttribute(myCache->getDetachedPosition());

         if (hasNormalsChanged() && myParms.getNormals()
            && reader.hasNormals(scan_index))
        {
            transformer.addAttribute(myCache->getDetachedNormals());
        }

        GA_Range detached_pos_range(
                myGdp->getPointMap(), GA_Offset(pt_idx),
                GA_Offset(pt_idx + num_pts));
        transformer.transform(detached_pos_range, rbx);
    }

    pt_idx += num_pts;

    return true;
}

bool
LidarImporter::updateE57ColorFromImages(E57Reader &reader)
{
    GA_Attribute *ncolors_attrib = myGdp->addIntTuple(
            GA_ATTRIB_POINT, GA_SCOPE_PRIVATE, "__num_colors__", 1);
    GA_Attribute *color_attrib = myGdp->addDiffuseAttribute(GA_ATTRIB_POINT);

    GA_Offset start = myGdp->pointOffset(0);
    GA_Offset end;
    for (int i = 0; i < reader.getNumScans(); ++i)
    {
        // Compute the range of the scan.
        int range_good, range_size;
        computeRange(
                range_good, range_size, reader.getPointsInFile(),
                reader.getPointsInScan(i));

        exint increment = reader.getPointsInScan(i);
        if (isRangeValid(range_good, range_size))
        {
            increment = range_good
                        * (reader.getPointsInScan(i) / range_size);
            exint remaining_pts = reader.getPointsInFile() % range_size;
            increment += SYSmin(range_good, remaining_pts);
        }
        end = start + increment;

        // Project images with matching guid's onto the current scan.
        UT_StringRef guid = reader.getScanGroupGuid(i);
        for (int j = 0; j < reader.getNumImages(); ++j)
        {
            if (reader.getImageAssociatedScanGuid(j) != guid)
                continue;

            UT_UniquePtr<unsigned char[]> image_buffer = reader.getImageBuffer(
                    j, false);
            if (!image_buffer)
                continue;

            size_t image_size = reader.getImageSize(j);
            UT_IStream is(
                    (char *)image_buffer.get(), image_size, UT_ISTREAM_BINARY);

            UT_UniquePtr<IMG_File> img_file(IMG_File::open(is));

            UT_Array<PXL_Raster *> rasters;
            img_file->readImages(rasters);
            img_file->close();

            UT_Matrix4D rigid_body_transform_inv(reader.getImageRBXForm(j));
            rigid_body_transform_inv.invert();
            UTparallelFor(
                    GA_SplittableRange(
                            GA_Range(myGdp->getPointMap(), start, end)),
                    E57ColorFromImage(
                            *myCache, *color_attrib, *ncolors_attrib,
                            *rasters(0), reader, j, rigid_body_transform_inv));

            for (int i = 0, ni = rasters.entries(); i < ni; ++i)
                delete rasters(i);

            if (myBoss.wasInterrupted())
                return false;
        }

        start += increment;
    }

    myGdp->destroyPointAttrib(GA_SCOPE_PRIVATE, "__num_colors__");
    return true;
}

// Reads E57 file into the detail, by calling readE57Scan for N-scans.
bool
LidarImporter::readE57File()
{
    using namespace SOP_LidarImportEnums;
    try
        {
            E57Reader reader(myParms.getFilename().c_str());
            warnE57MissingAttribs(reader);

            if (!isReadPointsRequired())
            {
                if (hasColorChanged()
                    && (myParms.getColor() == Color::FROM_IMAGES)
                    && (reader.getNumImages() > 0))
                {
                    updateE57ColorFromImages(reader);
                }

                return true;
            }

            // Cache the total bounding box of all E57 scans
            myCache->setFileBBox(reader.getFileBoundingBox());

            // Get the total points to be read, and bind the detached position
            int num_scans = reader.getNumScans();
            exint pts_in_file = reader.getPointsInFile();
            if (isClearDetailRequired())
            {
                exint num_pts = 0;
                for (int i = 0; i < num_scans; ++i)
                {
                    int range_good, range_size;
                    computeRange(
                            range_good, range_size, pts_in_file,
                            reader.getPointsInScan(i));

                    exint increment = reader.getPointsInScan(i);
                    if (isRangeValid(range_good, range_size))
                    {
                        increment = range_good
                                    * (reader.getPointsInScan(i) / range_size);

                        exint remaining_pts = pts_in_file % range_size;
                        increment += SYSmin(range_good, remaining_pts);
                    }
                    num_pts += increment;
                }

                myGdp->appendPointBlock(num_pts);
                myCache->bindDetachedPosition(*myGdp);
                UT_ASSERT(myGdp->getNumPoints() == num_pts);
            }

            // Bind the detached normals if they are present
            bool has_normals = false;
            for (int i = 0; i < num_scans; ++i)
            {
                has_normals |= reader.hasNormals(i);
            }
            if (has_normals && myParms.getNormals())
                myCache->bindDetachedNormals(*myGdp);

            int64 pt_idx = 0;
            for (int i = 0; i < num_scans; ++i)
            {
                if (isClearDetailRequired())
                {
                    UT_StringHolder group_name(reader.getScanGroupGuid(i));
                    myCache->getScanGroupMap()[group_name].append(i);
		}

                if (!readE57Scan(reader, i, pt_idx))
                    return false;
            }

            if (hasColorChanged() && (myParms.getColor() == Color::FROM_IMAGES)
                && (reader.getNumImages() > 0))
            {
                updateE57ColorFromImages(reader);
            }

            if (myParms.getDelete_invalid())
            {
                GA_PointGroup invalid_pts(*myGdp);

                const char *grp_namelist[] = {
                        "e57_invalid_pos",
                        "e57_invalid_color",
                        "e57_invalid_intensity",
                        "e57_invalid_timestamp",
                        nullptr,
                };

                UT_StringArray invalid_groups(grp_namelist);

                for (int i = 0, ni = invalid_groups.entries(); i < ni; ++i)
                {
                    GA_PointGroup *invalid_grp
                            = myGdp->findPointGroup(invalid_groups(i).c_str());

                    if (invalid_grp)
                        invalid_pts |= *invalid_grp;
                }

                myGdp->destroyPoints(GA_Range(invalid_pts));
            }

            if (hasScanNamesChanged())
            {
                if (!myParms.getScannames())
                {
                    myGdp->destroyAttribute(GA_ATTRIB_GLOBAL, "scannames");
                }
                else
                {
                    UT_StringArray names(num_scans, num_scans);
                    for (int i = 0; i < num_scans; ++i)
                        names(i) = reader.getScanName(i);

                    GA_RWHandleSA names_h(
                            myGdp->addStringArray(GA_ATTRIB_GLOBAL, "scannames"));
                    names_h.set(GA_Offset(0), names);
                }
            }
        }
#ifdef E57_SOP_VERBOSE
        catch (e57::E57Exception &e)
        {
            std::string context = e.context();
            addError(SOP_ERR_LIDAR_READER_ERROR, context.c_str());
            e.report(__FILE__, __LINE__, __FUNCTION__);
            return false;
        }
#else
        catch (e57::E57Exception &e)
        {
            std::string context = e.context();
            myCookparms.sopAddError(SOP_ERR_LIDAR_READER_ERROR, context.c_str());
            return false;
        }
#endif
        return true;
}

//******************************************************************************
//*                 Lidar Importer: Read and Transform Methods                 *
//******************************************************************************

// Read or destroy any point cloud attributes, if neeeded.
bool
LidarImporter::readPoints()
{
    using namespace SOP_LidarImportEnums;
    UT_String filename;
    filename = myParms.getFilename();

    UT_String prefix;
    prefix = myParms.getGroup_prefix();

    if (myCookparms.error()->getSeverity() >= UT_ERROR_ABORT
        || !filename.isstring() || !myParms.getGroup_prefix().isstring()
        || myCookparms.getNode()->forceValidGroupPrefix(prefix, UT_ERROR_ABORT)
        || !(filename.matchFileExtension(".las")
             || filename.matchFileExtension(".laz")
             || filename.matchFileExtension(".e57")))
    {
        myCache->clearCache();
        return false;
    }

    // Must re-read everything:
    if (isClearDetailRequired())
    {
        myGdp->clearAndDestroy();
        myCache->clearCache();
    }

    // LAS/LAZ :
    if (filename.matchFileExtension(".las")
        || filename.matchFileExtension(".laz"))
    {
        if (!readLASFile())
        {
            myCookparms.sopAddError(
                    SOP_ERR_LIDAR_READER_ERROR,
                    "Failed to read the file as a valid LAS format.");
            return false;
        }
    }
    // E57 :
    else if (filename.matchFileExtension(".e57"))
    {
        if (!readE57File())
            return false;
    }

    return true;
}

template <typename Body>
void 
LidarImporter::forAllPoints(const Body& body)
{
    UTparallelFor(
            GA_SplittableRange(myGdp->getPointRange()),
            [&](const GA_SplittableRange &r) {
                GA_Offset start, end;
                for (GA_Iterator it(r); it.blockAdvance(start, end);)
                {
                    for (GA_Offset pt = start; pt != end; ++pt)
                        body(pt);
                }
            });
}

// Copies and transforms the detached positions or normals into the detail.
void
LidarImporter::copyAndTransform(const UT_Matrix4D &xform, GA_TypeInfo type)
{
    GA_Attribute *src;
    GA_Attribute *dest;

    switch (type)
    {
    case GA_TYPE_POINT:
    {
        src = myCache->getDetachedPosition();
        dest = myGdp->getP();
        break;
    }
    case GA_TYPE_NORMAL:
    {
        src = myCache->getDetachedNormals();
        dest = myGdp->findNormalAttribute(GA_ATTRIB_POINT);
        break;
    }
    default:
        return;
    }
 
    if (!src || !dest)
        return;

    GA_ROHandleV3D h_src(src);
    GA_RWHandleV3D h_dest(dest);

    if (h_src.isInvalid() || h_dest.isInvalid())
        return;

    // No transform needed:
    if (xform.isIdentity()
        || (type == GA_TYPE_NORMAL && UT_Matrix3D(xform).isIdentity()))
    {
        forAllPoints([&] (GA_Offset pt) {
                    h_dest.set(pt, h_src.get(pt));
        });
    }
    // Translate only:
    else if (UT_Matrix3D(xform).isIdentity())
    {
        UT_Vector3D t;
        xform.getTranslates(t);

        forAllPoints([&] (GA_Offset pt) {
                    h_dest.set(pt, h_src.get(pt) + t);
        });
    }
    // Transform normals:
    else if (type == GA_TYPE_NORMAL)
    {
        UT_Matrix4D ixform;
        xform.invert(ixform);

        if (UT_Matrix3D(xform).determinant() < 0)
            ixform.scale(-1, -1, -1);

        forAllPoints([&](GA_Offset pt) {
                    UT_Vector3F n = h_src.get(pt);
                    n.colVecMult3(ixform);
                    h_dest.set(pt, n);
        });
    }
    // Transform points:
    else 
    {
        forAllPoints([&](GA_Offset pt) {
                h_dest.set(pt, h_src.get(pt) * xform);
        });
    }
}

bool
LidarImporter::importAndTransformCloud()
{
    // Read any required lidar data into the detail.
    if (!readPoints())
    {
        myGdp->clearAndDestroy();
        myCache->clearCache();
        return false;
    }

    // Set detail's position precision
    if (hasPrecisionChanged())
    {
        GA_Storage pos_storage;
        if (myParms.getPrecision() == "64")
            pos_storage = GA_STORE_REAL64;
        else
            pos_storage = GA_STORE_REAL32;

        GA_Attribute *pos = myGdp->getP();
        const GA_AIFTuple *tuple = pos->getAIFTuple();
        tuple->setStorage(pos, pos_storage);
    }

    UT_Matrix4D xform = buildXform();

    bool update = isClearDetailRequired() || hasTransformChanged();
    if (update || (hasPrecisionChanged() && myParms.getPrecision() == "64"))
    {
        copyAndTransform(xform, GA_TYPE_POINT);
    }

    if (update || (hasNormalsChanged() && myParms.getNormals()))
    {
        copyAndTransform(xform, GA_TYPE_NORMAL);
    }

    myCache->updateCachedParms(myParms);
    return true;
}

// Double precision version of guSolidCube() called in GU_Detail::cube()
void
LidarImporter::buildInfoCube(const UT_BoundingBoxD &bbox)
{
#define QUAD_VERTEX_OFFSET(prim_num, vtx_num) ((prim_num) * 4 + (vtx_num))

    GA_Offset offset = myGdp->appendPointBlock(8);
    GA_Offset start_vtxoff;
    myGdp->appendPrimitivesAndVertices(GA_PRIMPOLY, 6, 4, start_vtxoff, true);
   
    // Left Bottom Front corner
    myGdp->setPos3(offset, bbox.xmin(), bbox.ymin(), bbox.zmin());
    myGdp->setVertexPoint(start_vtxoff + QUAD_VERTEX_OFFSET(3, 0), offset);
    myGdp->setVertexPoint(start_vtxoff + QUAD_VERTEX_OFFSET(0, 3), offset);
    myGdp->setVertexPoint(start_vtxoff + QUAD_VERTEX_OFFSET(4, 2), offset);

    // Right Bottom Front corner
    ++offset;
    myGdp->setPos3(offset, bbox.xmax(), bbox.ymin(), bbox.zmin());
    myGdp->setVertexPoint(start_vtxoff + QUAD_VERTEX_OFFSET(0, 0), offset);
    myGdp->setVertexPoint(start_vtxoff + QUAD_VERTEX_OFFSET(1, 3), offset);
    myGdp->setVertexPoint(start_vtxoff + QUAD_VERTEX_OFFSET(4, 1), offset);

    // Right Bottom Back corner
    ++offset;
    myGdp->setPos3(offset, bbox.xmax(), bbox.ymin(), bbox.zmax());
    myGdp->setVertexPoint(start_vtxoff + QUAD_VERTEX_OFFSET(1, 0), offset);
    myGdp->setVertexPoint(start_vtxoff + QUAD_VERTEX_OFFSET(2, 3), offset);
    myGdp->setVertexPoint(start_vtxoff + QUAD_VERTEX_OFFSET(4, 0), offset);

    // Left Bottom Back corner
    ++offset;
    myGdp->setPos3(offset, bbox.xmin(), bbox.ymin(), bbox.zmax());
    myGdp->setVertexPoint(start_vtxoff + QUAD_VERTEX_OFFSET(2, 0), offset);
    myGdp->setVertexPoint(start_vtxoff + QUAD_VERTEX_OFFSET(3, 3), offset);
    myGdp->setVertexPoint(start_vtxoff + QUAD_VERTEX_OFFSET(4, 3), offset);

    // Left Top Front corner
    ++offset;
    myGdp->setPos3(offset, bbox.xmin(), bbox.ymax(), bbox.zmin());
    myGdp->setVertexPoint(start_vtxoff + QUAD_VERTEX_OFFSET(3, 1), offset);
    myGdp->setVertexPoint(start_vtxoff + QUAD_VERTEX_OFFSET(0, 2), offset);
    myGdp->setVertexPoint(start_vtxoff + QUAD_VERTEX_OFFSET(5, 3), offset);

    // Right Top Front corner
    ++offset;
    myGdp->setPos3(offset, bbox.xmax(), bbox.ymax(), bbox.zmin());
    myGdp->setVertexPoint(start_vtxoff + QUAD_VERTEX_OFFSET(0, 1), offset);
    myGdp->setVertexPoint(start_vtxoff + QUAD_VERTEX_OFFSET(1, 2), offset);
    myGdp->setVertexPoint(start_vtxoff + QUAD_VERTEX_OFFSET(5, 0), offset);

    // Right Top Back corner
    ++offset;
    myGdp->setPos3(offset, bbox.xmax(), bbox.ymax(), bbox.zmax());
    myGdp->setVertexPoint(start_vtxoff + QUAD_VERTEX_OFFSET(1, 1), offset);
    myGdp->setVertexPoint(start_vtxoff + QUAD_VERTEX_OFFSET(2, 2), offset);
    myGdp->setVertexPoint(start_vtxoff + QUAD_VERTEX_OFFSET(5, 1), offset);

    // Left Top Back corner
    ++offset;
    myGdp->setPos3(offset, bbox.xmin(), bbox.ymax(), bbox.zmax());
    myGdp->setVertexPoint(start_vtxoff + QUAD_VERTEX_OFFSET(2, 1), offset);
    myGdp->setVertexPoint(start_vtxoff + QUAD_VERTEX_OFFSET(3, 2), offset);
    myGdp->setVertexPoint(start_vtxoff + QUAD_VERTEX_OFFSET(5, 2), offset);
}

// Read metadata into an info or bounding box info detail.
bool
LidarImporter::readInfo()
{
    using namespace SOP_LidarImportEnums;
    UT_String filename;
    filename = myParms.getFilename();

    if (myCookparms.error()->getSeverity() >= UT_ERROR_ABORT
        || !filename.isstring()
        || !(filename.matchFileExtension(".las")
             || filename.matchFileExtension(".laz")
             || filename.matchFileExtension(".e57")))
    {
        myCache->clearCache();
        return false;
    }

    if (!isReadInfoRequired())
        return true;

    myGdp->clearAndDestroy();
    myCache->clearCache();

    // Scrape metadata.
    exint num_scans = 0;
    exint total_points = 0;
    UT_BoundingBoxD total_box;

    UT_StringArray scannames;
    UT_ExintArray pointcounts;
    UT_Array<UT_BoundingBoxD> boxes;
    UT_Array<UT_Matrix4D> xforms;

    if (filename.matchFileExtension(".las")
        || filename.matchFileExtension(".laz"))
    {
        LASReader reader(myParms.getFilename().c_str());
        if (!reader.isValid())
        {
            myCookparms.sopAddError(
                    SOP_ERR_LIDAR_READER_ERROR, "Failed to open the file.");
            return false;
        }

        // LAS is single-scan.
        num_scans = 1;
        total_points = reader.getPointCount();
        total_box = reader.getBoundingBox();

        scannames.append(filename.pathUpToExtension().fileName());
        pointcounts.append(reader.getPointCount());
        boxes.append(reader.getBoundingBox());

        // LAS doesn't have a transform.
        UT_Matrix4D mat;
        mat.identity();
        xforms.append(mat);
    }
    else if (filename.matchFileExtension(".e57"))
    {
        try
        {
            E57Reader reader(filename.c_str());
            num_scans = reader.getNumScans();
            total_points = reader.getPointsInFile();
            total_box = reader.getFileBoundingBox();

            for (int i = 0; i < num_scans; ++i)
            {
                scannames.append(reader.getScanName(i));
                pointcounts.append(reader.getPointsInScan(i));
                boxes.append(reader.getScanBoundingBox(i));
                xforms.append(reader.getScanRBXForm(i));
            }
        }
#ifdef E57_SOP_VERBOSE
        catch (e57::E57Exception &e)
        {
            std::string context = e.context();
            addError(SOP_ERR_LIDAR_READER_ERROR, context.c_str());
            e.report(__FILE__, __LINE__, __FUNCTION__);
            return false;
        }
#else
        catch (e57::E57Exception &e)
        {
            std::string context = e.context();
            myCookparms.sopAddError(
                    SOP_ERR_LIDAR_READER_ERROR, context.c_str());
            return false;
        }
#endif
    }

    // Set precision for position and bounds
    GA_Storage pos_storage;
    if (myParms.getPrecision() == "64")
        pos_storage = GA_STORE_REAL64;
    else
        pos_storage = GA_STORE_REAL32;

    GA_Attribute *pos = myGdp->getP();
    const GA_AIFTuple *ptuple = pos->getAIFTuple();
    ptuple->setStorage(pos, pos_storage);

    // Common attribs between both info loadtypes:
    GA_Attribute *bounds = myGdp->addFloatTuple(GA_ATTRIB_POINT, "bounds", 6);
    const GA_AIFTuple *boundstuple = bounds->getAIFTuple();
    boundstuple->setStorage(bounds, pos_storage);

    GA_RWHandleD h_bounds(bounds);
    GA_RWHandleI h_pointcount(
            myGdp->addIntTuple(GA_ATTRIB_POINT, "pointcount", 1));
    GA_RWHandleS h_filename(
            myGdp->addStringTuple(GA_ATTRIB_POINT, "filename", 1));

    // Load metadata into the detail:
    switch (myParms.getLoadtype())
    {
    case Loadtype::INFOBBOX:
    {
        if (total_box.isValid())
            buildInfoCube(total_box);
        else
            myGdp->appendPointOffset();

        for (GA_Iterator it(myGdp->getPointRange()); !it.atEnd(); ++it)
        {
            if (h_filename.isValid())
                h_filename.set(*it, filename.c_str());
            if (h_pointcount.isValid())
                h_pointcount.set(*it, total_points);
            if (h_bounds.isValid())
                h_bounds.setV(*it, total_box.data(), 6);
        }
        break;
    }
    case Loadtype::INFO:
    {
        GA_RWHandleS h_scanname;
        GA_RWHandleI h_scanpointcount;
        GA_RWHandleD h_transform;
        
        if (filename.matchFileExtension(".e57"))
        {
            h_scanname.bind(
                    myGdp->addStringTuple(GA_ATTRIB_POINT, "scanname", 1));
            h_scanpointcount.bind(
                    myGdp->addIntTuple(GA_ATTRIB_POINT, "scanpointcount", 1));
            h_transform.bind(
                    myGdp->addFloatTuple(GA_ATTRIB_POINT, "transform", 16));
        }

         // Each point in the detail corresponds to a scan.
        for (int i = 0; i < num_scans; ++i)
        {
            GA_Offset ptoff;
            ptoff = myGdp->appendPointOffset();

            if (boxes(i).isValid())
                myGdp->setPos3(ptoff, boxes(i).center());
            
            if (h_filename.isValid())
                h_filename.set(ptoff, filename.c_str());
            if (h_pointcount.isValid())
                h_pointcount.set(ptoff, total_points);
            if (h_scanname.isValid())
                h_scanname.set(ptoff, scannames(i).c_str());
            if (h_scanpointcount.isValid())
                h_scanpointcount.set(ptoff, pointcounts(i));
            if (h_transform.isValid())
                h_transform.setV(ptoff, xforms(i).data(), 16);
            if (h_bounds.isValid())
                h_bounds.setV(ptoff, boxes(i).data(), 6);
        }
        break;
    }
    default:
        return false;
    }

    myCache->setFileBBox(total_box);
    myCache->setPointBBox(total_box);

    return true;
}

bool
LidarImporter::importAndTransformInfo()
{
    // Read any required lidar data into the detail.
    if (!readInfo())
    {
        myGdp->clearAndDestroy();
        myCache->clearCache();
        return false;
    }

    UT_Matrix4D xform = buildXform();
    myGdp->transformPoints<fpreal64>(xform, myGdp->getPointRange());

    return true;
}

bool
LidarImporter::import()
{
    using namespace SOP_LidarImportEnums;

    switch (myParms.getLoadtype())
    {
    case Loadtype::POINTS:
    {
        return importAndTransformCloud();
    }
    case Loadtype::INFOBBOX:
    case Loadtype::INFO:
    {
        return importAndTransformInfo();
    }
    default:
        return false;
    }
}

} // namespace

//******************************************************************************
//*                         SOP Cook and Callbacks                             *
//******************************************************************************

void
SOP_LidarImportVerb::cook(const SOP_NodeVerb::CookParms &cookparms) const
{
    LidarImporter importer(cookparms);
    importer.import();
}

// Moves the centroid and handle to the origin. This method follows that of
// SOP_Transform, without the "invertxform" toggle.
// If metadata_centroid is toggled on, the center of the file metadata bounding
// box is used. Otherwise, the computed geometry centroid is used.
bool
SOP_LidarImport::moveCentroidToOrigin(fpreal now)
{
    using namespace SOP_LidarImportEnums;

    UT_AutoUndoBlock u("Move Centroid To Origin", ANYLEVEL);
    auto *cache = static_cast<SOP_LidarImportCache *>(myNodeVerbCache);

    // Build the pre-xform matrix
    UT_Matrix4D pre_xform;
    OP_Node::buildXform(
            PRETRANSFORM_TRS(), PRETRANSFORM_XYZ(), PRETRANSFORM_TX(now),
            PRETRANSFORM_TY(now), PRETRANSFORM_TZ(now), PRETRANSFORM_RX(now),
            PRETRANSFORM_RY(now), PRETRANSFORM_RZ(now), PRETRANSFORM_SX(now),
            PRETRANSFORM_SY(now), PRETRANSFORM_SZ(now),
            PRETRANSFORM_SHEAR_XY(now), PRETRANSFORM_SHEAR_XZ(now),
            PRETRANSFORM_SHEAR_YZ(now),
            0.0, 0.0, 0.0,
            pre_xform);

    UT_Matrix4D pre_xform_inv = pre_xform;
    pre_xform_inv.invert();

    // Build the xform matrix
    UT_Matrix4D xform;
    const fpreal scale = SCALE(now);
    const UT_Vector3D pr(PIVOT_RX(now), PIVOT_RY(now), PIVOT_RZ(now));
    UT_Vector3D p(PX(now), PY(now), PZ(now));

    OP_Node::buildXform(
            TRS(), XYZ(), TX(now), TY(now), TZ(now), RX(now), RY(now), RZ(now),
            SX(now) * scale, SY(now) * scale, SZ(now) * scale, SHEAR_XY(now),
            SHEAR_XZ(now), SHEAR_YZ(now), UT_Matrix4D::PivotSpace(p, pr),
            xform);

    xform *= pre_xform;

    // Use the file metadata or point bounding box center as the centroid.
    UT_Vector3D centroid;
    if (evalInt("centroid", 0, 0) == (exint)Centroid::CALCULATED)
    {
        if (!cache->isPointBBoxValid())
            cache->updatePointBBox();
        
        centroid = cache->getPointBBox().center();
    }
    else
    {
        centroid = cache->getFileBBox().center();
    }

    // Translate shift for moving the centroid to the origin.
    UT_Vector3D delta = centroid;

    // Current centroid's location. Translate xform by -1*centroid to center it.
    centroid *= xform;
    centroid *= -1.0;
    xform.translate(centroid.x(), centroid.y(), centroid.z());

    // Target is the point that the pre-transform moves to the origin. We want
    // to include the target offset in our delta.
    UT_Vector3D target_pivot(0.0, 0.0, 0.0);
    target_pivot *= pre_xform_inv;
    delta -= target_pivot;

    // With a nontrivial pivot rotate set, we would need to apply our inverse
    // pivot-rotate to -p to get the correct translate value. However, it would
    // cancel out with itself in getPivotParmValue(). We get the same effect by
    // specifying a translate of -p and no pivot-rotate.
    p = OP_Node::getPivotParmValue(
            TRS(), -delta.x(), -delta.y(), -delta.z(),
            target_pivot.x(), target_pivot.y(), target_pivot.z());

    // Snap pivot values close to default
    for (int i = 0; i < 3; i++)
    {
        if (SYSequalZero(p(i)))
            p(i) = 0.0;
    }

    // Undo the pre-transform. The centroid offset from earlier is still applied.
    xform *= pre_xform_inv;

    // Explode the shifted xform with our pivot value.
    UT_XformOrder xord;
    OP_Node::buildXformOrder(TRS(), XYZ(), xord);

    UT_Vector3D r, s, t, shear;
    xform.explode(xord, r, s, t, UT_Matrix4D::PivotSpace(p, pr), &shear);

    r.radToDeg();
    s /= SCALE(now);

    // Set xform parameters
    blockModify(true);
    for (int i = 0; i < 3; i++)
    {
        // Snap values close to the default exactly to avoid them from
        // showing up bolded in the parm pane.
        if (SYSequalZero(t(i)))
            t(i) = 0.0;
        if (SYSequalZero(r(i)))
            r(i) = 0.0;
        if (SYSisEqual(s(i), 1.0))
            s(i) = 1.0;
        if (SYSequalZero(shear(i)))
            shear(i) = 0.0;

        setFloat("t", i, now, t(i));
        setFloat("r", i, now, r(i));
        setFloat("s", i, now, s(i));
        setFloat("shear", i, now, shear(i));
        setFloat("p", i, now, p(i));
    }
    setFloat("scale", 0, now, scale);
    blockModify(false);

    return true;
}

// Moves pivot to the centroid of the detail.
// If metadata_centroid is toggled on, the center of the file metadata bounding
// box is used. Otherwise, the computed geometry centroid is used.
bool
SOP_LidarImport::movePivotToCentroid(fpreal now)
{
    using namespace SOP_LidarImportEnums;

    UT_AutoUndoBlock u("Move Pivot To Centroid", ANYLEVEL);
    auto *cache = static_cast<SOP_LidarImportCache *>(myNodeVerbCache);

    // Build the pre-xform matrix
    UT_Matrix4D pre_xform;
    OP_Node::buildXform(
            PRETRANSFORM_TRS(), PRETRANSFORM_XYZ(), PRETRANSFORM_TX(now),
            PRETRANSFORM_TY(now), PRETRANSFORM_TZ(now), PRETRANSFORM_RX(now),
            PRETRANSFORM_RY(now), PRETRANSFORM_RZ(now), PRETRANSFORM_SX(now),
            PRETRANSFORM_SY(now), PRETRANSFORM_SZ(now),
            PRETRANSFORM_SHEAR_XY(now), PRETRANSFORM_SHEAR_XZ(now),
            PRETRANSFORM_SHEAR_YZ(now),
            0.0, 0.0, 0.0,
            pre_xform);

    // Build the xform matrix
    UT_Matrix4D xform;
    const fpreal scale = SCALE(now);
    const UT_Vector3D pr(PIVOT_RX(now), PIVOT_RY(now), PIVOT_RZ(now));
    UT_Vector3D p(PX(now), PY(now), PZ(now));

    OP_Node::buildXform(
	    TRS(), XYZ(),
	    TX(now), TY(now), TZ(now),
	    RX(now), RY(now), RZ(now),
	    SX(now)*scale, SY(now)*scale, SZ(now)*scale,
	    SHEAR_XY(now), SHEAR_XZ(now), SHEAR_YZ(now),
	    UT_Matrix4D::PivotSpace(p, pr),
	    xform);

    // Set pivot to the centroid
    UT_Vector3D centroid;
    if (evalInt("centroid", 0, 0) == (exint)Centroid::CALCULATED)
    {
        if (cache->isPointBBoxValid())
            cache->updatePointBBox();

        centroid = cache->getPointBBox().center();
    }
    else
    {
        centroid = cache->getFileBBox().center();
    }
    p = centroid;

    // Snap pivot values close to default
    for (int i = 0; i < 3; i++)
    {
        if (SYSequalZero(p(i)))
            p(i) = 0.0;
    }

    // Build the new xform with the updated pivot, p
    UT_Matrix4D xform_new;
    OP_Node::buildXform(
	TRS(), XYZ(),
	TX(now), TY(now), TZ(now),
	RX(now), RY(now), RZ(now),
	SX(now)*scale, SY(now)*scale, SZ(now)*scale,
	SHEAR_XY(now), SHEAR_XZ(now), SHEAR_YZ(now),
	UT_Matrix4D::PivotSpace(p, pr),
	xform_new);

    // Get new pre-transform matrix
    UT_Matrix4D composition = xform * pre_xform;

    // Compute new matrix
    UT_Matrix4D xform_new_inv = xform_new;
    xform_new_inv.invert();
    UT_Matrix4D pre_xform_new = xform_new_inv * composition;

    // Explode new pre-transform matrix
    UT_XformOrder xord;
    OP_Node::buildXformOrder(PRETRANSFORM_TRS(), PRETRANSFORM_XYZ(), xord);

   UT_Vector3D r, s, t, shear;
    pre_xform_new.explode(xord, r, s, t, &shear);

    r.radToDeg();
    s /= scale;

    // Set xform parameters
    blockModify(true);
    for (int i = 0; i < 3; i++)
    {
        // Snap values close to the default exactly to avoid them from
        // showing up bolded in the parm pane.
        if (SYSequalZero(t(i)))
            t(i) = 0.0;
        if (SYSequalZero(r(i)))
            r(i) = 0.0;
        if (SYSisEqual(s(i), 1.0))
            s(i) = 1.0;
        if (SYSequalZero(shear(i)))
            shear(i) = 0.0;

        setFloat("p", i, now, p(i));
        setFloat("prexform_t", i, now, t(i));
        setFloat("prexform_r", i, now, r(i));
        setFloat("prexform_s", i, now, s(i));
        setFloat("prexform_shear", i, now, shear(i));
    }
    blockModify(false);

    return true;
}

int
SOP_LidarImport::moveCentroidToOriginCB(
	void *data, int, fpreal t, const PRM_Template *)
{
    return static_cast<SOP_LidarImport *>(data)->moveCentroidToOrigin(t);
}

int
SOP_LidarImport::movePivotToCentroidCB(
	void *data, int, fpreal t, const PRM_Template *)
{
    return static_cast<SOP_LidarImport *>(data)->movePivotToCentroid(t);
}
