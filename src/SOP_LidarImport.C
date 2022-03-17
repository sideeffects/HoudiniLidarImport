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
	    name    "group_prefix"
	    label   "Group Prefix"
	    type    string
	    default { "lidar_group" }
    }
        groupsimple {
	name	    "filtering"
	label	    "Filtering"
	grouptag    { "group_type" "simple" }

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
	    default { "0" "0" }
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
	    name    "row_col"
	    label   "Row and Column"
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
    parm {
	    name    "normals"
	    label   "Surface Normals"
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
	    label   "Scan Names"
	    type    toggle
	    default { "off" }
    }
    } // Attributes
}
)THEDSFILE";

SOP_LidarImport::SOP_LidarImport(OP_Network *net, const char *name, OP_Operator *op)
    : SOP_Node(net, name, op)
{
}

SOP_LidarImport::~SOP_LidarImport() {}

OP_ERROR
SOP_LidarImport::cookMySop(OP_Context &context)
{
    return cookMyselfAsVerb(context);
}

class SOP_LidarImportCache : public SOP_NodeCache
{
public:
    SOP_LidarImportCache() : SOP_NodeCache() {}
    ~SOP_LidarImportCache() override {}

    SOP_LidarImportParms myParms;
    UT_StringMap<UT_IntArray> myScanGroupMap;

    void clearCache() 
    { 
	myParms = SOP_LidarImportParms();
        myScanGroupMap.clear();
    }

    // Convenience methods:
    // return true if attribute has been changed for this cook
    bool hasGroupPrefixChanged(const SOP_LidarImportParms &sopparms) const
    { return sopparms.getGroup_prefix() != myParms.getGroup_prefix(); }

    bool hasColorChanged(const SOP_LidarImportParms &sopparms) const
    { return sopparms.getColor() != myParms.getColor(); }

    bool hasIntensityChanged(const SOP_LidarImportParms &sopparms) const
    { return sopparms.getIntensity() != myParms.getIntensity();}

    bool hasRowColChanged(const SOP_LidarImportParms &sopparms) const
    { return sopparms.getRow_col() != myParms.getRow_col(); }

    bool hasRetDataChanged(const SOP_LidarImportParms &sopparms) const
    { return sopparms.getRet_data() != myParms.getRet_data(); }

    bool hasTimestampChanged(const SOP_LidarImportParms &sopparms) const
    { return sopparms.getTimestamp() != myParms.getTimestamp(); }

    bool hasNormalsChanged(const SOP_LidarImportParms &sopparms) const
    { return sopparms.getNormals() != myParms.getNormals(); }

    bool hasRigidTransformsChanged(const SOP_LidarImportParms &sopparms) const
    { return sopparms.getRigidtransforms() != myParms.getRigidtransforms(); }

    bool hasScanNamesChanged(const SOP_LidarImportParms &sopparms) const
    { return sopparms.getScannames() != myParms.getScannames(); }

    void updateCache(const SOP_LidarImportParms& sopparms)
    {
        myParms.setFilename(sopparms.getFilename());
        myParms.setGroup_prefix(sopparms.getGroup_prefix());
        myParms.setFilter_type(sopparms.getFilter_type());
        myParms.setSelect_range(sopparms.getSelect_range());
        myParms.setMax_points(sopparms.getMax_points());
        myParms.setDelete_invalid(sopparms.getDelete_invalid());
        myParms.setColor(sopparms.getColor());
        myParms.setIntensity(sopparms.getIntensity());
        myParms.setRow_col(sopparms.getRow_col());
        myParms.setRet_data(sopparms.getRet_data());
        myParms.setTimestamp(sopparms.getTimestamp());
        myParms.setNormals(sopparms.getNormals());
        myParms.setRigidtransforms(sopparms.getRigidtransforms());
        myParms.setScannames(sopparms.getScannames());
    }

    // Returns true if the entire Detail needs to be cleared and re-generated.
    bool isClearDetailDataRequired(const SOP_LidarImportParms &sopparms) const;
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

//******************************************************************************
//*				  UTILITIES                                    *
//******************************************************************************
namespace
{
static bool
isRangeValid(exint range_good, exint range_size)
{
    return (range_good < range_size && range_good > 0 && range_size > 0);
}

static void
computeRangeFromMaxPoints(
        exint max_points,
        exint pts_in_scan,
        int &range_good,
        int &range_size)
{
    range_good = (int)max_points;
    range_size = (int)pts_in_scan;

    // Each iteration of the loop reduces the number of points that will
    // be read from the scan. However, the range values must be small so
    // that the point cloud is sampled evenly, and detail loss is reduced.
    while (range_good > 10)
    {
        range_good = SYSfloor(range_good / 10.0);
        range_size = SYSceil(range_size / 10.0);
    }
}
} // namespace

bool
SOP_LidarImportCache::
isClearDetailDataRequired(const SOP_LidarImportParms &sopparms) const
{
    // Filename changed:
    if (myParms.getFilename() != sopparms.getFilename())
        return true;

    // Filter type and relevant parms changed (if parms are valid values):
    bool result = myParms.getFilter_type() != sopparms.getFilter_type();
    switch (sopparms.getFilter_type())
    {
    case SOP_LidarImportEnums::Filter_type::RANGE_FILTER:
    {
        result |= myParms.getSelect_range() != sopparms.getSelect_range()
                  && isRangeValid(
                          sopparms.getSelect_range()[0],
                          sopparms.getSelect_range()[1]);
        break;
    }
    case SOP_LidarImportEnums::Filter_type::MAX_FILTER:
    {
        result |= myParms.getMax_points() != sopparms.getMax_points()
                  && myParms.getMax_points() > 0;
        break;
    }
    case SOP_LidarImportEnums::Filter_type::NO_FILTER:
    default:
        break;
    }

    if (result)
        return true;

    // E57 invalid group handling changed:
    UT_String filename;
    filename = myParms.getFilename();
    if (filename.matchFileExtension(".e57"))
    {
        if (sopparms.getDelete_invalid() && !myParms.getDelete_invalid())
            return true;
    }

    return false;
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
    ~LASReader();

    // Open file and read metadata. Returns true if successful.
    bool openStream(std::istream &stream);
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
    SYS_FORCE_INLINE bool hasClassificationFlagOverlap() const
    {
        return myHasClassificationFlagOverlap;
    }
    SYS_FORCE_INLINE bool hasScannerChannel() const
    {
        return myHasScannerChannel;
    }

    SYS_FORCE_INLINE UT_Vector3D getScale() const { return myScale; }
    SYS_FORCE_INLINE UT_Vector3D getOffset() const { return myOffset; }
    SYS_FORCE_INLINE UT_BoundingBoxD getBoundingBox() const
    {
        return myBoundingBox;
    }

    // myScale and myOffset must be applied to get the true xyz coords.
    SYS_FORCE_INLINE void getXYZ(UT_Vector3F &xyz) const;
    SYS_FORCE_INLINE void getIntensity(fpreal32 &in) const;
    SYS_FORCE_INLINE void getUserData(int8 &in) const;
    SYS_FORCE_INLINE void getPointSourceID(int16 &in) const;
    SYS_FORCE_INLINE void getGPSTime(fpreal64 &in) const;
    SYS_FORCE_INLINE void getRGB(UT_Vector3F &rgb) const;
    SYS_FORCE_INLINE void getNIR(fpreal32 &in) const;

    // These methods differ between legacy formats (0-5) and LAS 1.4 (6-10).
    // my<Var> and my<Var>Mask members are set based on the point format
    // to get the correct behavior.
    SYS_FORCE_INLINE void getReturnNumber(uint8 &in) const;
    SYS_FORCE_INLINE void getReturnCount(uint8 &in) const;
    SYS_FORCE_INLINE void getClassificationFlagSynthetic(bool &in) const;
    SYS_FORCE_INLINE void getClassificationFlagKeyPoint(bool &in) const;
    SYS_FORCE_INLINE void getClassificationFlagWithheld(bool &in) const;
    SYS_FORCE_INLINE void getClassificationFlagOverlap(bool &in) const;
    SYS_FORCE_INLINE void getScannerChannel(uint8 &in) const;
    SYS_FORCE_INLINE void getScanDirectionFlag(bool &in) const;
    SYS_FORCE_INLINE void getEdgeFlightLineFlag(bool &in) const;
    SYS_FORCE_INLINE void getClassification(uint8 &in) const;
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
    UT_BoundingBoxD myBoundingBox;

    // Available attributes that differ between point formats
    bool myHasGPSTime;
    bool myHasRGB;
    bool myHasNIR;
    bool myHasClassificationFlagOverlap; // LAS 1.4 only.
    bool myHasScannerChannel;            // LAS 1.4 only.

    void initializeAvailableAttribs();
};

LASReader::~LASReader()
{
    if (myReader)
    {
        if (laszip_close_reader(myReader))
            UTdebugPrint("LASzip failed: laszip_close_reader");

        if (laszip_destroy(myReader))
            UTdebugPrint("LASzip failed: laszip_destroy");
    }
}

void
LASReader::initializeAvailableAttribs()
{
    if (myPointFormat > 5)
    {
        myHasClassificationFlagOverlap = true;
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

bool
LASReader::openStream(std::istream &stream)
{
    if (laszip_create(&myReader))
    {
        UTdebugPrint("LASzip failed: laszip_create");
        myReader = nullptr;
        return false;
    }

    if (laszip_open_reader_stream(myReader, stream, &myIsCompressed))
    {
        UTdebugPrint("LASzip failed: laszip_open_reader_stream");

        if (laszip_destroy(myReader))
            UTdebugPrint("LASzip failed: laszip_destroy");

        myReader = nullptr;
        return false;
    }

    if (laszip_get_header_pointer(myReader, &myHeader))
    {
        UTdebugPrint("LASzip failed: laszip_get_header_pointer");
        return false;
    }

    if (laszip_get_point_pointer(myReader, &myPoint))
    {
        UTdebugPrint("LASzip failed: laszip_get_point_pointer");
        return false;
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

    myBoundingBox.setBounds(
            myHeader->min_x, myHeader->min_y, myHeader->min_z, myHeader->max_x,
            myHeader->max_y, myHeader->max_z);

    // Ensure that the header was read correctly
    myPointFormat = myHeader->point_data_format;
    if (myPointFormat > 10)
    {
        return false;
    }

    initializeAvailableAttribs();
    return true;
}

void
LASReader::getXYZ(UT_Vector3F &xyz) const
{
    xyz = UT_Vector3D(myPoint->X, myPoint->Y, myPoint->Z) * myScale + myOffset;
}

void
LASReader::getIntensity(fpreal32 &in) const
{
    in = static_cast<fpreal32>(myPoint->intensity) / UINT16_MAX;
}

void
LASReader::getUserData(int8 &in) const
{
    in = myPoint->user_data;
}

void
LASReader::getPointSourceID(int16 &in) const
{
    in = myPoint->point_source_ID;
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
LASReader::getClassificationFlagSynthetic(bool &in) const
{
    if (myPointFormat > 5)
        in = myPoint->extended_classification_flags & 0x1;
    else
        in = myPoint->synthetic_flag;
}

void
LASReader::getClassificationFlagKeyPoint(bool &in) const
{
    if (myPointFormat > 5)
        in = myPoint->extended_classification_flags & 0x2;
    else
        in = myPoint->keypoint_flag;
}

void
LASReader::getClassificationFlagWithheld(bool &in) const
{
    if (myPointFormat > 5)
        in = myPoint->extended_classification_flags & 0x4;
    else
        in = myPoint->withheld_flag;
}

void
LASReader::getClassificationFlagOverlap(bool &in) const
{
    in = myPoint->extended_classification_flags & 0x8;
}

void
LASReader::getScannerChannel(uint8 &in) const
{
    in = myPoint->extended_scanner_channel;
}

void
LASReader::getScanDirectionFlag(bool &in) const
{
    in = myPoint->scan_direction_flag;
}

void
LASReader::getEdgeFlightLineFlag(bool &in) const
{
    in = myPoint->edge_of_flight_line;
}

void
LASReader::getClassification(uint8 &in) const
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
        in = myPoint->extended_scan_angle * 0.006;
    else
        in = myPoint->scan_angle_rank;
}

// Read a .las or .laz file for a SOP cook. Returns true if successful.
bool
readLASFile(
        std::istream &stream,
        GU_Detail *gdp,
        UT_AutoInterrupt &boss,
        const SOP_NodeVerb::CookParms &cookparms)
{
    using namespace SOP_LidarImportEnums;
    auto &&sopparms = cookparms.parms<SOP_LidarImportParms>();
    SOP_LidarImportCache *cache
            = static_cast<SOP_LidarImportCache *>(cookparms.cache());

    // Pass stream to reader and scrape its metadata.
    LASReader reader;
    if (!reader.openStream(stream))
        return false;

    // Configure addition or removal of attributes
    GA_RWPageHandleV3 positionPH;
    if (cache->isClearDetailDataRequired(sopparms))
        positionPH.bind(gdp->getP());

    GA_RWPageHandleV3 colorPH;
    if (cache->hasColorChanged(sopparms))
    {
        if (sopparms.getColor() == Color::FROM_PTCLOUD)
        {
            if (reader.hasRGB())
                colorPH.bind(gdp->addDiffuseAttribute(GA_ATTRIB_POINT));
            else
		cookparms.sopAddWarning(SOP_WARN_ATTRIBS_NOT_FOUND, "Color");
        }
        else
        {
	    gdp->destroyDiffuseAttribute(GA_ATTRIB_POINT);
        }
    }

    GA_RWPageHandleF intensityPH;
    if (cache->hasIntensityChanged(sopparms))
    {
        if (sopparms.getIntensity())
	    intensityPH.bind(gdp->addFloatTuple(GA_ATTRIB_POINT, "intensity", 1));
        else
            gdp->destroyPointAttrib("intensity");
    }

    GA_PageHandleScalar<uint8>::RWType returnIndexPH;
    GA_PageHandleScalar<uint8>::RWType returnCountPH;
    if (cache->hasRetDataChanged(sopparms))
    {
        if (sopparms.getRet_data())
        {
            returnIndexPH.bind(
                    gdp->addIntTuple(GA_ATTRIB_POINT, "return_index", 1));
            returnCountPH.bind(
                    gdp->addIntTuple(GA_ATTRIB_POINT, "return_count", 1));
        }
        else
        {
            gdp->destroyPointAttrib("return_index");
            gdp->destroyPointAttrib("return_count");
        }
    }

    GA_RWPageHandleD timestampPH;
    if (cache->hasTimestampChanged(sopparms))
    {
        if (sopparms.getTimestamp())
        {
            if (reader.hasGPSTime())
                timestampPH.bind(
                        gdp->addFloatTuple(GA_ATTRIB_POINT, "timestamp", 1));
            else
                cookparms.sopAddWarning(
                        SOP_WARN_ATTRIBS_NOT_FOUND, "Timestamp");
        }
        else
        {
            gdp->destroyPointAttrib("timestamp");
        }
    }

    // Stop if no new attributes need to be read.
    if (!(positionPH.isValid() || colorPH.isValid() || intensityPH.isValid()
          || returnIndexPH.isValid() || timestampPH.isValid()))
        return true;

    // Configure filtering
    exint pts_in_file = reader.getPointCount();
    int range_good = GA_PAGE_SIZE;
    int range_size = GA_PAGE_SIZE;
    switch (sopparms.getFilter_type())
    {
    case Filter_type::RANGE_FILTER:
    {
        UT_Vector2I range_params = sopparms.getSelect_range();
        if (isRangeValid(range_params[0], range_params[1]))
        {
            range_good = range_params[0];
            range_size = range_params[1];
        }
        break;
    }
    case Filter_type::MAX_FILTER:
    {
        exint max_pts = sopparms.getMax_points();
        if (isRangeValid(max_pts, pts_in_file))
            computeRangeFromMaxPoints(
                    max_pts, pts_in_file, range_good, range_size);
        break;
    }
    case Filter_type::NO_FILTER:
    default:
        break;
    }

    // Compute the detail's point size, from the filter params.
    exint num_pts = pts_in_file;
    if (isRangeValid(range_good, range_size))
    {
        num_pts = pts_in_file / range_size;
        num_pts *= range_good;

        exint remaining_pts = pts_in_file % range_size;
        num_pts += SYSmin(range_good, remaining_pts);
    }

    // Reserve geometry if position needs to be read.
    if (positionPH.isValid())
        gdp->appendPointBlock(num_pts);
    UT_ASSERT(gdp->getNumPoints() == num_pts);

    // Main loops for reading data into the page handles:
    exint p_index = 0;
    exint range = 0;
    exint seek_size = static_cast<exint>(range_size - range_good);
    GA_Offset start, end;
    for (GA_Iterator it(gdp->getPointRange()); it.blockAdvance(start, end);)
    {
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
        if (timestampPH.isValid())
            timestampPH.setPage(start);

        for (GA_Offset ptoff = start; ptoff < end; ++ptoff, ++range, ++p_index)
        {
            // Skip over points outside of the filter range.
            if (seek_size && range == range_good)
            {
                range = 0;
                p_index += seek_size;
                reader.seekPoint(p_index);
            }

            // Read point and fill page elements.
            reader.readNextPoint();

            if (positionPH.isValid())
                reader.getXYZ(positionPH.value(ptoff));
            if (colorPH.isValid())
                reader.getRGB(colorPH.value(ptoff));
            if (intensityPH.isValid())
                reader.getIntensity(intensityPH.value(ptoff));
            if (returnIndexPH.isValid() && returnCountPH.isValid())
            {
                reader.getReturnNumber(returnIndexPH.value(ptoff));
                reader.getReturnCount(returnCountPH.value(ptoff));
            }
            if (timestampPH.isValid())
            {
                reader.getGPSTime(timestampPH.value(ptoff));
            }
        }

        if (boss.wasInterrupted())
        {
            cache->clearCache();
            return true;
        }
    }
    return true;
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
	Builder(e57::ImageFile handle, int idx, sop_ScanInfo info, GU_Detail *gdp,
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

sop_E57PointReader::Builder::Builder(e57::ImageFile handle, int idx, 
	sop_ScanInfo info, GU_Detail *gdp, int range_good, int range_size)
    : myFileHandle(handle)
    , myScanInfo(info)
    , myScanIndex(idx)
    , myGdp(gdp)
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
    GA_Attribute *attrib = myGdp->getP();
    const char *e57_names[] = { "cartesianX", "cartesianY", "cartesianZ", 
	nullptr };

    myBufferInfos.append(new sop_E57BufferInfoV3T<fpreal32>(e57_names, attrib,
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
    GA_Attribute *attrib = myGdp->getP();
    const char *e57_names[] = 
	{ "sphericalRange", "sphericalAzimuth", "sphericalElevation", nullptr };

    auto callback = [](fpreal32 *b, GA_Offset start, GA_Offset end)
    {
	for (int i = 0, ni = end - start; i < ni; ++i)
	{
	    fpreal32 range = b[i*3];
	    fpreal32 azimuth = b[i*3+1];
	    fpreal32 elevation = b[i*3+2];

	    b[i*3] = range * SYScos(elevation) * SYScos(azimuth);
	    b[i*3+1] = range * SYScos(elevation) * SYSsin(azimuth);
	    b[i*3+2] = range * SYSsin(elevation);
	}
    };

    myBufferInfos.append(new sop_E57BufferInfoV3T<fpreal32>(e57_names, attrib, 
		myGdp, callback));
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
    GA_Attribute *attrib = myGdp->addFloatTuple(GA_ATTRIB_POINT, "timestamp", 1);

    myBufferInfos.append(new sop_E57BufferInfoT<fpreal32>("timeStamp", attrib, 
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
    GA_Attribute *attrib = myGdp->addNormalAttribute(GA_ATTRIB_POINT);
    const char *e57_names[] = { "nor:normalX", "nor:normalY", "nor:normalZ", 
	nullptr };

    myBufferInfos.append(new sop_E57BufferInfoV3T<fpreal32>(e57_names, attrib, 
		myGdp));
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

    int getNumScans() const;
    int getNumImages() const;

    exint getPointsInFile() const;
    exint getPointsInScan(int idx) const;

    UT_Matrix4D getScanRBXForm(int idx) const;
    UT_Matrix4D getImageRBXForm(int idx) const;

    const UT_StringHolder &getScanName(int idx) const
    { return myScans(idx).myName; }

    sop_E57PointReader::Builder createPointReaderBuilder(int idx, GU_Detail *gdp,
	    int range_good, int range_size);

    UT_UniquePtr<unsigned char[]> getImageBuffer(int idx,
                                                 bool read_ref_images) const;

    void getImageDimensions(int idx, exint &width, exint &height) const;
    size_t getImageSize(int idx) const;
    bool hasValidProjectionType(int idx) const;

    UT_StringHolder getScanGroupGuid(int idx);
    UT_StringHolder getImageAssociatedScanGuid(int idx);

    void computePointProjection(int idx, const UT_Vector4 pos, 
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
};

E57Reader::E57Reader(const char *filename)
    : myFileHandle(filename, "r")
{
    // NOTE: some of this code may throw an exception; it should be caught in cookMySop
    e57::StructureNode root_node = myFileHandle.root();
    e57::VectorNode scan_root_node(root_node.get("/data3D"));

    auto set_rbxform = [](e57::StructureNode parent, UT_Matrix4D &rbxform)
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


	myScans.append(scan);
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

int
E57Reader::getNumScans() const
{
    return myScans.entries();
}

int
E57Reader::getNumImages() const
{
    return myImages.entries();
}

exint
E57Reader::getPointsInFile() const
{
    return myPointsInFile;
}

exint
E57Reader::getPointsInScan(int idx) const
{
    return myScans(idx).myNumPoints;
}

UT_Matrix4D
E57Reader::getScanRBXForm(int idx) const
{
    return myScans(idx).myRBXForm;
}

UT_Matrix4D
E57Reader::getImageRBXForm(int idx) const
{
    return myImages(idx).myRBXForm;
}

sop_E57PointReader::Builder
E57Reader::createPointReaderBuilder(int idx, GU_Detail *gdp, 
						  int range_good, int range_size)
{
    return sop_E57PointReader::Builder(myFileHandle, idx, myScans(idx), gdp, 
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

UT_StringHolder
E57Reader::getScanGroupGuid(int idx)
{
    return myScans(idx).myGuid;
}

UT_StringHolder
E57Reader::getImageAssociatedScanGuid(int idx)
{
    return myImages(idx).myAssociatedScanGuid;
}

void
E57Reader::computePointProjection(int idx, const UT_Vector4 pos, 
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

bool
readE57Scan(
	    GU_Detail *gdp,
	    UT_AutoInterrupt &boss,
	    const SOP_NodeVerb::CookParms& cookparms,
	    UT_StringArray &missing_attribs,
	    E57Reader &reader,
	    int scan_index,
	    int64 &pt_idx,
	    exint scan_max_pts)
{
    using namespace SOP_LidarImportEnums;
    auto &&sopparms = cookparms.parms<SOP_LidarImportParms>();
    SOP_LidarImportCache *cache
            = static_cast<SOP_LidarImportCache *>(cookparms.cache());

    int range_good = GA_PAGE_SIZE;
    int range_size = GA_PAGE_SIZE;
    exint pts_in_scan = reader.getPointsInScan(scan_index);

    switch (sopparms.getFilter_type())
    {
    case Filter_type::RANGE_FILTER:
    {
        UT_Vector2I range_params = sopparms.getSelect_range();
        if (isRangeValid(range_params[0], range_params[1]))
        {
            range_good = range_params[0];
            range_size = range_params[1];
        }
        break;
    }
    case Filter_type::MAX_FILTER:
    {
        if (isRangeValid(scan_max_pts, pts_in_scan))
            computeRangeFromMaxPoints(
                    scan_max_pts, pts_in_scan, range_good, range_size);
        break;
    }
    case Filter_type::NO_FILTER:
    default:
        break;
    }
    sop_E57PointReader::Builder pt_reader_builder(
            reader.createPointReaderBuilder(
                    scan_index, gdp, range_good, range_size));

    
    bool read_file_required = false;
    bool reimport_required = cache->isClearDetailDataRequired(sopparms);
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
            cookparms.sopAddError(SOP_ERR_MISSING_POSITION);
            return true;
        }
    }

    if (cache->hasColorChanged(sopparms)
        && sopparms.getColor() == Color::FROM_PTCLOUD)
    {
        if (reader.hasColor(scan_index))
        {
            pt_reader_builder.color();

            if (reader.hasColorInvalid(scan_index))
                pt_reader_builder.colorInvalid();

            read_file_required = true;
        }
        else
        {
            missing_attribs.append(UT_StringHolder("color"));
        }
    }
    else if (cache->hasColorChanged(sopparms)
            && sopparms.getColor() != Color::FROM_PTCLOUD)
    {
        gdp->destroyDiffuseAttribute(GA_ATTRIB_POINT);

        GA_PointGroup *invalid_group = gdp->findPointGroup("e57_invalid_color");
        if (invalid_group)
            gdp->destroyPointGroup(invalid_group);
    }

    if (cache->hasIntensityChanged(sopparms) && sopparms.getIntensity())
    {
        if (reader.hasIntensity(scan_index))
        {
            pt_reader_builder.intensity();

            if (reader.hasIntensityInvalid(scan_index))
                pt_reader_builder.intensityInvalid();

            read_file_required = true;
        }
        else
        {
            missing_attribs.append(UT_StringHolder("intensity"));
        }
    }
    else if (cache->hasIntensityChanged(sopparms) && !sopparms.getIntensity())
    {
        gdp->destroyPointAttrib("intensity");

        GA_PointGroup *invalid_group
                = gdp->findPointGroup("e57_invalid_intensity");
        if (invalid_group)
            gdp->destroyPointGroup(invalid_group);
    }

    if (cache->hasRowColChanged(sopparms) && sopparms.getRow_col())
    {
        if (reader.hasRowCol(scan_index))
        {
            pt_reader_builder.rowIndex();
            pt_reader_builder.columnIndex();

            read_file_required = true;
        }
        else
        {
            missing_attribs.append(UT_StringHolder("row and column indices"));
        }
    }
    else if (cache->hasRowColChanged(sopparms) && !sopparms.getRow_col())
    {
        gdp->destroyPointAttrib("row_index");
        gdp->destroyPointAttrib("column_index");
    }

    if (cache->hasRetDataChanged(sopparms) && sopparms.getRet_data())
    {
        if (reader.hasRetData(scan_index))
        {
            pt_reader_builder.returnIndex();
            pt_reader_builder.returnCount();

            read_file_required = true;
        }
        else
        {
            missing_attribs.append(UT_StringHolder("return data"));
        }
    }
    else if (cache->hasRetDataChanged(sopparms) && !sopparms.getRet_data())
    {
        gdp->destroyPointAttrib("return_index");
        gdp->destroyPointAttrib("return_count");
    }

    if (cache->hasTimestampChanged(sopparms) && sopparms.getTimestamp())
    {
        if (reader.hasTimestamp(scan_index))
        {
            pt_reader_builder.timestamp();

            if (reader.hasTimestampInvalid(scan_index))
                pt_reader_builder.timestampInvalid();

            read_file_required = true;
        }
        else
        {
            missing_attribs.append(UT_StringHolder("timestamps"));
        }
    }
    else if (cache->hasTimestampChanged(sopparms) && !sopparms.getTimestamp())
    {
        gdp->destroyPointAttrib("timestamp");

        GA_PointGroup *invalid_group
                = gdp->findPointGroup("e57_invalid_timestamp");
        if (invalid_group)
            gdp->destroyPointGroup(invalid_group);
    }

    if (cache->hasNormalsChanged(sopparms) && sopparms.getNormals())
    {
        if (reader.hasNormals(scan_index))
        {
            pt_reader_builder.normals();

            read_file_required = true;
        }
        else
        {
            missing_attribs.append(UT_StringHolder("surface normals"));
        }
    }
    else if (cache->hasNormalsChanged(sopparms) && !sopparms.getNormals())
    {
        gdp->destroyNormalAttribute(GA_ATTRIB_POINT);
    }

    if (reimport_required
	|| cache->hasGroupPrefixChanged(sopparms)
        || cache->hasRigidTransformsChanged(sopparms))
    {
        UT_WorkBuffer name(sopparms.getGroup_prefix().c_str());
        name.appendFormat("transform{}", scan_index + 1);
        gdp->destroyAttribute(GA_ATTRIB_GLOBAL, name);

        if (sopparms.getRigidtransforms())
        {
            name = sopparms.getGroup_prefix();
            name.appendFormat("transform{}", scan_index + 1);

            GA_RWHandleM4D xform_h(gdp->addFloatTuple(
                    GA_ATTRIB_GLOBAL, name, 16,
                    GA_Defaults(GA_Defaults::matrix4()), 0, 0,
                    GA_STORE_REAL64));
            // Take transform from reader directly to avoid precision lost
            xform_h.set(GA_Offset(0), reader.getScanRBXForm(scan_index));
        }
    }

    GA_PointGroup *scan_group = nullptr;
    if (reimport_required)
    {
        UT_WorkBuffer group_name(sopparms.getGroup_prefix().c_str());
        group_name.appendFormat("{}", scan_index + 1);
        scan_group = gdp->newPointGroup(group_name.buffer());
    }
    else if (cache->hasGroupPrefixChanged(sopparms))
    {
        UT_WorkBuffer old_group_name(cache->myParms.getGroup_prefix().c_str());
        old_group_name.appendFormat("{}", scan_index + 1);
        UT_WorkBuffer new_group_name(sopparms.getGroup_prefix().c_str());
        new_group_name.appendFormat("{}", scan_index + 1);

        gdp->getElementGroupTable(GA_ATTRIB_POINT)
                .renameGroup(old_group_name.buffer(), new_group_name.buffer());
    }

    if (!read_file_required)
        return false;

    exint num_pts = pts_in_scan;
    if (isRangeValid(range_good, range_size))
    {
        num_pts = pts_in_scan / range_size;
        num_pts *= range_good;

        exint remaining_pts = pts_in_scan % range_size;
        num_pts += SYSmin(range_good, remaining_pts);
    }

    if (reimport_required)
        gdp->appendPointBlock(num_pts);

    sop_E57PointReader pt_reader(pt_reader_builder.build());

    GA_Range pt_range(
            gdp->getPointMap(), GA_Offset(pt_idx), GA_Offset(pt_idx + num_pts));

    if (reimport_required)
        scan_group->setElement(pt_range, true);

    GA_Offset start, end;
    for (GA_Iterator it(pt_range);
         it.blockAdvance(start, end) && pt_reader.readBlock(start, end);)
    {
        if (boss.wasInterrupted())
        {
            cache->clearCache();
            return true;
        }
    }

    pt_idx += num_pts;

    UT_Matrix4 rigid_body_transform(reader.getScanRBXForm(scan_index));
    if (reimport_required && !rigid_body_transform.isIdentity())
        gdp->transformPoints(rigid_body_transform, scan_group, nullptr, true);

    return false;
}

class sop_colorFromImageParallel
{
public:
    sop_colorFromImageParallel(
            const GU_Detail *gdp,
            GA_Attribute *c_attr,
            GA_Attribute *nc_attr,
            const PXL_Raster *raster,
            const E57Reader &reader,
            int img_idx,
            const UT_Matrix4 rbxform_inv)
        : myGdp(gdp)
        , myColorAttrib(c_attr)
        , myNumColorsAttrib(nc_attr)
        , myRaster(raster)
        , myReader(reader)
        , myImageIndex(img_idx)
        , myRBXFormInv(rbxform_inv)
    {
    }

    void operator()(const GA_SplittableRange &r) const
    {
        UT_ASSERT(
                myReader.hasValidProjectionType(myImageIndex)
                && "Invalid projection type");
        UT_Interrupt *boss = UTgetInterrupt();

        GA_RWPageHandleV3 color_ph(myColorAttrib);
        GA_RWPageHandleI num_colors_ph(myNumColorsAttrib);

        GA_Offset start, end;
        for (GA_Iterator it(r); it.blockAdvance(start, end);)
        {
            if (boss->opInterrupt())
                return;

            color_ph.setPage(start);
            num_colors_ph.setPage(start);

            for (GA_Offset ptoff = start; ptoff < end; ++ptoff)
            {
                UT_Vector4 pos(myGdp->getPos3(ptoff));
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
                    fpreal r = (*(unsigned char *)myRaster->getPixel(
                                       img_x, img_y, 0))
                               / 255.0;
                    fpreal g = (*(unsigned char *)myRaster->getPixel(
                                       img_x, img_y, 1))
                               / 255.0;
                    fpreal b = (*(unsigned char *)myRaster->getPixel(
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
    const GU_Detail *myGdp;
    GA_Attribute *myColorAttrib;
    GA_Attribute *myNumColorsAttrib;
    const PXL_Raster *myRaster;
    const E57Reader &myReader;
    const int myImageIndex;
    const UT_Matrix4 myRBXFormInv;
};

bool
updateColourFromImage(
        int image_index,
        E57Reader &reader,
	GU_Detail *gdp,
        UT_AutoInterrupt &boss,
        const SOP_NodeVerb::CookParms &cookparms)
{
    auto &&sopparms = cookparms.parms<SOP_LidarImportParms>();
    SOP_LidarImportCache *cache
            = static_cast<SOP_LidarImportCache *>(cookparms.cache());

    UT_Matrix4 rigid_body_transform_inv(reader.getImageRBXForm(image_index));
    rigid_body_transform_inv.invertDouble();

    UT_UniquePtr<unsigned char[]> image_buffer = reader.getImageBuffer(
            image_index, false);

    if (!image_buffer)
        return false;

    size_t image_size = reader.getImageSize(image_index);

    UT_IStream is((char *)image_buffer.get(), image_size, UT_ISTREAM_BINARY);

    UT_UniquePtr<IMG_File> img_file(IMG_File::open(is));

    UT_Array<PXL_Raster *> rasters;
    img_file->readImages(rasters);

    img_file->close();

    GA_Attribute *ncolors_attrib = gdp->addIntTuple(
            GA_ATTRIB_POINT, GA_SCOPE_PRIVATE, "__num_colors__", 1);
    GA_Attribute *color_attrib = gdp->addDiffuseAttribute(GA_ATTRIB_POINT);

    GA_RWHandleV3 color_h(color_attrib);
    GA_RWHandleI num_colors_h(ncolors_attrib);

    UT_StringHolder group_name(reader.getImageAssociatedScanGuid(image_index));

    const UT_IntArray &scan_groups = cache->myScanGroupMap[group_name];

    GA_PointGroup image_ptgroup(*gdp);
    for (int i = 0, ni = scan_groups.entries(); i < ni; ++i)
    {
        int scan_index = scan_groups(i);

        GA_PointGroup *scan_group = nullptr;
        UT_WorkBuffer scan_group_name(sopparms.getGroup_prefix().c_str());
        scan_group_name.appendFormat("{}", scan_index + 1);
        scan_group = gdp->findPointGroup(scan_group_name.buffer());

        image_ptgroup |= *scan_group;
    }

    GA_Range group_range(image_ptgroup);
    UTparallelFor(
            GA_SplittableRange(group_range),
            sop_colorFromImageParallel(
                    gdp, color_attrib, ncolors_attrib, rasters(0), reader,
                    image_index, rigid_body_transform_inv));

    if (boss.wasInterrupted())
    {
        cache->clearCache();
        return true;
    }

    for (int i = 0, ni = rasters.entries(); i < ni; ++i)
        delete rasters(i);

    return false;
}

// Reads E57 file into the detail, by calling readE57Scan N-number of times.
void 
readE57File(
        const SOP_NodeVerb::CookParms &cookparms,
        GU_Detail *gdp,
        UT_AutoInterrupt &boss)
{
    using namespace SOP_LidarImportEnums;
    auto &&sopparms = cookparms.parms<SOP_LidarImportParms>();
    SOP_LidarImportCache *cache
            = static_cast<SOP_LidarImportCache *>(cookparms.cache());

    try
        {
            E57Reader reader(sopparms.getFilename().c_str());

            int num_scans = reader.getNumScans();

            int64 idx = 0;
            bool stop_cook = false;
            UT_StringMap<UT_IntArray> missing_attrib_map;
            UT_StringArray file_missing_attribs;

            exint pts_in_file = reader.getPointsInFile();
            UT_ExintArray max_pts_in_scans;
            max_pts_in_scans.setCapacity(num_scans);

            for (int i = 0; i < num_scans; ++i)
            {
                fpreal64 scan_proportion = fpreal64(reader.getPointsInScan(i))
                                           / pts_in_file;
                max_pts_in_scans.append(sopparms.getMax_points() * scan_proportion);
            }

            for (int i = 0; i < num_scans && !stop_cook; ++i)
            {
                if (boss.wasInterrupted())
                {
                    cache->clearCache();
                    return;
                }

                if (cache->isClearDetailDataRequired(sopparms))
                {
                    UT_StringHolder group_name(reader.getScanGroupGuid(i));
                    cache->myScanGroupMap[group_name].append(i);
		}

                UT_StringArray scan_missing_attribs;
                stop_cook |= readE57Scan(
                        gdp, boss, cookparms, scan_missing_attribs, reader, i,
                        idx, max_pts_in_scans(i));

                if (scan_missing_attribs.entries() != 0)
                {
                    for (int j = 0, nj = scan_missing_attribs.entries(); j < nj;
                         ++j)
                    {
                        missing_attrib_map[scan_missing_attribs(j)].append(i);

                        if (file_missing_attribs.find(scan_missing_attribs(j))
                            == -1)
                            file_missing_attribs.append(
                                    scan_missing_attribs(j));
                    }
                }
            }

            if (file_missing_attribs.entries() != 0)
            {
                UT_WorkBuffer warning_msg;
                for (int i = 0, ni = file_missing_attribs.entries(); i < ni;
                     ++i)
                {
                    UT_StringRef attrib(file_missing_attribs(i));
                    UT_IntArray &scans_without_attrib
                            = missing_attrib_map[attrib];

                    warning_msg.append("- ");
                    warning_msg.append(attrib);
                    warning_msg.append(" in scan");

                    if (scans_without_attrib.entries() > 1)
                        warning_msg.append("s");

                    for (int j = 0, nj = scans_without_attrib.entries(); j < nj;
                         ++j)
                    {
                        warning_msg.appendFormat(
                                " {}", scans_without_attrib(j) + 1);
                        if (j < nj - 1)
                            warning_msg.append(",");
                    }

                    warning_msg.append('\n');
                }

                cookparms.sopAddWarning(
                        SOP_WARN_ATTRIBS_NOT_FOUND, warning_msg.buffer());
            }

            if (cache->hasColorChanged(sopparms)
                && sopparms.getColor() == Color::FROM_IMAGES)
            {
                int num_images = reader.getNumImages();

                for (int i = 0; i < num_images && !stop_cook; ++i)
                {
                    if (boss.wasInterrupted())
                    {
                        cache->clearCache();
                        return;
                    }

                    stop_cook |= 
			updateColourFromImage(i, reader, gdp, boss, cookparms);
                }

                // Delete internal attribute once everything's done
                gdp->destroyPointAttrib(GA_SCOPE_PRIVATE, "__num_colors__");
            }

            if (sopparms.getDelete_invalid())
            {
                GA_PointGroup invalid_pts(*gdp);

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
                            = gdp->findPointGroup(invalid_groups(i).c_str());

                    if (invalid_grp)
                        invalid_pts |= *invalid_grp;
                }

                gdp->destroyPoints(GA_Range(invalid_pts));
            }

            if (cache->hasScanNamesChanged(sopparms))
            {
                if (!sopparms.getScannames())
                {
                    gdp->destroyAttribute(GA_ATTRIB_GLOBAL, "scannames");
                }
                else
                {
                    UT_StringArray names(num_scans, num_scans);
                    for (int i = 0; i < num_scans; ++i)
                        names(i) = reader.getScanName(i);

                    GA_RWHandleSA names_h(
                            gdp->addStringArray(GA_ATTRIB_GLOBAL, "scannames"));
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
        }
#else
        catch (e57::E57Exception &e)
        {
            std::string context = e.context();
            cookparms.sopAddError(SOP_ERR_LIDAR_READER_ERROR, context.c_str());
        }
#endif
        return;
}
} // namespace

void
SOP_LidarImportVerb::cook(const SOP_NodeVerb::CookParms &cookparms) const
{
    using namespace SOP_LidarImportEnums;
    auto &&sopparms = cookparms.parms<SOP_LidarImportParms>();
    GU_Detail *gdp = cookparms.gdh().gdpNC();
    SOP_LidarImportCache *cache = static_cast<SOP_LidarImportCache *>(
            cookparms.cache());

    // Set cook parameters
    UT_String filename;
    filename = sopparms.getFilename();

    // Must re-read everything:
    if (cache->isClearDetailDataRequired(sopparms))
    {
        gdp->clearAndDestroy();
        cache->clearCache();
    }

    UT_AutoInterrupt boss("Reading lidar file");

    // LAS/LAZ :
    if (filename.matchFileExtension(".las")
        || filename.matchFileExtension(".laz"))
    {
        UT_StringArray inapplicable_parms;
        if (sopparms.getColor() == Color::FROM_IMAGES)
            inapplicable_parms.append("Color From Images");
        if (sopparms.getDelete_invalid())
            inapplicable_parms.append("Delete Invalid Points");
        if (sopparms.getRow_col())
            inapplicable_parms.append("Row and Column");
        if (sopparms.getNormals())
            inapplicable_parms.append("Surface Normals");
        if (inapplicable_parms.entries() != 0)
        {
            UT_WorkBuffer warning_msg;
            for (int i = 0, ni = inapplicable_parms.entries(); i < ni; ++i)
            {
                warning_msg.append(inapplicable_parms(i));
                if (i < ni - 1)
                    warning_msg.append(", ");
            }

            cookparms.sopAddWarning(
                    SOP_WARN_PARMS_NOT_APPLICABLE, warning_msg.buffer());
        }

        std::ifstream stream(filename, std::ios_base::binary);
        if (!stream.is_open())
        {
            cookparms.sopAddError(
                    SOP_ERR_LIDAR_READER_ERROR, "Failed to open the file.");
        }
        else if (!readLASFile(stream, gdp, boss, cookparms))
        {
            cookparms.sopAddError(
                    SOP_ERR_LIDAR_READER_ERROR,
                    "Failed to read the file as a valid LAS format.");
        }
        return;
    }

    // E57 :
    if (filename.matchFileExtension(".e57"))
    {
	// All SOP warnings and errors are handled in the E57 read functions.
        readE57File(cookparms, gdp, boss);
    }

    // Cache all parameters from this cook
    cache->updateCache(sopparms);
}
