/*
 * Copyright (c) 2020
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

#include "SOP_LidarImport.h"

#include <E57/E57Foundation.h>
#include <E57/E57Simple.h>

#include <liblas/liblas.hpp>

#include <GU/GU_Detail.h>
#include <GA/GA_PageHandle.h>
#include <GA/GA_SplittableRange.h>
#include <OP/OP_Operator.h>
#include <OP/OP_OperatorTable.h>
#include <PRM/PRM_Include.h>
#include <PRM/PRM_SpareData.h>
#include <CH/CH_LocalVariable.h>
#include <CH/CH_Manager.h>
#include <PXL/PXL_Raster.h>
#include <IMG/IMG_File.h>
#include <IMG/IMG_Format.h>
#include <UT/UT_DSOVersion.h>
#include <UT/UT_Interrupt.h>
#include <UT/UT_String.h>
#include <UT/UT_VectorTypes.h>
#include <UT/UT_Matrix4.h>
#include <UT/UT_Quaternion.h>
#include <UT/UT_IStream.h>
#include <UT/UT_UniquePtr.h>
#include <UT/UT_Assert.h>

#include <vector>
#include <functional>
#include <exception>

//#define E57_SOP_VERBOSE

// when LAZ support is added, *.laz should be added here
static PRM_SpareData fileExtraData(PRM_SpareArgs()
	<< PRM_SpareToken(PRM_SpareData::getFileChooserPatternToken(), 
			  "*.e57 *.las")
	<< PRM_SpareToken(PRM_SpareData::getFileChooserModeToken(), 
			  PRM_SpareData::getFileChooserModeValRead()));


static PRM_Name parmNames[] = {
    PRM_Name("filename", "File"),
    PRM_Name("color", "Color"),
    PRM_Name("intensity", "Intensity"),
    PRM_Name("row_col", "Row and Column"),
    PRM_Name("ret_data", "Return Data"),
    PRM_Name("timestamp", "Timestamp"),
    PRM_Name("normals", "Surface Normals"),
    PRM_Name("group_prefix", "Group Prefix"),
    PRM_Name("filter_type", "Filter Type"),
    PRM_Name("select_range", "Select _ of _"),
    PRM_Name("max_points", "Max Number of Points"),
    PRM_Name("delete_invalid", "Delete Invalid Points"),
};

static PRM_Default defaultPrefix(0, "lidar_group");

static PRM_Name colorOptionNames[] = {
    PRM_Name("none", "None"),
    PRM_Name("from_ptcloud", "From Point Cloud"),
    PRM_Name("from_images", "From Images"),
    PRM_Name(0),
};

static PRM_ChoiceList colorOptions(
	(PRM_ChoiceListType)(PRM_CHOICELIST_EXCLUSIVE | PRM_CHOICELIST_REPLACE),
	colorOptionNames);

static PRM_Name filterSectionName("filtering");
static PRM_Default filterSectionLabel(4, "Filtering");

static PRM_Name attribSectionName("attribs");
static PRM_Default attribSectionLabel(6, "Attributes");

static PRM_Name filterOptionNames[] = {
    PRM_Name("no_filter", "None"),
    PRM_Name("range_filter", "Range"),
    PRM_Name("max_filter", "Maximum"),
    PRM_Name(0),
};

static PRM_ChoiceList filterOptions(
	(PRM_ChoiceListType)(PRM_CHOICELIST_EXCLUSIVE | PRM_CHOICELIST_REPLACE),
	filterOptionNames);

PRM_Template
SOP_LidarImport::myTemplateList[] = {
    PRM_Template(PRM_FILE, PRM_Template::PRM_EXPORT_TBX, 1, &parmNames[0],
	    nullptr, nullptr, nullptr, PRM_Callback(), &fileExtraData),
    PRM_Template(PRM_STRING, 1, &parmNames[7], &defaultPrefix),
    PRM_Template(PRM_SWITCHER,	1, &filterSectionName, &filterSectionLabel, 
	    0, 0, 0, &PRM_SpareData::groupTypeSimple),
    PRM_Template(PRM_ORD, 1, &parmNames[8], nullptr, &filterOptions),
    PRM_Template(PRM_INT, 2, &parmNames[9]),
    PRM_Template(PRM_INT, 1, &parmNames[10]),
    PRM_Template(PRM_TOGGLE, 1, &parmNames[11]),
    PRM_Template(PRM_SWITCHER,	1, &attribSectionName, &attribSectionLabel,
		 0, 0, 0, &PRM_SpareData::groupTypeSimple),
    PRM_Template(PRM_ORD, 1, &parmNames[1], nullptr, &colorOptions),
    PRM_Template(PRM_TOGGLE, 1, &parmNames[2]),
    PRM_Template(PRM_TOGGLE, 1, &parmNames[3]),
    PRM_Template(PRM_TOGGLE, 1, &parmNames[4]),
    PRM_Template(PRM_TOGGLE, 1, &parmNames[5]),
    PRM_Template(PRM_TOGGLE, 1, &parmNames[6]),
    PRM_Template()
};

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
            SOP_LidarImport::myTemplateList,
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

SOP_LidarImport::SOP_LidarImport(OP_Network *net, 
				     const char *name, 
				     OP_Operator *op)
    : SOP_Node(net, name, op)
    , myCachedFileName("")
    , myCachedGroupPrefix("")
    , myCachedUseColor(ColorDataSrc::NONE)
    , myCachedUseIntensity(false)
    , myCachedUseRowCol(false)
    , myCachedUseReturnData(false)
    , myCachedUseTimestamp(false)
    , myCachedUseNormals(false)
    , myCachedFilterType(FilterType::NOFILTER)
    , myCachedRangeGood(0)
    , myCachedRangeSize(0)
    , myCachedMaxPoints(0)
    , myCachedDeleteInvalid(false)
{
}

SOP_LidarImport::~SOP_LidarImport() {}

bool
SOP_LidarImport::updateParmsFlags()
{
    bool changed = false;

    fpreal now = CHgetEvalTime();
    int filter_type = evalInt("filter_type", 0, now);

    switch (filter_type)
    {
	case NOFILTER:
	    changed |= enableParm("select_range", false);
	    changed |= enableParm("max_points", false);
	    break;
	case RANGE:
	    changed |= enableParm("select_range", true);
	    changed |= enableParm("max_points", false);
	    break;
	case MAX:
	    changed |= enableParm("select_range", false);
	    changed |= enableParm("max_points", true);
	    break;
    }

    return changed;
}

namespace
{
static bool isRangeValid(exint range_good, exint range_size)
{
    return (range_good < range_size && range_good > 0 && range_size > 0);
}

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
    UT_Matrix4			myRBXForm;
    UT_StringHolder		myGuid;
    exint			myNumPoints;

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
    UT_Matrix4			myRBXForm;
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
}

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

class SOP_LidarImport::E57Reader
{
public:

    E57Reader(const char *filename);
    virtual ~E57Reader() {}

    int getNumScans() const;
    int getNumImages() const;

    exint getPointsInFile() const;
    exint getPointsInScan(int idx) const;

    UT_Matrix4 getScanRBXForm(int idx) const;
    UT_Matrix4 getImageRBXForm(int idx) const;

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

SOP_LidarImport::E57Reader::E57Reader(const char *filename)
    : myFileHandle(filename, "r")
{
    // NOTE: some of this code may throw an exception; it should be caught in cookMySop
    e57::StructureNode root_node = myFileHandle.root();
    e57::VectorNode scan_root_node(root_node.get("/data3D"));

    auto set_rbxform = [](e57::StructureNode parent, UT_Matrix4 &rbxform)
    {
	if (parent.isDefined("pose"))
	{
	    e57::StructureNode rbxform_node(parent.get("pose"));

	    fpreal		rw, rx, ry, rz;
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

	    fpreal		tx, ty, tz;
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
SOP_LidarImport::E57Reader::getNumScans() const
{
    return myScans.entries();
}

int
SOP_LidarImport::E57Reader::getNumImages() const
{
    return myImages.entries();
}

exint
SOP_LidarImport::E57Reader::getPointsInFile() const
{
    return myPointsInFile;
}

exint
SOP_LidarImport::E57Reader::getPointsInScan(int idx) const
{
    return myScans(idx).myNumPoints;
}

UT_Matrix4
SOP_LidarImport::E57Reader::getScanRBXForm(int idx) const
{
    return myScans(idx).myRBXForm;
}

UT_Matrix4
SOP_LidarImport::E57Reader::getImageRBXForm(int idx) const
{
    return myImages(idx).myRBXForm;
}

sop_E57PointReader::Builder
SOP_LidarImport::E57Reader::createPointReaderBuilder(int idx, GU_Detail *gdp, 
						  int range_good, int range_size)
{
    return sop_E57PointReader::Builder(myFileHandle, idx, myScans(idx), gdp, 
				       range_good, range_size);
}

UT_UniquePtr<unsigned char[]>
SOP_LidarImport::E57Reader::getImageBuffer(int idx, bool read_ref_images) const
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
SOP_LidarImport::E57Reader::getImageDimensions(int idx, exint &width, exint &height)
    const
{
    width = myImages(idx).myWidth;
    height = myImages(idx).myHeight;
}

size_t
SOP_LidarImport::E57Reader::getImageSize(int idx) const
{
    return myImages(idx).mySize;
}

bool
SOP_LidarImport::E57Reader::hasValidProjectionType(int idx) const
{
    return myImages(idx).myProjectionType > e57::Image2DProjection::E57_VISUAL;
}

UT_StringHolder
SOP_LidarImport::E57Reader::getScanGroupGuid(int idx)
{
    return myScans(idx).myGuid;
}

UT_StringHolder
SOP_LidarImport::E57Reader::getImageAssociatedScanGuid(int idx)
{
    return myImages(idx).myAssociatedScanGuid;
}

void
SOP_LidarImport::E57Reader::computePointProjection(int idx, const UT_Vector4 pos, 
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

class SOP_LidarImport::LASReader
{
public:
    struct RequestedAttribs
    {
	bool myReadPosition;
	bool myReadColor;
	bool myReadIntensity;
	bool myReadRetData;
	bool myReadTimestamp;

	bool isReadRequired() const
	{
	    return myReadPosition || myReadColor || myReadIntensity 
		|| myReadRetData || myReadTimestamp;
	}
    };

    LASReader(const char *filename, RequestedAttribs requested_attrs, 
	      GU_Detail *gdp);
    virtual ~LASReader() {}

    exint getPointsInFile() const;
    void setRange(int range_good, int range_size);
    void readBlock(GA_Offset start, GA_Offset end);

    bool hasColorData() const;
    bool hasTimestampData() const;
private:
    bool readNextPoint();
    
    int myRangeGood;
    int myRangeSize;
    int myRangePos;
    exint myPointRecordPos;
    exint myPointRecordCount;

    struct ReaderWrapper
    {
	ReaderWrapper(liblas::ReaderFactory &factory, std::ifstream &ifs)
	    : myReader(factory.CreateWithStream(ifs))
	{}

	liblas::Reader myReader;
    };

    UT_UniquePtr<ReaderWrapper> myReaderWrapper;
    // The stream is stored here so that it isn't closed until the end of the read
    std::ifstream myFileStream;

    GA_RWPageHandleV3 myPosPH;
    GA_RWPageHandleV3 myColorPH;
    GA_RWPageHandleF myIntensityPH;
    GA_RWPageHandleI myReturnIndexPH;
    GA_RWPageHandleI myReturnCountPH;
    GA_RWPageHandleF myTimestampPH;
};

SOP_LidarImport::LASReader::LASReader(const char *filename,
	RequestedAttribs requested_attrs, GU_Detail *gdp)
    : myRangeGood(GA_PAGE_SIZE)
    , myRangeSize(GA_PAGE_SIZE)
    , myRangePos(0)
    , myPointRecordPos(0)
    , myFileStream(filename, std::ios::in | std::ios::binary)
{
    liblas::ReaderFactory reader_factory;
    myReaderWrapper = UTmakeUnique<ReaderWrapper>(reader_factory, myFileStream);

    liblas::Header header = myReaderWrapper->myReader.GetHeader();
    myPointRecordCount = header.GetPointRecordsCount();

    liblas::PointFormatName pt_format = header.GetDataFormatId();
    switch (pt_format)
    {
	case liblas::ePointFormat0:
	    requested_attrs.myReadColor = false;
	    requested_attrs.myReadTimestamp = false;
	    break;
	case liblas::ePointFormat1:
	case liblas::ePointFormat4:
	    requested_attrs.myReadColor = false;
	    break;
	case liblas::ePointFormat2:
	    requested_attrs.myReadTimestamp = false;
	    break;
	case liblas::ePointFormat3:
	case liblas::ePointFormat5:
	default:
	    break;
    }

    if (requested_attrs.myReadPosition)
	myPosPH.bind(gdp->getP());

    if (requested_attrs.myReadColor)
	myColorPH.bind(gdp->addDiffuseAttribute(GA_ATTRIB_POINT));

    if (requested_attrs.myReadIntensity)
	myIntensityPH.bind(gdp->addFloatTuple(GA_ATTRIB_POINT, "intensity", 1));

    if (requested_attrs.myReadRetData)
    {
	myReturnIndexPH.bind(gdp->addIntTuple(GA_ATTRIB_POINT, "return_index", 1));
	myReturnCountPH.bind(gdp->addIntTuple(GA_ATTRIB_POINT, "return_count", 1));
    }

    if (requested_attrs.myReadTimestamp)
	myTimestampPH.bind(gdp->addFloatTuple(GA_ATTRIB_POINT, "timestamp", 1));

}

exint
SOP_LidarImport::LASReader::getPointsInFile() const
{
    return myPointRecordCount;
}

void 
SOP_LidarImport::LASReader::setRange(int range_good, int range_size)
{
    myRangeGood = range_good;
    myRangeSize = range_size;
}

bool 
SOP_LidarImport::LASReader::hasColorData() const
{
    return myColorPH.isValid();
}

bool 
SOP_LidarImport::LASReader::hasTimestampData() const
{
    return myTimestampPH.isValid();
}

void
SOP_LidarImport::LASReader::readBlock(GA_Offset start, GA_Offset end)
{
    if (myPosPH.isValid())
	myPosPH.setPage(start);

    if (myColorPH.isValid())
	myColorPH.setPage(start);

    if (myIntensityPH.isValid())
	myIntensityPH.setPage(start);

    if (myReturnIndexPH.isValid() && myReturnCountPH.isValid())
    {
	myReturnIndexPH.setPage(start);
	myReturnCountPH.setPage(start);
    }

    if (myTimestampPH.isValid())
	myTimestampPH.setPage(start);

    GA_Offset ptoff;
    for (ptoff = start; ptoff < end && readNextPoint(); ++ptoff)
    {
	liblas::Point pt = myReaderWrapper->myReader.GetPoint();

	if (myPosPH.isValid())
	{
	    fpreal x = pt.GetX();
	    fpreal y = pt.GetY();
	    fpreal z = pt.GetZ();

	    myPosPH.value(ptoff) = UT_Vector3(x, y, z);
	}

	if (myColorPH.isValid())
	{
	    liblas::Color color = pt.GetColor();
	    fpreal r = fpreal(color.GetRed()) / SYS_UINT16_MAX;
	    fpreal g = fpreal(color.GetGreen()) / SYS_UINT16_MAX;
	    fpreal b = fpreal(color.GetBlue()) / SYS_UINT16_MAX;

	    myColorPH.value(ptoff) = UT_Vector3(r, g, b);
	}

	if (myIntensityPH.isValid())
	    myIntensityPH.value(ptoff) = fpreal(pt.GetIntensity()) / SYS_UINT16_MAX;

	if (myReturnIndexPH.isValid() && myReturnCountPH.isValid())
	{
	    myReturnIndexPH.value(ptoff) = pt.GetReturnNumber();
	    myReturnCountPH.value(ptoff) = pt.GetNumberOfReturns();
	}

	if (myTimestampPH.isValid())
	    myTimestampPH.value(ptoff) = pt.GetTime();
    }
}


bool
SOP_LidarImport::LASReader::readNextPoint()
{
    UT_ASSERT(myRangePos >= 0 && myRangePos <= myRangeSize);
    if (myRangePos >= myRangeGood)
    {
	myPointRecordPos += myRangeSize - myRangePos;
	myRangePos = 0;

	if (myPointRecordPos >= myPointRecordCount)
	    return false;

	myReaderWrapper->myReader.Seek(myPointRecordPos);
    }

    myPointRecordPos++;
    myRangePos++;

    return myReaderWrapper->myReader.ReadNextPoint();
}

OP_ERROR
SOP_LidarImport::cookMySop(OP_Context &context)
{
    fpreal now = context.getTime();

    UT_String filename;
    evalString(filename, "filename", 0, now);

    UT_String prefix;
    evalString(prefix, "group_prefix", 0, now);

    int filter_type = evalInt("filter_type", 0, now);
    int range_good = evalInt("select_range", 0, now);
    int range_size = evalInt("select_range", 1, now);
    int max_pts = evalInt("max_points", 0, now);

    bool delete_invalid_pts = evalInt("delete_invalid", 0, now);

    int use_color = evalInt("color", 0, now);
    bool use_intensity = evalInt("intensity", 0, now);
    bool use_row_col = evalInt("row_col", 0, now);
    bool use_ret_data = evalInt("ret_data", 0, now);
    bool use_timestamp = evalInt("timestamp", 0, now);
    bool use_normals = evalInt("normals", 0, now);

    if (error() >= UT_ERROR_ABORT || !filename.isstring() || !prefix.isstring() 
	    || forceValidGroupPrefix(prefix, UT_ERROR_ABORT))
    {
	clearSopNodeCache();
	return error();
    }

    bool file_changed = (filename != myCachedFileName);

    bool delete_invalid_changed = (myCachedDeleteInvalid != delete_invalid_pts);

    bool filter_changed = (filter_type != myCachedFilterType);
    bool range_parms_changed = (myCachedRangeGood != range_good)
	|| (myCachedRangeSize != range_size);
    bool max_pts_changed = (myCachedMaxPoints != max_pts);

    switch (filter_type)
    {
	case RANGE:
	    filter_changed |= range_parms_changed;
	    break;
	case MAX:
	    filter_changed |= max_pts_changed;
	    break;
	case NOFILTER:
	default:
	    break;
    }

    bool color_changed = (use_color != myCachedUseColor);
    bool intensity_changed = (use_intensity != myCachedUseIntensity);
    bool row_col_changed = (use_row_col != myCachedUseRowCol);
    bool ret_data_changed = (use_ret_data != myCachedUseReturnData);
    bool timestamp_changed = (use_timestamp != myCachedUseTimestamp);
    bool normals_changed = (use_normals != myCachedUseNormals);

    bool clear_cache_required = file_changed || filter_changed
	|| delete_invalid_changed || delete_invalid_pts;

    if (clear_cache_required)
    {
	gdp->clearAndDestroy();
	clearSopNodeCache();
    }

    // Cache all the current parm values
    myCachedFileName = filename;
    myCachedFilterType = filter_type;
    myCachedRangeGood = range_good;
    myCachedRangeSize = range_size;
    myCachedMaxPoints = max_pts;
    myCachedDeleteInvalid = delete_invalid_pts;
    myCachedUseColor = use_color;
    myCachedUseIntensity = use_intensity;
    myCachedUseRowCol = use_row_col;
    myCachedUseReturnData = use_ret_data;
    myCachedUseTimestamp = use_timestamp;
    myCachedUseNormals = use_normals;
    // Group prefix is cached after all the groups have been created/renamed

    UT_AutoInterrupt boss("Reading lidar file");

    
    bool is_las_file = filename.matchFileExtension(".las");

    if (is_las_file)
    {
	UT_StringArray inapplicable_parms;
	if (use_color == IMAGES)
	    inapplicable_parms.append("Color From Images");

	if (delete_invalid_pts)
	    inapplicable_parms.append("Delete Invalid Points");

	if (use_row_col)
	    inapplicable_parms.append("Row and Column");

	if (use_normals)
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

	    addWarning(SOP_WARN_PARMS_NOT_APPLICABLE, warning_msg.buffer());
	}
	
	try
	{
	    readLASFile(filename.c_str(), boss, clear_cache_required, 
			color_changed, intensity_changed, ret_data_changed,
		       	timestamp_changed, max_pts);
	}
	catch (std::exception &e)
       	{
	    addError(SOP_ERR_LIDAR_READER_ERROR, e.what());
	}
	
	return error();
    }

    try
    {
	E57Reader reader(filename.c_str());

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
	    fpreal64 scan_proportion = 
		fpreal64(reader.getPointsInScan(i)) / pts_in_file;
	    max_pts_in_scans.append(max_pts * scan_proportion);
	}

	for (int i = 0; i < num_scans && !stop_cook; ++i)
	{
	    if (boss.wasInterrupted())
	    {
		clearSopNodeCache();
		return error();
	    }

	    if (clear_cache_required)
		updateScanGroupMap(i, reader);

	    UT_StringArray scan_missing_attribs;
	    stop_cook |= readE57Scan(i, idx, reader, boss, scan_missing_attribs,
				  clear_cache_required, color_changed, 
				  intensity_changed, row_col_changed, 
				  ret_data_changed, timestamp_changed, 
				  normals_changed, prefix, max_pts_in_scans(i));

	    if (scan_missing_attribs.entries() != 0)
	    {
		for (int j = 0, nj = scan_missing_attribs.entries(); j < nj; ++j)
		{
		    missing_attrib_map[scan_missing_attribs(j)].append(i);
		    
		    if (file_missing_attribs.find(scan_missing_attribs(j)) == -1)
			file_missing_attribs.append(scan_missing_attribs(j));
		}
	    }
	}

	if (file_missing_attribs.entries() != 0)
	{
	    UT_WorkBuffer warning_msg;
	    for (int i = 0, ni = file_missing_attribs.entries(); i < ni; ++i)
	    {
		UT_StringRef attrib(file_missing_attribs(i));
		UT_IntArray &scans_without_attrib = missing_attrib_map[attrib];

		warning_msg.append("- ");
		warning_msg.append(attrib);
		warning_msg.append(" in scan");

		if (scans_without_attrib.entries() > 1)
		    warning_msg.append("s");

		for (int j = 0, nj = scans_without_attrib.entries(); j < nj; ++j)
		{
		    warning_msg.appendFormat(" {}", scans_without_attrib(j) + 1);
		    if (j < nj - 1)
			warning_msg.append(",");
		}


		warning_msg.append('\n');
	    }

	    addWarning(SOP_WARN_ATTRIBS_NOT_FOUND, warning_msg.buffer());
	}

	myCachedGroupPrefix = prefix;

	if (color_changed && myCachedUseColor == ColorDataSrc::IMAGES)
	{
	    int num_images = reader.getNumImages();

	    for (int i = 0; i < num_images && !stop_cook; ++i)
	    {
		if (boss.wasInterrupted())
		{
		    clearSopNodeCache();
		    return error();
		}

		stop_cook |= updateColourFromImage(i, reader, boss);
	    }

	    // Delete internal attribute once everything's done
	    gdp->destroyPointAttrib("__num_colors__");
	}

	if (delete_invalid_pts)
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
		GA_PointGroup *invalid_grp = 
			gdp->findPointGroup(invalid_groups(i).c_str());

		if (invalid_grp)
		    invalid_pts |= *invalid_grp;
	    }

	    gdp->destroyPoints(GA_Range(invalid_pts));
	}
    }
#ifdef E57_SOP_VERBOSE
    catch (e57::E57Exception &e)
    {
	std::string	context = e.context();
	addError(SOP_ERR_LIDAR_READER_ERROR, context.c_str());
	e.report(__FILE__, __LINE__, __FUNCTION__);
    }
#else
    catch (e57::E57Exception &e)
    {
	std::string	context = e.context();
	addError(SOP_ERR_LIDAR_READER_ERROR, context.c_str());
    }
#endif

    return error();
}

namespace
{
    static void computeRangeFromMaxPoints(exint max_points, exint pts_in_scan, 
					  int &range_good, int &range_size)
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
}

void
SOP_LidarImport::readLASFile(const char *filename, UT_AutoInterrupt &boss, 
			     bool clear_cache_required, bool color_changed, 
			     bool intensity_changed, bool ret_data_changed, 
			     bool timestamp_changed, exint file_max_pts)
{
    LASReader::RequestedAttribs requested_attrs;

    requested_attrs.myReadPosition = clear_cache_required;
    requested_attrs.myReadColor = color_changed && myCachedUseColor == PTCLOUD;
    requested_attrs.myReadIntensity = intensity_changed && myCachedUseIntensity;
    requested_attrs.myReadRetData = ret_data_changed && myCachedUseReturnData;
    requested_attrs.myReadTimestamp = timestamp_changed && myCachedUseTimestamp;

    if (color_changed && myCachedUseColor != PTCLOUD)
	gdp->destroyDiffuseAttribute(GA_ATTRIB_POINT);

    if (intensity_changed && !myCachedUseIntensity)
	gdp->destroyPointAttrib("intensity");

    if (ret_data_changed && !myCachedUseReturnData)
    {
	gdp->destroyPointAttrib("return_index");
	gdp->destroyPointAttrib("return_count");
    }

    if (timestamp_changed && !myCachedUseTimestamp)
	gdp->destroyPointAttrib("timestamp");

    LASReader reader(filename, requested_attrs, gdp);

    UT_StringArray missing_attribs;
    if (!reader.hasColorData() && requested_attrs.myReadColor)
    {
	requested_attrs.myReadColor = false;
	missing_attribs.append("Color");
    }

    if (!reader.hasTimestampData() && requested_attrs.myReadTimestamp)
    {
	requested_attrs.myReadTimestamp = false;
	missing_attribs.append("Timestamp");
    }

    if (missing_attribs.entries() != 0)
    {
	UT_WorkBuffer warning_msg;
	for (int i = 0, ni = missing_attribs.entries(); i < ni; ++i)
	{
	    warning_msg.append(missing_attribs(i));
	    if (i < ni - 1)
		warning_msg.append(", ");
	}

	addWarning(SOP_WARN_ATTRIBS_NOT_FOUND, warning_msg.buffer());
    }

    if (!requested_attrs.isReadRequired())
	return;

    exint pts_in_file = reader.getPointsInFile();
    int range_good = GA_PAGE_SIZE;
    int range_size = GA_PAGE_SIZE;

    switch (myCachedFilterType)
    {
	case RANGE:
	    if (isRangeValid(myCachedRangeGood, myCachedRangeSize))
	    {
		range_good = myCachedRangeGood;
		range_size = myCachedRangeSize;
	    }
	    break;
	case MAX:
	    if (isRangeValid(file_max_pts, pts_in_file))
		computeRangeFromMaxPoints(file_max_pts, pts_in_file, range_good,
					  range_size);
	    break;
	case NOFILTER:
	default:
	    break;
    }
    reader.setRange(range_good, range_size);

    exint num_pts = pts_in_file;
    if (isRangeValid(range_good, range_size))
    {
	num_pts = pts_in_file / range_size;
	num_pts *= range_good;

	exint remaining_pts = pts_in_file % range_size;
	num_pts += SYSmin(range_good, remaining_pts);
    }

    if (clear_cache_required)
	gdp->appendPointBlock(num_pts);

    GA_Range pt_range(gdp->getPointMap(), GA_Offset(0), GA_Offset(num_pts));

    GA_Offset start, end;
    for (GA_Iterator it(pt_range); it.blockAdvance(start, end); )
    {
	reader.readBlock(start, end);

	if (boss.wasInterrupted())
	{
	    clearSopNodeCache();
	    return;
	}
    }
}

void
SOP_LidarImport::updateScanGroupMap(int scan_index, E57Reader &reader)
{
    UT_StringHolder group_name(reader.getScanGroupGuid(scan_index));
    myScanGroupMap[group_name].append(scan_index);
}

bool
SOP_LidarImport::readE57Scan(int scan_index, int64 &pt_idx, E57Reader &reader,
       		        UT_AutoInterrupt &boss,
			UT_StringArray &missing_attribs,
			bool clear_cache_required,
			bool color_changed,
			bool intensity_changed,
			bool row_col_changed,
			bool ret_data_changed,
			bool timestamp_changed,
			bool normals_changed,
			UT_StringHolder current_prefix,
			exint scan_max_pts)
{
    UT_Matrix4 rigid_body_transform(reader.getScanRBXForm(scan_index));

    bool read_file_required = false;

    int range_good = GA_PAGE_SIZE;
    int range_size = GA_PAGE_SIZE;
    exint pts_in_scan = reader.getPointsInScan(scan_index);

    switch (myCachedFilterType)
    {
	case RANGE:
	    if (isRangeValid(myCachedRangeGood, myCachedRangeSize))
	    {
		range_good = myCachedRangeGood;
		range_size = myCachedRangeSize;
	    }
	    break;
	case MAX:
	    if (isRangeValid(scan_max_pts, pts_in_scan))
		computeRangeFromMaxPoints(scan_max_pts, pts_in_scan, range_good, 
					  range_size);
	    break;
	case NOFILTER:
	default:
	    break;
    }
    sop_E57PointReader::Builder pt_reader_builder(
	    reader.createPointReaderBuilder(scan_index, gdp, range_good, 
					    range_size));

    if (clear_cache_required)
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
	    addError(SOP_ERR_MISSING_POSITION);
	    return true;
	}
    }

    if (color_changed && myCachedUseColor == ColorDataSrc::PTCLOUD)
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
    else if (color_changed && myCachedUseColor != ColorDataSrc::PTCLOUD)
    {
	gdp->destroyDiffuseAttribute(GA_ATTRIB_POINT);
	
	GA_PointGroup *invalid_group = gdp->findPointGroup("e57_invalid_color");
	if (invalid_group)
	    gdp->destroyPointGroup(invalid_group);
    }

    if (intensity_changed && myCachedUseIntensity)
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
    else if (intensity_changed && !myCachedUseIntensity)
    {
	gdp->destroyPointAttrib("intensity");
	
	GA_PointGroup *invalid_group = gdp->findPointGroup("e57_invalid_intensity");
	if (invalid_group)
	    gdp->destroyPointGroup(invalid_group);
    }

    if (row_col_changed && myCachedUseRowCol)
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
    else if (row_col_changed && !myCachedUseRowCol)
    {
	gdp->destroyPointAttrib("row_index");
	gdp->destroyPointAttrib("column_index");
    }

    if (ret_data_changed && myCachedUseReturnData)
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
    else if (ret_data_changed && !myCachedUseReturnData)
    {
	gdp->destroyPointAttrib("return_index");
	gdp->destroyPointAttrib("return_count");
    }

    if (timestamp_changed && myCachedUseTimestamp)
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
    else if (timestamp_changed && !myCachedUseTimestamp)
    {
	gdp->destroyPointAttrib("timestamp");

	GA_PointGroup *invalid_group = gdp->findPointGroup("e57_invalid_timestamp");
	if (invalid_group)
	    gdp->destroyPointGroup(invalid_group);
    }

    if (normals_changed && myCachedUseNormals)
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
    else if (normals_changed && !myCachedUseNormals)
    {
	gdp->destroyNormalAttribute(GA_ATTRIB_POINT);
    }

    GA_PointGroup *scan_group = nullptr;
    if (clear_cache_required)
    {
	UT_WorkBuffer group_name(current_prefix.c_str());
	group_name.appendFormat("{}", scan_index + 1);
	scan_group = gdp->newPointGroup(group_name.buffer());
    }
    else if (current_prefix != myCachedGroupPrefix)
    {
	UT_WorkBuffer old_group_name(myCachedGroupPrefix.c_str());
	old_group_name.appendFormat("{}", scan_index + 1);
	UT_WorkBuffer new_group_name(current_prefix.c_str());
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

    if (clear_cache_required)
	gdp->appendPointBlock(num_pts);

    sop_E57PointReader pt_reader(pt_reader_builder.build());

    GA_Range pt_range(gdp->getPointMap(), GA_Offset(pt_idx), 
	GA_Offset(pt_idx + num_pts));

    if (clear_cache_required)
	scan_group->setElement(pt_range, true);

    GA_Offset start, end;
    for (GA_Iterator it(pt_range); it.blockAdvance(start, end) 
	    && pt_reader.readBlock(start, end); )
    {
	if (boss.wasInterrupted())
	{
	    clearSopNodeCache();
	    return true;
	}
    }

    pt_idx += num_pts;

    if (clear_cache_required && !rigid_body_transform.isIdentity())
	gdp->transformPoints(rigid_body_transform, scan_group, nullptr, true);

    return false;
}

namespace
{
class sop_colorFromImageParallel
{
public:
    sop_colorFromImageParallel(const GU_Detail *gdp, 
			       GA_Attribute *c_attr, GA_Attribute *nc_attr,
			       const PXL_Raster *raster, 
			       const SOP_LidarImport::E57Reader &reader,
			       int img_idx,
			       const UT_Matrix4 rbxform_inv)
	: myGdp(gdp)
	, myColorAttrib(c_attr)
	, myNumColorsAttrib(nc_attr)
	, myRaster(raster)
	, myReader(reader)
	, myImageIndex(img_idx)
	, myRBXFormInv(rbxform_inv)
    {}

    void operator() (const GA_SplittableRange &r) const
    {
	UT_ASSERT(myReader.hasValidProjectionType(myImageIndex)
		  && "Invalid projection type");
        UT_Interrupt *boss = UTgetInterrupt();

	GA_RWPageHandleV3 color_ph(myColorAttrib);
	GA_RWPageHandleI num_colors_ph(myNumColorsAttrib);

	GA_Offset start, end;
	for (GA_Iterator it(r); it.blockAdvance(start, end); )
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
		myReader.computePointProjection(myImageIndex, pos, img_x, img_y);

		exint img_width, img_height;
		myReader.getImageDimensions(myImageIndex, img_width, img_height);

		if (img_x >= 0 && img_x < img_width 
			&& img_y >= 0 && img_y < img_height)
		{
		    UT_Vector3 old_color = color_ph.value(ptoff);
		    int32 old_num_colors = num_colors_ph.value(ptoff);
		    fpreal r = 
			(*(unsigned char *)myRaster->getPixel(img_x, img_y, 0))
		       	/ 255.0;
		    fpreal g = 
			(*(unsigned char *)myRaster->getPixel(img_x, img_y, 1)) 
			/ 255.0;
		    fpreal b = 
			(*(unsigned char *)myRaster->getPixel(img_x, img_y, 2)) 
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
    const GU_Detail			*myGdp;
    GA_Attribute			*myColorAttrib;
    GA_Attribute			*myNumColorsAttrib;
    const PXL_Raster			*myRaster;
    const SOP_LidarImport::E57Reader	&myReader;
    const int				 myImageIndex;
    const UT_Matrix4			 myRBXFormInv;
};
}

bool
SOP_LidarImport::updateColourFromImage(int image_index, E57Reader &reader, 
				     UT_AutoInterrupt &boss)
{

    UT_Matrix4 rigid_body_transform_inv(reader.getImageRBXForm(image_index));
    rigid_body_transform_inv.invertDouble();

    UT_UniquePtr<unsigned char[]> image_buffer =
        reader.getImageBuffer(image_index, false);

    if (!image_buffer)
	return false;

    size_t image_size = reader.getImageSize(image_index);

    UT_IStream is((char *)image_buffer.get(), image_size, UT_ISTREAM_BINARY);

    UT_UniquePtr<IMG_File> img_file(IMG_File::open(is));

    UT_Array<PXL_Raster *> rasters;
    img_file->readImages(rasters);

    img_file->close();

    GA_Attribute *ncolors_attrib = gdp->addIntTuple(GA_ATTRIB_POINT, 
						    GA_SCOPE_PRIVATE,
						    "__num_colors__", 1);
    GA_Attribute *color_attrib = gdp->addDiffuseAttribute(GA_ATTRIB_POINT);

    GA_RWHandleV3 color_h(color_attrib);
    GA_RWHandleI num_colors_h(ncolors_attrib);

    UT_StringHolder group_name(reader.getImageAssociatedScanGuid(image_index));

    const UT_IntArray &scan_groups = myScanGroupMap[group_name];

    GA_PointGroup image_ptgroup(*gdp);
    for (int i = 0, ni = scan_groups.entries(); i < ni; ++i)
    {
	int scan_index = scan_groups(i);

	GA_PointGroup *scan_group = nullptr;
	UT_WorkBuffer scan_group_name(myCachedGroupPrefix.c_str());
	scan_group_name.appendFormat("{}", scan_index + 1);
	scan_group = gdp->findPointGroup(scan_group_name.buffer());

	image_ptgroup |= *scan_group;
    }

    GA_Range group_range(image_ptgroup);
    UTparallelFor(GA_SplittableRange(group_range), 
		  sop_colorFromImageParallel(gdp, color_attrib, ncolors_attrib,
					     rasters(0), reader, image_index,
					     rigid_body_transform_inv));

    if (boss.wasInterrupted())
    {
	clearSopNodeCache();
	return true;
    }

    for (int i = 0, ni = rasters.entries(); i < ni; ++i)
	delete rasters(i);

    return false;
}

void
SOP_LidarImport::clearSopNodeCache()
{
    myCachedFileName.clear();
    myCachedGroupPrefix.clear();
    myCachedUseColor = ColorDataSrc::NONE;
    myCachedUseIntensity = false;
    myCachedUseRowCol = false;
    myCachedUseReturnData = false;
    myCachedUseTimestamp = false;
    myCachedUseNormals = false;

    myCachedFilterType = FilterType::NOFILTER;
    myCachedRangeGood = 0;
    myCachedRangeSize = 0;
    myCachedMaxPoints = 0;

    myCachedDeleteInvalid = false;

    myScanGroupMap.clear();
}
