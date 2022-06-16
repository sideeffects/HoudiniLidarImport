/*
 * Copyright (c) 2022
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

#include <SOP/SOP_Node.h>
#include <SOP/SOP_NodeVerb.h>
#include <UT/UT_StringHolder.h>
#include <UT/UT_Interrupt.h>

class SOP_LidarImport : public SOP_Node 
{
public:
    static OP_Node          *myConstructor( OP_Network *net, const char *name,
                                        OP_Operator *op);
    static PRM_Template     *buildTemplates();
    static void              installSOP(OP_OperatorTable *table);

    // Callbacks
        static int           reloadGeometryCB(
                                                void *data,
                                                int index,
                                                fpreal now,
                                                const PRM_Template *);
    static int               moveCentroidToOriginCB(
                                                void *data,
                                                int index,
                                                fpreal now,
                                                const PRM_Template *);
    static int               movePivotToCentroidCB(
                                                void *data,
                                                int index,
                                                fpreal now,
                                                const PRM_Template *);

protected:
    SOP_LidarImport(OP_Network *net, const char *name, OP_Operator *op);
    ~SOP_LidarImport() override;

    const SOP_NodeVerb      *cookVerb() const override;
    OP_ERROR                 cookMySop(OP_Context &context) override;

    void		     syncNodeVersion(
				    const char *old_version,
				    const char *cur_version,
				    bool *node_deleted) override;

    void		     getDescriptiveParmName(UT_String &name) const override
			     { name = "filename"; }

private:
    // Callback helpers
    bool	 moveCentroidToOrigin(fpreal now);
    bool	 movePivotToCentroid(fpreal now);

    // Callback parm accessors
    int		 TRS() const		    { return evalInt("xOrd", 0, 0); }
    int		 XYZ() const		    { return evalInt("rOrd", 0, 0); }

    fpreal	 TX(fpreal t) const	    { return evalFloat("t", 0, t); }
    fpreal	 TY(fpreal t) const	    { return evalFloat("t", 1, t); }
    fpreal	 TZ(fpreal t) const	    { return evalFloat("t", 2, t); }

    fpreal	 RX(fpreal t) const	    { return evalFloat("r", 0, t); }
    fpreal	 RY(fpreal t) const	    { return evalFloat("r", 1, t); }
    fpreal	 RZ(fpreal t) const	    { return evalFloat("r", 2, t); }

    fpreal	 SX(fpreal t) const	    { return evalFloat("s", 0, t); }
    fpreal	 SY(fpreal t) const	    { return evalFloat("s", 1, t); }
    fpreal	 SZ(fpreal t) const	    { return evalFloat("s", 2, t); }

    fpreal	 SHEAR_XY(fpreal t) const   { return evalFloat("shear", 0, t); }
    fpreal	 SHEAR_XZ(fpreal t) const   { return evalFloat("shear", 1, t); }
    fpreal	 SHEAR_YZ(fpreal t) const   { return evalFloat("shear", 2, t); }

    fpreal	 PX(fpreal t) const
		    { return evalFloat("p", 0, t); }
    fpreal	 PY(fpreal t) const
		    { return evalFloat("p", 1, t); }
    fpreal	 PZ(fpreal t) const
		    { return evalFloat("p", 2, t); }

    fpreal	 PIVOT_RX(fpreal t) const
		    { return evalFloat("pr", 0, t); }
    fpreal	 PIVOT_RY(fpreal t) const
		    { return evalFloat("pr", 1, t); }
    fpreal	 PIVOT_RZ(fpreal t) const
		    { return evalFloat("pr", 2, t); }

    fpreal	 SCALE(fpreal t) const	    { return evalFloat("scale", 0, t); }

    int		 PRETRANSFORM_TRS() const
		    { return evalInt("prexform_xOrd", 0, 0); }
    int		 PRETRANSFORM_XYZ() const
		    { return evalInt("prexform_rOrd", 0, 0); }
    fpreal	 PRETRANSFORM_TX(fpreal t) const
		    { return evalFloat("prexform_t", 0, t); }
    fpreal	 PRETRANSFORM_TY(fpreal t) const
		    { return evalFloat("prexform_t", 1, t); }
    fpreal	 PRETRANSFORM_TZ(fpreal t) const
		    { return evalFloat("prexform_t", 2, t); }
    fpreal	 PRETRANSFORM_RX(fpreal t) const
		    { return evalFloat("prexform_r", 0, t); }
    fpreal	 PRETRANSFORM_RY(fpreal t) const
		    { return evalFloat("prexform_r", 1, t); }
    fpreal	 PRETRANSFORM_RZ(fpreal t) const
		    { return evalFloat("prexform_r", 2, t); }
    fpreal	 PRETRANSFORM_SX(fpreal t) const
		    { return evalFloat("prexform_s", 0, t); }
    fpreal	 PRETRANSFORM_SY(fpreal t) const
		    { return evalFloat("prexform_s", 1, t); }
    fpreal	 PRETRANSFORM_SZ(fpreal t) const
		    { return evalFloat("prexform_s", 2, t); }
    fpreal	 PRETRANSFORM_SHEAR_XY(fpreal t) const
		    { return evalFloat("prexform_shear", 0, t); }
    fpreal	 PRETRANSFORM_SHEAR_XZ(fpreal t) const
		    { return evalFloat("prexform_shear", 1, t); }
    fpreal	 PRETRANSFORM_SHEAR_YZ(fpreal t) const
		    { return evalFloat("prexform_shear", 2, t); }
};

#endif
