// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "ScArticulationTendonSim.h"
#include "ScArticulationTendonCore.h"
#include "ScArticulationAttachmentCore.h"
#include "ScArticulationTendonJointCore.h"
#include "ScArticulationJointCore.h"
#include "ScScene.h"
#include "DyArticulationTendon.h"
#include "ScArticulationSim.h"


using namespace ev4sio_physx;

ev4sio_Sc::ArticulationSpatialTendonSim::ArticulationSpatialTendonSim(ArticulationSpatialTendonCore& tendon, Scene& scene) :
	mTendonCore(tendon), mScene(scene)
{
	mTendonCore.setSim(this);
	mLLTendon.mStiffness = tendon.mStiffness;
	mLLTendon.mDamping = tendon.mDamping;
	mLLTendon.mOffset = tendon.mOffset;
	mLLTendon.mLimitStiffness = tendon.mLimitStiffness;
}


ev4sio_Sc::ArticulationSpatialTendonSim::~ArticulationSpatialTendonSim()
{
	mTendonCore.setSim(NULL);
}


void ev4sio_Sc::ArticulationSpatialTendonSim::setStiffness(const PxReal stiffness)
{
	mLLTendon.mStiffness = stiffness;

	ev4sio_Dy::FeatherstoneArticulation* llArticulation = static_cast<ev4sio_Dy::FeatherstoneArticulation*>(mArtiSim->getLowLevelArticulation());
	llArticulation->setGpuDirtyFlag(ev4sio_Dy::ArticulationDirtyFlag::eDIRTY_SPATIAL_TENDON);
}

PxReal ev4sio_Sc::ArticulationSpatialTendonSim::getStiffness() const
{
	return mLLTendon.mStiffness;
}

void ev4sio_Sc::ArticulationSpatialTendonSim::setDamping(const PxReal damping)
{
	mLLTendon.mDamping = damping;

	ev4sio_Dy::FeatherstoneArticulation* llArticulation = static_cast<ev4sio_Dy::FeatherstoneArticulation*>(mArtiSim->getLowLevelArticulation());
	llArticulation->setGpuDirtyFlag(ev4sio_Dy::ArticulationDirtyFlag::eDIRTY_SPATIAL_TENDON);
}

PxReal ev4sio_Sc::ArticulationSpatialTendonSim::getDamping() const
{
	return mLLTendon.mDamping;
}

void ev4sio_Sc::ArticulationSpatialTendonSim::setLimitStiffness(const PxReal stiffness)
{
	mLLTendon.mLimitStiffness = stiffness;

	ev4sio_Dy::FeatherstoneArticulation* llArticulation = static_cast<ev4sio_Dy::FeatherstoneArticulation*>(mArtiSim->getLowLevelArticulation());
	llArticulation->setGpuDirtyFlag(ev4sio_Dy::ArticulationDirtyFlag::eDIRTY_FIXED_TENDON);
}

PxReal ev4sio_Sc::ArticulationSpatialTendonSim::getLimitStiffness() const
{
	return mLLTendon.mLimitStiffness;
}

void ev4sio_Sc::ArticulationSpatialTendonSim::setOffset(const PxReal offset)
{
	mLLTendon.mOffset = offset;

	ev4sio_Dy::FeatherstoneArticulation* llArticulation = static_cast<ev4sio_Dy::FeatherstoneArticulation*>(mArtiSim->getLowLevelArticulation());
	llArticulation->setGpuDirtyFlag(ev4sio_Dy::ArticulationDirtyFlag::eDIRTY_SPATIAL_TENDON);
}

PxReal ev4sio_Sc::ArticulationSpatialTendonSim::getOffset() const
{
	return mLLTendon.mOffset;
}


void ev4sio_Sc::ArticulationSpatialTendonSim::setAttachmentCoefficient(ArticulationAttachmentCore& core, const PxReal coefficient)
{
	const PxU32 index = core.mAttachmentIndex;

	ev4sio_Dy::ArticulationAttachment& attachment = mLLTendon.getAttachment(index);

	attachment.coefficient = coefficient;

	ev4sio_Dy::FeatherstoneArticulation* llArticulation = static_cast<ev4sio_Dy::FeatherstoneArticulation*>(mArtiSim->getLowLevelArticulation());
	llArticulation->setGpuDirtyFlag(ev4sio_Dy::ArticulationDirtyFlag::eDIRTY_SPATIAL_TENDON_ATTACHMENT);
}

void ev4sio_Sc::ArticulationSpatialTendonSim::setAttachmentRelativeOffset(ArticulationAttachmentCore& core, const PxVec3& offset)
{
	const PxU32 index = core.mAttachmentIndex;

	ev4sio_Dy::ArticulationAttachment& attachment = mLLTendon.getAttachment(index);

	attachment.relativeOffset = offset;

	ev4sio_Dy::FeatherstoneArticulation* llArticulation = static_cast<ev4sio_Dy::FeatherstoneArticulation*>(mArtiSim->getLowLevelArticulation());
	llArticulation->setGpuDirtyFlag(ev4sio_Dy::ArticulationDirtyFlag::eDIRTY_SPATIAL_TENDON_ATTACHMENT);
}

void ev4sio_Sc::ArticulationSpatialTendonSim::setAttachmentLimits(ArticulationAttachmentCore& core, const PxReal lowLimit, const PxReal highLimit)
{
	const PxU32 index = core.mAttachmentIndex;

	ev4sio_Dy::ArticulationAttachment& attachment = mLLTendon.getAttachment(index);

	attachment.lowLimit = lowLimit;
	attachment.highLimit = highLimit;

	ev4sio_Dy::FeatherstoneArticulation* llArticulation = static_cast<ev4sio_Dy::FeatherstoneArticulation*>(mArtiSim->getLowLevelArticulation());
	llArticulation->setGpuDirtyFlag(ev4sio_Dy::ArticulationDirtyFlag::eDIRTY_SPATIAL_TENDON_ATTACHMENT);
}

void ev4sio_Sc::ArticulationSpatialTendonSim::setAttachmentRestLength(ArticulationAttachmentCore& core, const PxReal restLength)
{
	const PxU32 index = core.mAttachmentIndex;
	ev4sio_Dy::ArticulationAttachment& attachment = mLLTendon.getAttachment(index);
	attachment.restLength = restLength;

	ev4sio_Dy::FeatherstoneArticulation* llArticulation = static_cast<ev4sio_Dy::FeatherstoneArticulation*>(mArtiSim->getLowLevelArticulation());
	llArticulation->setGpuDirtyFlag(ev4sio_Dy::ArticulationDirtyFlag::eDIRTY_SPATIAL_TENDON_ATTACHMENT);

}


void ev4sio_Sc::ArticulationSpatialTendonSim::addAttachment(ArticulationAttachmentCore& core)
{

	const PxU32 index = mLLTendon.getNewID();

	ev4sio_Dy::ArticulationAttachment& attachment = mLLTendon.getAttachment(index);

	attachment.relativeOffset = core.mRelativeOffset;
	attachment.linkInd = PxU16(core.mLLLinkIndex);
	attachment.lowLimit = core.mLowLimit;
	attachment.highLimit = core.mHighLimit;
	attachment.coefficient = core.mCoefficient;
	attachment.myInd = index;
	attachment.children = 0;
	attachment.childCount = 0;
	attachment.restLength = core.mRestLength;

	core.mAttachmentIndex = index;
	core.mTendonSim = this;

	if (core.mParent)
	{
		const PxU32 parentIndex = core.mParent->mAttachmentIndex;
		attachment.parent = parentIndex;
		mLLTendon.getAttachment(parentIndex).children |= ev4sio_Dy::ArticulationAttachmentBitField(1) << index;
		mLLTendon.getAttachment(parentIndex).childCount++;
	}
	else
	{
		attachment.parent = DY_ARTICULATION_ATTACHMENT_NONE;
	}

}

void ev4sio_Sc::ArticulationSpatialTendonSim::removeAttachment(ArticulationAttachmentCore& core)
{
	const PxU32 index = core.mAttachmentIndex;

	ev4sio_Dy::ArticulationAttachment& attachment = mLLTendon.getAttachment(index);

	PX_ASSERT(attachment.childCount == 0);

	if (attachment.parent != DY_ARTICULATION_ATTACHMENT_NONE)
	{
		ev4sio_Dy::ArticulationAttachment& parent = mLLTendon.getAttachment(attachment.parent);
		parent.children &= ~(ev4sio_Dy::ArticulationAttachmentBitField(1) << index);
		parent.childCount--;
	}

	mLLTendon.freeID(index);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

ev4sio_Sc::ArticulationFixedTendonSim::ArticulationFixedTendonSim(ArticulationFixedTendonCore& tendon, Scene& scene) :
	mTendonCore(tendon), mScene(scene)
{
	mTendonCore.setSim(this);
	mLLTendon.mStiffness = tendon.mStiffness;
	mLLTendon.mDamping = tendon.mDamping;
	mLLTendon.mOffset = tendon.mOffset;
	mLLTendon.mLimitStiffness = tendon.mLimitStiffness;
	mLLTendon.mLowLimit = tendon.mLowLimit;
	mLLTendon.mHighLimit = tendon.mHighLimit;
	mLLTendon.mRestLength = tendon.mRestLength;
}

ev4sio_Sc::ArticulationFixedTendonSim::~ArticulationFixedTendonSim()
{
	mTendonCore.setSim(NULL);
}

void ev4sio_Sc::ArticulationFixedTendonSim::setStiffness(const PxReal stiffness)
{
	mLLTendon.mStiffness = stiffness;

	ev4sio_Dy::FeatherstoneArticulation* llArticulation = static_cast<ev4sio_Dy::FeatherstoneArticulation*>(mArtiSim->getLowLevelArticulation());
	llArticulation->setGpuDirtyFlag(ev4sio_Dy::ArticulationDirtyFlag::eDIRTY_FIXED_TENDON);
}

PxReal ev4sio_Sc::ArticulationFixedTendonSim::getStiffness() const
{
	return mLLTendon.mStiffness;
}

void ev4sio_Sc::ArticulationFixedTendonSim::setDamping(const PxReal damping)
{
	mLLTendon.mDamping = damping;

	ev4sio_Dy::FeatherstoneArticulation* llArticulation = static_cast<ev4sio_Dy::FeatherstoneArticulation*>(mArtiSim->getLowLevelArticulation());
	llArticulation->setGpuDirtyFlag(ev4sio_Dy::ArticulationDirtyFlag::eDIRTY_FIXED_TENDON);
}

PxReal ev4sio_Sc::ArticulationFixedTendonSim::getDamping() const
{
	return mLLTendon.mDamping;
}

void ev4sio_Sc::ArticulationFixedTendonSim::setLimitStiffness(const PxReal stiffness)
{
	mLLTendon.mLimitStiffness = stiffness;

	ev4sio_Dy::FeatherstoneArticulation* llArticulation = static_cast<ev4sio_Dy::FeatherstoneArticulation*>(mArtiSim->getLowLevelArticulation());
	llArticulation->setGpuDirtyFlag(ev4sio_Dy::ArticulationDirtyFlag::eDIRTY_FIXED_TENDON);
}

PxReal ev4sio_Sc::ArticulationFixedTendonSim::getLimitStiffness() const
{
	return mLLTendon.mLimitStiffness;
}

void ev4sio_Sc::ArticulationFixedTendonSim::setOffset(const PxReal offset)
{
	mLLTendon.mOffset = offset;

	mArtiSim->setArticulationDirty(ev4sio_Dy::ArticulationDirtyFlag::eDIRTY_FIXED_TENDON);
}

PxReal ev4sio_Sc::ArticulationFixedTendonSim::getOffset() const
{
	return mLLTendon.mOffset;
}

void ev4sio_Sc::ArticulationFixedTendonSim::setSpringRestLength(const PxReal restLength)
{
	mLLTendon.mRestLength = restLength;

	mArtiSim->setArticulationDirty(ev4sio_Dy::ArticulationDirtyFlag::eDIRTY_FIXED_TENDON);
}

PxReal ev4sio_Sc::ArticulationFixedTendonSim::getSpringRestLength() const
{
	return mLLTendon.mRestLength;
}


void ev4sio_Sc::ArticulationFixedTendonSim::setLimitRange(const PxReal lowLimit, const PxReal highLimit)
{
	mLLTendon.mLowLimit = lowLimit;
	mLLTendon.mHighLimit = highLimit;

	mArtiSim->setArticulationDirty(ev4sio_Dy::ArticulationDirtyFlag::eDIRTY_FIXED_TENDON);
}

void ev4sio_Sc::ArticulationFixedTendonSim::getLimitRange(PxReal& lowLimit, PxReal& highLimit) const
{
	lowLimit = mLLTendon.mLowLimit;
	highLimit = mLLTendon.mHighLimit;
}

void ev4sio_Sc::ArticulationFixedTendonSim::addTendonJoint(ArticulationTendonJointCore& tendonJointCore)
{

	const PxU32 jointIndex = mLLTendon.getNewID();

	ev4sio_Dy::ArticulationTendonJoint& tendonJoint = mLLTendon.getTendonJoint(jointIndex);

	tendonJoint.axis = PxU16(tendonJointCore.axis);
	tendonJoint.coefficient = tendonJointCore.coefficient;
	tendonJoint.recipCoefficient = tendonJointCore.recipCoefficient;
	tendonJoint.linkInd = PxU16(tendonJointCore.mLLLinkIndex);
	tendonJoint.children = 0;
	tendonJoint.childCount = 0;

	tendonJointCore.mLLTendonJointIndex = jointIndex;
	//tendonJointCore.mLLTendonJoint = &tendonJoint;
	tendonJointCore.mTendonSim = this;

	if (tendonJointCore.mParent)
	{
		const PxU32 parentIndex = tendonJointCore.mParent->mLLTendonJointIndex;
		tendonJoint.parent = parentIndex;
		mLLTendon.getTendonJoint(parentIndex).children |= ev4sio_Dy::ArticulationAttachmentBitField(1) << jointIndex;
		mLLTendon.getTendonJoint(parentIndex).childCount++;
	}
	else
	{
		tendonJoint.parent = DY_ARTICULATION_ATTACHMENT_NONE;
	}
	
}

void ev4sio_Sc::ArticulationFixedTendonSim::removeTendonJoint(ArticulationTendonJointCore& core)
{
	const PxU32 index = core.mLLTendonJointIndex;

	ev4sio_Dy::ArticulationTendonJoint& tendonJoint = mLLTendon.getTendonJoint(index);

	PX_ASSERT(tendonJoint.childCount == 0);

	if (tendonJoint.parent != DY_ARTICULATION_ATTACHMENT_NONE)
	{
		ev4sio_Dy::ArticulationTendonJoint& parent = mLLTendon.getTendonJoint(tendonJoint.parent);
		parent.children &= ~(ev4sio_Dy::ArticulationAttachmentBitField(1) << index);
		parent.childCount--;
	}

	mLLTendon.freeID(index);
}

void ev4sio_Sc::ArticulationFixedTendonSim::setTendonJointCoefficient(ArticulationTendonJointCore& core, const PxArticulationAxis::Enum axis, const float coefficient, const float recipCoefficient)
{
	const PxU32 index = core.mLLTendonJointIndex;

	ev4sio_Dy::ArticulationTendonJoint& tendonJoint = mLLTendon.getTendonJoint(index);
	tendonJoint.axis = PxU16(axis);
	tendonJoint.coefficient = coefficient;
	tendonJoint.recipCoefficient = recipCoefficient;

	mArtiSim->setArticulationDirty(ev4sio_Dy::ArticulationDirtyFlag::eDIRTY_FIXED_TENDON_JOINT);

}


