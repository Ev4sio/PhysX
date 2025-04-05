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

#include "ScArticulationTendonCore.h"
#include "ScArticulationTendonSim.h"

using namespace ev4sio_physx;

void ev4sio_Sc::ArticulationSpatialTendonCore::setStiffness(const PxReal stiffness)
{
	mStiffness = stiffness;

	if (mSim)
		mSim->setStiffness(stiffness);
}

PxReal ev4sio_Sc::ArticulationSpatialTendonCore::getStiffness() const
{
	return mStiffness;
}

void ev4sio_Sc::ArticulationSpatialTendonCore::setDamping(const PxReal damping)
{
	mDamping = damping;

	if (mSim)
		mSim->setDamping(damping);
}

PxReal ev4sio_Sc::ArticulationSpatialTendonCore::getDamping() const
{
	return mDamping;
}

void ev4sio_Sc::ArticulationSpatialTendonCore::setLimitStiffness(const PxReal stiffness)
{
	mLimitStiffness = stiffness;

	if (mSim)
		mSim->setLimitStiffness(stiffness);
}

PxReal ev4sio_Sc::ArticulationSpatialTendonCore::getLimitStiffness() const
{
	return mLimitStiffness;
}

void ev4sio_Sc::ArticulationSpatialTendonCore::setOffset(const PxReal offset)
{
	mOffset = offset;

	if (mSim)
		mSim->setOffset(offset);
}

PxReal ev4sio_Sc::ArticulationSpatialTendonCore::getOffset() const
{
	return mOffset;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////

void ev4sio_Sc::ArticulationFixedTendonCore::setStiffness(const PxReal stiffness)
{
	mStiffness = stiffness;

	if (mSim)
		mSim->setStiffness(stiffness);
}

PxReal ev4sio_Sc::ArticulationFixedTendonCore::getStiffness() const
{
	return mStiffness;
}

void ev4sio_Sc::ArticulationFixedTendonCore::setDamping(const PxReal damping)
{
	mDamping = damping;

	if (mSim)
		mSim->setDamping(damping);
}

PxReal ev4sio_Sc::ArticulationFixedTendonCore::getDamping() const
{
	return mDamping;
}

void ev4sio_Sc::ArticulationFixedTendonCore::setLimitStiffness(const PxReal stiffness)
{
	mLimitStiffness = stiffness;
	if (mSim)
		mSim->setLimitStiffness(stiffness);
}

PxReal ev4sio_Sc::ArticulationFixedTendonCore::getLimitStiffness() const
{
	return mLimitStiffness;
}

void ev4sio_Sc::ArticulationFixedTendonCore::setSpringRestLength(const PxReal restLength)
{
	mRestLength = restLength;
	if (mSim)
		mSim->setSpringRestLength(restLength);
}

PxReal	ev4sio_Sc::ArticulationFixedTendonCore::getSpringRestLength() const
{
	return mRestLength;
}

void ev4sio_Sc::ArticulationFixedTendonCore::setLimitRange(const PxReal lowLimit, const PxReal highLimit)
{
	mLowLimit = lowLimit;
	mHighLimit = highLimit;

	if (mSim)
		mSim->setLimitRange(lowLimit, highLimit);
}

void ev4sio_Sc::ArticulationFixedTendonCore::getLimitRange(PxReal& lowLimit, PxReal& highLimit) const
{
	lowLimit = mLowLimit;
	highLimit = mHighLimit;
}

void ev4sio_Sc::ArticulationFixedTendonCore::setOffset(const PxReal offset)
{
	mOffset = offset;
	if (mSim)
		mSim->setOffset(offset);
}

PxReal ev4sio_Sc::ArticulationFixedTendonCore::getOffset() const
{
	return mOffset;
}
