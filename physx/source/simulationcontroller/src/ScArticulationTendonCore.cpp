// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
