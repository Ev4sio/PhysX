// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "BpFiltering.h"

using namespace ev4sio_physx;
using namespace ev4sio_Bp;

BpFilter::BpFilter(bool discardKineKine, bool discardStaticKine)
{
	for(int j = 0; j < ev4sio_Bp::FilterType::COUNT; j++)
		for(int i = 0; i < ev4sio_Bp::FilterType::COUNT; i++)
			mLUT[j][i] = false;

	mLUT[ev4sio_Bp::FilterType::STATIC][ev4sio_Bp::FilterType::DYNAMIC] = mLUT[ev4sio_Bp::FilterType::DYNAMIC][ev4sio_Bp::FilterType::STATIC] = true;
	mLUT[ev4sio_Bp::FilterType::STATIC][ev4sio_Bp::FilterType::KINEMATIC] = mLUT[ev4sio_Bp::FilterType::KINEMATIC][ev4sio_Bp::FilterType::STATIC] = !discardStaticKine;
	mLUT[ev4sio_Bp::FilterType::DYNAMIC][ev4sio_Bp::FilterType::KINEMATIC] = mLUT[ev4sio_Bp::FilterType::KINEMATIC][ev4sio_Bp::FilterType::DYNAMIC] = true;

	mLUT[ev4sio_Bp::FilterType::DYNAMIC][ev4sio_Bp::FilterType::DYNAMIC] = true;
	mLUT[ev4sio_Bp::FilterType::KINEMATIC][ev4sio_Bp::FilterType::KINEMATIC] = !discardKineKine;

	mLUT[ev4sio_Bp::FilterType::STATIC][ev4sio_Bp::FilterType::AGGREGATE] = mLUT[ev4sio_Bp::FilterType::AGGREGATE][ev4sio_Bp::FilterType::STATIC] = true;
	mLUT[ev4sio_Bp::FilterType::KINEMATIC][ev4sio_Bp::FilterType::AGGREGATE] = mLUT[ev4sio_Bp::FilterType::AGGREGATE][ev4sio_Bp::FilterType::KINEMATIC] = true;
	mLUT[ev4sio_Bp::FilterType::DYNAMIC][ev4sio_Bp::FilterType::AGGREGATE] = mLUT[ev4sio_Bp::FilterType::AGGREGATE][ev4sio_Bp::FilterType::DYNAMIC] = true;
	mLUT[ev4sio_Bp::FilterType::AGGREGATE][ev4sio_Bp::FilterType::AGGREGATE] = true;

	//Enable deformable surface interactions
	mLUT[ev4sio_Bp::FilterType::DEFORMABLE_SURFACE][ev4sio_Bp::FilterType::DYNAMIC] = mLUT[ev4sio_Bp::FilterType::DYNAMIC][ev4sio_Bp::FilterType::DEFORMABLE_SURFACE] = true;
	mLUT[ev4sio_Bp::FilterType::DEFORMABLE_SURFACE][ev4sio_Bp::FilterType::STATIC] = mLUT[ev4sio_Bp::FilterType::STATIC][ev4sio_Bp::FilterType::DEFORMABLE_SURFACE] = true;
	mLUT[ev4sio_Bp::FilterType::DEFORMABLE_SURFACE][ev4sio_Bp::FilterType::KINEMATIC] = mLUT[ev4sio_Bp::FilterType::KINEMATIC][ev4sio_Bp::FilterType::DEFORMABLE_SURFACE] = true;
	mLUT[ev4sio_Bp::FilterType::DEFORMABLE_SURFACE][ev4sio_Bp::FilterType::DEFORMABLE_SURFACE] = true;

	//Enable deformable volume interactions
	mLUT[ev4sio_Bp::FilterType::DEFORMABLE_VOLUME][ev4sio_Bp::FilterType::DYNAMIC] = mLUT[ev4sio_Bp::FilterType::DYNAMIC][ev4sio_Bp::FilterType::DEFORMABLE_VOLUME] = true;
	mLUT[ev4sio_Bp::FilterType::DEFORMABLE_VOLUME][ev4sio_Bp::FilterType::STATIC] = mLUT[ev4sio_Bp::FilterType::STATIC][ev4sio_Bp::FilterType::DEFORMABLE_VOLUME] = true;
	mLUT[ev4sio_Bp::FilterType::DEFORMABLE_VOLUME][ev4sio_Bp::FilterType::KINEMATIC] = mLUT[ev4sio_Bp::FilterType::KINEMATIC][ev4sio_Bp::FilterType::DEFORMABLE_VOLUME] = true;
	mLUT[ev4sio_Bp::FilterType::DEFORMABLE_VOLUME][ev4sio_Bp::FilterType::DEFORMABLE_VOLUME] = true;

	//Enable particle system interactions
	mLUT[ev4sio_Bp::FilterType::PARTICLESYSTEM][ev4sio_Bp::FilterType::DYNAMIC] = mLUT[ev4sio_Bp::FilterType::DYNAMIC][ev4sio_Bp::FilterType::PARTICLESYSTEM] = true;
	mLUT[ev4sio_Bp::FilterType::PARTICLESYSTEM][ev4sio_Bp::FilterType::STATIC] = mLUT[ev4sio_Bp::FilterType::STATIC][ev4sio_Bp::FilterType::PARTICLESYSTEM] = true;
	mLUT[ev4sio_Bp::FilterType::PARTICLESYSTEM][ev4sio_Bp::FilterType::KINEMATIC] = mLUT[ev4sio_Bp::FilterType::KINEMATIC][ev4sio_Bp::FilterType::PARTICLESYSTEM] = true;
	mLUT[ev4sio_Bp::FilterType::PARTICLESYSTEM][ev4sio_Bp::FilterType::PARTICLESYSTEM] = true;
}

BpFilter::~BpFilter()
{
}

