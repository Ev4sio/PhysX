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

