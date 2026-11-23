// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SC_ARTICULATION_MIMIC_JOINT_SIM_H
#define SC_ARTICULATION_MIMIC_JOINT_SIM_H

#include "foundation/PxUserAllocated.h"
#include "PxArticulationReducedCoordinate.h"
#include "DyFeatherstoneArticulation.h"
#include "DyArticulationMimicJointCore.h"

namespace ev4sio_physx
{
namespace ev4sio_Sc
{
class Scene;
class ArticulationMimicJointCore;
class ArticulationCore;
class ArticulationSim;

class ArticulationMimicJointSim : public PxUserAllocated
{

	PX_NOCOPY(ArticulationMimicJointSim)

	
public:
	ArticulationMimicJointSim(ev4sio_Sc::ArticulationMimicJointCore& core, ev4sio_Sc::Scene& scene);

	~ArticulationMimicJointSim();

	PX_FORCE_INLINE ev4sio_Sc::Scene& getScene() { return mScene; }
	PX_FORCE_INLINE const ev4sio_Sc::Scene& getScene() const { return mScene; }

	PX_FORCE_INLINE void setLowLevelIndex(const PxU32 llIndex) { mLLIndex = llIndex;}
	PX_FORCE_INLINE PxU32 getLowLevelIndex() const { return mLLIndex; }
	
	PX_FORCE_INLINE ev4sio_Sc::ArticulationMimicJointCore& getCore() { return mCore; }
	PX_FORCE_INLINE const ev4sio_Sc::ArticulationMimicJointCore& getCore() const { return mCore; }

	PX_FORCE_INLINE ev4sio_Dy::ArticulationMimicJointCore& getLLMimicJoint() { return mLLMimicJoint; }

	void setGearRatio(const PxReal gearRatio);
	void setOffset(const PxReal offset);
	void setNaturalFrequency(const PxReal naturalFrequency);
	void setDampingRatio(const PxReal dampingRatio);

	ev4sio_Sc::Scene&							mScene;
	ev4sio_Sc::ArticulationMimicJointCore&		mCore;
	ev4sio_Sc::ArticulationSim*				mArticulationSim;
	ev4sio_Dy::ArticulationMimicJointCore		mLLMimicJoint;
	PxU32								mLLIndex;
};

}
}

#endif //SC_ARTICULATION_MIMIC_JOINT_SIM_H


