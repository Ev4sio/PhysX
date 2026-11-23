// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "foundation/PxPreprocessor.h"

#if PX_SUPPORT_GPU_PHYSX

#include "ScParticleSystemSim.h"

using namespace ev4sio_physx;
using namespace ev4sio_Dy;

ev4sio_Sc::ParticleSystemSim::ParticleSystemSim(ParticleSystemCore& core, Scene& scene) :
	GPUActorSim(scene, core, &core.getShapeCore())
{
	createLowLevelVolume();

	mLLParticleSystem = scene.createLLParticleSystem(this);

	mNodeIndex = scene.getSimpleIslandManager()->addNode(false, false, ev4sio_IG::Node::ePARTICLESYSTEM_TYPE, mLLParticleSystem);

	scene.getSimpleIslandManager()->activateNode(mNodeIndex);

	//mCore.setSim(this);

	mLLParticleSystem->setElementId(mShapeSim.getElementID());

	PxParticleSystemGeometry geometry;
	core.getShapeCore().setGeometry(geometry);
	
	mLLParticleSystem->setShapeCore(&core.getShapeCore());
}

ev4sio_Sc::ParticleSystemSim::~ParticleSystemSim()
{
	if (!mLLParticleSystem)
		return;

	mScene.destroyLLParticleSystem(*mLLParticleSystem);

	mScene.getSimpleIslandManager()->removeNode(mNodeIndex);

	mCore.setSim(NULL);
}

void ev4sio_Sc::ParticleSystemSim::createLowLevelVolume()
{
	//PX_ASSERT(getWorldBounds().isFinite());

	const PxU32 index = mShapeSim.getElementID();

	if (!(static_cast<ev4sio_Sc::ParticleSystemSim&>(mShapeSim.getActor()).getCore().getFlags() & PxParticleFlag::eDISABLE_RIGID_COLLISION))
	{
		mScene.getBoundsArray().setBounds(PxBounds3(PxVec3(PX_MAX_BOUNDS_EXTENTS), PxVec3(-PX_MAX_BOUNDS_EXTENTS)), index);
		mShapeSim.setInBroadPhase();
	}
	else
		mScene.getAABBManager()->reserveSpaceForBounds(index);

	addToAABBMgr(ev4sio_Bp::FilterType::PARTICLESYSTEM);
}

bool ev4sio_Sc::ParticleSystemSim::isSleeping() const
{
	return false;
}

void ev4sio_Sc::ParticleSystemSim::sleepCheck(PxReal dt)
{
	PX_UNUSED(dt);
}

/*void ev4sio_Sc::ParticleSystemSim::activate()
{
	activateInteractions(*this);
}

void ev4sio_Sc::ParticleSystemSim::deactivate()
{
	deactivateInteractions(*this);
}*/

#endif //PX_SUPPORT_GPU_PHYSX
