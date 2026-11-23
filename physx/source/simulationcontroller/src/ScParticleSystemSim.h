// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SC_PARTICLESYSTEM_SIM_H
#define SC_PARTICLESYSTEM_SIM_H

#include "foundation/PxPreprocessor.h"
#if PX_SUPPORT_GPU_PHYSX
#include "foundation/PxUserAllocated.h"
#include "ScGpuActorSim.h"
#include "ScParticleSystemCore.h" 

namespace ev4sio_physx
{
	namespace ev4sio_Sc
	{
		class Scene;

		class ParticleSystemSim : public GPUActorSim
		{
			PX_NOCOPY(ParticleSystemSim)
		public:
			ParticleSystemSim(ParticleSystemCore& core, Scene& scene);
			~ParticleSystemSim();

			PX_INLINE	ev4sio_Dy::ParticleSystem*		getLowLevelParticleSystem() const { return mLLParticleSystem; }
			PX_INLINE	ParticleSystemCore&		getCore() const { return static_cast<ParticleSystemCore&>(mCore); }

			virtual			PxActor*		getPxActor() const PX_OVERRIDE { return getCore().getPxActor(); }

			bool							isSleeping() const;
			bool							isActive() const { return true; }
			void							sleepCheck(PxReal dt);

			void							setActive(bool active, bool asPartOfCreation=false);

			void							createLowLevelVolume();

		private:
			ev4sio_Dy::ParticleSystem*				mLLParticleSystem;

// PT: as far as I can tell these are never actually called
//								void			activate();
//								void			deactivate();
		};

	} // namespace ev4sio_Sc
}
#endif

#endif
