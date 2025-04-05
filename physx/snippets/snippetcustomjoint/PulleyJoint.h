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

#ifndef PULLEY_JOINT_H
#define PULLEY_JOINT_H

#include "PxPhysicsAPI.h"

// a pulley joint constrains two actors such that the sum of their distances from their respective anchor points at their attachment points 
// is a fixed value (the parameter 'distance'). Only dynamic actors are supported.
//
// The constraint equation is as follows:
//
// |anchor0 - attachment0| + |anchor1 - attachment1| * ratio = distance
// 
// where 'ratio' provides mechanical advantage.
//
// The above equation results in a singularity when the anchor point is coincident with the attachment point; for simplicity
// the constraint does not attempt to handle this case robustly.

class PulleyJoint : public ev4sio_physx::PxConstraintConnector
{
public:

	static const ev4sio_physx::PxU32 TYPE_ID = ev4sio_physx::PxConcreteType::eFIRST_USER_EXTENSION;

	PulleyJoint(ev4sio_physx::PxPhysics& physics, 
				ev4sio_physx::PxRigidBody& body0, const ev4sio_physx::PxTransform& localFrame0, const ev4sio_physx::PxVec3& attachment0,
			    ev4sio_physx::PxRigidBody& body1, const ev4sio_physx::PxTransform& localFrame1, const ev4sio_physx::PxVec3& attachment1);

	void release();

	// attribute accessor and mutators

	void			setAttachment0(const ev4sio_physx::PxVec3& pos);
	ev4sio_physx::PxVec3	getAttachment0() const;

	void			setAttachment1(const ev4sio_physx::PxVec3& pos);
	ev4sio_physx::PxVec3	getAttachment1() const;

	void			setDistance(ev4sio_physx::PxReal totalDistance);
	ev4sio_physx::PxReal	getDistance() const;
	
	void			setRatio(ev4sio_physx::PxReal ratio);
	ev4sio_physx::PxReal	getRatio() const;

	// PxConstraintConnector boilerplate

	void*			prepareData();
	void			onConstraintRelease();
	void			onComShift(ev4sio_physx::PxU32 actor);
	void			onOriginShift(const ev4sio_physx::PxVec3& shift);
	void*			getExternalReference(ev4sio_physx::PxU32& typeID);

	bool			updatePvdProperties(ev4sio_physx::pvdsdk::PvdDataStream&,
										const ev4sio_physx::PxConstraint*,
										ev4sio_physx::PxPvdUpdateType::Enum) const { return true; }
	void			updateOmniPvdProperties() const { }
	ev4sio_physx::PxBase*	getSerializable() { return NULL; }

	virtual ev4sio_physx::PxConstraintSolverPrep getPrep() const;

	virtual const void* getConstantBlock() const { return &mData; }

	struct PulleyJointData
	{
		ev4sio_physx::PxTransform c2b[2];
	
		ev4sio_physx::PxVec3 attachment0;
		ev4sio_physx::PxVec3 attachment1;

		ev4sio_physx::PxReal distance;
		ev4sio_physx::PxReal ratio;
		ev4sio_physx::PxReal tolerance;
	};

	ev4sio_physx::PxRigidBody*		mBody[2];
	ev4sio_physx::PxTransform		mLocalPose[2];

	ev4sio_physx::PxConstraint*	mConstraint;
	PulleyJointData			mData;

	~PulleyJoint() {}
};

#endif
