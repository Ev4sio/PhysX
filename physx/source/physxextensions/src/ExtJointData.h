// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef EXT_JOINT_DATA_H
#define EXT_JOINT_DATA_H

#include "extensions/PxJointLimit.h"

namespace ev4sio_physx
{
namespace ev4sio_Ext
{
	struct JointData
	{
				PxConstraintInvMassScale	invMassScale;
				PxTransform32				c2b[2];
	// Ev4sio.Native owns copies of joint data and needs a public destructor.
		        ~JointData()	{}
	};

} // namespace ev4sio_Ext
}

#endif
