// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_INTERSECTION_SPHERE_BOX_H
#define GU_INTERSECTION_SPHERE_BOX_H

namespace ev4sio_physx
{
namespace ev4sio_Gu
{
	class Sphere;
	class Box;

	/**
	Checks if a sphere intersects a box. Based on: Jim Arvo, A Simple Method for Box-Sphere Intersection Testing, Graphics Gems, pp. 247-250.

	\param sphere	[in] sphere
	\param box		[in] box

	\return	true if sphere overlaps box (or exactly touches it)
	*/
	bool intersectSphereBox(const ev4sio_Gu::Sphere& sphere, const ev4sio_Gu::Box& box);

} // namespace ev4sio_Gu

}

#endif
