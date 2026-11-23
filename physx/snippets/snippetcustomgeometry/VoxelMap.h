// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef VOXEL_MAP_H
#define VOXEL_MAP_H

#include "PxPhysicsAPI.h"

/*
VoxelMap inherits ev4sio_physx::PxCustomGeometry::Callbacks interface and provides
implementations for 5 callback functions:

- getLocalBounds
- generateContacts
- raycast
- overlap
- sweep

It should be passed to PxCustomGeometry constructor.

*/
struct VoxelMap : ev4sio_physx::PxCustomGeometry::Callbacks, ev4sio_physx::PxUserAllocated
{
	void setDimensions(int x, int y, int z);
	int dimX() const;
	int dimY() const;
	int dimZ() const;

	void setVoxelSize(float x, float y, float z);
	const ev4sio_physx::PxVec3& voxelSize() const;
	float voxelSizeX() const;
	float voxelSizeY() const;
	float voxelSizeZ() const;

	ev4sio_physx::PxVec3 extents() const;

	void setVoxel(int x, int y, int z, bool yes = true);

	bool voxel(int x, int y, int z) const;

	void clearVoxels();

	void setFloorVoxels(int layers);

	void setWaveVoxels();

	void voxelize(const ev4sio_physx::PxGeometry& geom, const ev4sio_physx::PxTransform& pose, bool add = true);

	ev4sio_physx::PxVec3 voxelPos(int x, int y, int z) const;

	void pointCoords(const ev4sio_physx::PxVec3& p, int& x, int& y, int& z) const;

	void getVoxelRegion(const ev4sio_physx::PxBounds3& b, int& sx, int& sy, int& sz, int& ex, int& ey, int& ez) const;

	// ev4sio_physx::PxCustomGeometry::Callbacks overrides

	DECLARE_CUSTOM_GEOMETRY_TYPE
	virtual ev4sio_physx::PxBounds3 getLocalBounds(const ev4sio_physx::PxGeometry&) const;
	virtual bool generateContacts(const ev4sio_physx::PxGeometry& geom0, const ev4sio_physx::PxGeometry& geom1, const ev4sio_physx::PxTransform& pose0, const ev4sio_physx::PxTransform& pose1,
		const ev4sio_physx::PxReal contactDistance, const ev4sio_physx::PxReal meshContactMargin, const ev4sio_physx::PxReal toleranceLength,
		ev4sio_physx::PxContactBuffer& contactBuffer) const;
	virtual ev4sio_physx::PxU32 raycast(const ev4sio_physx::PxVec3& origin, const ev4sio_physx::PxVec3& unitDir, const ev4sio_physx::PxGeometry& geom, const ev4sio_physx::PxTransform& pose,
		ev4sio_physx::PxReal maxDist, ev4sio_physx::PxHitFlags hitFlags, ev4sio_physx::PxU32 maxHits, ev4sio_physx::PxGeomRaycastHit* rayHits, ev4sio_physx::PxU32 stride, ev4sio_physx::PxRaycastThreadContext*) const;
	virtual bool overlap(const ev4sio_physx::PxGeometry& geom0, const ev4sio_physx::PxTransform& pose0, const ev4sio_physx::PxGeometry& geom1, const ev4sio_physx::PxTransform& pose1, ev4sio_physx::PxOverlapThreadContext*) const;
	virtual bool sweep(const ev4sio_physx::PxVec3& unitDir, const ev4sio_physx::PxReal maxDist,
		const ev4sio_physx::PxGeometry& geom0, const ev4sio_physx::PxTransform& pose0, const ev4sio_physx::PxGeometry& geom1, const ev4sio_physx::PxTransform& pose1,
		ev4sio_physx::PxGeomSweepHit& sweepHit, ev4sio_physx::PxHitFlags hitFlags, const ev4sio_physx::PxReal inflation, ev4sio_physx::PxSweepThreadContext*) const;
	virtual void visualize(const ev4sio_physx::PxGeometry&, ev4sio_physx::PxRenderOutput&, const ev4sio_physx::PxTransform&, const ev4sio_physx::PxBounds3&) const;
	virtual void computeMassProperties(const ev4sio_physx::PxGeometry&, ev4sio_physx::PxMassProperties&) const {}
	virtual bool usePersistentContactManifold(const ev4sio_physx::PxGeometry&, ev4sio_physx::PxReal&) const { return true; }

private:

	int pacsX() const;
	int pacsY() const;
	int pacsZ() const;

	int m_dimensions[3];
	ev4sio_physx::PxVec3 m_voxelSize;
	ev4sio_physx::PxArray<ev4sio_physx::PxU64> m_packs;
};

#endif
