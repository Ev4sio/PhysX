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

#ifndef __HEIGHTFIELD_CUH__
#define __HEIGHTFIELD_CUH__

#include "geometry/PxHeightFieldSample.h"
#include "geometry/PxMeshScale.h"
#include "triangle.cuh"
#include "assert.h"

__device__	static inline ev4sio_physx::PxU32	getMinRow(ev4sio_physx::PxReal x, const ev4sio_physx::PxU32 rows)
{
	using namespace ev4sio_physx;
	return (PxU32)PxClamp(PxI32(PxFloor(x)), PxI32(0), PxI32(rows - 2));
}
__device__	static inline ev4sio_physx::PxU32	getMaxRow(ev4sio_physx::PxReal x, const ev4sio_physx::PxU32 rows)
{
	using namespace ev4sio_physx;
	return (PxU32)PxClamp(PxI32(PxCeil(x)), PxI32(0), PxI32(rows - 1));
}

__device__	static inline ev4sio_physx::PxU32	getMinColumn(ev4sio_physx::PxReal z, const ev4sio_physx::PxU32 columns)
{
	using namespace ev4sio_physx;
	return (PxU32)PxClamp(PxI32(PxFloor(z)), PxI32(0), PxI32(columns - 2));
}

__device__	static inline  ev4sio_physx::PxU32	getMaxColumn(ev4sio_physx::PxReal z, const  ev4sio_physx::PxU32 columns)
{
	using namespace ev4sio_physx;
	return (PxU32)PxClamp(PxI32(PxCeil(z)), PxI32(0), PxI32(columns - 1));
}

__device__	static inline bool isValidVertex(ev4sio_physx::PxU32 vertexIndex, const  ev4sio_physx::PxU32 rows, const  ev4sio_physx::PxU32 columns)
{
	using namespace ev4sio_physx;
	return vertexIndex < rows*columns;
}

__device__	static inline  bool	isFirstTriangle(ev4sio_physx::PxU32 triangleIndex)
{
	return ((triangleIndex & 0x1) == 0);
}

__device__	static inline const ev4sio_physx::PxHeightFieldSample& getSample(ev4sio_physx::PxU32 vertexIndex, const ev4sio_physx::PxHeightFieldSample* samples)
{

	return samples[vertexIndex];
}

__device__	static inline  ev4sio_physx::PxReal getHeight(ev4sio_physx::PxU32 vertexIndex, const ev4sio_physx::PxHeightFieldSample* samples)
{
	using namespace ev4sio_physx;
	return PxReal(getSample(vertexIndex, samples).height);
}

__device__	static inline  ev4sio_physx::PxU16	getMaterialIndex0(ev4sio_physx::PxU32 vertexIndex, const ev4sio_physx::PxHeightFieldSample* samples)
{
	using namespace ev4sio_physx;
	return getSample(vertexIndex, samples).materialIndex0;
}

__device__	static inline  ev4sio_physx::PxU16	getMaterialIndex1(ev4sio_physx::PxU32 vertexIndex, const ev4sio_physx::PxHeightFieldSample* samples)
{
	using namespace ev4sio_physx;
	return getSample(vertexIndex, samples).materialIndex1;
}

__device__ static inline bool isZerothVertexShared(ev4sio_physx::PxU32 vertexIndex, const ev4sio_physx::PxHeightFieldSample* samples)
{
	return getSample(vertexIndex, samples).tessFlag() != 0;
}


__device__ static inline ev4sio_physx::PxVec3 getVertex(ev4sio_physx::PxU32 vertexIndex, const ev4sio_physx::PxU32 columns, const ev4sio_physx::PxHeightFieldSample* samples)
{
	using namespace ev4sio_physx;
	const PxU32 row = vertexIndex / columns;
	const PxU32 column = vertexIndex % columns;
	return PxVec3(PxReal(row), getHeight(vertexIndex, samples), PxReal(column));
}

__device__ static inline ev4sio_physx::PxVec3 hf2shapep(const ev4sio_physx::PxVec3& v, const ev4sio_physx::PxReal rowScale, const ev4sio_physx::PxReal heightScale, const ev4sio_physx::PxReal columnScale) 
{
	using namespace ev4sio_physx;
	return PxVec3(v.x *rowScale, v.y * heightScale, v.z * columnScale);
}

__device__ static inline void getTriangleVertexIndices(ev4sio_physx::PxU32 triangleIndex, ev4sio_physx::PxU32& vertexIndex0, ev4sio_physx::PxU32& vertexIndex1, ev4sio_physx::PxU32& vertexIndex2, const ev4sio_physx::PxU32 columns, const ev4sio_physx::PxHeightFieldSample* samples)
{
	using namespace ev4sio_physx;
	const PxU32 cell = triangleIndex >> 1;
	if (isZerothVertexShared(cell, samples))
	{
		//      <---- COL  
		//      0----2  1 R
		//      | 1 /  /| O
		//      |  /  / | W
		//      | /  /  | |
		//      |/  / 0 | |
		//      1  2----0 V
		//      
		if (isFirstTriangle(triangleIndex))
		{
			vertexIndex0 = cell + columns;
			vertexIndex1 = cell;
			vertexIndex2 = cell + columns + 1;
		}
		else
		{
			vertexIndex0 = cell + 1;
			vertexIndex1 = cell + columns + 1;
			vertexIndex2 = cell;
		}
	}
	else
	{
		//      <---- COL  
		//      2  1----0 R
		//      |\  \ 0 | O
		//      | \  \  | W
		//      |  \  \ | |
		//      | 1 \  \| |
		//      0----1  2 V
		//                   
		if (isFirstTriangle(triangleIndex))
		{
			vertexIndex0 = cell;
			vertexIndex1 = cell + 1;
			vertexIndex2 = cell + columns;
		}
		else
		{
			vertexIndex0 = cell + columns + 1;
			vertexIndex1 = cell + columns;
			vertexIndex2 = cell + 1;
		}
	}
}

__device__ static inline void getTriangleAdjacencyIndices(ev4sio_physx::PxU32 triangleIndex, ev4sio_physx::PxU32& adjacencyIndex0, ev4sio_physx::PxU32& adjacencyIndex1, ev4sio_physx::PxU32& adjacencyIndex2, const ev4sio_physx::PxU32 rows, const ev4sio_physx::PxU32 columns,
	const ev4sio_physx::PxHeightFieldSample* samples)
{
	using namespace ev4sio_physx;
	const PxU32 cell = triangleIndex >> 1;
	if (isZerothVertexShared(cell, samples))
	{
		//      <---- COL  
		//      0----2  1 R
		//      | 1 /  /| O
		//      |  /  / | W
		//      | /  /  | |
		//      |/  / 0 | |
		//      1  2----0 V
		//      
		if (isFirstTriangle(triangleIndex))
		{
			adjacencyIndex0 = BOUNDARY;
			adjacencyIndex1 = triangleIndex + 1;
			adjacencyIndex2 = BOUNDARY;

			if ((cell % (columns) != 0))
			{
				adjacencyIndex0 = triangleIndex - 1;
			}

			if ((cell / columns != rows - 2))
			{
				adjacencyIndex2 = ((cell + columns) * 2) + 1;
			}
		}
		else
		{
			adjacencyIndex0 = BOUNDARY;
			adjacencyIndex1 = triangleIndex - 1;
			adjacencyIndex2 = BOUNDARY;

			if (cell % (columns) < (columns - 2))
			{
				adjacencyIndex0 = triangleIndex + 1;
			}

			if (cell >= columns - 1)
			{
				adjacencyIndex2 = (cell - columns) * 2;
			}
		}
	}
	else
	{
		//      <---- COL  
		//      2  1----0 R
		//      |\  \ 0 | O
		//      | \  \  | W
		//      |  \  \ | |
		//      | 1 \  \| |
		//      0----1  2 V
		//                   
		if (isFirstTriangle(triangleIndex))
		{
			adjacencyIndex0 = BOUNDARY;
			adjacencyIndex1 = triangleIndex + 1;
			adjacencyIndex2 = BOUNDARY;

			if (cell >= columns - 1)
			{
				adjacencyIndex0 = ((cell - (columns)) * 2) + 1;
			}

			if ((cell % (columns) != 0))
			{
				adjacencyIndex2 = triangleIndex - 1;
			}
		}
		else
		{
			adjacencyIndex0 = BOUNDARY;
			adjacencyIndex1 = triangleIndex - 1;
			adjacencyIndex2 = BOUNDARY;

			if ((cell / columns != rows - 2))
			{
				adjacencyIndex0 = (cell + (columns)) * 2;
			}

			if (cell % (columns) < (columns - 2))
			{
				adjacencyIndex2 = triangleIndex + 1;
			}
		}
	}
}



__device__ static void getTriangle(ev4sio_physx::PxVec3& triLocV0, ev4sio_physx::PxVec3& triLocV1, ev4sio_physx::PxVec3& triLocV2, uint4* adjacencyIndices, const ev4sio_physx::PxU32 triangleIndex, const ev4sio_physx::PxMeshScale& scale, const ev4sio_physx::PxU32 rows, const ev4sio_physx::PxU32 columns, const ev4sio_physx::PxHeightFieldSample* samples)
{
	using namespace ev4sio_physx;

	const PxReal rowScale = scale.scale.x;
	const PxReal heightScale = scale.scale.y;
	const PxReal columnScale = scale.scale.z;
	PxVec3 handedness(1.0f);	// Vector to invert normal coordinates according to the heightfield scales
	bool wrongHanded = false;
	if (columnScale < 0.f)
	{
		wrongHanded = !wrongHanded;
		handedness.z = -1.0f;
	}
	if (rowScale < 0.f)
	{
		wrongHanded = !wrongHanded;
		handedness.x = -1.0f;
	}

	//PxU32 tVertexIndices[3];
	//getTriangleVertexIndices(triangleIndex, tVertexIndices[0], tVertexIndices[1 + wrongHanded], tVertexIndices[2 - wrongHanded], columns, samples);

	PxU32 tVert0, tVert1, tVert2;
	getTriangleVertexIndices(triangleIndex, tVert0, tVert1, tVert2, columns, samples);

	if (adjacencyIndices)
	{
		/*PxU32 tAdjInd0 = wrongHanded ? adjacencyIndices->z : adjacencyIndices->x;
		PxU32 tAdjInd1 = adjacencyIndices->y;
		PxU32 tAdjInd2 = wrongHanded ? adjacencyIndices->x : adjacencyIndices->z;*/

		PxU32 tAdjInd0, tAdjInd1, tAdjInd2;
		getTriangleAdjacencyIndices(triangleIndex, tAdjInd0, tAdjInd1, tAdjInd2, rows, columns, samples);

		adjacencyIndices->x = wrongHanded ? tAdjInd2 : tAdjInd0;
		adjacencyIndices->y = tAdjInd1;
		adjacencyIndices->z = wrongHanded ? tAdjInd0 : tAdjInd2;
	}


	const PxVec3 vertex0 = getVertex(tVert0, columns, samples);
	const PxVec3 vertex1 = getVertex(wrongHanded ? tVert2 : tVert1, columns, samples);
	const PxVec3 vertex2 = getVertex(wrongHanded ? tVert1 : tVert2, columns, samples);

	triLocV0 = hf2shapep(vertex0, rowScale, heightScale, columnScale);
	triLocV1 = hf2shapep(vertex1, rowScale, heightScale, columnScale);
	triLocV2 = hf2shapep(vertex2, rowScale, heightScale, columnScale);
}

__device__ static bool testLocalPoint(const ev4sio_physx::PxVec3& pnt, const ev4sio_physx::PxU32 rows, const ev4sio_physx::PxU32 columns, const ev4sio_physx::PxHeightFieldSample* samples, float& height)
{
	using namespace ev4sio_physx;

    if (pnt.x < 0 || pnt.x > rows - 1 || pnt.z < 0 || pnt.z > columns - 1)
        return false;

    PxU32 row = getMinRow(pnt.x, rows);
    PxU32 column = getMinColumn(pnt.z, columns);
    PxU32 vertex = row * columns + column;
    PxReal h0 = getHeight(vertex, samples);
    PxReal h1 = getHeight(vertex + 1, samples);
    PxReal h2 = getHeight(vertex + columns, samples);
    PxReal h3 = getHeight(vertex + columns + 1, samples);
    PxReal s = pnt.x - PxFloor(pnt.x);
    PxReal t = pnt.z - PxFloor(pnt.z);
    PxReal h = h0 * (1 - s) * (1 - t) + h1 * s * (1 - t) + h2 * (1 - s) * t + h3 * s * t; // @@@ That's wrong. Should find point on triangle
	height = h;
    return h > pnt.y;
}


__device__ static inline ev4sio_physx::PxU32 heightfieldComputePairs(
	const ev4sio_physx::PxU32 minColumn,
	const ev4sio_physx::PxU32 maxColumn,
	const ev4sio_physx::PxU32 minRow,
	const ev4sio_physx::PxU32 maxRow,
	const ev4sio_physx::PxU32 nbRows,
	const ev4sio_physx::PxU32 nbCols,
	ev4sio_physx::PxHeightFieldSample* samples,
	const ev4sio_physx::PxU32 miny,
	const ev4sio_physx::PxU32 maxy
)
{
	using namespace ev4sio_physx;

	PxU32 nbPairs = 0;

	const PxU32 columnSpan = maxColumn - minColumn;

	//we have two materials corresponding to one vertexIndex, so each thread will deal with one of the materials
	const PxU32 totalNumProcessed = (maxRow - minRow) * columnSpan * 2;

	for (PxU32 i = 0; i < totalNumProcessed; ++i)
	{

		const PxU32 index = i / 2;
		const PxU32 vertexIndex = (minRow + index / columnSpan) * nbCols + (minColumn + index % columnSpan);
		assert(isValidVertex(vertexIndex, nbRows, nbCols));
		PxReal h0 = getHeight(vertexIndex, samples);
		PxReal h1 = getHeight(vertexIndex + 1, samples);
		PxReal h2 = getHeight(vertexIndex + nbCols, samples);
		PxReal h3 = getHeight(vertexIndex + nbCols + 1, samples);
		const bool con0 = maxy < h0 && maxy < h1 && maxy < h2 && maxy < h3;
		const bool con1 = miny > h0 && miny > h1 && miny > h2 && miny > h3;

		if (!(con0 || con1))
		{
			const PxHeightFieldSample& sample = getSample(vertexIndex, samples);

			const bool isMaterial1 = (i & 1) ? 1 : 0;
			PxU32 material = isMaterial1 ? sample.materialIndex1 : sample.materialIndex0;
			if (material != PxHeightFieldMaterial::eHOLE)
				nbPairs++;
		}
	}//end of totalNumProcessed

	return nbPairs;
}

__device__ static inline void heightfieldOutputPairs(
	const ev4sio_physx::PxU32 minColumn,
	const ev4sio_physx::PxU32 maxColumn,
	const ev4sio_physx::PxU32 minRow,
	const ev4sio_physx::PxU32 maxRow,
	const ev4sio_physx::PxU32 nbRows,
	const ev4sio_physx::PxU32 nbCols,
	ev4sio_physx::PxHeightFieldSample* samples,
	const ev4sio_physx::PxU32 miny,
	const ev4sio_physx::PxU32 maxy,

	const ev4sio_physx::PxU32 cmInd,
	const ev4sio_physx::PxU32 elementInd, //either tetrahedron id or triangle id
	const ev4sio_physx::PxU32 startOffset,
	const ev4sio_physx::PxU32 size,

	uint4* stackPtr						//output

)
{
	using namespace ev4sio_physx;

	PxU32 writeIndex = startOffset;
	const PxU32 columnSpan = maxColumn - minColumn;

	//we have two materials corresponding to one vertexIndex, so each thread will deal with one of the materials
	const PxU32 totalNumProcessed = (maxRow - minRow) * columnSpan * 2;

	for (PxU32 i = 0; i < totalNumProcessed && writeIndex < size; ++i)
	{

		PxU32 triangleIdx = 0xFFffFFff;

		const PxU32 index = i / 2;
		const PxU32 vertexIndex = (minRow + index / columnSpan) * nbCols + (minColumn + index % columnSpan);
		assert(isValidVertex(vertexIndex, nbRows, nbCols));
		PxReal h0 = getHeight(vertexIndex, samples);
		PxReal h1 = getHeight(vertexIndex + 1, samples);
		PxReal h2 = getHeight(vertexIndex + nbCols, samples);
		PxReal h3 = getHeight(vertexIndex + nbCols + 1, samples);
		const bool con0 = maxy < h0 && maxy < h1 && maxy < h2 && maxy < h3;
		const bool con1 = miny > h0 && miny > h1 && miny > h2 && miny > h3;

		if (!(con0 || con1))
		{
			const PxHeightFieldSample& sample = getSample(vertexIndex, samples);

			const bool isMaterial1 = (i & 1) ? 1 : 0;
			PxU32 material = isMaterial1 ? sample.materialIndex1 : sample.materialIndex0;
			if (material != PxHeightFieldMaterial::eHOLE)
			{
				triangleIdx = isMaterial1 ? ((vertexIndex << 1) + 1) : (vertexIndex << 1);

				stackPtr[writeIndex] = make_uint4(cmInd, elementInd, triangleIdx, 0);

				writeIndex++;
			}
		}
	}//end of totalNumProcessed
}

#endif