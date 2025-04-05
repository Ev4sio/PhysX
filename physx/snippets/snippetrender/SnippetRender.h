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

#ifndef PHYSX_SNIPPET_RENDER_H
#define PHYSX_SNIPPET_RENDER_H

#include "PxPhysicsAPI.h"
#include "foundation/PxPreprocessor.h"

#if PX_WINDOWS
	#include <windows.h>
	#pragma warning(disable: 4505)
	#include <GL/glew.h>
	#include <GL/freeglut.h>
#elif PX_LINUX_FAMILY
	#if PX_CLANG
		#pragma clang diagnostic push
		#pragma clang diagnostic ignored "-Wreserved-identifier"
	#endif
	#include <GL/glew.h>
	#include <GL/freeglut.h>	
	#if PX_CLANG
		#pragma clang diagnostic pop
	#endif
#elif PX_OSX
	#include <GL/glew.h>
	#include <GLUT/glut.h>	
#else
	#error platform not supported.
#endif

typedef	void	(*KeyboardCallback)	(unsigned char key, const ev4sio_physx::PxTransform& camera);
typedef	void	(*RenderCallback)	();
typedef	void	(*ExitCallback)		();

namespace Snippets
{
	class Camera;
	void setupDefault(const char* name, Camera* camera, KeyboardCallback kbcb, RenderCallback rdcb, ExitCallback excb);
	Camera* getCamera();
	ev4sio_physx::PxVec3 computeWorldRayF(float xs, float ys, const ev4sio_physx::PxVec3& camDir);
	PX_FORCE_INLINE ev4sio_physx::PxVec3 computeWorldRay(int xs, int ys, const ev4sio_physx::PxVec3& camDir)
	{
		return computeWorldRayF(float(xs), float(ys), camDir);
	}

	ev4sio_physx::PxU32	getScreenWidth();
	ev4sio_physx::PxU32	getScreenHeight();

	void	enableVSync(bool vsync);

	void	startRender(const Camera* camera, float nearClip = 1.0f, float farClip = 10000.0f, float fov=60.0f, bool setupLighting=true);
	void	finishRender();
	void	print(const char* text);

	class TriggerRender
	{
		public:
		virtual	bool	isTrigger(ev4sio_physx::PxShape*)	const	= 0;
	};

#if PX_SUPPORT_GPU_PHYSX
	class SharedGLBuffer
	{
	public:
		SharedGLBuffer();
		~SharedGLBuffer();
		void initialize(ev4sio_physx::PxCudaContextManager* contextManager);
		void allocate(ev4sio_physx::PxU32 sizeInBytes);
		void release();
		void* map();
		void unmap();

	private:
		ev4sio_physx::PxCudaContextManager* cudaContextManager;
		void* vbo_res;
		void* devicePointer;
	public:
		GLuint vbo; //Opengl vertex buffer object
		ev4sio_physx::PxU32 size;
	};
#endif

	void	initFPS();
	void	showFPS(int updateIntervalMS = 30, const char* info = NULL);

	void	renderDeformableVolume(ev4sio_physx::PxDeformableVolume* deformableVolume, const ev4sio_physx::PxVec4* deformedPositionsInvMass, bool shadows, const ev4sio_physx::PxVec3& color = ev4sio_physx::PxVec3(0.0f, 0.75f, 0.0f));
	void	renderActors(ev4sio_physx::PxRigidActor** actors, const ev4sio_physx::PxU32 numActors, bool shadows = false, const ev4sio_physx::PxVec3& color = ev4sio_physx::PxVec3(0.0f, 0.75f, 0.0f), TriggerRender* cb = NULL, bool changeColorForSleepingActors = true, bool wireframePass=true);
//	void	renderGeoms(const ev4sio_physx::PxU32 nbGeoms, const ev4sio_physx::PxGeometry* geoms, const ev4sio_physx::PxTransform* poses, bool shadows, const ev4sio_physx::PxVec3& color);
	void	renderGeoms(const ev4sio_physx::PxU32 nbGeoms, const ev4sio_physx::PxGeometryHolder* geoms, const ev4sio_physx::PxTransform* poses, bool shadows, const ev4sio_physx::PxVec3& color);
	void	renderMesh(ev4sio_physx::PxU32 nbVerts, const ev4sio_physx::PxVec3* verts, ev4sio_physx::PxU32 nbTris, const ev4sio_physx::PxU32* indices, const ev4sio_physx::PxVec3& color, const ev4sio_physx::PxVec3* normals = NULL, bool flipFaceOrientation = false);
	void	renderMesh(ev4sio_physx::PxU32 nbVerts, const ev4sio_physx::PxVec4* verts, ev4sio_physx::PxU32 nbTris, const ev4sio_physx::PxU32* indices, const ev4sio_physx::PxVec3& color, const ev4sio_physx::PxVec4* normals = NULL, bool flipFaceOrientation = false);
	void	renderMesh(ev4sio_physx::PxU32 nbVerts, const ev4sio_physx::PxVec4* verts, ev4sio_physx::PxU32 nbTris, const void* indices, bool hasSixteenBitIndices, const ev4sio_physx::PxVec3& color, const ev4sio_physx::PxVec4* normals = NULL, bool flipFaceOrientation = false, bool enableBackFaceCulling = true);

	void	DrawLine(const ev4sio_physx::PxVec3& p0, const ev4sio_physx::PxVec3& p1, const ev4sio_physx::PxVec3& color);
	void	DrawPoints(const ev4sio_physx::PxArray<ev4sio_physx::PxVec3>& pts, const ev4sio_physx::PxVec3& color, float scale);
	void	DrawPoints(const ev4sio_physx::PxArray<ev4sio_physx::PxVec3>& pts, const ev4sio_physx::PxArray<ev4sio_physx::PxVec3>& colors, float scale);
	void	DrawPoints(const ev4sio_physx::PxArray<ev4sio_physx::PxVec4>& pts, const ev4sio_physx::PxVec3& color, float scale);
	void	DrawPoints(const ev4sio_physx::PxArray<ev4sio_physx::PxVec4>& pts, const ev4sio_physx::PxArray<ev4sio_physx::PxVec3>& colors, float scale);
	void	DrawPoints(GLuint vbo, ev4sio_physx::PxU32 numPoints, const ev4sio_physx::PxVec3& color, float scale, ev4sio_physx::PxU32 coordinatesPerPoint = 3, ev4sio_physx::PxU32 stride = 4 * sizeof(float), size_t offset = 0);
	
	void	DrawLines(GLuint vbo, ev4sio_physx::PxU32 numPoints, const ev4sio_physx::PxVec3& color, float scale, ev4sio_physx::PxU32 coordinatesPerPoint = 3, ev4sio_physx::PxU32 stride = 4 * sizeof(float), size_t offset = 0);
	
	void	DrawIcosahedraPoints(const ev4sio_physx::PxArray<ev4sio_physx::PxVec3>& pts, const ev4sio_physx::PxVec3& color, float radius);
	void	DrawFrame(const ev4sio_physx::PxVec3& pt, float scale=1.0f);
	void	DrawBounds(const ev4sio_physx::PxBounds3& box);
	void	DrawBounds(const ev4sio_physx::PxBounds3& box, const ev4sio_physx::PxVec3& color);
	void	DrawMeshIndexedNoNormals(GLuint vbo, GLuint elementbuffer, GLuint numTriangles, const ev4sio_physx::PxVec3& color, ev4sio_physx::PxU32 stride = 4 * sizeof(float));
	void	DrawMeshIndexed(GLuint vbo, GLuint elementbuffer, GLuint numTriangles, const ev4sio_physx::PxVec3& color, ev4sio_physx::PxU32 stride = 6 * sizeof(float));

	GLuint	CreateTexture(ev4sio_physx::PxU32 width, ev4sio_physx::PxU32 height, const GLubyte* buffer, bool createMipmaps);
	void	UpdateTexture(GLuint texId, ev4sio_physx::PxU32 width, ev4sio_physx::PxU32 height, const GLubyte* buffer, bool createMipmaps);
	void	ReleaseTexture(GLuint texId);
	void	DrawRectangle(float x_start, float x_end, float y_start, float y_end, const ev4sio_physx::PxVec3& color_top, const ev4sio_physx::PxVec3& color_bottom, float alpha, ev4sio_physx::PxU32 screen_width, ev4sio_physx::PxU32 screen_height, bool draw_outline, bool texturing);
	void	DisplayTexture(GLuint texId, ev4sio_physx::PxU32 size, ev4sio_physx::PxU32 margin);
}

#endif //PHYSX_SNIPPET_RENDER_H
