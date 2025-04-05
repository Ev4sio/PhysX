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

#include "PxPhysXConfig.h"

#if PX_SUPPORT_GPU_PHYSX

#include "gpu/PxGpu.h"

#ifndef PX_PHYSX_GPU_STATIC

namespace ev4sio_physx
{
	//forward declare stuff from PxPhysXGpuModuleLoader.cpp
	void PxLoadPhysxGPUModule(const char* appGUID);

	typedef ev4sio_physx::PxCudaContextManager* (PxCreateCudaContextManager_FUNC)(ev4sio_physx::PxFoundation& foundation, const ev4sio_physx::PxCudaContextManagerDesc& desc, ev4sio_physx::PxProfilerCallback* profilerCallback, bool launchSynchronous);
	typedef int (PxGetSuggestedCudaDeviceOrdinal_FUNC)(ev4sio_physx::PxErrorCallback& errc);
	typedef void (PxSetPhysXGpuProfilerCallback_FUNC)(ev4sio_physx::PxProfilerCallback* cbk);
	typedef void (PxSetPhysXGpuFoundationInstance_FUNC)(ev4sio_physx::PxFoundation& foundation);
	typedef void (PxCudaRegisterFunction_FUNC)(int, const char*);
	typedef void** (PxCudaRegisterFatBinary_FUNC)(void*);
	typedef ev4sio_physx::PxKernelIndex* (PxGetCudaFunctionTable_FUNC)();
	typedef PxU32 (PxGetCudaFunctionTableSize_FUNC)();
	typedef void** PxGetCudaModuleTable_FUNC();
	typedef PxPhysicsGpu* PxCreatePhysicsGpu_FUNC();

	extern PxCreateCudaContextManager_FUNC*  g_PxCreateCudaContextManager_Func;
	extern PxGetSuggestedCudaDeviceOrdinal_FUNC* g_PxGetSuggestedCudaDeviceOrdinal_Func;
	extern PxSetPhysXGpuProfilerCallback_FUNC* g_PxSetPhysXGpuProfilerCallback_Func;
	extern PxSetPhysXGpuFoundationInstance_FUNC* g_PxSetPhysXGpuFoundationInstance_Func;
	extern PxCudaRegisterFunction_FUNC* g_PxCudaRegisterFunction_Func;
	extern PxCudaRegisterFatBinary_FUNC* g_PxCudaRegisterFatBinary_Func;
	extern PxGetCudaFunctionTable_FUNC* g_PxGetCudaFunctionTable_Func;
	extern PxGetCudaFunctionTableSize_FUNC* g_PxGetCudaFunctionTableSize_Func;
	extern PxGetCudaFunctionTableSize_FUNC* g_PxGetCudaModuleTableSize_Func;
	extern PxGetCudaModuleTable_FUNC* g_PxGetCudaModuleTable_Func;
	extern PxCreatePhysicsGpu_FUNC* g_PxCreatePhysicsGpu_Func;

} // end of physx namespace



ev4sio_physx::PxCudaContextManager* ev4sio_PxCreateCudaContextManager(ev4sio_physx::PxFoundation& foundation, const ev4sio_physx::PxCudaContextManagerDesc& desc, ev4sio_physx::PxProfilerCallback* profilerCallback, bool launchSynchronous)
{
	ev4sio_physx::PxLoadPhysxGPUModule(desc.appGUID);

	if (ev4sio_physx::g_PxCreateCudaContextManager_Func)
		return ev4sio_physx::g_PxCreateCudaContextManager_Func(foundation, desc, profilerCallback, launchSynchronous);
	else
		return NULL;
}

int ev4sio_PxGetSuggestedCudaDeviceOrdinal(ev4sio_physx::PxErrorCallback& errc)
{
	ev4sio_physx::PxLoadPhysxGPUModule(NULL);

	if (ev4sio_physx::g_PxGetSuggestedCudaDeviceOrdinal_Func)
		return ev4sio_physx::g_PxGetSuggestedCudaDeviceOrdinal_Func(errc);
	else
		return -1;
}

void ev4sio_PxSetPhysXGpuProfilerCallback(ev4sio_physx::PxProfilerCallback* profilerCallback)
{
	ev4sio_physx::PxLoadPhysxGPUModule(NULL);

	if (ev4sio_physx::g_PxSetPhysXGpuProfilerCallback_Func)
		ev4sio_physx::g_PxSetPhysXGpuProfilerCallback_Func(profilerCallback);
}

void ev4sio_PxSetPhysXGpuFoundationInstance(ev4sio_physx::PxFoundation& foundation)
{
	ev4sio_physx::PxLoadPhysxGPUModule(NULL);

	if (ev4sio_physx::g_PxSetPhysXGpuFoundationInstance_Func)
		ev4sio_physx::g_PxSetPhysXGpuFoundationInstance_Func(foundation);
}

void ev4sio_PxCudaRegisterFunction(int moduleIndex, const char* functionName)
{
	ev4sio_physx::PxLoadPhysxGPUModule(NULL);

	if (ev4sio_physx::g_PxCudaRegisterFunction_Func)
		ev4sio_physx::g_PxCudaRegisterFunction_Func(moduleIndex, functionName);
}

void** ev4sio_PxCudaRegisterFatBinary(void* fatBin)
{
	ev4sio_physx::PxLoadPhysxGPUModule(NULL);

	if (ev4sio_physx::g_PxCudaRegisterFatBinary_Func)
		return ev4sio_physx::g_PxCudaRegisterFatBinary_Func(fatBin);

	return NULL;
}


ev4sio_physx::PxKernelIndex* ev4sio_PxGetCudaFunctionTable()
{
	ev4sio_physx::PxLoadPhysxGPUModule(NULL);

	if(ev4sio_physx::g_PxGetCudaFunctionTable_Func)
		return ev4sio_physx::g_PxGetCudaFunctionTable_Func();

	return NULL;
}

ev4sio_physx::PxU32 ev4sio_PxGetCudaFunctionTableSize()
{
	ev4sio_physx::PxLoadPhysxGPUModule(NULL);

	if(ev4sio_physx::g_PxGetCudaFunctionTableSize_Func)
		return ev4sio_physx::g_PxGetCudaFunctionTableSize_Func();

	return 0;
}

void** ev4sio_PxGetCudaModuleTable() 
{
	ev4sio_physx::PxLoadPhysxGPUModule(NULL);

	if(ev4sio_physx::g_PxGetCudaModuleTable_Func)
		return ev4sio_physx::g_PxGetCudaModuleTable_Func();

	return NULL;
}


ev4sio_physx::PxU32 ev4sio_PxGetCudaModuleTableSize()
{
	ev4sio_physx::PxLoadPhysxGPUModule(NULL);

	if(ev4sio_physx::g_PxGetCudaModuleTableSize_Func)
		return ev4sio_physx::g_PxGetCudaModuleTableSize_Func();

	return 0;
}


ev4sio_physx::PxPhysicsGpu* ev4sio_PxGetPhysicsGpu()
{
	ev4sio_physx::PxLoadPhysxGPUModule(NULL);

	if (ev4sio_physx::g_PxCreatePhysicsGpu_Func)
		return ev4sio_physx::g_PxCreatePhysicsGpu_Func();

	return NULL;
}

#endif // PX_PHYSX_GPU_STATIC

#endif // PX_SUPPORT_GPU_PHYSX

