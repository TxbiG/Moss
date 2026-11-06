//                        MIT License
//
//                  Copyright (c) 2026 Toby
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//     /$$      /$$        
//    | $$$    /$$$         
//    | $$$$  /$$$$  /$$$$$$   /$$$$$$$ /$$$$$$$
//    | $$ $$/$$ $$ /$$__  $$ /$$_____//$$_____/
//    | $$  $$$| $$| $$  \ $$|  $$$$$$|  $$$$$$
//    | $$\  $ | $$| $$  | $$ \____  $$\____  $$
//    | $$ \/  | $$|  $$$$$$/ /$$$$$$$//$$$$$$$/
//    |__/     |__/ \______/ |_______/|_______/
//

#pragma once

#include <Moss/Moss_stdinc.h>
#include <Moss/Moss_stdinc.h>

#include <Moss/Variants/Vector/Float2.h>
#include <Moss/Variants/Vector/Float3.h>
#include <Moss/Variants/Vector/Float4.h>
#include <Moss/Variants/Vector/Vec4.h>
#include <Moss/Variants/Vector/UVec4.h>
#include <Moss/Variants/Vector/Vec2.h>
#include <Moss/Variants/Vector/Vec3.h>
#include <Moss/Variants/Vector/Double2.h>
#include <Moss/Variants/Vector/Double3.h>
#include <Moss/Variants/Vector/Double4.h>
#include <Moss/Variants/Vector/Int2.h>
#include <Moss/Variants/Vector/Int3.h>
#include <Moss/Variants/Vector/Int4.h>
#include <Moss/Variants/Vector/BVec16.h>
#include <Moss/Variants/Vector/TVec.h>

#include <Moss/Variants/Quat.h>
#include <Moss/Variants/Matrix/Mat22.h>
#include <Moss/Variants/Matrix/Mat33.h>
#include <Moss/Variants/Matrix/Mat44.h>
#include <Moss/Variants/Matrix/TMatrix.h>
#include <Moss/Variants/Math/Real.h>
#include <Moss/Variants/Math/EigenValueSymmetric.h>
#include <Moss/Variants/Math/GaussianElimination.h>
#include <Moss/Variants/Math/HalfFloat.h>
#include <Moss/Variants/Math/Trigonometry.h>

#include <Moss/Variants/AABB2.h>
#include <Moss/Variants/AABB3.h>
#include <Moss/Variants/Color.h>
#include <Moss/Variants/OBB2.h>
#include <Moss/Variants/OBB3.h>
#include <Moss/Variants/Plane.h>
#include <Moss/Variants/Rect.h>
#include <Moss/Variants/TArray.h>
#include <Moss/Variants/TMap.h>
#include <Moss/Variants/TSet.h>
#include <Moss/Variants/TStaticArray.h>

#include <Moss/Geometry/Triangle.h>
#include <Moss/Resources/Json.h>
#include <Moss/Resources/Timer.h>
#include <Moss/TriangleSplitter/TriangleSplitter.h>

#include <Moss/Core/BinaryHeap.h>
#include <Moss/Core/ByteBuffer.h>
#include <Moss/Core/FPControlWord.h>
#include <Moss/Core/FPException.h>
#include <Moss/Core/FPFlushDenormals.h>
#include <Moss/Core/Factory.h>
#include <Moss/Core/FixedSizeFreeList.h>
#include <Moss/Core/FreeBlockAlloc.h>
#include <Moss/Core/HashCombine.h>
#include <Moss/Core/HashTable.h>
#include <Moss/Core/InsertionSort.h>
#include <Moss/Core/JobSystem.h>
#include <Moss/Core/JobSystemSingleThreaded.h>
#include <Moss/Core/JobSystemThreadPool.h>
#include <Moss/Core/JobSystemWithBarrier.h>
#include <Moss/Core/LinearCurve.h>
#include <Moss/Core/LockFreeHashMap.h>
#include <Moss/Core/Mutex.h>
#include <Moss/Core/MutexArray.h>
#include <Moss/Core/NonCopyable.h>
#include <Moss/Core/Profiler.h>
#include <Moss/Core/QuickSort.h>
#include <Moss/Core/RTTI.h>
#include <Moss/Core/Reference.h>
#include <Moss/Core/Result.h>
#include <Moss/Core/STLAlignedAllocator.h>
#include <Moss/Core/STLAllocator.h>
#include <Moss/Core/STLLocalAllocator.h>
#include <Moss/Core/STLTempAllocator.h>
#include <Moss/Core/ScopeExit.h>
#include <Moss/Core/Semaphore.h>
#include <Moss/Core/StreamIn.h>
#include <Moss/Core/StreamOut.h>
#include <Moss/Core/StreamUtils.h>
#include <Moss/Core/StreamWrapper.h>
#include <Moss/Core/StridedPtr.h>
#include <Moss/Core/StringTools.h>
#include <Moss/Core/TempAllocator.h>
#include <Moss/Core/TickCounter.h>

#include <Moss/Moss_Platform.h>
#include <Moss/Moss_GPU.h>
#include <Moss/Moss_Renderer.h>
#include <Moss/Moss_GUI.h>
#include <Moss/Moss_Audio.h>
//#include <Moss/Moss_Physics.h>
#include <Moss/Moss_Network.h>
#include <Moss/Moss_XR.h>
//#include <Moss/Moss_Navigation.h>