# Moss Framework API Reference

> Declaration index generated from the Moss header archive supplied for this documentation update. The source files in the archive map to `include/Moss/` in the repository.

## Umbrella headers

| Header | Responsibility |
|---|---|
| [`Moss.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Moss.h) | Framework umbrella; includes standard types and main subsystem headers. |
| [`Moss_stdinc.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Moss_stdinc.h) | Platform/compiler abstraction and common macros/types. |
| [`Moss_Platform.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Moss_Platform.h) | Platform, window, input, system, storage and filesystem APIs. |
| [`Moss_GPU.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Moss_GPU.h) | Backend-neutral GPU API. |
| [`Moss_Renderer.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Moss_Renderer.h) | High-level renderer and render resources. |
| [`Moss_Audio.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Moss_Audio.h) | Audio sources, streams, effects, listeners and capture. |
| [`Moss_GUI.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Moss_GUI.h) | GUI context, windows and widgets. |
| [`Moss_Physics.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Moss_Physics.h) | Physics-facing declarations and utilities. |
| [`Moss_Navigation.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Moss_Navigation.h) | Navigation mesh and pathfinding API. |
| [`Moss_Network.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Moss_Network.h) | ENet and transport/network APIs. |
| [`Moss_XR.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Moss_XR.h) | XR subsystem API. |

## Top-level API map

| Header | Main API families | Export-marked declarations |
|---|---|---:|
| [`Moss_stdinc.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Moss_stdinc.h) | Compiler/platform macros, type aliases, logging, assertions and common support. | 34 |
| [`Moss_Platform.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Moss_Platform.h) | Window and monitor control, input, gamepads, touch, haptics, cameras, paths, storage and dialogs. | 181 |
| [`Moss_GPU.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Moss_GPU.h) | GPU device, buffers, textures, samplers, shaders, pipelines, command buffers, swapchain and synchronization. | 241 |
| [`Moss_Renderer.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Moss_Renderer.h) | Renderer lifecycle, materials, resource sets, framebuffers, mesh/model/font/sprite and debug draw. | 195 |
| [`Moss_Audio.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Moss_Audio.h) | Audio subsystem, source loading, effects, channels, streams, listeners, ray acoustics and microphones. | 113 |
| [`Moss_GUI.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Moss_GUI.h) | GUI context, window/container lifecycle, widgets, sliders, drag controls and text/scalar input. | 73 |
| [`Moss_Physics.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Moss_Physics.h) | Layer/filter setup, collision dispatch, contact manifold access, skeleton and triangle helpers. | 39 |
| [`Moss_Navigation.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Moss_Navigation.h) | Recast/Detour mesh/query/corridor/crowd/tile-cache classes; class-oriented declarations. | 0 |
| [`Moss_Network.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Moss_Network.h) | ENet socket/host/peer/packet APIs, transport, HTTP/TLS, WebSocket and WebRTC declarations. | 146 |
| [`Moss_XR.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Moss_XR.h) | XR initialization, session/frame/views, actions, haptics, tracking and optional rendering features. | 41 |

## Complete header inventory

Each link opens the corresponding source file in `include/Moss/`. The rightmost count shows declarations beginning with `MOSS_API` in that header; zero does not mean the header has no public API (it may expose class members, templates, inline methods, typedefs, or preprocessor declarations).

### Core (40 headers)

Foundational C++ utilities.

| Header | Export-marked declarations |
|---|---:|
| [`Core/BinaryHeap.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/BinaryHeap.h) | 0 |
| [`Core/ByteBuffer.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/ByteBuffer.h) | 0 |
| [`Core/FPControlWord.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/FPControlWord.h) | 0 |
| [`Core/FPException.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/FPException.h) | 0 |
| [`Core/FPFlushDenormals.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/FPFlushDenormals.h) | 0 |
| [`Core/Factory.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/Factory.h) | 0 |
| [`Core/FixedSizeFreeList.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/FixedSizeFreeList.h) | 0 |
| [`Core/FreeBlockAlloc.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/FreeBlockAlloc.h) | 0 |
| [`Core/HashCombine.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/HashCombine.h) | 0 |
| [`Core/HashTable.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/HashTable.h) | 0 |
| [`Core/InsertionSort.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/InsertionSort.h) | 0 |
| [`Core/JobSystem.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/JobSystem.h) | 0 |
| [`Core/JobSystemSingleThreaded.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/JobSystemSingleThreaded.h) | 0 |
| [`Core/JobSystemThreadPool.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/JobSystemThreadPool.h) | 0 |
| [`Core/JobSystemWithBarrier.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/JobSystemWithBarrier.h) | 0 |
| [`Core/LinearCurve.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/LinearCurve.h) | 0 |
| [`Core/LockFreeHashMap.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/LockFreeHashMap.h) | 0 |
| [`Core/Memory.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/Memory.h) | 11 |
| [`Core/Mutex.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/Mutex.h) | 0 |
| [`Core/MutexArray.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/MutexArray.h) | 0 |
| [`Core/NonCopyable.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/NonCopyable.h) | 0 |
| [`Core/Profiler.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/Profiler.h) | 4 |
| [`Core/QuickSort.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/QuickSort.h) | 0 |
| [`Core/RTTI.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/RTTI.h) | 0 |
| [`Core/Reference.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/Reference.h) | 0 |
| [`Core/Result.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/Result.h) | 0 |
| [`Core/STLAlignedAllocator.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/STLAlignedAllocator.h) | 0 |
| [`Core/STLAllocator.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/STLAllocator.h) | 0 |
| [`Core/STLLocalAllocator.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/STLLocalAllocator.h) | 0 |
| [`Core/STLTempAllocator.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/STLTempAllocator.h) | 0 |
| [`Core/ScopeExit.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/ScopeExit.h) | 0 |
| [`Core/Semaphore.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/Semaphore.h) | 0 |
| [`Core/StreamIn.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/StreamIn.h) | 0 |
| [`Core/StreamOut.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/StreamOut.h) | 0 |
| [`Core/StreamUtils.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/StreamUtils.h) | 0 |
| [`Core/StreamWrapper.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/StreamWrapper.h) | 0 |
| [`Core/StridedPtr.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/StridedPtr.h) | 0 |
| [`Core/StringTools.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/StringTools.h) | 6 |
| [`Core/TempAllocator.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/TempAllocator.h) | 0 |
| [`Core/TickCounter.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Core/TickCounter.h) | 0 |

### Geometry (9 headers)

Geometry algorithms and primitives.

| Header | Export-marked declarations |
|---|---:|
| [`Geometry/ClipPoly.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Geometry/ClipPoly.h) | 0 |
| [`Geometry/ClosestPoint.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Geometry/ClosestPoint.h) | 0 |
| [`Geometry/ConvexHullBuilder.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Geometry/ConvexHullBuilder.h) | 0 |
| [`Geometry/ConvexSupport.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Geometry/ConvexSupport.h) | 0 |
| [`Geometry/EPAConvexHullBuilder.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Geometry/EPAConvexHullBuilder.h) | 0 |
| [`Geometry/EPAPenetrationDepth.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Geometry/EPAPenetrationDepth.h) | 0 |
| [`Geometry/GJKClosestPoint.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Geometry/GJKClosestPoint.h) | 0 |
| [`Geometry/RayCylinder.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Geometry/RayCylinder.h) | 0 |
| [`Geometry/Triangle.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Geometry/Triangle.h) | 0 |

### Physics (69 headers)

Simulation, body, shape, collision, constraint and query implementation headers.

| Header | Export-marked declarations |
|---|---:|
| [`Physics/AABBTree/AABBTreeBuilder.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/AABBTree/AABBTreeBuilder.h) | 0 |
| [`Physics/AABBTree/AABBTreeToBuffer.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/AABBTree/AABBTreeToBuffer.h) | 0 |
| [`Physics/AABBTree/NodeCodec/NodeCodecQuadTreeHalfFloat.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/AABBTree/NodeCodec/NodeCodecQuadTreeHalfFloat.h) | 0 |
| [`Physics/AABBTree/NodeCodec/NodeCodecQuadTreeHalfFloat2D.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/AABBTree/NodeCodec/NodeCodecQuadTreeHalfFloat2D.h) | 0 |
| [`Physics/AABBTree/TriangleCodec/TriangleCodecIndexed8BitPackSOA4Flags.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/AABBTree/TriangleCodec/TriangleCodecIndexed8BitPackSOA4Flags.h) | 0 |
| [`Physics/AABBTree/TriangleCodec/TriangleCodecIndexed8BitPackSOA4Flags2D.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/AABBTree/TriangleCodec/TriangleCodecIndexed8BitPackSOA4Flags2D.h) | 0 |
| [`Physics/Body/Body.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Body/Body.h) | 1 |
| [`Physics/Body/BodyID.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Body/BodyID.h) | 0 |
| [`Physics/Character/Character.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Character/Character.h) | 0 |
| [`Physics/Cloth/Cloth.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Cloth/Cloth.h) | 0 |
| [`Physics/Collision/ActiveEdges.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/ActiveEdges.h) | 0 |
| [`Physics/Collision/BroadPhase/BroadPhase.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/BroadPhase/BroadPhase.h) | 0 |
| [`Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceMask.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceMask.h) | 0 |
| [`Physics/Collision/BroadPhase/QuadTree.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/BroadPhase/QuadTree.h) | 0 |
| [`Physics/Collision/CastConvexVsTriangles.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/CastConvexVsTriangles.h) | 0 |
| [`Physics/Collision/CastResult.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/CastResult.h) | 0 |
| [`Physics/Collision/CastSphereVsTriangles.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/CastSphereVsTriangles.h) | 0 |
| [`Physics/Collision/CollideConvexVsTriangles.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/CollideConvexVsTriangles.h) | 0 |
| [`Physics/Collision/CollidePointResult.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/CollidePointResult.h) | 0 |
| [`Physics/Collision/CollideShape.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/CollideShape.h) | 0 |
| [`Physics/Collision/CollideShapeVsShapePerLeaf.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/CollideShapeVsShapePerLeaf.h) | 0 |
| [`Physics/Collision/CollideSoftBodyVertexIterator.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/CollideSoftBodyVertexIterator.h) | 0 |
| [`Physics/Collision/CollideSoftBodyVerticesVsTriangles.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/CollideSoftBodyVerticesVsTriangles.h) | 0 |
| [`Physics/Collision/CollideSphereVsTriangles.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/CollideSphereVsTriangles.h) | 0 |
| [`Physics/Collision/CollisionCollector.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/CollisionCollector.h) | 0 |
| [`Physics/Collision/CollisionCollectorImpl.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/CollisionCollectorImpl.h) | 0 |
| [`Physics/Collision/CollisionDispatch.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/CollisionDispatch.h) | 0 |
| [`Physics/Collision/CollisionGroup.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/CollisionGroup.h) | 0 |
| [`Physics/Collision/ContactListener.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/ContactListener.h) | 0 |
| [`Physics/Collision/EstimateCollisionResponse.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/EstimateCollisionResponse.h) | 1 |
| [`Physics/Collision/GroupFilter.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/GroupFilter.h) | 0 |
| [`Physics/Collision/GroupFilterTable.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/GroupFilterTable.h) | 0 |
| [`Physics/Collision/InternalEdgeRemovingCollector.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/InternalEdgeRemovingCollector.h) | 0 |
| [`Physics/Collision/ManifoldBetweenTwoFaces.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/ManifoldBetweenTwoFaces.h) | 2 |
| [`Physics/Collision/NarrowPhaseQuery.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/NarrowPhaseQuery.h) | 0 |
| [`Physics/Collision/NarrowPhaseStats.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/NarrowPhaseStats.h) | 0 |
| [`Physics/Collision/ObjectLayer.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/ObjectLayer.h) | 0 |
| [`Physics/Collision/ObjectLayerPairFilterMask.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/ObjectLayerPairFilterMask.h) | 0 |
| [`Physics/Collision/ObjectLayerPairFilterTable.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/ObjectLayerPairFilterTable.h) | 0 |
| [`Physics/Collision/PhysicsMaterial.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/PhysicsMaterial.h) | 0 |
| [`Physics/Collision/PhysicsMaterialSimple.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/PhysicsMaterialSimple.h) | 0 |
| [`Physics/Collision/RayCast.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/RayCast.h) | 0 |
| [`Physics/Collision/Shape/CompoundShapeVisitors.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/Shape/CompoundShapeVisitors.h) | 0 |
| [`Physics/Collision/Shape/ConvexHullShape.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/Shape/ConvexHullShape.h) | 0 |
| [`Physics/Collision/Shape/GetTrianglesContext.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/Shape/GetTrianglesContext.h) | 0 |
| [`Physics/Collision/Shape/PolyhedronSubmergedVolumeCalculator.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/Shape/PolyhedronSubmergedVolumeCalculator.h) | 0 |
| [`Physics/Collision/Shape/Shapes.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/Shape/Shapes.h) | 0 |
| [`Physics/Collision/Shape/SubShapeID.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/Shape/SubShapeID.h) | 0 |
| [`Physics/Collision/Shape/SubShapeIDPair.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/Shape/SubShapeIDPair.h) | 0 |
| [`Physics/Collision/ShapeCast.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/ShapeCast.h) | 0 |
| [`Physics/Collision/ShapeFilter.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/ShapeFilter.h) | 0 |
| [`Physics/Collision/SimShapeFilter.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/SimShapeFilter.h) | 0 |
| [`Physics/Collision/SimShapeFilterWrapper.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/SimShapeFilterWrapper.h) | 0 |
| [`Physics/Collision/SortReverseAndStore.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/SortReverseAndStore.h) | 0 |
| [`Physics/Collision/TransformedShape.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Collision/TransformedShape.h) | 0 |
| [`Physics/Constraints/ConstraintManager.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Constraints/ConstraintManager.h) | 0 |
| [`Physics/Constraints/Constraints.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Constraints/Constraints.h) | 0 |
| [`Physics/Hair/Hair.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Hair/Hair.h) | 0 |
| [`Physics/IslandBuilder.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/IslandBuilder.h) | 0 |
| [`Physics/LargeIslandSplitter.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/LargeIslandSplitter.h) | 0 |
| [`Physics/PhysicsLock.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/PhysicsLock.h) | 0 |
| [`Physics/PhysicsScene.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/PhysicsScene.h) | 0 |
| [`Physics/PhysicsSystem.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/PhysicsSystem.h) | 0 |
| [`Physics/PhysicsUpdateContext.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/PhysicsUpdateContext.h) | 0 |
| [`Physics/Ragdoll/Ragdoll.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Ragdoll/Ragdoll.h) | 0 |
| [`Physics/SoftBody/Softbody.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/SoftBody/Softbody.h) | 0 |
| [`Physics/StateRecorder.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/StateRecorder.h) | 0 |
| [`Physics/StateRecorderImpl.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/StateRecorderImpl.h) | 0 |
| [`Physics/Vehicle/Vehicle.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Physics/Vehicle/Vehicle.h) | 0 |

### Renderer (65 headers)

Renderer resources and graphics backend headers.

| Header | Export-marked declarations |
|---|---:|
| [`Renderer/DX12/CommandQueueDX12.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/DX12/CommandQueueDX12.h) | 0 |
| [`Renderer/DX12/ConstantBufferDX12.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/DX12/ConstantBufferDX12.h) | 0 |
| [`Renderer/DX12/DescriptorHeapDX12.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/DX12/DescriptorHeapDX12.h) | 0 |
| [`Renderer/DX12/LightingDX12.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/DX12/LightingDX12.h) | 0 |
| [`Renderer/DX12/PipelineStateDX12.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/DX12/PipelineStateDX12.h) | 0 |
| [`Renderer/DX12/RenderInstancesDX12.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/DX12/RenderInstancesDX12.h) | 0 |
| [`Renderer/DX12/RenderPrimitiveDX12.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/DX12/RenderPrimitiveDX12.h) | 0 |
| [`Renderer/DX12/RendererDX12.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/DX12/RendererDX12.h) | 0 |
| [`Renderer/DX12/ShaderDX12.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/DX12/ShaderDX12.h) | 0 |
| [`Renderer/DX12/TextureDX12.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/DX12/TextureDX12.h) | 0 |
| [`Renderer/DebugRenderer.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/DebugRenderer.h) | 0 |
| [`Renderer/DebugRendererPlayback.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/DebugRendererPlayback.h) | 0 |
| [`Renderer/DebugRendererRecorder.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/DebugRendererRecorder.h) | 0 |
| [`Renderer/DebugRendererSimple.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/DebugRendererSimple.h) | 0 |
| [`Renderer/FogVolume.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/FogVolume.h) | 0 |
| [`Renderer/Font.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/Font.h) | 0 |
| [`Renderer/GL/FontGL.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/GL/FontGL.h) | 0 |
| [`Renderer/GL/LightingGL.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/GL/LightingGL.h) | 0 |
| [`Renderer/GL/MeshGL.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/GL/MeshGL.h) | 0 |
| [`Renderer/GL/PipelineStateGL.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/GL/PipelineStateGL.h) | 0 |
| [`Renderer/GL/PostProcess.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/GL/PostProcess.h) | 0 |
| [`Renderer/GL/ReflectionProbe.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/GL/ReflectionProbe.h) | 0 |
| [`Renderer/GL/Renderer_GL.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/GL/Renderer_GL.h) | 0 |
| [`Renderer/GL/ShaderGL.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/GL/ShaderGL.h) | 0 |
| [`Renderer/GL/StorageBufferGL.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/GL/StorageBufferGL.h) | 0 |
| [`Renderer/GL/SubViewportGL.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/GL/SubViewportGL.h) | 0 |
| [`Renderer/GL/SurfaceGL.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/GL/SurfaceGL.h) | 0 |
| [`Renderer/GL/TextureGL.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/GL/TextureGL.h) | 0 |
| [`Renderer/GL/glad.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/GL/glad.h) | 0 |
| [`Renderer/MTL/PipelineStateMTL.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/MTL/PipelineStateMTL.h) | 0 |
| [`Renderer/MTL/RenderInstancesMTL.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/MTL/RenderInstancesMTL.h) | 0 |
| [`Renderer/MTL/RenderPrimitiveMTL.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/MTL/RenderPrimitiveMTL.h) | 0 |
| [`Renderer/MTL/RendererMTL.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/MTL/RendererMTL.h) | 0 |
| [`Renderer/MTL/ShaderMTL.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/MTL/ShaderMTL.h) | 0 |
| [`Renderer/MTL/TextureMTL.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/MTL/TextureMTL.h) | 0 |
| [`Renderer/Managers/DecalManager.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/Managers/DecalManager.h) | 0 |
| [`Renderer/Managers/FontManager.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/Managers/FontManager.h) | 0 |
| [`Renderer/Managers/LightingManager.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/Managers/LightingManager.h) | 0 |
| [`Renderer/Managers/MeshManager3D.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/Managers/MeshManager3D.h) | 0 |
| [`Renderer/Managers/ParticleEffectManager.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/Managers/ParticleEffectManager.h) | 0 |
| [`Renderer/Managers/SurfaceManager.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/Managers/SurfaceManager.h) | 0 |
| [`Renderer/Material2.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/Material2.h) | 0 |
| [`Renderer/Material3.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/Material3.h) | 0 |
| [`Renderer/Mesh.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/Mesh.h) | 0 |
| [`Renderer/Model.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/Model.h) | 0 |
| [`Renderer/PipelineState.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/PipelineState.h) | 0 |
| [`Renderer/Renderer.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/Renderer.h) | 0 |
| [`Renderer/Shader.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/Shader.h) | 0 |
| [`Renderer/SkyBox.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/SkyBox.h) | 0 |
| [`Renderer/StorageBuffer.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/StorageBuffer.h) | 0 |
| [`Renderer/SubViewport.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/SubViewport.h) | 0 |
| [`Renderer/Surface.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/Surface.h) | 0 |
| [`Renderer/Texture.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/Texture.h) | 0 |
| [`Renderer/VK/ConstantBufferVK.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/VK/ConstantBufferVK.h) | 0 |
| [`Renderer/VK/LightingManagerVK.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/VK/LightingManagerVK.h) | 0 |
| [`Renderer/VK/MeshVK.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/VK/MeshVK.h) | 0 |
| [`Renderer/VK/PipelineStateVK.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/VK/PipelineStateVK.h) | 0 |
| [`Renderer/VK/RenderInstancesVK.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/VK/RenderInstancesVK.h) | 0 |
| [`Renderer/VK/RenderPrimitiveVK.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/VK/RenderPrimitiveVK.h) | 0 |
| [`Renderer/VK/RendererVK.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/VK/RendererVK.h) | 0 |
| [`Renderer/VK/Renderer_VK.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/VK/Renderer_VK.h) | 4 |
| [`Renderer/VK/ShaderVK.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/VK/ShaderVK.h) | 0 |
| [`Renderer/VK/StorageBufferVK.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/VK/StorageBufferVK.h) | 0 |
| [`Renderer/VK/SubViewportVK.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/VK/SubViewportVK.h) | 0 |
| [`Renderer/VK/TextureVK.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Renderer/VK/TextureVK.h) | 0 |

### Resources (21 headers)

File/resource and serialization headers.

| Header | Export-marked declarations |
|---|---:|
| [`Resources/BezierCurve.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Resources/BezierCurve.h) | 0 |
| [`Resources/Config.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Resources/Config.h) | 0 |
| [`Resources/Gradient.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Resources/Gradient.h) | 0 |
| [`Resources/Json.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Resources/Json.h) | 0 |
| [`Resources/ObjectStream/GetPrimitiveTypeOfType.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Resources/ObjectStream/GetPrimitiveTypeOfType.h) | 0 |
| [`Resources/ObjectStream/ObjectStream.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Resources/ObjectStream/ObjectStream.h) | 4 |
| [`Resources/ObjectStream/ObjectStreamBinaryIn.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Resources/ObjectStream/ObjectStreamBinaryIn.h) | 0 |
| [`Resources/ObjectStream/ObjectStreamBinaryOut.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Resources/ObjectStream/ObjectStreamBinaryOut.h) | 0 |
| [`Resources/ObjectStream/ObjectStreamIn.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Resources/ObjectStream/ObjectStreamIn.h) | 0 |
| [`Resources/ObjectStream/ObjectStreamOut.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Resources/ObjectStream/ObjectStreamOut.h) | 0 |
| [`Resources/ObjectStream/ObjectStreamTextIn.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Resources/ObjectStream/ObjectStreamTextIn.h) | 0 |
| [`Resources/ObjectStream/ObjectStreamTextOut.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Resources/ObjectStream/ObjectStreamTextOut.h) | 0 |
| [`Resources/ObjectStream/ObjectStreamTypes.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Resources/ObjectStream/ObjectStreamTypes.h) | 0 |
| [`Resources/ObjectStream/SerializableAttribute.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Resources/ObjectStream/SerializableAttribute.h) | 0 |
| [`Resources/ObjectStream/SerializableAttributeEnum.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Resources/ObjectStream/SerializableAttributeEnum.h) | 0 |
| [`Resources/ObjectStream/SerializableAttributeTyped.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Resources/ObjectStream/SerializableAttributeTyped.h) | 0 |
| [`Resources/ObjectStream/SerializableObject.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Resources/ObjectStream/SerializableObject.h) | 0 |
| [`Resources/ObjectStream/TypeDeclarations.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Resources/ObjectStream/TypeDeclarations.h) | 0 |
| [`Resources/PCK.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Resources/PCK.h) | 0 |
| [`Resources/Timer.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Resources/Timer.h) | 0 |
| [`Resources/Tween.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Resources/Tween.h) | 0 |

### Skeleton (1 headers)

Skeleton and animation declarations.

| Header | Export-marked declarations |
|---|---:|
| [`Skeleton/Skeleton.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Skeleton/Skeleton.h) | 0 |

### TriangleSplitter (1 headers)

Triangle splitting utilities.

| Header | Export-marked declarations |
|---|---:|
| [`TriangleSplitter/TriangleSplitter.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/TriangleSplitter/TriangleSplitter.h) | 0 |

### Variants (53 headers)

Math, geometry and container value types.

| Header | Export-marked declarations |
|---|---:|
| [`Variants/AABB2.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/AABB2.h) | 0 |
| [`Variants/AABB3.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/AABB3.h) | 0 |
| [`Variants/Basis.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Basis.h) | 0 |
| [`Variants/Color.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Color.h) | 0 |
| [`Variants/Math/EigenValueSymmetric.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Math/EigenValueSymmetric.h) | 0 |
| [`Variants/Math/GaussianElimination.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Math/GaussianElimination.h) | 0 |
| [`Variants/Math/HalfFloat.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Math/HalfFloat.h) | 0 |
| [`Variants/Math/MathTypes.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Math/MathTypes.h) | 0 |
| [`Variants/Math/Real.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Math/Real.h) | 0 |
| [`Variants/Math/Trigonometry.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Math/Trigonometry.h) | 0 |
| [`Variants/Matrix/DMat44.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Matrix/DMat44.h) | 0 |
| [`Variants/Matrix/DMat4x3.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Matrix/DMat4x3.h) | 0 |
| [`Variants/Matrix/DynMatrix.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Matrix/DynMatrix.h) | 0 |
| [`Variants/Matrix/Mat22.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Matrix/Mat22.h) | 0 |
| [`Variants/Matrix/Mat2x3.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Matrix/Mat2x3.h) | 0 |
| [`Variants/Matrix/Mat2x4.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Matrix/Mat2x4.h) | 0 |
| [`Variants/Matrix/Mat33.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Matrix/Mat33.h) | 0 |
| [`Variants/Matrix/Mat3x2.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Matrix/Mat3x2.h) | 0 |
| [`Variants/Matrix/Mat3x4.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Matrix/Mat3x4.h) | 0 |
| [`Variants/Matrix/Mat44.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Matrix/Mat44.h) | 0 |
| [`Variants/Matrix/Mat4x2.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Matrix/Mat4x2.h) | 0 |
| [`Variants/Matrix/Mat4x3.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Matrix/Mat4x3.h) | 0 |
| [`Variants/Matrix/TMatrix.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Matrix/TMatrix.h) | 0 |
| [`Variants/OBB2.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/OBB2.h) | 0 |
| [`Variants/OBB3.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/OBB3.h) | 0 |
| [`Variants/Plane.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Plane.h) | 0 |
| [`Variants/Quat.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Quat.h) | 0 |
| [`Variants/Rect.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Rect.h) | 0 |
| [`Variants/TArray.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/TArray.h) | 0 |
| [`Variants/TMap.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/TMap.h) | 0 |
| [`Variants/TSet.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/TSet.h) | 0 |
| [`Variants/TStaticArray.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/TStaticArray.h) | 0 |
| [`Variants/Vector/BVec16.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Vector/BVec16.h) | 0 |
| [`Variants/Vector/DVec2.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Vector/DVec2.h) | 0 |
| [`Variants/Vector/DVec3.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Vector/DVec3.h) | 0 |
| [`Variants/Vector/DVec4.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Vector/DVec4.h) | 0 |
| [`Variants/Vector/Double2.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Vector/Double2.h) | 0 |
| [`Variants/Vector/Double3.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Vector/Double3.h) | 0 |
| [`Variants/Vector/Double4.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Vector/Double4.h) | 0 |
| [`Variants/Vector/Float2.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Vector/Float2.h) | 0 |
| [`Variants/Vector/Float3.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Vector/Float3.h) | 0 |
| [`Variants/Vector/Float4.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Vector/Float4.h) | 0 |
| [`Variants/Vector/Int2.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Vector/Int2.h) | 0 |
| [`Variants/Vector/Int3.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Vector/Int3.h) | 0 |
| [`Variants/Vector/Int4.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Vector/Int4.h) | 0 |
| [`Variants/Vector/TVec.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Vector/TVec.h) | 0 |
| [`Variants/Vector/UVec4.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Vector/UVec4.h) | 0 |
| [`Variants/Vector/Vec2.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Vector/Vec2.h) | 1 |
| [`Variants/Vector/Vec3.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Vector/Vec3.h) | 1 |
| [`Variants/Vector/Vec4.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Vector/Vec4.h) | 0 |
| [`Variants/Vector/iVec2.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Vector/iVec2.h) | 1 |
| [`Variants/Vector/iVec3.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Vector/iVec3.h) | 1 |
| [`Variants/Vector/iVec4.h`](https://github.com/TxbiG/Moss/blob/main/include/Moss/Variants/Vector/iVec4.h) | 1 |

## Export-marked declaration index

The following section reproduces active declaration lines whose source line begins with `MOSS_API`. Multiline declarations are joined for readability; names, parameter types, default arguments and qualifiers are retained. Source links point to the declaration start line. This section is deliberately not a replacement for the class/member APIs in the header inventory.

### `Moss_Audio.h`

[Open source header](https://github.com/TxbiG/Moss/blob/main/include/Moss/Moss_Audio.h)

```cpp
MOSS_API int Moss_Init_Audio();
MOSS_API void Moss_Terminate_Audio();
MOSS_API void Moss_AudioUpdate(float deltaTime);
MOSS_API Moss_AudioSource* Moss_AudioLoadWavFile(const char* filename);
MOSS_API Moss_AudioSource* Moss_AudioLoadWav();
MOSS_API Moss_AudioSource* Moss_AudioLoadOgg(const char* filename, AudioLoadType type);
MOSS_API Moss_AudioSource* Moss_AudioLoadMP3(const char* filename);
MOSS_API void Moss_AudioSourceDestroy(Moss_AudioSource* source);
MOSS_API Moss_AudioSource* Moss_AudioCaptureMicrophone(Moss_Microphone* mic);
MOSS_API AudioEffect Moss_AudioCreateEffect(AudioEffectType type);
MOSS_API void Moss_AudioCreateEffect(AudioEffect* effect, const char* paramName, float value);
MOSS_API void Moss_AudioRemoveEffect(AudioEffect* effect);
MOSS_API ChannelID Moss_AudioCreateChannel(ChannelID channel);
MOSS_API void Audio_RemoveChannel(ChannelID channel);
MOSS_API ChannelID Moss_AudioGetMasterChannel();
MOSS_API void Moss_AudioSetChannelVolume(ChannelID channel, float volume);
MOSS_API void Moss_AudioSetChannelMute(ChannelID channel, bool mute);
MOSS_API void Moss_AudioAddChannelEffect(ChannelID channel, AudioEffect* effect);
MOSS_API void Moss_AudioRemoveChannelEffect(ChannelID channel, AudioEffect* effect);
MOSS_API void Moss_AudioRemoveAllChannelEffects(ChannelID channel);
MOSS_API AudioStream* Moss_AudioStreamCreate();
MOSS_API void Moss_AudioStreamPlay(AudioStream* audiostream);
MOSS_API void Moss_AudioStreamStop(AudioStream* audiostream);
MOSS_API void Moss_AudioStreamSetVolume(AudioStream* audiostream, float volume);
MOSS_API void Moss_AudioStreamSetPitch(AudioStream* audiostream, float pitch);
MOSS_API void Moss_AudioStreamSetPlaybackRate(AudioStream* audiostream, float rate);
MOSS_API void Moss_AudioStreamSetPan(AudioStream* audiostream, float pan);
MOSS_API void Moss_AudioStreamSetLoop(AudioStream* audiostream, bool loop);
MOSS_API void Moss_AudioStreamRemove(AudioStream* audiostream);
MOSS_API AudioStream2D* Moss_AudioStream2DCreate();
MOSS_API void Moss_AudioStream2DPlay(AudioStream2D* audiostream);
MOSS_API void Moss_AudioStream2DStop(AudioStream2D* audiostream);
MOSS_API void Moss_AudioStream2DSetVolume(AudioStream2D* audiostream, float volume);
MOSS_API void Moss_AudioStream2DSetPitch(AudioStream2D* audiostream, float pitch);
MOSS_API void Moss_AudioStream2DSetPan(AudioStream2D* audiostream, float pan);
MOSS_API void Moss_AudioStream2DSetLoop(AudioStream2D* audiostream, bool loop);
MOSS_API void Moss_AudioStream3DSetPosition(AudioStream2D* audiostream);
MOSS_API void Moss_AudioStream3DSetVelocity(AudioStream2D* audiostream);
MOSS_API void Moss_AudioStream3DSetMaxDistance(AudioStream2D* audiostream);
MOSS_API void Moss_AudioStream2DRemove(AudioStream2D* audiostream);
MOSS_API AudioStream3D* Moss_AudioStream3DCreate();
MOSS_API void Moss_AudioStream3DPlay(AudioStream3D* audiostream);
MOSS_API void Moss_AudioStream3DStop(AudioStream3D* audiostream);
MOSS_API void Moss_AudioStream3DSetVolume(AudioStream3D* audiostream, float volume);
MOSS_API void Moss_AudioStream3DSetPitch(AudioStream3D* audiostream, float pitch);
MOSS_API void Moss_AudioStream3DSetPlaybackRate(AudioStream3D* audiostream, float rate);
MOSS_API void Moss_AudioStream3DSetPan(AudioStream3D* audiostream, float pan);
MOSS_API void Moss_AudioStream3DSetLoop(AudioStream3D* audiostream, bool loop);
MOSS_API void Moss_AudioStream3DSetPosition(AudioStream3D* audiostream);
MOSS_API void Moss_AudioStream3DSetVelocity(AudioStream3D* audiostream);
MOSS_API void Moss_AudioStream3DSetMaxDistance(AudioStream3D* audiostream);
MOSS_API void Moss_AudioStream3DSetDistanceModel(AudioStream3D* stream, DistanceModel model);
MOSS_API void Moss_AudioStream3DRemove(AudioStream3D* audiostream);
MOSS_API AudioListener2D* Moss_AudioCreateAudioListener2D(Float2 position);
MOSS_API AudioListener3D* Moss_AudioCreateAudioListener3D(Float3 position);
MOSS_API RayAudioListener2D* Moss_AudioCreateRayAudioListener2D(Float2 position);
MOSS_API RayAudioListener3D* Moss_AudioCreateRayAudioListener3D(Float3 position);
MOSS_API void Moss_AudioRemoveAudioListener2D(AudioListener2D* listener);
MOSS_API void Moss_AudioRemoveAudioListener3D(AudioListener3D* listener);
MOSS_API void Moss_AudioRemoveRayAudioListener2D(RayAudioListener2D* listener);
MOSS_API void Moss_AudioRemoveRayAudioListener3D(RayAudioListener3D* listener);
MOSS_API void Moss_AudioActivateAudioListener2D(AudioListener2D* listener, bool activate);
MOSS_API void Moss_AudioActivateAudioListener3D(AudioListener3D* listener, bool activate);
MOSS_API void Moss_AudioActivateRayAudioListener2D(RayAudioListener2D* listener, bool activate);
MOSS_API void Moss_AudioActivateRayAudioListener3D(RayAudioListener3D* listener, bool activate);
MOSS_API void Moss_AudioListenerSetOrientation(AudioListener3D* listener, const Vec3& forward, const Vec3& up);
MOSS_API void Moss_AudioRayListener2DSetPhysics(RayAudioListener2D* listener, PhysicsSystem* physics, Moss_AudioRaycast2DCallback raycast, void* user_data);
MOSS_API void Moss_AudioRayListener3DSetPhysics(RayAudioListener3D* listener, PhysicsSystem* physics, Moss_AudioRaycast3DCallback raycast, void* user_data);
MOSS_API void Moss_AudioRayListener2DSetTraceSettings(RayAudioListener2D* listener, float max_distance, uint32_t reflection_rays);
MOSS_API void Moss_AudioRayListener3DSetTraceSettings(RayAudioListener3D* listener, float max_distance, uint32_t reflection_rays);
MOSS_API bool Moss_AudioRayTrace2D(const Moss_AudioRayTrace2DDesc* desc, Moss_AudioRayTraceResult* out_result);
MOSS_API bool Moss_AudioRayTrace3D(const Moss_AudioRayTrace3DDesc* desc, Moss_AudioRayTraceResult* out_result);
MOSS_API bool Moss_AudioRayTraceFromListener2D(RayAudioListener2D* listener, const Vec2* source_position, Moss_AudioRayTraceResult* out_result);
MOSS_API bool Moss_AudioRayTraceFromListener3D(RayAudioListener3D* listener, const Vec3* source_position, Moss_AudioRayTraceResult* out_result);
MOSS_API float Moss_AudioRayTraceComputeGain(const Moss_AudioRayTraceResult* result);
MOSS_API bool Moss_IsSpeakerDeviceReady();
MOSS_API void Moss_AudioSpeakerOpen();
MOSS_API void Moss_AudioSpeakerPause();
MOSS_API void Moss_AudioSpeakerResume();
MOSS_API bool Moss_AudioSpeakerIsPaused();
MOSS_API bool Moss_AudioSelectSpeakerDevice(int id);
MOSS_API int Moss_GetCurrentSpeakerDeviceID();
MOSS_API const char* Moss_GetSpeakerDeviceName(int id);
MOSS_API int Moss_ListSpeakerDevices();
MOSS_API uint32_t Moss_MicrophoneGetDeviceCount();
MOSS_API const char* Moss_MicrophoneGetDeviceName(uint32_t device_index);
MOSS_API Moss_Microphone* Moss_MicrophoneOpen(const Moss_MicrophoneDesc* desc);
MOSS_API void Moss_MicrophoneClose(Moss_Microphone* microphone);
MOSS_API bool Moss_MicrophoneStart(Moss_Microphone* microphone);
MOSS_API void Moss_MicrophoneStop(Moss_Microphone* microphone);
MOSS_API uint32_t Moss_MicrophoneRead(Moss_Microphone* microphone, float* out_samples, uint32_t max_frames);
MOSS_API void Moss_MicrophoneSetGain(Moss_Microphone* microphone, float gain);
MOSS_API uint32_t Moss_MicrophoneGetSampleRate(Moss_Microphone* microphone);
MOSS_API uint32_t Moss_MicrophoneGetChannels(Moss_Microphone* microphone);
MOSS_API void Moss_MicrophoneSetCallback(Moss_Microphone* microphone, MicrophoneCallback callback, void* userData);
MOSS_API Moss_MicrophoneLevels Moss_MicrophoneGetLevels(Moss_Microphone* microphone);
MOSS_API float Moss_MicrophoneGetLevelRMS(Moss_Microphone* microphone);
MOSS_API float Moss_MicrophoneGetLevelPeak(Moss_Microphone* microphone);
MOSS_API float Moss_MicrophoneGetSmoothedVolume(Moss_Microphone* microphone);
MOSS_API float Moss_MicrophoneGetVoiceActivity(Moss_Microphone* microphone);
MOSS_API bool Moss_IsMicrophoneDeviceReady();
MOSS_API int Moss_AudioMicrophoneOpen();
MOSS_API void Moss_AudioMicrophoneClose();
MOSS_API void Moss_AudioMicrophonePlay();
MOSS_API void Moss_AudioMicrophoneStop();
MOSS_API int Moss_AudioMicrophoneID();
MOSS_API bool Moss_AudioSelectMicrophoneDevice(int id);
MOSS_API const char* Moss_GetMicrophoneDeviceName(int index);
MOSS_API int Moss_ListMicrophoneDevices();
MOSS_API void Moss_AudioMicrophoneSetGain(Moss_Microphone* microphone, float gain);
MOSS_API int Moss_AudioMicrophoneGetSampleRate(Moss_Microphone* microphone);
MOSS_API int Moss_AudioMicrophoneGetChannels(Moss_Microphone* microphone);
MOSS_API void Moss_AudioStreamSetCallback(AudioStream* stream, AudioStreamCallback callback, void* userData);
```

### `Moss_GPU.h`

[Open source header](https://github.com/TxbiG/Moss/blob/main/include/Moss/Moss_GPU.h)

```cpp
MOSS_API Moss_GPUCommandBuffer* Moss_AcquireGPUCommandBuffer(Moss_GPUDevice* device);
MOSS_API void Moss_BeginGPUComputePass(Moss_GPUCommandBuffer* cmd);
MOSS_API void Moss_BeginGPUCopyPass(Moss_GPUCommandBuffer* cmd);
MOSS_API Moss_GPUBuffer* Moss_CreateGPUBuffer(Moss_GPUDevice* device, const Moss_GPUBufferDesc* desc);
MOSS_API void Moss_DestroyGPUDevice(Moss_GPUDevice* device);
MOSS_API void Moss_BeginGPURenderPass(Moss_GPUCommandBuffer* cmd, Moss_Framebuffer* framebuffer);
MOSS_API void Moss_BindGPUComputeSamplers(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, Moss_GPUSampler* const* samplers, uint32_t count);
MOSS_API void Moss_BindGPUComputeTextures(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, Moss_GPUTexture* const* textures, uint32_t texture_count);
MOSS_API void Moss_BindGPUComputeUniformBuffers(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, const Moss_GPUBufferBinding* bindings, uint32_t binding_count);
MOSS_API void Moss_BindGPUFragmentSamplers(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, Moss_GPUSampler* const* samplers, uint32_t sampler_count);
MOSS_API void Moss_BindGPUFragmentTextures(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, Moss_GPUTexture* const* textures, uint32_t texture_count);
MOSS_API void Moss_BindGPUFragmentUniformBuffers(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, const Moss_GPUBufferBinding* bindings, uint32_t binding_count);
MOSS_API void Moss_BindGPUIndexBuffer(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* buffer, uint64_t offset, EGPUIndexType index_type);
MOSS_API void Moss_BindGPUPipeline(Moss_GPUCommandBuffer* cmd, Moss_PipelineState* pipeline);
MOSS_API void Moss_BindGPUResourceSet(Moss_GPUCommandBuffer* cmd, uint32_t set_index, Moss_ResourceSet* set);
MOSS_API void Moss_BindGPUVertexBuffers(Moss_GPUCommandBuffer* cmd, uint32_t first_binding, const Moss_GPUBufferBinding* bindings, uint32_t binding_count);
MOSS_API void Moss_BindGPUVertexSamplers(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, Moss_GPUSampler* const* samplers, uint32_t sampler_count);
MOSS_API void Moss_BindGPUVertexTextures(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, Moss_GPUTexture* const* textures, uint32_t texture_count);
MOSS_API void Moss_BindGPUVertexUniformBuffers(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, const Moss_GPUBufferBinding* bindings, uint32_t binding_count);
MOSS_API void Moss_BlitGPUTexture(Moss_GPUCommandBuffer* cmd, Moss_Texture* src, Moss_Texture* dst, const Moss_GPUBlitRegion* region);
MOSS_API uint32_t Moss_CalculateGPUTextureFormatSize(ETextureFormat format, uint32_t width, uint32_t height, uint32_t depth);
MOSS_API void Moss_CancelGPUCommandBuffer(Moss_GPUCommandBuffer* cmd);
MOSS_API void Moss_CopyGPUBufferToBuffer(Moss_GPUCommandBuffer* cmd,Moss_GPUBuffer* src,Moss_GPUBuffer* dst,const Moss_GPUBufferRegion* src_region,const Moss_GPUBufferRegion* dst_region);
MOSS_API void Moss_CopyGPUBufferToTexture(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* src, Moss_GPUTexture* dst, const Moss_GPUBufferRegion* src_region, const Moss_GPUTextureRegion* dst_region);
MOSS_API void Moss_CopyGPUTextureToBuffer(Moss_GPUCommandBuffer* cmd, Moss_GPUTexture* src, Moss_GPUBuffer* dst, const Moss_GPUTextureRegion* src_region, const Moss_GPUBufferRegion* dst_region);
MOSS_API void Moss_CopyGPUTextureToTexture(Moss_GPUCommandBuffer* cmd, Moss_GPUTexture* src, Moss_GPUTexture* dst, const Moss_GPUTextureRegion* src_region, const Moss_GPUTextureRegion* dst_region);
MOSS_API Moss_GPUSampler* Moss_CreateGPUSampler( Moss_GPUDevice* device, const Moss_GPUSamplerCreateInfo* create_info);
MOSS_API Moss_GPUShader* Moss_CreateGPUShader(Moss_GPUDevice* device, const Moss_GPUShaderCreateInfo* create_info);
MOSS_API Moss_ComputePipelineState* Moss_CreateGPUComputePipeline(Moss_GPUDevice* device, const Moss_GPUComputePipelineCreateInfo* create_info);
MOSS_API Moss_GPUTexture* Moss_CreateGPUTexture(Moss_GPUDevice* device, const Moss_GPUTextureCreateInfo* create_info);
MOSS_API Moss_GPUTextureView* Moss_CreateGPUTextureView(Moss_GPUDevice* device, const Moss_GPUTextureViewCreateInfo* create_info);
MOSS_API Moss_GPUTransferBuffer* Moss_CreateGPUTransferBuffer(Moss_GPUDevice* device, const Moss_GPUTransferBufferCreateInfo* create_info);
MOSS_API Moss_Framebuffer* Moss_CreateGPUFramebuffer(Moss_GPUDevice* device, const Moss_GPUFramebufferCreateInfo* create_info);
MOSS_API Moss_ResourceSetLayout* Moss_CreateGPUResourceSetLayout(Moss_GPUDevice* device, const Moss_GPUResourceSetLayoutCreateInfo* create_info);
MOSS_API Moss_ResourceSet* Moss_CreateGPUResourceSet(Moss_GPUDevice* device, const Moss_GPUResourceSetCreateInfo* create_info);
MOSS_API Moss_GPUQueryPool* Moss_CreateGPUQueryPool(Moss_GPUDevice* device, const Moss_GPUQueryPoolCreateInfo* create_info);
MOSS_API void Moss_DispatchGPUCompute(Moss_GPUCommandBuffer* cmd, uint32_t group_count_x, uint32_t group_count_y, uint32_t group_count_z);
MOSS_API void Moss_DispatchGPUComputeIndirect(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* indirect_buffer, uint64_t offset);
MOSS_API void Moss_DownloadFromGPUBuffer(Moss_GPUDevice* device, Moss_GPUBuffer* src_buffer, void* dst_data, uint64_t size, uint64_t src_offset);
MOSS_API void Moss_DownloadFromGPUTexture(Moss_GPUDevice* device, Moss_GPUTexture* src_texture, const Moss_GPUTextureRegion* src_region, void* dst_data, uint32_t dst_row_pitch, uint32_t dst_slice_pitch);
MOSS_API void Moss_DrawGPUIndexedPrimitives(Moss_GPUCommandBuffer* cmd, uint32_t index_count, uint32_t instance_count, uint32_t first_index, int32_t vertex_offset, uint32_t first_instance);
MOSS_API void Moss_DrawGPUIndexedPrimitivesIndirect(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* indirect_buffer, uint64_t offset, uint32_t draw_count, uint32_t stride);
MOSS_API void Moss_DrawGPUPrimitives(Moss_GPUCommandBuffer* cmd, uint32_t vertex_count, uint32_t instance_count, uint32_t first_vertex, uint32_t first_instance);
MOSS_API void Moss_DrawGPUPrimitivesIndirect(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* indirect_buffer, uint64_t offset, uint32_t draw_count, uint32_t stride);
MOSS_API void Moss_EndGPUComputePass(Moss_GPUCommandBuffer* cmd);
MOSS_API void Moss_EndGPUCopyPass(Moss_GPUCommandBuffer* cmd);
MOSS_API void Moss_EndGPURenderPass(Moss_GPUCommandBuffer* cmd);
MOSS_API void Moss_GDKResumeGPU(Moss_GPUDevice* device);
MOSS_API void Moss_GDKSuspendGPU(Moss_GPUDevice* device);
MOSS_API void Moss_GenerateMipmapsForGPUTexture(Moss_GPUCommandBuffer* cmd, Moss_GPUTexture* texture);
MOSS_API const char* Moss_GetGPUDriver( uint32_t driver_index);
MOSS_API Moss_GPUBackendType Moss_GetCompiledGPUBackendType(void);
MOSS_API Moss_GPUBackendType Moss_GetGPUDeviceBackendType(Moss_GPUDevice* device);
MOSS_API const char* Moss_GPUBackendGetName(Moss_GPUBackendType backend);
MOSS_API uint32_t Moss_GPUBackendGetShaderFormats(Moss_GPUBackendType backend);
MOSS_API bool Moss_GPUBackendIsCompiled(Moss_GPUBackendType backend);
MOSS_API bool Moss_GPUBackendIsSupportedOnPlatform(Moss_GPUBackendType backend);
MOSS_API bool Moss_GetGPUBackendInfo(Moss_GPUBackendType backend, Moss_GPUBackendInfo* out_info);
MOSS_API bool Moss_RegisterNativeGPUBackend(Moss_GPUBackendType backend, const void* native_table);
MOSS_API bool Moss_HasNativeGPUBackend(Moss_GPUBackendType backend);
MOSS_API uint32_t Moss_GetGPUShaderFormats( Moss_GPUDevice* device);
MOSS_API ETextureFormat Moss_GetGPUTextureFormatFromPixelFormat(EPixelFormat pixel_format);
MOSS_API uint32_t Moss_GetNumGPUDrivers(void);
MOSS_API EPixelFormat Moss_GetPixelFormatFromGPUTextureFormat(ETextureFormat texture_format);
MOSS_API bool Moss_GPUSupportsProperties(Moss_GPUDevice* device);
MOSS_API bool Moss_GPUSupportsShaderFormats(Moss_GPUDevice* device, uint32_t shader_format_mask);
MOSS_API uint32_t Moss_GPUTextureFormatTexelBlockSize( ETextureFormat format);
MOSS_API bool Moss_GPUTextureSupportsFormat(Moss_GPUDevice* device,ETextureFormat format, ETextureUsage usage);
MOSS_API bool Moss_GPUTextureSupportsSampleCount(Moss_GPUDevice* device,ETextureFormat format, uint32_t sample_count);
MOSS_API void Moss_InsertGPUDebugLabel(Moss_GPUCommandBuffer* cmd, const char* label);
MOSS_API void* Moss_MapGPUTransferBuffer(Moss_GPUDevice* device, Moss_GPUTransferBuffer* transfer_buffer);
MOSS_API void Moss_PopGPUDebugGroup(Moss_GPUCommandBuffer* cmd);
MOSS_API void Moss_PushGPUComputeUniformData(Moss_GPUCommandBuffer* cmd, uint32_t slot, const void* data, uint32_t size);
MOSS_API void Moss_PushGPUDebugGroup(Moss_GPUCommandBuffer* cmd, const char* label);
MOSS_API void Moss_PushGPUFragmentUniformData(Moss_GPUCommandBuffer* cmd, uint32_t slot, const void* data, uint32_t size);
MOSS_API void Moss_PushGPUVertexUniformData(Moss_GPUCommandBuffer* cmd, uint32_t slot, const void* data, uint32_t size);
MOSS_API bool Moss_QueryGPUFence(Moss_GPUDevice* device, Moss_GPUFence* fence);
MOSS_API void Moss_ReleaseGPUBuffer(Moss_GPUDevice* device, Moss_GPUBuffer* buffer);
MOSS_API void Moss_ReleaseGPUFence(Moss_GPUDevice* device, Moss_GPUFence* fence);
MOSS_API void Moss_ReleaseGPUSampler(Moss_GPUDevice* device, Moss_GPUSampler* sampler);
MOSS_API void Moss_ReleaseGPUShader(Moss_GPUDevice* device, Moss_GPUShader* shader);
MOSS_API void Moss_ReleaseGPUComputePipeline(Moss_GPUDevice* device, Moss_ComputePipelineState* pipeline);
MOSS_API void Moss_ReleaseGPUTexture(Moss_GPUDevice* device, Moss_GPUTexture* texture);
MOSS_API void Moss_ReleaseGPUTextureView(Moss_GPUDevice* device, Moss_GPUTextureView* texture_view);
MOSS_API void Moss_ReleaseGPUTransferBuffer(Moss_GPUDevice* device, Moss_GPUTransferBuffer* transfer_buffer);
MOSS_API void Moss_ReleaseGPUFramebuffer(Moss_GPUDevice* device, Moss_Framebuffer* framebuffer);
MOSS_API void Moss_ReleaseGPUResourceSetLayout(Moss_GPUDevice* device, Moss_ResourceSetLayout* layout);
MOSS_API void Moss_ReleaseGPUResourceSet(Moss_GPUDevice* device, Moss_ResourceSet* set);
MOSS_API void Moss_ReleaseGPUQueryPool(Moss_GPUDevice* device, Moss_GPUQueryPool* query_pool);
MOSS_API void Moss_SetGPUAllowedFramesInFlight(Moss_GPUDevice* device, uint32_t frames_in_flight);
MOSS_API void Moss_SetGPUBlendConstants(Moss_GPUCommandBuffer* cmd, float r, float g, float b, float a);
MOSS_API void Moss_SetGPUBufferName(Moss_GPUDevice* device, Moss_GPUBuffer* buffer, const char* name);
MOSS_API void Moss_SetGPUScissor(Moss_GPUCommandBuffer* cmd, const Moss_Rect* scissor);
MOSS_API void Moss_SetGPUStencilReference(Moss_GPUCommandBuffer* cmd,uint32_t reference);
MOSS_API void Moss_SetGPUTextureName(Moss_GPUDevice* device, Moss_GPUTexture* texture, const char* name);
MOSS_API void Moss_SetGPUViewport(Moss_GPUCommandBuffer* cmd, const Moss_GPUViewport* viewport);
MOSS_API void Moss_SetGPUScissor(Moss_GPUCommandBuffer* cmd, const Moss_Rect* scissors);
MOSS_API void Moss_SubmitGPUCommandBuffer(Moss_GPUDevice* device, Moss_GPUCommandBuffer* cmd);
MOSS_API Moss_GPUFence* Moss_SubmitGPUCommandBufferAndAcquireFence(Moss_GPUDevice* device, Moss_GPUCommandBuffer* cmd);
MOSS_API void Moss_BindGPUComputePipeline(Moss_GPUCommandBuffer* cmd, Moss_ComputePipelineState* pipeline);
MOSS_API void Moss_BarrierGPUResources(Moss_GPUCommandBuffer* cmd, const Moss_GPUBarrierInfo* barrier_info);
MOSS_API void Moss_ResetGPUQueryPool(Moss_GPUCommandBuffer* cmd, Moss_GPUQueryPool* query_pool, uint32_t first_query, uint32_t query_count);
MOSS_API void Moss_WriteGPUTimestamp(Moss_GPUCommandBuffer* cmd, Moss_GPUQueryPool* query_pool, uint32_t query_index);
MOSS_API void Moss_BeginGPUQuery(Moss_GPUCommandBuffer* cmd, Moss_GPUQueryPool* query_pool, uint32_t query_index);
MOSS_API void Moss_EndGPUQuery(Moss_GPUCommandBuffer* cmd, Moss_GPUQueryPool* query_pool, uint32_t query_index);
MOSS_API bool Moss_GetGPUQueryResults(Moss_GPUDevice* device, Moss_GPUQueryPool* query_pool, uint32_t first_query, uint32_t query_count, void* data, uint64_t data_size, uint64_t stride, EGPUQueryResultFlags flags);
MOSS_API void Moss_TransitionGPUBuffer(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* buffer, EResourceState old_state, EResourceState new_state);
MOSS_API void Moss_TransitionGPUTexture(Moss_GPUCommandBuffer* cmd, Moss_GPUTexture* texture, EResourceState old_state, EResourceState new_state);
MOSS_API void Moss_UnmapGPUTransferBuffer(Moss_GPUDevice* device, Moss_GPUTransferBuffer* transfer_buffer);
MOSS_API void Moss_UploadToGPUBuffer(Moss_GPUDevice* device, Moss_GPUBuffer* dst_buffer, const void* src_data, uint64_t size, uint64_t dst_offset);
MOSS_API void Moss_UploadToGPUTexture(Moss_GPUDevice* device, Moss_GPUTexture* dst_texture, const Moss_GPUTextureTransferInfo* transfer_info);
MOSS_API void Moss_WaitForGPUFences(Moss_GPUDevice* device, Moss_GPUFence* const* fences, uint32_t fence_count, bool wait_all, uint64_t timeout_ns);
MOSS_API void Moss_WaitForGPUIdle(Moss_GPUDevice* device);
MOSS_API bool Moss_WindowSupportsGPUPresentMode(Moss_Window* window, EGPUPresentMode present_mode);
MOSS_API Moss_GPUBlitInfo();
MOSS_API Moss_GPUBlitRegion();
MOSS_API Moss_GPUBufferBinding();
MOSS_API Moss_GPUBufferCreateInfo();
MOSS_API Moss_GPUBufferLocation();
MOSS_API Moss_GPUBufferRegion();
MOSS_API Moss_GPUColorTargetBlendState();
MOSS_API Moss_GPUColorTargetDescription();
MOSS_API Moss_GPUColorTargetInfo();
MOSS_API Moss_GPUDepthStencilState();
MOSS_API Moss_GPUDepthStencilTargetInfo();
MOSS_API Moss_GPUIndexedIndirectDrawCommand();
MOSS_API Moss_GPUIndirectDispatchCommand();
MOSS_API Moss_GPUIndirectDrawCommand();
MOSS_API Moss_GPUMultisampleState();
MOSS_API Moss_GPURasterizerState();
MOSS_API Moss_GPUSamplerCreateInfo();
MOSS_API Moss_GPUShaderCreateInfo();
MOSS_API Moss_GPUStencilOpState();
MOSS_API Moss_GPUTextureCreateInfo();
MOSS_API Moss_GPUTextureLocation();
MOSS_API Moss_GPUTextureRegion();
MOSS_API Moss_GPUTextureSamplerBinding();
MOSS_API Moss_GPUTextureTransferInfo();
MOSS_API Moss_GPUTransferBufferCreateInfo();
MOSS_API Moss_GPUTransferBufferLocation();
MOSS_API Moss_GPUVertexAttribute();
MOSS_API Moss_GPUVertexBufferDescription();
MOSS_API Moss_GPUVertexInputState();
MOSS_API Moss_GPUViewport();
MOSS_API void Moss_BindGPUComputeStorageBuffers(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, const Moss_GPUStorageBufferReadWriteBinding* bindings, uint32_t binding_count);
MOSS_API void Moss_BindGPUComputeStorageTextures( Moss_GPUCommandBuffer* cmd, uint32_t first_slot, const Moss_GPUStorageTextureReadWriteBinding* bindings, uint32_t binding_count);
MOSS_API void Moss_BindGPUFragmentStorageBuffers(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, const Moss_GPUStorageBufferReadWriteBinding* bindings, uint32_t binding_count);
MOSS_API void Moss_BindGPUFragmentStorageTextures(Moss_GPUCommandBuffer* cmd, uint32_t first_slot,const Moss_GPUStorageTextureReadWriteBinding* bindings, uint32_t binding_count);
MOSS_API void Moss_BindGPUVertexStorageBuffers(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, const Moss_GPUStorageBufferReadWriteBinding* bindings, uint32_t binding_count);
MOSS_API void Moss_BindGPUVertexStorageTextures(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, const Moss_GPUStorageTextureReadWriteBinding* bindings, uint32_t binding_count);
MOSS_API const char* Moss_GetGPUDeviceDriver(Moss_GPUDevice* device);
MOSS_API const Moss_GPUDeviceProperties* Moss_GetGPUDeviceProperties(Moss_GPUDevice* device);
MOSS_API void Moss_ReleaseWindowFromGPUDevice(Moss_GPUDevice* device, Moss_Window* window);
MOSS_API bool Moss_ClaimWindowForGPUDevice(Moss_GPUDevice* device, Moss_Window* window);
MOSS_API void Moss_UpdateGPUResourceSet(Moss_GPUDevice* device, Moss_ResourceSet* set, const Moss_GPUResourceBinding* bindings, uint32_t binding_count);
MOSS_API Moss_GPUDevice* Moss_CreateGPUDevice(const Moss_GPUDeviceDesc* desc);
MOSS_API Moss_GPUDevice* Moss_CreateGPUDeviceWithProperties(const Moss_GPUDeviceDesc* desc, const Moss_GPUDeviceProperties* requested_properties);
MOSS_API Moss_GPUBuffer* Moss_GPUBufferCreate(Moss_GPUDevice* device, const Moss_GPUBufferDesc* desc);
MOSS_API void Moss_GPUBufferUpload(Moss_GPUDevice* device, Moss_GPUBuffer* buffer, const void* data, uint64_t size, uint64_t offset);
MOSS_API void* Moss_GPUBufferMap(Moss_GPUDevice* device, Moss_GPUBuffer* buffer);
MOSS_API void Moss_GPUBufferUnmap(Moss_GPUDevice* device, Moss_GPUBuffer* buffer);
MOSS_API void Moss_GPUBufferDestroy(Moss_GPUDevice* device, Moss_GPUBuffer* buffer);
MOSS_API bool Moss_AcquireGPUSwapchainTexture(Moss_GPUDevice* device, Moss_Window* window, Moss_GPUTexture** texture);
MOSS_API EGPUSwapchainResult Moss_AcquireGPUSwapchainTextureStatus(Moss_GPUDevice* device, Moss_Window* window, Moss_GPUTexture** texture);
MOSS_API ETextureFormat Moss_GetGPUSwapchainTextureFormat(Moss_GPUDevice* device, Moss_Window* window);
MOSS_API bool Moss_SetGPUSwapchainParameters(Moss_GPUDevice* device, Moss_Window* window, uint32_t width, uint32_t height, EGPUPresentMode present_mode);
MOSS_API EGPUSwapchainResult Moss_ResizeGPUSwapchain(Moss_GPUDevice* device, Moss_Window* window, uint32_t width, uint32_t height);
MOSS_API bool Moss_WaitAndAcquireGPUSwapchainTexture(Moss_GPUDevice* device, Moss_Window* window, Moss_GPUTexture** texture, uint64_t timeout_ns);
MOSS_API EGPUSwapchainResult Moss_WaitAndAcquireGPUSwapchainTextureStatus(Moss_GPUDevice* device, Moss_Window* window, Moss_GPUTexture** texture, uint64_t timeout_ns);
MOSS_API bool Moss_WaitForGPUSwapchain(Moss_GPUDevice* device, Moss_Window* window, uint64_t timeout_ns);
MOSS_API bool Moss_WindowSupportsGPUSwapchainComposition(Moss_Window* window);
MOSS_API bool Moss_PresentGPUSwapchain(Moss_GPUDevice* device, Moss_Window* window, Moss_GPUTexture* texture);
MOSS_API EGPUSwapchainResult Moss_PresentGPUSwapchainStatus(Moss_GPUDevice* device, Moss_Window* window, Moss_GPUTexture* texture);
MOSS_API void Moss_GPUClear(Moss_GPUCommandBuffer* cmd, float r, float g, float b, float a);
MOSS_API Moss_GPUSampler* Moss_GPUSamplerCreate(Moss_GPUDevice* renderer, const Moss_GPUSamplerDesc* desc);
MOSS_API void Moss_GPUSamplerDestroy(Moss_GPUDevice* device, Moss_GPUSampler* sampler);
MOSS_API Moss_Texture* Moss_TextureCreate(Moss_GPUDevice* device, const Moss_TextureDesc* desc);
MOSS_API void Moss_TextureDestroy(Moss_GPUDevice* device, Moss_Texture* texture);
MOSS_API void Moss_TextureUpload(Moss_GPUDevice* device, Moss_Texture* texture, const Moss_TextureUploadDesc* desc);
MOSS_API void Moss_TextureGenerateMips(Moss_GPUDevice* device, Moss_Texture* texture);
MOSS_API uint32_t Moss_GPUGetExternalFeatureMask(void);
MOSS_API bool Moss_GPUExternalFeatureAvailable(Moss_GPUExternalFeature feature);
MOSS_API void Moss_GPUGetExternalFeatures(Moss_GPUExternalFeatures* out_features);
MOSS_API const char* Moss_GPUExternalFeatureName(Moss_GPUExternalFeature feature);
MOSS_API void Moss_FSR1BuildEasuConstants(uint32_t out_constants16[16], float input_viewport_width, float input_viewport_height, float input_width, float input_height, float output_width, float output_height);
MOSS_API float Moss_FSR2GetUpscaleRatio(Moss_FSR2QualityMode quality);
MOSS_API bool Moss_FSR2GetRenderResolution(uint32_t display_width, uint32_t display_height, Moss_FSR2QualityMode quality, Moss_FSR2RenderResolution* out_resolution);
MOSS_API int32_t Moss_FSR2GetJitterPhaseCount(int32_t render_width, int32_t display_width);
MOSS_API bool Moss_FSR2GetJitterOffset(float* out_x, float* out_y, int32_t index, int32_t phase_count);
MOSS_API size_t Moss_MeshGenerateVertexRemap(uint32_t* destination, const uint32_t* indices, size_t index_count, const void* vertices, size_t vertex_count, size_t vertex_size);
MOSS_API bool Moss_MeshOptimizeVertexCache(uint32_t* destination, const uint32_t* indices, size_t index_count, size_t vertex_count);
MOSS_API bool Moss_MeshOptimizeOverdraw(uint32_t* destination, const uint32_t* indices, size_t index_count, const float* vertex_positions, size_t vertex_count, size_t vertex_positions_stride, float threshold);
MOSS_API size_t Moss_MeshOptimizeVertexFetch(void* destination, uint32_t* indices, size_t index_count, const void* vertices, size_t vertex_count, size_t vertex_size);
MOSS_API bool Moss_SMOLVEncode(const void* spirv_data, size_t spirv_size, void* out_smolv_data, size_t* inout_smolv_size, uint32_t flags);
MOSS_API size_t Moss_SMOLVGetDecodedSize(const void* smolv_data, size_t smolv_size);
MOSS_API bool Moss_SMOLVDecode(const void* smolv_data, size_t smolv_size, void* out_spirv_data, size_t spirv_size, uint32_t flags);
MOSS_API bool Moss_GPUVulkanMemoryAllocatorAvailable(void);
MOSS_API Moss_GPUClothSolver* Moss_GPUClothSolverCreate(Moss_GPUDevice* device, const Moss_GPUClothSolverDesc* desc);
MOSS_API void Moss_GPUClothSolverDestroy(Moss_GPUDevice* device, Moss_GPUClothSolver* solver);
MOSS_API bool Moss_GPUClothSolverDispatch(Moss_GPUCommandBuffer* cmd, Moss_GPUClothSolver* solver, const Moss_GPUClothDispatchDesc* desc);
MOSS_API Moss_GPUCommandBuffer* Moss_GPUCommandBufferBegin(Moss_GPUDevice* device, ECommandQueue queue);
MOSS_API void Moss_GPUCommandBufferEnd(Moss_GPUCommandBuffer* cmd);
MOSS_API void Moss_GPUCommandBufferSubmit(Moss_GPUDevice* device, Moss_GPUCommandBuffer* cmd);
MOSS_API void Moss_CmdBeginRenderPass(Moss_GPUCommandBuffer* cmd, Moss_Framebuffer* fb);
MOSS_API void Moss_CmdEndRenderPass(Moss_GPUCommandBuffer* cmd);
MOSS_API void Moss_CmdBindPipeline(Moss_GPUCommandBuffer* cmd, Moss_PipelineState* pipeline);
MOSS_API void Moss_CmdBindResourceSet(Moss_GPUCommandBuffer* cmd, uint32_t set_index, Moss_ResourceSet* set);
MOSS_API void Moss_CmdBindVertexBuffer(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* buffer, uint64_t offset);
MOSS_API void Moss_CmdBindIndexBuffer(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* buffer, uint64_t offset);
MOSS_API void Moss_CmdDraw(Moss_GPUCommandBuffer* cmd, uint32_t vertex_count, uint32_t first_vertex);
MOSS_API void Moss_CmdDrawIndexed(Moss_GPUCommandBuffer* cmd, uint32_t index_count, uint32_t first_index, int32_t vertex_offset);
MOSS_API void Moss_CmdDispatch(Moss_GPUCommandBuffer* cmd, uint32_t x, uint32_t y, uint32_t z);
MOSS_API Moss_RGTexture* Moss_RGPassReadTexture(Moss_RGPass* pass, Moss_RGTexture* texture, EResourceState state);
MOSS_API Moss_RGTexture* Moss_RGPassWriteTexture(Moss_RGPass* pass, Moss_RGTexture* texture, EResourceState state);
MOSS_API int Moss_GPUDeviceSupportsAsyncCompute(Moss_GPUDevice* renderer);
MOSS_API void Moss_TextureSetResidency(Moss_Texture* texture, EResidencyState state);
MOSS_API Moss_Shader* Moss_ShaderCreate(Moss_GPUDevice* device, const Moss_ShaderDesc* desc);
MOSS_API void Moss_ShaderDestroy(Moss_GPUDevice* device, Moss_Shader* shader);
MOSS_API Moss_BindlessHandle Moss_BindlessRegisterTexture(Moss_GPUDevice* renderer, void* texture);
MOSS_API Moss_BindlessHandle Moss_BindlessRegisterBuffer(Moss_GPUDevice* renderer, Moss_GPUBuffer* buffer);
MOSS_API void Moss_BindlessUnregister(Moss_GPUDevice* renderer, Moss_BindlessHandle handle);
MOSS_API Moss_PipelineState* Moss_PipelineCreate(Moss_GPUDevice* device, const Moss_PipelineDesc* desc);
MOSS_API void Moss_PipelineDestroy(Moss_GPUDevice* device, Moss_PipelineState* pipeline);
MOSS_API void Moss_WindowResizeVK(VkPhysicalDevice physicalDevice, VkDevice device, VkSwapchainKHR& swapchain, VkExtent2D& swapchainExtent, std::vector<VkFramebuffer>& swapchainFramebuffers, std::vector<VkImageView>& swapchainImageViews, VkImage& depthImage, VkDeviceMemory& depthImageMemory, VkImageView& depthImageView);
MOSS_API void Moss_GPUFatalError(Moss_Result result);
MOSS_API void Moss_SetGPUViewports(Moss_GPUCommandBuffer* cmd, const Moss_GPUViewport* viewports, uint32_t count);
MOSS_API void Moss_SetGPUScissors(Moss_GPUCommandBuffer* cmd, const Moss_Rect* scissors, uint32_t count);
MOSS_API void Moss_ClearGPUTexture(Moss_GPUCommandBuffer* cmd, Moss_GPUTexture* texture, const float color[4]);
MOSS_API void Moss_ClearGPUDepthStencilTexture(Moss_GPUCommandBuffer* cmd, Moss_GPUTexture* texture, float depth, uint8_t stencil);
MOSS_API void Moss_DrawGPUIndexedPrimitivesIndirectCount(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* indirect_buffer, uint64_t offset, Moss_GPUBuffer* count_buffer, uint64_t count_offset, uint32_t max_draw_count, uint32_t stride);
MOSS_API void Moss_DrawGPUPrimitivesIndirectCount(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* indirect_buffer, uint64_t offset, Moss_GPUBuffer* count_buffer, uint64_t count_offset, uint32_t max_draw_count, uint32_t stride);
MOSS_API void Moss_SetGPUDepthBias(Moss_GPUCommandBuffer* cmd, float constant_factor, float clamp, float slope_factor);
MOSS_API Moss_GPUSemaphore* Moss_CreateGPUSemaphore(Moss_GPUDevice* device);
MOSS_API void Moss_ReleaseGPUSemaphore(Moss_GPUDevice* device, Moss_GPUSemaphore* semaphore);
MOSS_API void Moss_SignalGPUSemaphoreValue(Moss_GPUDevice* device, Moss_GPUSemaphore* semaphore, uint64_t value);
MOSS_API bool Moss_WaitGPUSemaphoreValue(Moss_GPUDevice* device, Moss_GPUSemaphore* semaphore, uint64_t value, uint64_t timeout_ns);
MOSS_API void Moss_SubmitGPUCommandBufferWithSemaphores(Moss_GPUDevice* device, Moss_GPUCommandBuffer* cmd, Moss_GPUSemaphore* const* wait_semaphores, const uint64_t* wait_values, uint32_t wait_count, Moss_GPUSemaphore* const* signal_semaphores, const uint64_t* signal_values, uint32_t signal_count);
MOSS_API uint32_t Moss_GetGPUBindlessCapacity(Moss_GPUDevice* device, Moss_ShaderResourceType type);
MOSS_API Moss_GPUPipelineCache* Moss_CreateGPUPipelineCache(Moss_GPUDevice* device, const void* initial_data, size_t initial_size);
MOSS_API size_t Moss_GetGPUPipelineCacheData(Moss_GPUDevice* device, Moss_GPUPipelineCache* cache, void* out_data, size_t out_size);
MOSS_API void Moss_ReleaseGPUPipelineCache(Moss_GPUDevice* device, Moss_GPUPipelineCache* cache);
```

### `Moss_GUI.h`

[Open source header](https://github.com/TxbiG/Moss/blob/main/include/Moss/Moss_GUI.h)

```cpp
MOSS_API GuiContext* CreateContext(Moss_FontAtlas* shared_font_atlas = NULL);
MOSS_API void DestroyContext(GuiContext* ctx = NULL);
MOSS_API GuiContext* GetCurrentContext();
MOSS_API void SetCurrentContext(GuiContext* ctx);
MOSS_API bool BeginWindow(const char* name, bool* p_open = NULL, Moss_GuiWindowFlags flags = 0);
MOSS_API void EndWindow();
MOSS_API bool BeginWindowChild(const char* str_id, const Float2& size = Float2(0, 0), Moss_GuiChildFlags child_flags = 0, Moss_GuiWindowFlags window_flags = 0);
MOSS_API bool BeginWindowChild(uint32_t id, const Float2& size = Float2(0, 0), Moss_GuiChildFlags child_flags = 0, Moss_GuiWindowFlags window_flags = 0);
MOSS_API void EndWindowChild();
MOSS_API void BeginContainer(Rect& rect);
MOSS_API void EndContainer();
MOSS_API void BeginVContainer(Rect& rect, float spacing = 50.0f);
MOSS_API void EndVContainer();
MOSS_API void BeginHContainer(Rect& rect, float spacing = 50.0f);
MOSS_API void EndHContainer();
MOSS_API void BeginCenterContainer(Rect& rect);
MOSS_API void EndCenterContainer();
MOSS_API void BeginMarginContainer(float leftMargin = 4.0f, float rightMargin = 4.0f, float topMargin = 4.0f, float bottomMargin = 4.0f);
MOSS_API void EndMarginContainer();
MOSS_API void BeginVScrollContainer();
MOSS_API void EndVScrollContainer();
MOSS_API void BeginHScrollContainer();
MOSS_API void EndHScrollContainer();
MOSS_API void BeginGridContainer();
MOSS_API void EndGridContainer();
MOSS_API void ColorRect(Color color, const Float2& size);
MOSS_API void Text(const char* fmt, ...);
MOSS_API void RichText(const char* label);
MOSS_API void ProgressBar();
MOSS_API bool TextLink(const char* label);
MOSS_API void TextLinkOpenURL(const char* label, const char* url = NULL);
MOSS_API bool Button(const char* label, const Rect* rectPtr, const Mat44& vpMatrix, float defaultWidth = 100.0f, float defaultHeight = 24.0f);
MOSS_API bool ToggleSwitch(bool active);
MOSS_API bool RadioButton(const char* label, bool active);
MOSS_API bool Checkbox(const char* label, bool* value, const Rect* rectPtr, const Mat44& vpMatrix, float defaultSize = 20.0f);
MOSS_API void SliderFloat(float min_value, float max_value, float value);
MOSS_API void SliderDouble(double min_value, double max_value, double value);
MOSS_API void SliderInt(int min_value, int max_value, int value);
MOSS_API void SliderFloat2(Float2 min_value, Float2 max_value, Float2 value);
MOSS_API void SliderDouble2(Double2 min_value, Double2 max_value, Double2 value);
MOSS_API void SliderInt2(Int2 min_value, Int2 max_value, Int2 value);
MOSS_API void SliderFloat3(Float3 min_value, Float3 max_value, Float3 value);
MOSS_API void SliderDouble3(Double3 min_value, Double3 max_value, Double3 value);
MOSS_API void SliderInt3(Int3 min_value, Int3 max_value, Int3 value);
MOSS_API void SliderFloat4(Float4 min_value, Float4 max_value, Float4 value);
MOSS_API void SliderDouble4(Double4 min_value, Double4 max_value, Double4 value);
MOSS_API void SliderInt4(Int4 min_value, Int4 max_value, Int4 value);
MOSS_API bool DragFloat(const char* label, float* value, float v_speed = 1.0f, float v_min = 0.0f, float v_max = 0.0f, const char* format = "%.3f", Moss_GuiSliderFlags flags = 0);
MOSS_API bool DragFloat2(const char* label, Float2 value, float v_speed = 1.0f, float v_min = 0.0f, float v_max = 0.0f, const char* format = "%.3f", Moss_GuiSliderFlags flags = 0);
MOSS_API bool DragFloat3(const char* label, Float3 value, float v_speed = 1.0f, float v_min = 0.0f, float v_max = 0.0f, const char* format = "%.3f", Moss_GuiSliderFlags flags = 0);
MOSS_API bool DragFloat4(const char* label, Float4 value, float v_speed = 1.0f, float v_min = 0.0f, float v_max = 0.0f, const char* format = "%.3f", Moss_GuiSliderFlags flags = 0);
MOSS_API bool DragFloatRange2(const char* label, float* v_current_min, float* v_current_max, float v_speed = 1.0f, float v_min = 0.0f, float v_max = 0.0f, const char* format = "%.3f", const char* format_max = NULL, Moss_GuiSliderFlags flags = 0);
MOSS_API bool DragInt(const char* label, int* v, float v_speed = 1.0f, int v_min = 0, int v_max = 0, const char* format = "%d", Moss_GuiSliderFlags flags = 0);
MOSS_API bool DragInt2(const char* label, int v[2], float v_speed = 1.0f, int v_min = 0, int v_max = 0, const char* format = "%d", Moss_GuiSliderFlags flags = 0);
MOSS_API bool DragInt3(const char* label, int v[3], float v_speed = 1.0f, int v_min = 0, int v_max = 0, const char* format = "%d", Moss_GuiSliderFlags flags = 0);
MOSS_API bool DragInt4(const char* label, int v[4], float v_speed = 1.0f, int v_min = 0, int v_max = 0, const char* format = "%d", Moss_GuiSliderFlags flags = 0);
MOSS_API bool DragIntRange2(const char* label, int* v_current_min, int* v_current_max, float v_speed = 1.0f, int v_min = 0, int v_max = 0, const char* format = "%d", const char* format_max = NULL, Moss_GuiSliderFlags flags = 0);
MOSS_API bool DragScalar(const char* label, Moss_GuiDataType data_type, void* p_data, float v_speed = 1.0f, const void* p_min = NULL, const void* p_max = NULL, const char* format = NULL, Moss_GuiSliderFlags flags = 0);
MOSS_API bool DragScalarN(const char* label, Moss_GuiDataType data_type, void* p_data, int components, float v_speed = 1.0f, const void* p_min = NULL, const void* p_max = NULL, const char* format = NULL, Moss_GuiSliderFlags flags = 0);
MOSS_API bool InputText(const char* label, char* buf, size_t buf_size, Moss_GuiInputTextFlags flags = 0, Moss_GuiInputTextCallback callback = NULL, void* user_data = NULL);
MOSS_API bool InputTextMultiline(const char* label, char* buf, size_t buf_size, const Float2& size = Float2(0, 0), Moss_GuiInputTextFlags flags = 0, Moss_GuiInputTextCallback callback = NULL, void* user_data = NULL);
MOSS_API bool InputTextWithHint(const char* label, const char* hint, char* buf, size_t buf_size, Moss_GuiInputTextFlags flags = 0, Moss_GuiInputTextCallback callback = NULL, void* user_data = NULL);
MOSS_API bool InputFloat(const char* label, float* v, float step = 0.0f, float step_fast = 0.0f, const char* format = "%.3f", Moss_GuiInputTextFlags flags = 0);
MOSS_API bool InputFloat2(const char* label, float v[2], const char* format = "%.3f", Moss_GuiInputTextFlags flags = 0);
MOSS_API bool InputFloat3(const char* label, float v[3], const char* format = "%.3f", Moss_GuiInputTextFlags flags = 0);
MOSS_API bool InputFloat4(const char* label, float v[4], const char* format = "%.3f", Moss_GuiInputTextFlags flags = 0);
MOSS_API bool InputInt(const char* label, int* v, int step = 1, int step_fast = 100, Moss_GuiInputTextFlags flags = 0);
MOSS_API bool InputInt2(const char* label, int v[2], Moss_GuiInputTextFlags flags = 0);
MOSS_API bool InputInt3(const char* label, int v[3], Moss_GuiInputTextFlags flags = 0);
MOSS_API bool InputInt4(const char* label, int v[4], Moss_GuiInputTextFlags flags = 0);
MOSS_API bool InputDouble(const char* label, double* v, double step = 0.0, double step_fast = 0.0, const char* format = "%.6f", Moss_GuiInputTextFlags flags = 0);
MOSS_API bool InputScalar(const char* label, Moss_GuiDataType data_type, void* p_data, const void* p_step = NULL, const void* p_step_fast = NULL, const char* format = NULL, Moss_GuiInputTextFlags flags = 0);
MOSS_API bool InputScalarN(const char* label, Moss_GuiDataType data_type, void* p_data, int components, const void* p_step = NULL, const void* p_step_fast = NULL, const char* format = NULL, Moss_GuiInputTextFlags flags = 0);
```

### `Moss_Network.h`

[Open source header](https://github.com/TxbiG/Moss/blob/main/include/Moss/Moss_Network.h)

```cpp
MOSS_API int Moss_Init_Network(void);
MOSS_API int enet_initialize_with_callbacks(const ENetCallbacks * inits);
MOSS_API void Moss_TerminateNetwork(void);
MOSS_API void enet_list_clear(ENetList*);
MOSS_API ENetListIterator enet_list_insert(ENetListIterator, void*);
MOSS_API void * enet_list_remove(ENetListIterator);
MOSS_API ENetListIterator enet_list_move(ENetListIterator, void*, void*);
MOSS_API size_t enet_list_size(ENetList*);
MOSS_API uint32_t enet_time_get(void);
MOSS_API void enet_time_set(uint32_t);
MOSS_API ENetSocket enet_socket_create(ENetAddressType adress_type, ENetSocketType socket_type);
MOSS_API int enet_socket_bind(ENetSocket, const ENetAddress*);
MOSS_API int enet_socket_get_address(ENetSocket, ENetAddress*);
MOSS_API int enet_socket_listen(ENetSocket, int);
MOSS_API ENetSocket enet_socket_accept(ENetSocket, ENetAddress *);
MOSS_API int enet_socket_connect(ENetSocket, const ENetAddress *);
MOSS_API int enet_socket_send(ENetSocket, const ENetAddress *, const ENetBuffer *, size_t);
MOSS_API int enet_socket_receive(ENetSocket, ENetAddress *, ENetBuffer *, size_t);
MOSS_API int enet_socket_wait(ENetSocket, uint32_t*, uint32_t);
MOSS_API int enet_socket_set_option(ENetSocket, ENetSocketOption, int);
MOSS_API int enet_socket_get_option(ENetSocket, ENetSocketOption, int *);
MOSS_API int enet_socket_shutdown(ENetSocket, ENetSocketShutdown);
MOSS_API void enet_socket_destroy(ENetSocket);
MOSS_API int enet_socketset_select(ENetSocket, ENetSocketSet *, ENetSocketSet *, uint32_t);
MOSS_API int enet_address_set_host_ip(ENetAddress* address, const char* hostName);
MOSS_API int enet_address_set_host(ENetAddress* address, ENetAddressType type, const char* hostName);
MOSS_API int enet_address_get_host_ip(const ENetAddress* address, char* hostName, size_t nameLength);
MOSS_API int enet_address_get_host(const ENetAddress* address, char* hostName, size_t nameLength);
MOSS_API int enet_address_equal_host(const ENetAddress* firstAddress, const ENetAddress * secondAddress);
MOSS_API int enet_address_is_broadcast(const ENetAddress* address);
MOSS_API void enet_address_build_any(ENetAddress* address, ENetAddressType type);
MOSS_API void enet_address_build_loopback(ENetAddress* address, ENetAddressType type);
MOSS_API void enet_address_convert_ipv6(ENetAddress* address);
MOSS_API uint32_t enet_crc32(const ENetBuffer*, size_t);
MOSS_API ENetHost* enet_host_create(ENetAddressType type, const ENetAddress*, size_t, size_t, uint32_t, uint32_t);
MOSS_API void enet_host_destroy(ENetHost*);
MOSS_API int enet_host_check_events(ENetHost*, ENetEvent*);
MOSS_API int enet_host_service(ENetHost*, ENetEvent*, uint32_t);
MOSS_API void enet_host_flush(ENetHost*);
MOSS_API void enet_host_broadcast(ENetHost*, uint8_t, ENetPacket *);
MOSS_API void enet_host_compress(ENetHost*, const ENetCompressor*);
MOSS_API int enet_host_compress_with_range_coder(ENetHost * host);
MOSS_API void enet_host_channel_limit(ENetHost*, size_t);
MOSS_API void enet_host_bandwidth_limit(ENetHost*, uint32_t, uint32_t);
MOSS_API void enet_host_bandwidth_throttle(ENetHost*);
MOSS_API uint32_t enet_host_random_seed(void);
MOSS_API uint32_t enet_host_random(ENetHost*);
MOSS_API ENetPeer* enet_host_connect(ENetHost*, const ENetAddress*, size_t, uint32_t);
MOSS_API int enet_peer_send(ENetPeer*, uint8_t, ENetPacket*);
MOSS_API void enet_peer_ping(ENetPeer*);
MOSS_API void enet_peer_ping_interval(ENetPeer*, uint32_t);
MOSS_API void enet_peer_timeout(ENetPeer*, uint32_t, uint32_t, uint32_t);
MOSS_API void enet_peer_reset(ENetPeer*);
MOSS_API void enet_peer_disconnect(ENetPeer*, uint32_t);
MOSS_API void enet_peer_disconnect_now(ENetPeer*, uint32_t);
MOSS_API void enet_peer_disconnect_later(ENetPeer*, uint32_t);
MOSS_API void enet_peer_throttle_configure(ENetPeer*, uint32_t, uint32_t, uint32_t);
MOSS_API int enet_peer_throttle(ENetPeer *, uint32_t);
MOSS_API void enet_peer_reset_queues(ENetPeer *);
MOSS_API int enet_peer_has_outgoing_commands(ENetPeer*);
MOSS_API void enet_peer_setup_outgoing_command(ENetPeer*, ENetOutgoingCommand*);
MOSS_API ENetOutgoingCommand* enet_peer_queue_outgoing_command(ENetPeer*, const ENetProtocol*, ENetPacket*, uint32_t, uint16_t);
MOSS_API ENetIncomingCommand* enet_peer_queue_incoming_command(ENetPeer*, const ENetProtocol*, const void*, size_t, uint32_t, uint32_t);
MOSS_API ENetAcknowledgement* enet_peer_queue_acknowledgement(ENetPeer*, const ENetProtocol*, uint16_t);
MOSS_API void enet_peer_dispatch_incoming_unreliable_commands(ENetPeer*, ENetChannel*, ENetIncomingCommand*);
MOSS_API void enet_peer_dispatch_incoming_reliable_commands(ENetPeer*, ENetChannel*, ENetIncomingCommand*);
MOSS_API void enet_peer_on_connect(ENetPeer*);
MOSS_API void enet_peer_on_disconnect(ENetPeer*);
MOSS_API ENetPacket* enet_packet_create(const void*, size_t, uint32_t);
MOSS_API void enet_packet_destroy(ENetPacket*);
MOSS_API int enet_packet_resize(ENetPacket*, size_t);
MOSS_API ENetPacket* enet_peer_receive(ENetPeer*, uint8_t* channelID);
MOSS_API void* enet_range_coder_create(void);
MOSS_API void enet_range_coder_destroy(void*);
MOSS_API size_t enet_range_coder_compress(void*, const ENetBuffer*, size_t, size_t, uint8_t*, size_t);
MOSS_API size_t enet_range_coder_decompress(void*, const uint8_t*, size_t, uint8_t*, size_t);
MOSS_API size_t enet_protocol_command_size(uint8_t);
MOSS_API void* enet_packet_get_data(const ENetPacket*);
MOSS_API void* enet_packet_get_user_data(const ENetPacket*);
MOSS_API void enet_packet_set_user_data(ENetPacket*, void* userData);
MOSS_API int enet_packet_get_length(const ENetPacket*);
MOSS_API void enet_packet_set_acknowledge_callback(ENetPacket*, ENetPacketAcknowledgedCallback);
MOSS_API void enet_packet_set_free_callback(ENetPacket*, ENetPacketFreeCallback);
MOSS_API int enet_packet_check_references(const ENetPacket*);
MOSS_API void enet_packet_dispose(ENetPacket*);
MOSS_API uint32_t enet_host_get_peers_count(const ENetHost*);
MOSS_API uint32_t enet_host_get_packets_sent(const ENetHost*);
MOSS_API uint32_t enet_host_get_packets_received(const ENetHost*);
MOSS_API uint32_t enet_host_get_bytes_sent(const ENetHost*);
MOSS_API uint32_t enet_host_get_bytes_received(const ENetHost*);
MOSS_API void enet_host_set_max_duplicate_peers(ENetHost*, uint16_t);
MOSS_API void enet_host_set_intercept_callback(ENetHost*, ENetInterceptCallback);
MOSS_API void enet_host_set_checksum_callback(ENetHost*, ENetChecksumCallback);
MOSS_API uint32_t enet_peer_get_id(const ENetPeer*);
MOSS_API int enet_peer_get_ip(const ENetPeer*, char*, size_t);
MOSS_API uint16_t enet_peer_get_port(const ENetPeer*);
MOSS_API uint32_t enet_peer_get_mtu(const ENetPeer*);
MOSS_API ENetPeerState enet_peer_get_state(const ENetPeer*);
MOSS_API uint32_t enet_peer_get_rtt(const ENetPeer*);
MOSS_API uint32_t enet_peer_get_last_rtt(const ENetPeer* peer);
MOSS_API uint32_t enet_peer_get_lastsendtime(const ENetPeer*);
MOSS_API uint32_t enet_peer_get_lastreceivetime(const ENetPeer*);
MOSS_API float enet_peer_get_packets_throttle(const ENetPeer*);
MOSS_API void* enet_peer_get_data(const ENetPeer*);
MOSS_API void enet_peer_set_data(ENetPeer*, const void*);
MOSS_API ENetHost* enet_host_create_with_transport(const ENetAddress* address, size_t peerCount, size_t channelLimit, uint32_t incomingBandwidth, uint32_t outgoingBandwidth, ENetTransportType transport);
MOSS_API ENetPeer* enet_host_connect(ENetHost* host, const ENetAddress* address, size_t channelCount, uint32_t data);
MOSS_API ENetPeer* enet_host_connect_ex(ENetHost* host, const char* endpoint, size_t channelCount, uint32_t data);
MOSS_API uint32_t enet_host_transport_caps(ENetHost* host);
MOSS_API void enet_transport_handle_event(ENetHost* host, const ENetTransportEvent* tevent);
MOSS_API void enet_peer_reset(ENetPeer* peer);
MOSS_API void enet_peer_timeout(ENetPeer* peer);
MOSS_API void enet_peer_ping(ENetPeer* peer);
MOSS_API size_t enet_packet_encode(const NetPacket* packet, uint8* outBuffer, size_t bufferSize);
MOSS_API bool enet_packet_decode(NetPacket* outPacket, const uint8* buffer, size_t bufferSize);
MOSS_API void enet_fragment_send(ENetPeer* peer, uint8 channelId, const uint8* data, size_t size);
MOSS_API bool enet_fragment_receive(ENetPeer* peer, const NetPacket* fragment, NetPacket* outPacket);
MOSS_API void enet_bandwidth_update(ENetHost* host, float deltaTime);
MOSS_API size_t enet_peer_send_window(const ENetPeer* peer);
MOSS_API int enet_peer_create_channel(ENetPeer* peer, bool reliable, bool ordered);
MOSS_API void enet_peer_reset_channels(ENetPeer* peer);
MOSS_API void enet_peer_disconnect(ENetPeer* peer, uint32_t_t reason);
MOSS_API void enet_peer_disconnect_now(ENetPeer* peer);
MOSS_API void enet_peer_get_stats(const ENetPeer* peer, float* rtt, float* packetLoss, uint32_t_t* sent, uint32_t_t* received);
MOSS_API void enet_host_flush(ENetHost* host);
MOSS_API void enet_host_broadcast(ENetHost* host, uint8 channelId, const uint8* data, size_t size);
MOSS_API void enet_host_set_protocol_id(ENetHost* host, uint16_t_t protocolId);
MOSS_API uint16_t_t enet_host_get_protocol_id(const ENetHost* host);
MOSS_API ENetHost* enet_host_create_with_transport( const ENetAddress*, size_t peerCount, size_t channelLimit, uint32_t incomingBandwidth, uint32_t outgoingBandwidth, ENetTransportType transport );
MOSS_API ENetPeer* enet_host_connect( ENetHost*, const ENetAddress*, size_t, uint32_t );
MOSS_API ENetPeer* enet_host_connect_ex( ENetHost*, const char* endpoint, size_t, uint32_t );
MOSS_API uint32_t enet_host_transport_caps(ENetHost*);
MOSS_API ENetHttpServer* enet_http_server_start( const char* bindAddress, enet_uint16_t port);
MOSS_API void enet_http_server_stop(ENetHttpServer* server);
MOSS_API void enet_http_register_endpoint(ENetHttpServer* server, const char* path, ENetHttpEndpointHandler handler, void* userData);
MOSS_API ENetHost* enet_host_create_secure(const ENetAddress* address, size_t peerCount, size_t channelLimit, uint32_t incomingBandwidth, uint32_t outgoingBandwidth, ENetTransportType transport, const ENetTransportSecurity* security);
MOSS_API ENetHttpRequest* enet_http_request_async(const ENetHttpRequestDesc* desc,ENetHttpResponseCallback cb, void* userData);
MOSS_API void enet_http_request_cancel(ENetHttpRequest* request);
MOSS_API void enet_http_request_release(ENetHttpRequest* request);
MOSS_API void enet_host_set_connect_validator(ENetHost* host, ENetConnectValidator validator);
MOSS_API int enet_transport_send_stream(...);
MOSS_API int enet_transport_receive_stream(...);
MOSS_API void enet_transport_ws_close(ENetTransportPeer*, int code);
MOSS_API void enet_transport_webrtc_signal(ENetTransport*, ENetTransportPeer*, const ENetWebRTCSignal*);
MOSS_API void enet_host_set_webrtc_signal_callback(ENetHost*, ENetWebRTCSignalCallback, void*);
MOSS_API int enet_webrtc_handle_signal(ENetHost*, ENetPeer*, const ENetWebRTCSignal*);
```

### `Moss_Physics.h`

[Open source header](https://github.com/TxbiG/Moss/blob/main/include/Moss/Moss_Physics.h)

```cpp
MOSS_API void CollideShapeResult_FreeMembers(CollideShapeResult* result);
MOSS_API void CollisionEstimationResult_FreeMembers(CollisionEstimationResult* result);
MOSS_API BroadPhaseLayerInterface* BroadPhaseLayerInterfaceMask_Create(uint32_t numBroadPhaseLayers);
MOSS_API void BroadPhaseLayerInterfaceMask_ConfigureLayer(BroadPhaseLayerInterface* bpInterface, BroadPhaseLayer broadPhaseLayer, uint32_t groupsToInclude, uint32_t groupsToExclude);
MOSS_API BroadPhaseLayerInterface* BroadPhaseLayerInterfaceTable_Create(uint32_t numObjectLayers, uint32_t numBroadPhaseLayers);
MOSS_API void BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(BroadPhaseLayerInterface* bpInterface, ObjectLayer objectLayer, BroadPhaseLayer broadPhaseLayer);
MOSS_API ObjectLayerPairFilter* ObjectLayerPairFilterMask_Create(void);
MOSS_API ObjectLayer ObjectLayerPairFilterMask_GetObjectLayer(uint32_t group, uint32_t mask);
MOSS_API uint32_t ObjectLayerPairFilterMask_GetGroup(ObjectLayer layer);
MOSS_API uint32_t ObjectLayerPairFilterMask_GetMask(ObjectLayer layer);
MOSS_API ObjectLayerPairFilter* ObjectLayerPairFilterTable_Create(uint32_t numObjectLayers);
MOSS_API void ObjectLayerPairFilterTable_DisableCollision(ObjectLayerPairFilter* objectFilter, ObjectLayer layer1, ObjectLayer layer2);
MOSS_API void ObjectLayerPairFilterTable_EnableCollision(ObjectLayerPairFilter* objectFilter, ObjectLayer layer1, ObjectLayer layer2);
MOSS_API bool ObjectLayerPairFilterTable_ShouldCollide(ObjectLayerPairFilter* objectFilter, ObjectLayer layer1, ObjectLayer layer2);
MOSS_API ObjectVsBroadPhaseLayerFilter* ObjectVsBroadPhaseLayerFilterMask_Create(const BroadPhaseLayerInterface* broadPhaseLayerInterface);
MOSS_API ObjectVsBroadPhaseLayerFilter* ObjectVsBroadPhaseLayerFilterTable_Create( BroadPhaseLayerInterface* broadPhaseLayerInterface, uint32_t numBroadPhaseLayers, ObjectLayerPairFilter* objectLayerPairFilter, uint32_t numObjectLayers);
MOSS_API void DrawSettings_InitDefault(DrawSettings* settings);
MOSS_API void ContactManifold_GetWorldSpaceNormal(const ContactManifold* manifold, Vec3* result);
MOSS_API float ContactManifold_GetPenetrationDepth(const ContactManifold* manifold);
MOSS_API SubShapeID ContactManifold_GetSubShapeID1(const ContactManifold* manifold);
MOSS_API SubShapeID ContactManifold_GetSubShapeID2(const ContactManifold* manifold);
MOSS_API uint32_t ContactManifold_GetPointCount(const ContactManifold* manifold);
MOSS_API void ContactManifold_GetWorldSpaceContactPointOn1(const ContactManifold* manifold, uint32_t index, Vec3* result);
MOSS_API void ContactManifold_GetWorldSpaceContactPointOn2(const ContactManifold* manifold, uint32_t index, Vec3* result);
MOSS_API bool CollisionDispatch_CollideShapeVsShape( const Shape* shape1, const Shape* shape2, const Vec3* scale1, const Vec3* scale2, const Mat44* centerOfMassTransform1, const Mat44* centerOfMassTransform2, const CollideShapeSettings* collideShapeSettings, CollideShapeCollectorCallback* callback, void* userData, const ShapeFilter* shapeFilter);
MOSS_API bool CollisionDispatch_CastShapeVsShapeLocalSpace( const Vec3* direction, const Shape* shape1, const Shape* shape2, const Vec3* scale1InShape2LocalSpace, const Vec3* scale2, Mat44* centerOfMassTransform1InShape2LocalSpace, Mat44* centerOfMassWorldTransform2, const ShapeCastSettings* shapeCastSettings, CastShapeCollectorCallback* callback, void* userData, const ShapeFilter* shapeFilter);
MOSS_API bool CollisionDispatch_CastShapeVsShapeWorldSpace( const Vec3* direction, const Shape* shape1, const Shape* shape2, const Vec3* scale1, const Vec3* inScale2, const Mat44* centerOfMassWorldTransform1, const Mat44* centerOfMassWorldTransform2, const ShapeCastSettings* shapeCastSettings, CastShapeCollectorCallback* callback, void* userData, const ShapeFilter* shapeFilter);
MOSS_API Skeleton* Skeleton_Create(void);
MOSS_API void Skeleton_Destroy(Skeleton* skeleton);
MOSS_API uint32_t Skeleton_AddJoint(Skeleton* skeleton, const char* name);
MOSS_API uint32_t Skeleton_AddJoint2(Skeleton* skeleton, const char* name, int parentIndex);
MOSS_API uint32_t Skeleton_AddJoint3(Skeleton* skeleton, const char* name, const char* parentName);
MOSS_API int Skeleton_GetJointCount(const Skeleton* skeleton);
MOSS_API void Skeleton_GetJoint(const Skeleton* skeleton, int index, SkeletonJoint* joint);
MOSS_API int Skeleton_GetJointIndex(const Skeleton* skeleton, const char* name);
MOSS_API void Skeleton_CalculateParentJointIndices(Skeleton* skeleton);
MOSS_API bool Skeleton_AreJointsCorrectlyOrdered(const Skeleton* skeleton);
MOSS_API void Indexify(const TriangleList &inTriangles, VertexList &outVertices, IndexedTriangleList &outTriangles, float inVertexWeldDistance = 1.0e-4f);
MOSS_API void Deindexify(const VertexList &inVertices, const IndexedTriangleList &inTriangles, TriangleList &outTriangles);
```

### `Moss_Platform.h`

[Open source header](https://github.com/TxbiG/Moss/blob/main/include/Moss/Moss_Platform.h)

```cpp
MOSS_API Moss_Window* Moss_CreateWindow(const char* title, int width, int height, Moss_Monitor* monitor, Moss_Window* share);
MOSS_API void Moss_TerminateWindow(Moss_Window* window);
MOSS_API bool Moss_CreateMessageBox(const char* title, const char* message, Moss_MessageBoxFlags flags, Moss_Window* window);
MOSS_API bool Moss_ShouldWindowClose(Moss_Window* window);
MOSS_API void Moss_PollEvents(void);
MOSS_API int Moss_GetWindowWidth();
MOSS_API int Moss_GetWindowHeight();
MOSS_API void Moss_SetWindowTitle(Moss_Window* window, const char* title);
MOSS_API void Moss_SetWindowIcon(Moss_Window* window, Moss_Image image);
MOSS_API void Moss_CloseWindow(Moss_Window* window);
MOSS_API Moss_Monitor* Moss_MonitorGetPrimary();
MOSS_API Moss_Monitor* Moss_MonitorGetSecondary();
MOSS_API void Moss_MonitorGetPhysicalSize(Moss_Monitor* monitor, int* width_mm, int* height_mm);
MOSS_API void Moss_MonitorGetContentScale(Moss_Monitor* monitor, float* xscale, float* yscale);
MOSS_API void Moss_MonitorGetPosition(Moss_Monitor* monitor, int* x, int* y);
MOSS_API const char* Moss_MonitorGetName(Moss_Monitor* monitor);
MOSS_API void Moss_MonitorSetGammaRamp(Moss_Monitor* monitor, const Moss_GammaRamp* gammaRamp);
MOSS_API Moss_GammaRamp* Moss_MonitorGetGammaRamp(Moss_Monitor* monitor);
MOSS_API void Moss_MonitorSetGamma(Moss_Monitor* monitor, float gamma);
MOSS_API bool Moss_IsKeyPressed(Keyboard key);
MOSS_API bool Moss_IsReleased(Keyboard key);
MOSS_API bool Moss_IsKeyJustPressed(Keyboard key);
MOSS_API bool Moss_IsKeyJustReleased(Keyboard key);
MOSS_API Keyboard Moss_InputGetKey();
MOSS_API bool Moss_IsMousePressed(Mouse button);
MOSS_API bool Moss_IsMouseReleased(Mouse button);
MOSS_API bool Moss_IsMouseJustPressed(Mouse button);
MOSS_API bool Moss_IsMouseJustReleased(Mouse button);
MOSS_API Mouse Moss_InputGetMouseButton();
MOSS_API void Moss_GetMousePosition(int* x, int* y);
MOSS_API void Moss_SetMousePosition(int x, int y);
MOSS_API void Moss_SetMouseVisible(bool visible);
MOSS_API int Moss_GetNumGamepads(void);
MOSS_API Moss_Gamepad* Moss_OpenGamepad(Moss_GamepadID id);
MOSS_API void Moss_CloseGamepad(Moss_Gamepad* gp);
MOSS_API bool Moss_GamepadConnected(Moss_Gamepad* gp);
MOSS_API void Moss_UpdateGamepads(void);
MOSS_API bool Moss_IsGamepadButtonPressed(Moss_Gamepad* gp, Moss_GamepadButton button);
MOSS_API bool Moss_IsGamepadButtonJustPressed(Moss_Gamepad* gp, Moss_GamepadButton button);
MOSS_API bool Moss_IsGamepadButtonJustReleased(Moss_Gamepad* gp, Moss_GamepadButton button);
MOSS_API float Moss_GetGamepadAxis(Moss_Gamepad* gp, GamepadAxis axis);
MOSS_API bool Moss_GetGamepadOrientation(Moss_Gamepad* gp, float* w, float* x, float* y, float* z);
MOSS_API void Moss_GetGamepadPlayerSpaceGyro(Moss_Gamepad* gp, float* outX, float* outY, const float yawRelaxFactor = 1.41f);
MOSS_API void Moss_GetGamepadWorldSpaceGyro(Moss_Gamepad* gp, float* outX, float* outY, const float sideReductionThreshold = 0.125f);
MOSS_API void Moss_ResetGamepadCalibration(Moss_Gamepad* gp);
MOSS_API bool Moss_RumbleGamepad(Moss_Gamepad* gp, uint16_t low, uint16_t high, uint32_t duration_ms);
MOSS_API bool Moss_RumbleGamepadTriggers(Moss_Gamepad* gp, uint16_t left, uint16_t right, uint32_t duration_ms);
MOSS_API bool Moss_SetGamepadLED(Moss_Gamepad* gp, uint8_t r, uint8_t g, uint8_t b);
MOSS_API const char* Moss_GetGamepadName(Moss_Gamepad* gp);
MOSS_API Moss_GamepadID Moss_GetGamepadID(Moss_Gamepad* gp);
MOSS_API int Moss_GetGamepadPlayerIndex(Moss_Gamepad* gp);
MOSS_API Moss_PowerState Moss_GetGamepadPowerInfo(Moss_Gamepad* gp, int* percent);
MOSS_API int Moss_GetNumGamepadTouchpads(Moss_Gamepad* gp);
MOSS_API int Moss_GetNumGamepadTouchpadFingers(Moss_Gamepad* gp);
MOSS_API bool Moss_GetGamepadTouchpadFinger(Moss_Gamepad* gp, int pad, int finger, bool* down, float* x, float* y, float* pressure);
MOSS_API const char* Moss_GetGamepadMapping(Moss_Gamepad* gp);
MOSS_API bool Moss_SetGamepadMapping(Moss_Gamepad* gp, const char* mapping);
MOSS_API void Moss_ReloadGamepadMappings(void);
MOSS_API Moss_GamepadButton Moss_InputGetGamepadButton();
MOSS_API GamepadAxis Moss_InputGetGamepadAxis();
MOSS_API Moss_PenDeviceType Moss_GetPenDeviceType(Moss_PenID instance_id);
MOSS_API const char* Moss_GetTouchDeviceName(Moss_TouchID touchID);
MOSS_API Moss_TouchID* Moss_GetTouchDevices(int* count);
MOSS_API Moss_TouchDeviceType Moss_GetTouchDeviceType(Moss_TouchID touchID);
MOSS_API Moss_Finger** Moss_GetTouchFingers(Moss_TouchID touchID, int* count);
MOSS_API bool Moss_GamepadHasSensor(Moss_Gamepad* gp, Moss_GamepadSensorType sensor);
MOSS_API bool Moss_SetGamepadSensorEnabled(Moss_Gamepad* gp, Moss_GamepadSensorType sensor, bool enabled);
MOSS_API bool Moss_GetGamepadSensorData(Moss_Gamepad* gp, Moss_GamepadSensorType sensor, Moss_GamepadSensorData* data);
MOSS_API Moss_Haptic* Moss_OpenHaptic(Moss_HapticID id);
MOSS_API void Moss_CloseHaptic(Moss_Haptic* haptic);
MOSS_API Moss_HapticID Moss_CreateHapticEffect(Moss_Haptic* haptic);
MOSS_API void Moss_DestroyHapticEffect(Moss_Haptic* haptic);
MOSS_API bool Moss_GetHapticEffectStatus(Moss_Haptic* haptic);
MOSS_API uint32_t Moss_GetHapticFeatures(Moss_Haptic* haptic);
MOSS_API Moss_Haptic* Moss_GetHapticFromID(Moss_Haptic* haptic);
MOSS_API Moss_HapticID* Moss_GetHapticID(Moss_Haptic* haptic);
MOSS_API const char* Moss_GetHapticName(Moss_Haptic* haptic);
MOSS_API const char* Moss_GetHapticNameForID(Moss_Haptic* haptic);
MOSS_API Moss_HapticID* Moss_GetHaptics(Moss_Haptic* haptic);
MOSS_API int Moss_GetMaxHapticEffects(Moss_Haptic* haptic);
MOSS_API int Moss_GetMaxHapticEffectsPlaying(Moss_Haptic* haptic);
MOSS_API int Moss_GetNumHapticAxes(Moss_Haptic* haptic);
MOSS_API bool Moss_HapticEffectSupported(Moss_Haptic* haptic);
MOSS_API bool Moss_HapticRumbleSupported(Moss_Haptic* haptic);
MOSS_API bool Moss_InitHapticRumble(Moss_Haptic* joystick);
MOSS_API bool Moss_IsJoystickHaptic(Moss_Gamepad* joystick);
MOSS_API bool Moss_IsMouseHaptic(void);
MOSS_API Moss_Haptic* Moss_OpenHapticFromJoystick(Moss_Gamepad* joystick);
MOSS_API Moss_Haptic* Moss_OpenHapticFromMouse(void);
MOSS_API bool Moss_PauseHaptic(Moss_Haptic* haptic);
MOSS_API bool Moss_PlayHapticRumble(Moss_Haptic* haptic, float strength, uint32_t length);
MOSS_API bool Moss_ResumeHaptic(Moss_Haptic* haptic);
MOSS_API bool Moss_RunHapticEffect(Moss_Haptic* haptic, uint32_t iterations);
MOSS_API bool Moss_SetHapticAutocenter(Moss_Haptic* haptic, int center);
MOSS_API bool Moss_SetHapticGain(Moss_Haptic* haptic, int gain);
MOSS_API bool Moss_StopHapticEffect(Moss_Haptic* haptic, Moss_HapticEffectID effect);
MOSS_API bool Moss_StopHapticEffects(Moss_Haptic* haptic);
MOSS_API bool Moss_StopHapticRumble(Moss_Haptic* haptic);
MOSS_API bool Moss_UpdateHapticEffect(Moss_Haptic* haptic, Moss_HapticEffectID effect, const Moss_HapticEffect* data);
MOSS_API int Moss_GetAvailableCPUCores(void);
MOSS_API int Moss_GetCPUCacheLineSize(void);
MOSS_API int Moss_GetSystemRAM(void);
MOSS_API bool Moss_OpenURL(const char* url);
MOSS_API Moss_Locale* Moss_GetLocale();
MOSS_API Moss_PowerState Moss_GetPowerInfo(int *seconds, int *percent);
MOSS_API bool Moss_IsProcessRunningByName(const char* executable_path);
MOSS_API void* Moss_LoadDynamicLibrary(const char* lib_path);
MOSS_API void* Moss_GetLibrarySymbol(void* handle, const char* symbol_name);
MOSS_API void Moss_UnloadDynamicLibrary(void* handle);
MOSS_API void Moss_SetFramebufferResizeCallback(Moss_FramebufferResizeCallback callback);
MOSS_API void Moss_SetWindowSizeCallback(Moss_WindowSizeCallback callback);
MOSS_API void Moss_SetWindowResizeCallback(Moss_WindowResizeCallback callback);
MOSS_API void Moss_SetWindowPositionCallback(Moss_WindowPositionCallback callback);
MOSS_API void Moss_SetWindowFocusCallback(Moss_WindowFocusCallback callback);
MOSS_API void Moss_SetWindowContentScaleCallback(Moss_WindowContentScaleCallback callback);
MOSS_API void Moss_SetMonitorCallback(Moss_MonitorCallback callback);
MOSS_API Moss_CameraID* Moss_GetCameras(int* count);
MOSS_API const char* Moss_GetCameraName(Moss_CameraID camera_id);
MOSS_API Moss_CameraPosition Moss_GetCameraPosition(Moss_CameraID camera_id);
MOSS_API const char* Moss_GetCurrentCameraDriver(void);
MOSS_API int Moss_GetNumCameraDrivers(void);
MOSS_API const Moss_CameraSpec* Moss_GetCameraSupportedFormats(Moss_CameraID camera_id, int* count);
MOSS_API Moss_Surface* Moss_AcquireCameraFrame(Moss_Capture* camera, uint64_t* timestamp_ns);
MOSS_API void Moss_ReleaseCameraFrame(Moss_Capture* camera, Moss_Surface* frame);
MOSS_API bool Moss_GetCameraFormat(Moss_Capture* camera, Moss_CameraSpec* out_spec);
MOSS_API Moss_CameraPermissionState Moss_GetCameraPermissionState(Moss_Capture* camera);
MOSS_API Moss_PropertiesID Moss_GetCameraProperties(Moss_Capture* camera);
MOSS_API void Moss_CloseCamera(Moss_Capture* camera);
MOSS_API Moss_CameraID Moss_GetCameraID(Moss_Capture* camera);
MOSS_API Moss_PropertiesID Moss_GetCameraProperties(Moss_Capture* camera);
MOSS_API Moss_Capture* Moss_OpenCapture(Moss_CameraID id, const Moss_CameraSpec* spec);
MOSS_API bool Moss_CopyFile(const char* src_path, const char* dst_path, bool overwrite);
MOSS_API bool Moss_CreateDirectory(const char* path, bool recursive);
MOSS_API bool Moss_RemovePath(const char* path, bool recursive);
MOSS_API bool Moss_RenamePath(const char* old_path, const char* new_path, bool overwrite);
MOSS_API bool Moss_GetPathInfo(const char* path, Moss_PathInfo* out_info);
MOSS_API bool Moss_GetCurrentDirectory(char* out_path, int max_len);
MOSS_API bool Moss_GetBasePath(char* out_path, int max_len);
MOSS_API bool Moss_GetUserFolder(Moss_UserFolder folder, char* out_path, int max_len);
MOSS_API bool Moss_GetPrefPath(const char* org_name, const char* app_name, char* out_path, int max_len);
MOSS_API bool Moss_EnumerateDirectory( const char* path, bool recursive, Moss_DirectoryIterateFn callback, void* user_data);
MOSS_API bool Moss_GlobDirectory(const char* pattern, Moss_DirectoryIterateFn callback, void* user_data);
MOSS_API void Moss_ShowFileDialogWithProperties(Moss_DialogFileCallback callback, void* userdata, Moss_Window* window, const Moss_DialogFileFilter* filters, int nfilters, const char* default_location);
MOSS_API void Moss_ShowOpenFolderDialog(Moss_DialogFileCallback callback, void* userdata, Moss_Window* window, const Moss_DialogFileFilter* filters, int nfilters, const char* default_location, bool allow_many);
MOSS_API void Moss_ShowOpenFileDialog(Moss_DialogFileCallback callback, void* userdata, Moss_Window* window, const char* default_location, bool allow_many);
MOSS_API void Moss_ShowSaveFileDialog(Moss_FileDialogType type, Moss_DialogFileCallback callback, void* userdata, Moss_PropertiesID props);
MOSS_API bool Moss_CloseStorage(Moss_Storage* storage);
MOSS_API bool Moss_CopyStorageFile(Moss_Storage* storage, const char *oldpath, const char* newpath);
MOSS_API bool Moss_CreateStorageDirectory(Moss_Storage* storage, const char* path);
MOSS_API bool Moss_EnumerateStorageDirectory(Moss_Storage* storage, const char* path, Moss_EnumerateDirectoryCallback callback, void* userdata);
MOSS_API bool Moss_GetStorageFileSize(Moss_Storage* storage, const char *path, uint64* length);
MOSS_API uint64_t Moss_GetStorageSpaceRemaining(Moss_Storage* storage);
MOSS_API bool Moss_GetStoragePathInfo(Moss_Storage* storage, const char *path, Moss_PathInfo* info);
MOSS_API char** Moss_GlobStorageDirectory(Moss_Storage* storage, const char* path, const char* pattern, Moss_GlobFlags flags, int* count);
MOSS_API Moss_Storage* Moss_OpenFileStorage(const char* path);
MOSS_API Moss_Storage* Moss_OpenStorage(const Moss_Storage* iface, void* userdata);
MOSS_API Moss_Storage* Moss_OpenTitleStorage(const char* override, Moss_PropertiesID props);
MOSS_API Moss_Storage* Moss_OpenUserStorage(const char* org, const char* app, Moss_PropertiesID props);
MOSS_API bool Moss_ReadStorageFile(Moss_Storage* storage, const char* path, void* destination, uint64 length);
MOSS_API bool Moss_RemoveStoragePath(Moss_Storage* storage, const char* path);
MOSS_API bool Moss_RenameStoragePath(Moss_Storage* storage, const char* oldpath, const char* newpath);
MOSS_API bool Moss_StorageReady(Moss_Storage* storage);
MOSS_API bool Moss_WriteStorageFile(Moss_Storage* storage, const char* path, const void* source, uint64 length);
MOSS_API Moss_Window* Moss_CreateEmbeddedWindow();
MOSS_API void Moss_Terminate_EmbeddedWindow();
MOSS_API void Moss_EmbeddedWindow_OnOrientationChanged(bool focused);
MOSS_API void Moss_EmbeddedWindow_OnResume();
MOSS_API void Moss_EmbeddedWindow_OnPause();
MOSS_API void Moss_MakeContextCurrent(Moss_Window* window);
MOSS_API void Moss_SwapBuffers();
MOSS_API void Moss_SwapBuffersInterval(int interval);
MOSS_API void* Moss_GetProcAddress(const char* procname);
MOSS_API VkResult Moss_CreateWindowSurface(Moss_Window* window, VkInstance vk_instance, const VkAllocationCallbacks* allocator, VkSurfaceKHR* vk_surface);
MOSS_API int Moss_VulkanSupported(void);
MOSS_API const char** Moss_GetRequiredInstanceExtensions(uint32_t* count);
MOSS_API void* Moss_GetInstanceProcAddress(VkInstance instance, const char* procname);
MOSS_API int Moss_GetPhysicalDevicePresentationSupport(Moss_Window* window, VkPhysicalDevice device, uint32_t queuefamily);
MOSS_API Moss_MetalView Moss_Metal_CreateView(Moss_Window *window);
MOSS_API void Moss_Metal_DestroyView(Moss_MetalView view);
MOSS_API void* Moss_Metal_GetLayer(Moss_MetalView view);
MOSS_API void Moss_Metal_Resize(Moss_MetalView handle, uint32_t width, uint32_t height);
```

### `Moss_Renderer.h`

[Open source header](https://github.com/TxbiG/Moss/blob/main/include/Moss/Moss_Renderer.h)

```cpp
MOSS_API Moss_Material* Moss_RendererCreateMaterial(Moss_Renderer* renderer, const Moss_MaterialDesc* desc);
MOSS_API void Moss_RendererDestroyMaterial(Moss_Renderer* renderer, Moss_Material* material);
MOSS_API int Moss_MaterialSetUniform(Moss_Material* material, const char* name, const void* data, uint32_t size);
MOSS_API int Moss_MaterialSetTexture(Moss_Material* material, MaterialTextureType type, Texture* texture, Moss_GPUSampler* sampler);
MOSS_API Moss_PipelineState* Moss_RendererCreatePipeline(Moss_Renderer* renderer, const Moss_PipelineDesc* desc);
MOSS_API void Moss_PipelineBind(Moss_PipelineState* pipeline);
MOSS_API void Moss_PipelineUnbind(Moss_PipelineState* pipeline);
MOSS_API void Moss_PipelineSetUniform(Moss_PipelineState* pipeline, const char* name, const void* data, uint32_t size);
MOSS_API void Moss_PipelineFlush(Moss_PipelineState* pipeline);
MOSS_API int Moss_PipelineValidate(Moss_PipelineState* pipeline);
MOSS_API Moss_ResourceSet* Moss_RendererCreateResourceSet(Moss_Renderer* renderer, const Moss_ResourceSetLayoutDesc* desc);
MOSS_API void Moss_RendererDestroyResourceSet(Moss_Renderer* renderer, Moss_ResourceSet* set);
MOSS_API void Moss_ResourceSetBindUniformBuffer(Moss_ResourceSet* set, uint32_t binding, Moss_GPUBuffer* buffer);
MOSS_API void Moss_ResourceSetBindUniformBufferRange(Moss_ResourceSet* set, uint32_t binding, Moss_GPUBuffer* buffer, size_t offset, size_t size);
MOSS_API void Moss_ResourceSetBindStorageBuffer(Moss_ResourceSet* set, uint32_t binding, Moss_GPUBuffer* buffer);
MOSS_API void Moss_ResourceSetBindTexture(Moss_ResourceSet* set, uint32_t binding, Texture* texture);
MOSS_API void Moss_ResourceSetBindSampler(Moss_ResourceSet* set, uint32_t binding, Moss_GPUSampler* sampler);
MOSS_API void Moss_ResourceSetBind(Moss_Renderer* renderer, Moss_PipelineState* pipeline, uint32_t set_index, Moss_ResourceSet* set);
MOSS_API Moss_Framebuffer* Moss_RendererCreateFramebuffer(Moss_Renderer* renderer, const Moss_FramebufferDesc* desc);
MOSS_API void Moss_RendererDestroyFramebuffer(Moss_Renderer* renderer, Moss_Framebuffer* framebuffer);
MOSS_API void Moss_FramebufferResize(Moss_Framebuffer* framebuffer, uint32_t width, uint32_t height);
MOSS_API void Moss_PostProcessExecute(Moss_Renderer* renderer, const Moss_PostProcessPass* pass);
MOSS_API void Moss_FramebufferBegin(Moss_Renderer* renderer, Moss_Framebuffer* framebuffer);
MOSS_API void Moss_FramebufferEnd(Moss_Renderer* renderer, Moss_Framebuffer* framebuffer);
MOSS_API void Moss_TextureBarrier(Moss_Renderer* renderer, Texture* texture, EResourceState old_state, EResourceState new_state);
MOSS_API Moss_Framebuffer* Moss_RendererGetBackbuffer(Moss_Renderer* renderer);
MOSS_API void Moss_RenderSubViewport(Moss_GPUCommandBuffer* cmd, const SubViewport* sub_viewport, Moss_SubViewportRecordFn record, void* user_data);
MOSS_API Moss_Renderer* Moss_RendererCreate(const Moss_RendererDesc* desc);
MOSS_API Moss_Renderer* Moss_CreateRenderer(Moss_Window* window);
MOSS_API void Moss_RendererDestroy(Moss_Renderer* renderer);
MOSS_API void Moss_TerminateRenderer(Moss_Renderer* renderer);
MOSS_API void Moss_RendererBeginFrame(Moss_Renderer* renderer);
MOSS_API void Moss_RendererBeginFrameEx(Moss_Renderer* renderer, const Moss_RenderFrameDesc* desc);
MOSS_API void Moss_RendererEndFrame(Moss_Renderer* renderer);
MOSS_API Moss_GPUDevice* Moss_RendererGetGPUDevice(Moss_Renderer* renderer);
MOSS_API Moss_Mesh* Moss_MeshLoadFBX(const char* file_path);
MOSS_API Moss_Mesh* Moss_MeshLoadOBJ(const char* file_path);
MOSS_API Moss_Mesh* Moss_MeshLoadGLB(const char* file_path);
MOSS_API Moss_Model* Moss_ModelLoadFBX(const char* file_path);
MOSS_API Moss_Model* Moss_ModelLoadOBJ(const char* file_path);
MOSS_API Moss_Model* Moss_ModelLoadGLB(const char* file_path);
MOSS_API Moss_Model* Moss_ModelLoad(const char* file_path);
MOSS_API Moss_ModelLoadError Moss_ModelGetLastLoadError();
MOSS_API const char* Moss_ModelGetLastLoadErrorMessage();
MOSS_API int Moss_ModelFindBone(const Moss_Model* model, const char* name);
MOSS_API int Moss_ModelFindMorphTarget(const Moss_Model* model, const char* name);
MOSS_API bool Moss_ModelSetBoneTransform(Moss_Model* model, uint32_t bone_index, const float* transform16);
MOSS_API bool Moss_ModelSetMorphWeight(Moss_Model* model, uint32_t morph_index, float weight);
MOSS_API void Moss_RendererDrawMesh(Moss_Renderer* renderer, const Moss_DrawMeshDesc* desc);
MOSS_API void Moss_RendererDrawModelDesc(Moss_Renderer* renderer, const Moss_DrawModelDesc* desc);
MOSS_API void Moss_RendererDrawModel(Moss_Renderer* renderer, const Moss_Model* model, const float* transform);
MOSS_API Moss_Font* Moss_RendererCreateFont(Moss_Renderer* renderer, const Moss_FontDesc* desc);
MOSS_API void Moss_RendererDestroyFont(Moss_Renderer* renderer, Moss_Font* font);
MOSS_API Moss_Font* Moss_FontLoad(const char* path, float pixel_size);
MOSS_API Moss_Font* Moss_FontCreateFromMemory(const void* data, size_t data_size, float pixel_size);
MOSS_API void Moss_FontDestroy(Moss_Font* font);
MOSS_API const Moss_FontMetrics* Moss_FontGetMetrics(const Moss_Font* font);
MOSS_API const Moss_FontGlyph* Moss_FontGetGlyph(const Moss_Font* font, uint32_t codepoint);
MOSS_API const unsigned char* Moss_FontGetAtlasPixels(const Moss_Font* font, uint32_t* width, uint32_t* height, uint32_t* stride);
MOSS_API Texture* Moss_FontGetAtlasTexture(Moss_Font* font);
MOSS_API bool Moss_FontUploadAtlas(Moss_Renderer* renderer, Moss_Font* font);
MOSS_API Moss_TextMeasure Moss_FontMeasureText(const Moss_Font* font, const char* text, float scale);
MOSS_API size_t Moss_FontBuildTextQuads(const Moss_DrawTextDesc* desc, Moss_FontVertex* out_vertices, size_t vertex_capacity);
MOSS_API void Moss_RendererDrawText(Moss_Renderer* renderer, const Moss_DrawTextDesc* desc);
MOSS_API uint32_t Moss_RendererGetFontBackendMask(void);
MOSS_API bool Moss_RendererBackendSupportsFonts(Moss_GPUBackendType backend);
MOSS_API void Moss_RendererDrawSprite(Moss_Renderer* renderer, const Moss_SpriteDesc* desc);
MOSS_API Moss_SpriteBatch* Moss_RendererCreateSpriteBatch(Moss_Renderer* renderer, uint32_t capacity);
MOSS_API void Moss_RendererDestroySpriteBatch(Moss_Renderer* renderer, Moss_SpriteBatch* batch);
MOSS_API void Moss_SpriteBatchClear(Moss_SpriteBatch* batch);
MOSS_API int Moss_SpriteBatchAdd(Moss_SpriteBatch* batch, const Moss_SpriteDesc* desc);
MOSS_API void Moss_RendererDrawSpriteBatch(Moss_Renderer* renderer, const Moss_SpriteBatch* batch);
MOSS_API Moss_DebugDrawList* Moss_RendererCreateDebugDrawList(Moss_Renderer* renderer, uint32_t capacity);
MOSS_API void Moss_RendererDestroyDebugDrawList(Moss_Renderer* renderer, Moss_DebugDrawList* list);
MOSS_API void Moss_DebugDrawListClear(Moss_DebugDrawList* list);
MOSS_API int Moss_DebugDrawLine(Moss_DebugDrawList* list, const Moss_DebugLineDesc* desc);
MOSS_API int Moss_DebugDrawBox(Moss_DebugDrawList* list, const Moss_DebugBoxDesc* desc);
MOSS_API int Moss_DebugDrawSphere(Moss_DebugDrawList* list, const Moss_DebugSphereDesc* desc);
MOSS_API int Moss_DebugDrawNavmesh(Moss_DebugDrawList* list, const Moss_DebugNavmeshDesc* desc);
MOSS_API int Moss_DebugDrawPhysicsContact(Moss_DebugDrawList* list, const Moss_DebugPhysicsContactDesc* desc);
MOSS_API void Moss_RendererDrawDebugList(Moss_Renderer* renderer, const Moss_DebugDrawList* list);
MOSS_API Moss_Camera* Moss_CameraCreate2D(const Moss_Camera2DDesc* desc);
MOSS_API Moss_Camera* Moss_CameraCreate3D(const Moss_Camera3DDesc* desc);
MOSS_API Moss_Camera* Moss_CameraCreateEditor(const Moss_EditorCameraDesc* desc);
MOSS_API void Moss_CameraDestroy(Moss_Camera* camera);
MOSS_API void Moss_RendererSetCamera(Moss_Renderer* renderer, Moss_Camera* camera);
MOSS_API Moss_ShadowMap* Moss_RendererCreateShadowMap(Moss_Renderer* renderer, const Moss_ShadowMapDesc* desc);
MOSS_API void Moss_RendererDestroyShadowMap(Moss_Renderer* renderer, Moss_ShadowMap* shadow_map);
MOSS_API void Moss_RendererBeginShadowPass(Moss_Renderer* renderer, Moss_ShadowMap* shadow_map);
MOSS_API void Moss_RendererEndShadowPass(Moss_Renderer* renderer, Moss_ShadowMap* shadow_map);
MOSS_API void Moss_MeshDraw(Moss_Mesh* mesh);
MOSS_API void Moss_ModelDraw(Moss_Model* model);
MOSS_API void Moss_MeshRemove(Moss_Mesh* mesh);
MOSS_API void Moss_ModelRemove(Moss_Model* model);
MOSS_API Moss_ResourceSet* Moss_ResourceSetCreate(Moss_Renderer* renderer, const Moss_ResourceSetLayoutDesc* desc);
MOSS_API void Moss_ResourceSetDestroy(Moss_ResourceSet* set);
MOSS_API Moss_Framebuffer* Moss_FramebufferCreate(Moss_Renderer* renderer, const Moss_FramebufferDesc* desc);
MOSS_API void Moss_FramebufferDestroy(Moss_Framebuffer* framebuffer);
MOSS_API void Moss_RendererSetUpscaler(Moss_Renderer* renderer, Moss_Upscaler upscaler);
MOSS_API void Moss_RendererSetUpscalerDesc(Moss_Renderer* renderer, const Moss_RendererUpscalerDesc* desc);
MOSS_API char* Moss_GPUGetPhysicalDeviceName();
MOSS_API char* Moss_GPUAPIGetName();
MOSS_API char* Moss_GPUAPIGetVersion();
MOSS_API void Moss_RendererDrawLine2D(Moss_Renderer* renderer, const Vec2* from, const Vec2* to, Color color, float thickness);
MOSS_API void Moss_RendererDrawLines2D(Moss_Renderer* renderer, const TArray<Vec2>* positions, Color color, float thickness);
MOSS_API void Moss_RendererDrawBox2D(Moss_Renderer* renderer, const Vec2* center, const Vec2* size, float rotation, Color color);
MOSS_API void Moss_RendererDrawCircle2D(Moss_Renderer* renderer, const Vec2* center, float radius, Color color);
MOSS_API void Moss_RendererDrawCylinder2D(Moss_Renderer* renderer, RMat44* matrix, float halfHeight, float radius, Color color);
MOSS_API void Moss_RendererDrawTriangle2D(Moss_Renderer* renderer, const Vec2* p0, const Vec2* p1, const Vec2* p2, Color color);
MOSS_API void Moss_RendererDrawArrow2D(Moss_Renderer* renderer, const Vec2* from, const Vec2* to, Color color, float headSize);
MOSS_API void Moss_RendererDrawMarker2D(Moss_Renderer* renderer, const Vec2* position, Color color, float size);
MOSS_API void Moss_RendererDrawBone2D(Moss_Renderer* renderer, RMat44* transform, float length, Color color);
MOSS_API void Moss_RendererDrawCoordinateSystem2D(Moss_Renderer* renderer, RMat44* matrix, float size);
MOSS_API void Moss_RendererDrawCapsule2D(Moss_Renderer* renderer, RMat44* matrix, float halfHeightOfCylinder, float radius, Color color);
MOSS_API void Moss_RendererDrawConvex2D(Moss_Renderer* renderer, const Vec2* points, uint32_t point_count, Color color);
MOSS_API void Moss_RendererDrawConvexHull2D(Moss_Renderer* renderer, const Vec2* points, uint32_t point_count, Color color);
MOSS_API void Moss_RendererDrawPolygonShape2D(Moss_Renderer* renderer, const Vec2* points, uint32_t point_count, Color color);
MOSS_API void Moss_RendererDrawGizmo2D(Moss_Renderer* renderer, RMat44* transform, float size);
MOSS_API void Moss_RendererDrawLineBatch2D(Moss_Renderer* renderer, RMat44* transform, Color& Colour, uint32_t count);
MOSS_API void Moss_RendererDrawBoxBatch2D(Moss_Renderer* renderer, RMat44* transform, Color& Colour, uint32_t count);
MOSS_API void Moss_RendererDrawCircleBatch2D(Moss_Renderer* renderer, RMat44* transform, Color& Colour, uint32_t count);
MOSS_API void Moss_RendererDrawArrowBatch2D(Moss_Renderer* renderer, RMat44* transform, Color& Colour, uint32_t count);
MOSS_API void Moss_RendererDrawMarkerBatch2D(Moss_Renderer* renderer, RMat44* transform, Color& Colour, uint32_t count);
MOSS_API void Moss_RendererDrawBoxInstanced2D(Moss_Renderer* renderer, const Vec2* size, RMat44* transform, Color& Colour, uint32_t count);
MOSS_API void Moss_RendererDrawCircleInstanced2D(Moss_Renderer* renderer, float radius, RMat44* transform, Color& Colour, uint32_t count);
MOSS_API void Moss_RendererDrawCapsuleInstanced2D(Moss_Renderer* renderer, float halfHeightOfCylinder, float radius, RMat44* transform, Color& Colour, uint32_t count);
MOSS_API void Moss_RendererDrawMarkerInstanced2D(Moss_Renderer* renderer, float size, RMat44* transform, Color& Colour, uint32_t count);
MOSS_API void Moss_RendererDrawGizmoInstanced2D(Moss_Renderer* renderer, float size, RMat44* transform, uint32_t count);
MOSS_API void Moss_RendererDrawLine3D(Moss_Renderer* renderer, const Vec3* from, const Vec3* to, Color color, float thickness);
MOSS_API void Moss_RendererDrawLines3D(Moss_Renderer* renderer, const TArray<Vec3>* positions, Color color, float thickness);
MOSS_API void Moss_RendererDrawBox3D(Moss_Renderer* renderer, const Vec3* center, const Vec3* size, RMat44* rotation, Color color);
MOSS_API void Moss_RendererDrawSphere3D(Moss_Renderer* renderer, const Vec3* center, float radius, Color color);
MOSS_API void Moss_RendererDrawCylinder3D(Moss_Renderer* renderer, const Vec3* base, const Vec3* top, float radius, Color color);
MOSS_API void Moss_RendererDrawCone3D(Moss_Renderer* renderer, const Vec3* base, const Vec3* tip, float radius, Color color);
MOSS_API void Moss_RendererDrawTriangle3D(Moss_Renderer* renderer, const Vec3* p0, const Vec3* p1, const Vec3* p2, Color color);
MOSS_API void Moss_RendererDrawArrow3D(Moss_Renderer* renderer, const Vec3* from, const Vec3* to, Color color, float headSize);
MOSS_API void Moss_RendererDrawMarker3D(Moss_Renderer* renderer, const Vec3* position, Color color, float size);
MOSS_API void Moss_RendererDrawPlane3D(Moss_Renderer* renderer, const Vec3* center, const Vec3* normal, float size, Color color);
MOSS_API void Moss_RendererDrawBone3D(Moss_Renderer* renderer, RMat44* transform, float length, Color color);
MOSS_API void Moss_RendererDrawCoordinateSystem3D(Moss_Renderer* renderer, RMat44* matrix, float size);
MOSS_API void Moss_RendererDrawCapsule3D(Moss_Renderer* renderer, RMat44* matrix, float halfHeightOfCylinder, float radius, Color color);
MOSS_API void Moss_RendererDrawConvex3D(Moss_Renderer* renderer, const Vec3* points, uint32_t point_count, Color color);
MOSS_API void Moss_RendererDrawConvexHull3D(Moss_Renderer* renderer, const Vec3* points, uint32_t point_count, Color color);
MOSS_API void Moss_RendererDrawPolygonShape3D(Moss_Renderer* renderer, const Vec3* points, uint32_t point_count, Color color);
MOSS_API void Moss_RendererDrawGizmo3D(Moss_Renderer* renderer, RMat44* transform, float size);
MOSS_API void Moss_RendererDrawLineBatch3D(Moss_Renderer* renderer, RMat44* transform, Color& Colour, uint32_t count);
MOSS_API void Moss_RendererDrawBoxBatch3D(Moss_Renderer* renderer, RMat44* transform, Color& Colour, uint32_t count);
MOSS_API void Moss_RendererDrawSphereBatch3D(Moss_Renderer* renderer, RMat44* transform, Color& Colour, uint32_t count);
MOSS_API void Moss_RendererDrawArrowBatch3D(Moss_Renderer* renderer, RMat44* transform, Color& Colour, uint32_t count);
MOSS_API void Moss_RendererDrawMarkerBatch3D(Moss_Renderer* renderer, RMat44* transform, Color& Colour, uint32_t count);
MOSS_API void Moss_RendererDrawBoxInstanced3D(Moss_Renderer* renderer, const Vec3* size, RMat44* transform, Color& Colour, uint32_t count);
MOSS_API void Moss_RendererDrawSphereInstanced3D(Moss_Renderer* renderer, float radius, RMat44* transform, Color& Colour, uint32_t count);
MOSS_API void Moss_RendererDrawCapsuleInstanced3D(Moss_Renderer* renderer, float halfHeightOfCylinder, float radius, RMat44* transform, Color& Colour, uint32_t count);
MOSS_API void Moss_RendererDrawCylinderInstanced3D(Moss_Renderer* renderer, float halfHeight, float radius, RMat44* transform, Color& Colour, uint32_t count);
MOSS_API void Moss_RendererDrawConeInstanced3D(Moss_Renderer* renderer, float height, float radius, RMat44* transform, Color& Colour, uint32_t count);
MOSS_API void Moss_RendererDrawMarkerInstanced3D(Moss_Renderer* renderer, float size, RMat44* transform, Color& Colour, uint32_t count);
MOSS_API void Moss_RendererDrawGizmoInstanced3D(Moss_Renderer* renderer, float size, RMat44* transform, uint32_t count);
MOSS_API VkDevice Moss_VulkanGetDevice(Moss_Renderer* renderer);
MOSS_API VkDescriptorPool Moss_VulkanGetDescriptorPool(Moss_Renderer* renderer);
MOSS_API VkDescriptorSetLayout Moss_VulkanGetDescriptorSetLayoutTexture(Moss_Renderer* renderer);
MOSS_API VkSampler Moss_VulkanGetTextureSamplerRepeat(Moss_Renderer* renderer);
MOSS_API VkSampler Moss_VulkanGetTextureSamplerShadow(Moss_Renderer* renderer);
MOSS_API VkRenderPass Moss_VulkanGetRenderPassShadow(Moss_Renderer* renderer);
MOSS_API VkRenderPass Moss_VulkanGetRenderPass(Moss_Renderer* renderer);
MOSS_API VkPipelineLayout Moss_VulkanGetPipelineLayout(Moss_Renderer* renderer);
MOSS_API VkCommandBuffer Moss_VulkanGetCommandBuffer(Moss_Renderer* renderer);
MOSS_API VkCommandBuffer Moss_VulkanStartTempCommandBuffer(Moss_Renderer* renderer);
MOSS_API void Moss_VulkanEndTempCommandBuffer(Moss_Renderer* renderer, VkCommandBuffer command_buffer);
MOSS_API void Moss_VulkanAllocateMemory(Moss_Renderer* renderer, VkDeviceSize size, uint32_t memory_type_bits, VkMemoryPropertyFlags properties, VkDeviceMemory* out_memory);
MOSS_API void Moss_VulkanFreeMemory(Moss_Renderer* renderer, VkDeviceMemory memory, VkDeviceSize size);
MOSS_API void Moss_VulkanCreateBuffer(Moss_Renderer* renderer, VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties, BufferVK* out_buffer);
MOSS_API void Moss_VulkanCopyBuffer(Moss_Renderer* renderer, VkBuffer src, VkBuffer dst, VkDeviceSize size);
MOSS_API void Moss_VulkanCreateDeviceLocalBuffer(Moss_Renderer* renderer, const void* data, VkDeviceSize size, VkBufferUsageFlags usage, BufferVK* out_buffer);
MOSS_API void Moss_VulkanFreeBuffer(Moss_Renderer* renderer, BufferVK* buffer);
MOSS_API ConstantBufferVK* Moss_VulkanCreateConstantBuffer(Moss_Renderer* renderer, VkDeviceSize buffer_size);
MOSS_API void Moss_VulkanCreateImage(Moss_Renderer* renderer, uint32_t width, uint32_t height, VkFormat format, VkImageTiling tiling, VkImageUsageFlags usage, VkMemoryPropertyFlags properties, VkImage* out_image, VkDeviceMemory* out_memory);
MOSS_API void Moss_VulkanDestroyImage(Moss_Renderer* renderer, VkImage image, VkDeviceMemory memory);
MOSS_API VkImageView Moss_VulkanCreateImageView(Moss_Renderer* renderer, VkImage image, VkFormat format, VkImageAspectFlags aspect_flags);
MOSS_API VkFormat Moss_VulkanFindDepthFormat(Moss_Renderer* renderer);
MOSS_API void Moss_Vulkan_EncodeSPIRV(const char* path);
MOSS_API void Moss_Vulkan_DecodeSPIRV(const char* path);
MOSS_API ID3D12Device* Moss_DX12GetDevice(Moss_Renderer* renderer);
MOSS_API ID3D12RootSignature* Moss_DX12GetRootSignature(Moss_Renderer* renderer);
MOSS_API ID3D12GraphicsCommandList* Moss_DX12GetCommandList(Moss_Renderer* renderer);
MOSS_API CommandQueueDX12* Moss_DX12GetUploadQueue(Moss_Renderer* renderer);
MOSS_API DescriptorHeapDX12* Moss_DX12GetDSVHeap(Moss_Renderer* renderer);
MOSS_API DescriptorHeapDX12* Moss_DX12GetSRVHeap(Moss_Renderer* renderer);
MOSS_API bool Moss_DX_CompileHLSL(const char* inputPath, const char* outputPath, const char* entryPoint, const char* target);
MOSS_API void Moss_DX_EnableDebugLayer(bool enable); /* Debug */
MOSS_API uint32_t Moss_DX_GetShaderModel(void); /* Info */
MOSS_API MTKView* Moss_MetalGetView(Moss_Renderer* renderer);
MOSS_API void* Moss_MetalGetDevice(Moss_Renderer* renderer);
MOSS_API void* Moss_MetalGetRenderEncoder(Moss_Renderer* renderer);
MOSS_API bool Moss_Metal_CompileMSL(const char* inputPath, const char* outputPath); /* Shader tools */
MOSS_API bool Moss_Metal_SupportsFamily(uint32_t family); /* Capabilities */
```

### `Moss_stdinc.h`

[Open source header](https://github.com/TxbiG/Moss/blob/main/include/Moss/Moss_stdinc.h)

```cpp
MOSS_API extern AllocateFunction Allocate;
MOSS_API extern ReallocateFunction Reallocate;
MOSS_API extern FreeFunction Free;
MOSS_API extern AlignedAllocateFunction AlignedAllocate;
MOSS_API extern AlignedFreeFunction AlignedFree;
MOSS_API void RegisterDefaultAllocator();
MOSS_API void *Allocate(size_t inSize);
MOSS_API void *Reallocate(void *inBlock, size_t inOldSize, size_t inNewSize);
MOSS_API void Free(void *inBlock);
MOSS_API void *AlignedAllocate(size_t inSize, size_t inAlignment);
MOSS_API void AlignedFree(void *inBlock);
MOSS_API int32 Moss_rand(int32 n);
MOSS_API float Moss_randf(void);
MOSS_API int Moss_randi(void);
MOSS_API void Moss_srand(int64 seed);
MOSS_API int32 Moss_randi_range(int32 from, int32 to);
MOSS_API float Moss_randf_range(float from, float to);
MOSS_API float Moss_rand_gaussian(float mean, float stddev);
MOSS_API float Moss_rand_unit();
MOSS_API Moss_Time Moss_GetTicks();
MOSS_API double Moss_GetSeconds(Moss_Time time);
MOSS_API float Moss_GetMilliseconds(Moss_Time ticks);
MOSS_API float Moss_GetMillisecondsAndReset(Moss_Time* ticks);
MOSS_API void Moss_Yield(void);
MOSS_API double Moss_Delay(double delay);
MOSS_API const char* Moss_LocalTime(void);
MOSS_API const char* Moss_TimeStamp(void);
MOSS_API const char* Moss_TimeNow(void);
MOSS_API const char* Moss_CTimeNow(void);
MOSS_API const char* Moss_TimeStampET(void);
MOSS_API const char* Moss_TimeStampCT(void);
MOSS_API const char* Moss_TimeStampMT(void);
MOSS_API const char* Moss_TimeStampPT(void);
MOSS_API const char* Moss_FormatTime(const char* fmt);
```

### `Moss_XR.h`

[Open source header](https://github.com/TxbiG/Moss/blob/main/include/Moss/Moss_XR.h)

```cpp
MOSS_API bool Moss_XR_Initialize(Moss_Renderer* renderer);
MOSS_API void Moss_XR_Shutdown(void);
MOSS_API const MossXR_Capabilities* Moss_XR_GetCapabilities(void);
MOSS_API bool Moss_XR_EnableValidationLayers();
MOSS_API MossXR_Session* Moss_XR_CreateSession(void);
MOSS_API void Moss_XR_DestroySession(MossXR_Session* session);
MOSS_API MossXR_SessionState Moss_XR_GetSessionState(MossXR_Session* session);
MOSS_API bool Moss_XR_BeginFrame(MossXR_Session* session);
MOSS_API void Moss_XR_EndFrame(MossXR_Session* session);
MOSS_API MossXR_Time Moss_XR_GetPredictedDisplayTime(MossXR_Session* session);
MOSS_API float Moss_XR_GetDeltaSeconds(void);
MOSS_API void Moss_XR_HandleEvents(MossXR_Session* session);
MOSS_API uint32_t Moss_XR_GetViewCount(MossXR_Session* session);
MOSS_API bool Moss_XR_GetView(MossXR_Session* session, uint32_t index, MossXR_View* outView);
MOSS_API MossXR_ActionSet* Moss_XR_CreateActionSet(const char* name);
MOSS_API void Moss_XR_DestroyActionSet(MossXR_ActionSet* set);
MOSS_API MossXR_Action* Moss_XR_CreateAction(MossXR_ActionSet* set, const char* name, MossXR_ActionType type);
MOSS_API void Moss_XR_DestroyAction(MossXR_Action* action);
MOSS_API void Moss_XR_AttachActionSet(MossXR_Session* session, MossXR_ActionSet* set);
MOSS_API void Moss_XR_SyncActions(MossXR_Session* session);
MOSS_API bool Moss_XR_GetActionBoolean(MossXR_Action* action);
MOSS_API float Moss_XR_GetActionFloat(MossXR_Action* action);
MOSS_API bool Moss_XR_GetActionPose(MossXR_Action* action, MossXR_Pose* outPose);
MOSS_API void Moss_XR_StopHaptic(MossXR_Action* action);
MOSS_API MossXR_HandModifier* Moss_XR_CreateHand(void);
MOSS_API bool Moss_XR_GetHandJointPose(MossXR_HandModifier* hand, XRHandJoint joint, MossXR_Pose* outPose);
MOSS_API void Moss_XR_DestroyHand(MossXR_HandModifier* hand);
MOSS_API bool Moss_XR_GetHandPose(MossXR_HandModifier* hand, MossXR_Pose* outPose);
MOSS_API MossXR_BodyModifier* Moss_XR_CreateBody(void);
MOSS_API bool Moss_XR_GetBodyJointPose(MossXR_BodyModifier* body, XRBodyJoint joint, MossXR_Pose* outPose);
MOSS_API void Moss_XR_DestroyBody(MossXR_BodyModifier* body);
MOSS_API MossXR_FaceModifier* Moss_XR_CreateFace(void);
MOSS_API void Moss_XR_DestroyFace(MossXR_FaceModifier* face);
MOSS_API void Moss_XR_UpdateBodyTracking(MossXR_Session* session);
MOSS_API void Moss_XR_UpdateHandTracking(MossXR_Session* session);
MOSS_API bool Moss_XR_EnablePassthrough(bool enable);
MOSS_API bool Moss_XR_EnableFoveatedRendering(bool enable);
MOSS_API const char* Moss_XR_GetBackendName(void);
MOSS_API uint32_t Moss_XR_GetBackendVersion(void);
MOSS_API void Moss_XR_BeginDebugLabel(const char* label);
MOSS_API void Moss_XR_EndDebugLabel(void);
```
