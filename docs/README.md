# Moss Framework Documentation

## Overview
- [Introduction](#) — What is MossFramework?
- [Architecture Overview](#) — Key systems/components.
- [Thirdparties](/docs/Thirdparties.md)
- [Licensing](LICENSE)

## Getting Started
[Your First 2D Game](/docs/2DGame#your-first-2d-game), [Your First 3D Game](/docs/3DGame#your-first-3d-game)

## Platform
- [Overview](/docs/Platform.md#overview)
    - [Window](/docs/Platform.md#window), [Monitor](/docs/Platform.md#monitor)
    - [Input](/docs/Platform.md#input), [Haptic Feedback](/docs/Platform.md#haptic-feedback)
    - [VideoCapture](/docs/Platform.md#videocapture)
    - [Miscellaneous](/docs/Platform.md#miscellaneous)
    - [Graphics API Specific](/docs/Platform.md#graphics-api-specific)

## Audio
- [Overview](/docs/Audio.md#overview)
  - [Channels](/docs/Audio.md#channels), [Effects](/docs/Audio.md#effects)
  - [AudioStream](/docs/Audio.md#audiostream), [AudioStream2](/docs/Audio.md#audiostream2), [AudioStream3](/docs/Audio.md#audiostream3)
  - [AudioListener2](/docs/Audio.md#audiolistener2), [AudioListener3](/docs/Audio.md#audiolistener3)
  - [Speaker](/docs/Audio.md#speaker), [Microphone](/docs/Audio.md#microphone)
  - [RayTrace Audio](/docs/Audio.md#raytraceaudio)

## Components
- 2D
    - [CPUParticle2D](/docs/2D.md#cpuparticle2d), [GPUParticle2D](/docs/2D.md#gpuparticle2d), [Sprite2D](/docs/2D.md#sprite2d), [Tilemap](/docs/2D.md#tilemap), DampedSpringJoint2D, GrooveJoint2D, PinJoint2D, Line2D, ParallaxLayer, Path2D, PathFollow
- 3D
    - [CPUParticle3D](/docs/3D.md#cpuparticle3d), [Decal](/docs/3D.md#decal),  [GPUParticle3D](/docs/3D.md#gpuparticle3d), [GridMap](), [Sprite3D](), [Terrain](/docs/3D.md#terrain), [World3D](), FogVolume, VehicleWheel3D, SpringArm3D, DampedSpringJoint3D, GrooveJoint3D, PinJoint3D

## Physics
- [Overview](/docs/Physics.md#overview)
### Physics 2D
- [Collision detection](/docs/), [Vehicles 2D](/docs/), [Softbody simulation](/docs/), [Animated ragdolls](/docs/), [Game character simulation](/docs/), [Water buoyancy calculations](/docs/), [Motors](/docs/)
### Physics 3D
- [Collision detection](/docs/), [Vehicles 3D](/docs/), [Softbody simulation](/docs/) [Animated ragdolls](/docs/), [Game character simulation](/docs/), [Water buoyancy calculations](/docs/), [Motors](/docs/)

## Renderer
- [Overview](/docs/Renderer.md#overview)
    - [Introduction](/docs/Renderer.md#introduction)
    - [Choosing a Renderer](/docs/Renderer.md#choosing-a-renderer)
    - [Renderer Life-cycle](/docs/Renderer.md#renderer-life-cycles)
    - [Textures](/docs/Renderer.md#), [Shaders](/docs/Renderer.md#shaders), [Surface](/docs/Renderer.md#surface), [Mesh](/docs/Renderer.md#mesh), [Model](/docs/Renderer.md#Model)
    - [Viewport & SubViewports](/docs/Renderer.md#), [Cameras](/docs/Renderer.md#)
    - [Lighting](/docs/Renderer.md#)
    - [Postprocessing](/docs/Renderer.md#post-processing), [Compositor]()
    - [Graphics API Specific](/docs/Renderer.md#graphics-api-specific)

## UI
- [Overview](/docs/UI.md#overview)
    - [Container](/docs/UI.md#), [CenterContainer](/docs/UI.md#), [VContainer](/docs/UI.md#), [HContainer](/docs/UI.md#), [GridContainer](/docs/UI.md#), [MarginContainer](/docs/UI.md#), [VScrollContainer](/docs/UI.md#), [HScrollContainer](/docs/UI.md#)
    - [TextureRect](/docs/UI.md#), [ColorRect](/docs/UI.md#)
    - [RichText](/docs/UI.md#), [Text](/docs/UI.md#)
    - [Button](UI.md#), [ToggleSwitch](/docs/UI.md#), [Checkbox](/docs/UI.md#), [RadioButton](/docs/UI.md#)
    - [ProgressBar](UI.md#), [ComboBox](/docs/UI.md#), [Drag](UI.md#), [Slider](#), [Input](/docs/UI.md#), [Color Edit/Picker](/docs/UI.md#), [TreeNode](/docs/UI.md#), [ListBox](/docs/UI.md#), [MenuBar](/docs/UI.md#), [Tooltip](/docs/UI.md#), [Popup](/docs/UI.md#), [Table](/docs/UI.md#), [TabBar](/docs/UI.md#), [Graphs](/docs/UI.md#), [AnimatedIcon](/docs/UI.md#), [Dialogue](/docs/UI.md#), [CodeEditor](/docs/UI.md#), [HexEditor](UI.md#), [DatePicker](/docs/UI.md#)
    - [TextLink](/docs/UI.md#), [TextLinkOpenURL](/docs/UI.md#)


## Variants
- [Overview](/docs/Variants.md#overview)
    - [Signed Integers](/docs/Variants.md#signed-integers), [Unsigned Integers](/docs/Variants.md#unsigned-integers)
    - [AABB2](/docs/Variants.md#aabb), [AABB3](/docs/Variants.md#aabb)
    - [OOB2](/docs/Variants.md#aabb), [OOB3](/docs/Variants.md#aabb)
    - [Color](/docs/Variants.md#color)
    - [Rect](/docs/Variants.md#rect), [iRect](/docs/Variants.md#recti)
    - [Curve](/docs/Variants.md#curve), [Curve2](/docs/Variants.md#curve2), [Curve3](/docs/Variants.md#curve3)
    - [Vec2](/docs/Variants.md#vec2), [Vec3](Variants.md#vec3), [Vec4](/docs/Variants.md#vec4)
    - [iVec2](/docs/Variants.md#vec2i), [iVec3](/docs/Variants.md#vec3i), [iVec4](/docs/Variants.md#vec4i)
    - [uVec2](/docs/Variants.md#uvec2), [uVec3](/docs/Variants.md#uvec3), [uVec4](/docs/Variants.md#uvec4)
    - [dVec2](/docs/Variants.md#bvec2), [dVec3](/docs/Variants.md#bvec3), [dVec4](/docs/Variants.md#bvec4)
    - [Float2](/docs/Variants.md#float2), [Float3](/docs/Variants.md#float3), [Float4](/docs/Variants.md#float4)
    - [Int2](/docs/Variants.md#float2), [Int3](/docs/Variants.md#float3), [Int4](/docs/Variants.md#float4)
    - [Double2](/docs/Variants.md#double2), [Double3](/docs/Variants.md#double3), [Double4](/docs/Variants.md#double4)
    - [Mat2x2](/docs/Variants.md#mat2x2), [Mat2x3](/docs/Variants.md#mat2x3), [Mat2x4](/docs/Variants.md#mat2x4), [Mat3x2](/docs/Variants.md#mat3x2), [Mat3x3](/docs/Variants.md#mat3x3), [Mat3x4](/docs/Variants.md#mat3x4), [Mat4x2](/docs/Variants.md#mat4x2), [Mat4x3](/docs/Variants.md#mat4x3), [Mat4x4](/docs/Variants.md#mat4x4)
    - [Quat](/docs/Variants.md#quat), [Basis](/docs/Variants.md#basis),
    - [TArray](/docs/Variants.md#), [TMap](/docs/Variants.md#), [TSet](/docs/Variants.md#), [TStaticArray](/docs/Variants.md#), [TMultiMap](/docs/Variants.md#), [TPair](/docs/Variants.md#)
    - [Hash](/docs/Variants.md#)

## Resources
- [Overview](/docs/#overview)
    - [Noise](/docs/), [Tween](/docs/)
    - [Font](/docs/), [Shader](/docs/), [Mesh](/docs/), [Texture](/docs/)
    - [Config](/docs/), [Json](/docs/), [PAK](/docs/), [Wav](/docs/),
    - [Animations](/docs/Animations.md#animations)

## Networking
- [Overview](/docs/Network.md#overview) 
    - [Client/Server](/docs/#), [RPC](/docs/#)
    - [HTTP/Rest](/docs/#)

## XR (Virtual/Augmented/Mixed Reality)
- [Overview](/docs/XR.md#overview)
    - [XROrigin](/docs/XR.md), [XRSpace](/docs/XR.md), [XRAnchor](/docs/XR.md), [XRSession](/docs/XR.md)
    - [XRCamera](/docs/XR.md)
    - [XRActionSet](/docs/XR.md), [XRActions](/docs/XR.md)
    - [XRControls](/docs/XR.md), [XRHaptic](/docs/XR.md)
    - [XRFaceModifier](/docs/XR.md),  [XRBodyModifier](/docs/XR.md), [XRHandModifier](/docs/XR.md),
    - [XRCharacterBody](/docs/XR.md)
