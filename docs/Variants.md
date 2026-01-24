# Variants
## Signed Integers & Unsigned Integers
```cpp
// Provided by Moss
typedef signed char int8;
typedef signed short int16;
typedef signed int int32;
typedef signed long long int64;

typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned int uint32;
typedef unsigned long long uint64;
```
#### Macros
```cpp
// Provided by Moss
#define MAX_INT8    ((int8)(0x7F))
#define MAX_INT16   ((int16)(0x7FFF))
#define MAX_INT32   ((int32)(0x7FFFFFFF))
#define MAX_INT64   ((int64)(0x7FFFFFFFFFFFFFFF))
#define MIN_INT8    ((int8)(~0x7F))
#define MIN_INT16   ((int16)~0x7FFF)
#define MIN_INT32   ((int32)(~0x7FFFFFFF))
#define MIN_INT64   ((int64)(~0x7FFFFFFFFFFFFFFF))

#define MAX_UINT8   ((uint8)(0xFF))
#define MAX_UINT16  ((uint16)(0xFFFF))
#define MAX_UINT32  ((uint32)(0xFFFFFFFFu))
#define MAX_UINT64  ((uint64)(0xFFFFFFFFFFFFFFFF))
#define MIN_UINT8   ((uint8)0x00)
#define MIN_UINT16  ((uint16)0x0000)
#define MIN_UINT32  ((uint32)0x00000000)
#define MIN_UINT64  ((uint64)(0x0000000000000000))
```

## AABB
```cpp
AABB();
AABB();
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## OOB

## Basis
```cpp
Basis();
Basis(axis: Vector3, angle: float);
Basis(from: Quaternion);
Basis(x_axis: Vec3, y_axis: Vec3, z_axis: Vec3);
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## Color
```cpp
Color();
Color(float r, float g, float b);
Color(float r, float g, float b, float a);
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## Rect
```cpp
Rect();
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## iRect
```cpp
Recti();
```
| Variable | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## Quat
```cpp
Quat();
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## Float2
```cpp
Float2();
Float2();
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## Float3
```cpp
Float3();
Float3();
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## Float4
```cpp
Float4();
Float4();
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## Double2
## Double3
## Double4

## Vec2
Vec2 is a ```float``` type variable. vec2 is used for 2D coordinates
```cpp
Vec2();
Vec2(float x, float y);
```
Example
```cpp
Vec2(float x, float y);
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## Vec3
Vec3 is a ```float``` type variable. vec3 is used for 3D coordinates
```cpp
Vec3();
Vec3(float x, float y, float z);
```
Example
```cpp
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## Vec4
Vec4 is a ```float``` type variable
```cpp
Vec4();
Vec4(float x, float y, float z, float w);
```
Example
```cpp
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## Vec2i
Vec2 is a ```int``` type variable
```cpp
Vec2i();
Vec2i(int x, int y);
```
Example
```cpp
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## Vec3i
Vec3 is a ```int``` type variable
```cpp
Vec3i();
Vec3i(int x, int y, int z);
```
Example
```cpp
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## Vec4i
Vec4i is a ```int``` type variable
```cpp
Vec4i();
Vec4i(int x, int y, int z, int w);
```
Example
```cpp
```

| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |


## uVec2
Example
```cpp
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## uVec3
Example
```cpp
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |
## uVec4
Example
```cpp
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## dVec2
Example
```cpp
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## dVec3
Example
```cpp
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## dVec4
Example
```cpp
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## dVec2
Example
```cpp
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## dVec3
Example
```cpp
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |
## dVec4
Example
```cpp
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## bVec2
Example
```cpp
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## bVec3
Example
```cpp
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## bVec4
Example
```cpp
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## Mat2x2
Example
```cpp
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## Mat2x3
Example
```cpp
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## Mat2x4
Example
```cpp
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## Mat3x2
Example
```cpp
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## Mat3x3
Example
```cpp
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## Mat3x4
Example
```cpp
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## Mat4x2
Example
```cpp
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## Mat4x3
Example
```cpp
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## Mat4x4
Example
```cpp
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## TArray
TArray is a ```template``` Array.
```cpp
template<T>
TArray<T>;
```
Example
```cpp
TArray<vec3> tarray;
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## TMap
TMap is a ```template``` unordered_map.
```cpp
template<T>
TMap<T>;
```
Example
```cpp
TMap<vec3> tvector;
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## TMultiMap
TMultiMap is a ```template```.
```cpp
template<T>
TMultiMap<T>;
```
Example
```cpp
TMultiMap<vec3> tvector;
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## TPair
TPair is a ```template```.
```cpp
template<T>
TPair<T>;
```
Example
```cpp
TPair<vec3> tvector;
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## TSet
TSet is a ```template``` Vector that is not to be confused with Vecs.
```cpp
template<T>
TVector<T>;
```
Example
```cpp
TVector<vec3> tvector;
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## TStaticArray
TStaticArray is a ```template``` Vector that is not to be confused with Vecs.
```cpp
template<T>
TStaticArray<T>;
```
Example
```cpp
TStaticArray<vec3> tvector;
```
| Variant | Operator |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |
