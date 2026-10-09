// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Core/RTTI.h>

MOSS_SUPPRESS_WARNINGS_END

/// Helper functions to get the underlying RTTI type of a type (so e.g. TArray<sometype> will return sometype)
template <class T>
const RTTI *GetPrimitiveTypeOfType(T *)
{
	return GetRTTIOfType((T *)nullptr);
}

template <class T>
const RTTI *GetPrimitiveTypeOfType(T **)
{
	return GetRTTIOfType((T *)nullptr);
}

template <class T>
const RTTI *GetPrimitiveTypeOfType(Ref<T> *)
{
	return GetRTTIOfType((T *)nullptr);
}

template <class T>
const RTTI *GetPrimitiveTypeOfType(RefConst<T> *)
{
	return GetRTTIOfType((T *)nullptr);
}

template <class T, class A>
const RTTI *GetPrimitiveTypeOfType(TArray<T, A> *)
{
	return GetPrimitiveTypeOfType((T *)nullptr);
}

template <class T, uint32_t N>
const RTTI *GetPrimitiveTypeOfType(StaticArray<T, N> *)
{
	return GetPrimitiveTypeOfType((T *)nullptr);
}

template <class T, uint32_t N>
const RTTI *GetPrimitiveTypeOfType(T (*)[N])
{
	return GetPrimitiveTypeOfType((T *)nullptr);
}

MOSS_SUPPRESS_WARNINGS_END
