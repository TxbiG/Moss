// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Core/Core.h>

MOSS_SUPRESS_WARNINGS_BEGIN

class Color;

using ColorArg = Color;

/// Class that holds an RGBA color with 8-bits per component
class MOSS_EXPORT_GCC_BUG_WORKAROUND [[nodiscard]] Color {
public:
	/// Constructors
	Color() = default; // uninitialized for perf
    constexpr Color(float inRed, float inGreen, float inBlue, float inAlpha = 1.0f) : r(inRed), g(inGreen), b(inBlue), a(inAlpha) {}
    Color(const Color& rhs) = default;
    Color& operator=(const Color& rhs) = default;

	// Comparison
    bool operator==(const Color& rhs) const { return r == rhs.r && g == rhs.g && b == rhs.b && a == rhs.a; }
    bool operator!=(const Color& rhs) const { return !(*this == rhs); }

    // Multiply two colors component-wise
    Color operator*(const Color& rhs) const { return Color(r * rhs.r, g * rhs.g, b * rhs.b, a * rhs.a); }

    // Multiply color by intensity [0..1]
    Color operator*(float intensity) const { return Color(r * intensity, g * intensity, b * intensity, a); }

    // Convert to Vec4 (assuming Vec4 uses floats)
    Vec4 ToVec4() const { return Vec4(r, g, b, a); }

    // Grayscale intensity (luminance approx)
    float GetIntensity() const { return 0.2126f * r + 0.7152f * g + 0.0722f * b; // standard Rec. 709 weights 
		}

	static Color			sGetDistinctColor(int inIndex);

	/// Predefined colors
	static const Color		sBlack;
	static const Color		sDarkRed;
	static const Color		sRed;
	static const Color		sDarkGreen;
	static const Color		sGreen;
	static const Color		sDarkBlue;
	static const Color		sBlue;
	static const Color		sYellow;
	static const Color		sPurple;
	static const Color		sCyan;
	static const Color		sOrange;
	static const Color		sDarkOrange;
	static const Color		sGrey;
	static const Color		sLightGrey;
	static const Color		sWhite;

	union
	{
		uint32				mU32;																	///< Combined value for red, green, blue and alpha
		struct
		{
			float			r;																		///< Red channel
			float			g;																		///< Green channel
			float			b;																		///< Blue channel
			float			a;																		///< Alpha channel
		};
	};
};

static_assert(std::is_trivial<Color>(), "Is supposed to be a trivial type!");

MOSS_SUPRESS_WARNINGS_END
