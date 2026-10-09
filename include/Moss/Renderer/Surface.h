// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Core/Reference.h>
#include <Moss/Core/Color.h>
#include <Moss/Core/StringTools.h>

/// Possible lock modes of a Surface
enum class ESurfaceLockMode : uint32_t
{
	None						= 0 << 0,															///< Not locked, cannot be used as a parameter
	Read						= 1 << 0,
	Write						= 2 << 0,
	ReadWrite					= Read | Write,
};

/// Possible surface formats, most significant bit (MSB) first
enum class ESurfaceFormat : uint32_t
{
	A4L4,																							///< 4 bit alpha, 4 bit luminance (grayscale)
	L8,																								///< 8 bit luminance (grayscale)
	A8,																								///< 8 bit alpha
	A8L8,																							///< 8 bit luminance and 8 bit alpha
	R5G6B5,																							///< 16 bit RGB
	X1R5G5B5,																						///< 16 bit RGB
	X4R4G4B4,																						///< 16 bit RGB
	A1R5G5B5,																						///< 16 bit RGBA
	A4R4G4B4,																						///< 16 bit RGBA
	R8G8B8,																							///< 24 bit RGB
	B8G8R8,																							///< 24 bit BGR
	X8R8G8B8,																						///< 32 bit RGB
	X8B8G8R8,																						///< 32 bit RGB
	A8R8G8B8,																						///< 32 bit RGBA
	A8B8G8R8,																						///< 32 bit BGRA
	Invalid,																						///< Invalid value
	Count						= Invalid,															///< Number of pixel formats
};

/// Description of a surface format
class FormatDescription
{
public:
	/// Constructor
								FormatDescription(const char *inFormatName, int inBitsPerPixel, int inNumberOfComponents, ESurfaceFormat inClosest8BitFormat, ESurfaceFormat inClosestAlphaFormat, uint32_t inRedMask, uint32_t inGreenMask, uint32_t inBlueMask, uint32_t inAlphaMask);

	/// General properties
	const string_view &			GetFormatName() const												{ return mFormatName; }
	int							GetBytesPerPixel() const											{ return (mBitsPerPixel + 7) >> 3; }
	int							GetNumberOfComponents() const										{ return mNumberOfComponents; }
	ESurfaceFormat				GetClosest8BitFormat() const										{ return mClosest8BitFormat; }
	ESurfaceFormat				GetClosestAlphaFormat() const										{ return mClosestAlphaFormat; }

	/// Bitcounts for the various components of the image
	int							GetBitsPerPixel() const												{ return mBitsPerPixel; }
	int							GetRedBitsPerPixel() const											{ return CountBits(mRedMask); }
	int							GetGreenBitsPerPixel() const										{ return CountBits(mGreenMask); }
	int							GetBlueBitsPerPixel() const											{ return CountBits(mBlueMask); }
	int							GetAlphaBitsPerPixel() const										{ return CountBits(mAlphaMask); }
	int							GetComponentBitCount(int inComponent) const							{ return CountBits(GetComponentMask(inComponent)); }

	/// Bitmasks indicating the various components of the image
	uint32_t						GetRedMask() const													{ return mRedMask; }
	uint32_t						GetGreenMask() const												{ return mGreenMask; }
	uint32_t						GetBlueMask() const													{ return mBlueMask; }
	uint32_t						GetAlphaMask() const												{ return mAlphaMask; }
	uint32_t						GetComponentMask(int inComponent) const								{ return *(&mRedMask + inComponent); }

	/// Convert a single color
	uint32_t						Encode(ColorArg inColor) const;
	const Color					Decode(uint32_t inColor) const;

private:
	string_view					mFormatName;														///< User displayable String describing the format
	int							mBitsPerPixel;														///< Number of bits per pixel
	int							mNumberOfComponents;												///< Number of color components per pixel
	ESurfaceFormat				mClosest8BitFormat;													///< Closest matching format that has 8 bit color components
	ESurfaceFormat				mClosestAlphaFormat;												///< Closest matching format that has an alpha channel

	uint32_t						mRedMask;															///< Bitmasks indicating which bits are used by which color components
	uint32_t						mGreenMask;
	uint32_t						mBlueMask;
	uint32_t						mAlphaMask;
};

/// Get the description for a specific surface format
const FormatDescription &		GetFormatDescription(ESurfaceFormat inFormat);

/// Class that contains an image in arbitrary format
class Surface : public RefTarget<Surface>
{
public:
	/// Constructor
								Surface(int inWidth, int inHeight, ESurfaceFormat inFormat);
	virtual						~Surface();

	/// Type of the image data
	const FormatDescription &	GetFormatDescription() const										{ return ::GetFormatDescription(mFormat); }

	const string_view &			GetFormatName() const												{ return GetFormatDescription().GetFormatName(); }
	int							GetBytesPerPixel() const											{ return GetFormatDescription().GetBytesPerPixel(); }
	int							GetNumberOfComponents() const										{ return GetFormatDescription().GetNumberOfComponents(); }
	ESurfaceFormat				GetClosest8BitFormat() const										{ return GetFormatDescription().GetClosest8BitFormat(); }

	int							GetBitsPerPixel() const												{ return GetFormatDescription().GetBitsPerPixel(); }
	int							GetRedBitsPerPixel() const											{ return GetFormatDescription().GetRedBitsPerPixel(); }
	int							GetGreenBitsPerPixel() const										{ return GetFormatDescription().GetGreenBitsPerPixel(); }
	int							GetBlueBitsPerPixel() const											{ return GetFormatDescription().GetBlueBitsPerPixel(); }
	int							GetAlphaBitsPerPixel() const										{ return GetFormatDescription().GetAlphaBitsPerPixel(); }
	int							GetComponentBitCount(int inComponent) const							{ return GetFormatDescription().GetComponentBitCount(inComponent); }

	uint32_t						GetRedMask() const													{ return GetFormatDescription().GetRedMask(); }
	uint32_t						GetGreenMask() const												{ return GetFormatDescription().GetGreenMask(); }
	uint32_t						GetBlueMask() const													{ return GetFormatDescription().GetBlueMask(); }
	uint32_t						GetAlphaMask() const												{ return GetFormatDescription().GetAlphaMask(); }
	uint32_t						GetComponentMask(int inComponent) const								{ return GetFormatDescription().GetComponentMask(inComponent); }

	/// Get properties of this surface
	inline ESurfaceFormat		GetFormat() const													{ return mFormat; }
	inline int					GetWidth() const													{ return mWidth; }
	inline int					GetHeight() const													{ return mHeight; }

	/// Sets the image to a specific color
	void						Clear(ColorArg inColor = Color::Black);

	/// Locking functions
	void						Lock(ESurfaceLockMode inMode) const;
	void						UnLock() const;

	/// Current lock state
	inline ESurfaceLockMode		GetLockMode() const													{ return mLockMode; }
	inline bool					IsLocked() const													{ return mLockMode != ESurfaceLockMode::None; }
	inline bool					IsLockedForRead() const												{ return (uint32_t(mLockMode) & uint32_t(ESurfaceLockMode::Read)) != 0; }
	inline bool					IsLockedForWrite() const											{ return (uint32_t(mLockMode) & uint32_t(ESurfaceLockMode::Write)) != 0; }
	inline bool					IsLockedForReadWrite() const										{ return IsLockedForRead() && IsLockedForWrite(); }

	/// Access to the image data
	inline const uint8 *		GetData() const														{ MOSS_ASSERT(IsLockedForRead()); return mData; }
	inline uint8 *				GetData()															{ MOSS_ASSERT(IsLockedForWrite()); return mData; }
	inline int					GetStride() const													{ MOSS_ASSERT(IsLocked()); return mStride; }
	inline int					GetLength() const													{ MOSS_ASSERT(IsLocked()); return mLength; }

	/// Get start of a specific scanline
	inline const uint8 *		GetScanLine(int inScanLine) const									{ MOSS_ASSERT(inScanLine >= 0 && inScanLine < GetHeight()); return GetData() + inScanLine * GetStride(); }
	inline uint8 *				GetScanLine(int inScanLine)											{ MOSS_ASSERT(inScanLine >= 0 && inScanLine < GetHeight()); return GetData() + inScanLine * GetStride(); }

protected:
	/// These functions must be overridden by the hardware buffer
	virtual void				HardwareLock() const = 0;
	virtual void				HardwareUnLock() const = 0;

	/// Data
	ESurfaceFormat				mFormat;															///< Pixel format of the surface
	int							mWidth;																///< Width of the image
	int							mHeight;															///< Height of the image
	mutable int					mLength;															///< Length in bytes of the image

	mutable ESurfaceLockMode	mLockMode;
	mutable int					mStride;															///< Width of one scanline in bytes
	mutable uint8 *				mData;																///< Pointer to image data, starting at top-left of locked rectangle
};

/// Class that contains an image in arbitrary format, backed by normal memory (not device specific)
class SoftwareSurface : public Surface
{
public:
	/// Constructor
								SoftwareSurface(int inWidth, int inHeight, ESurfaceFormat inFormat, int inStride = 0);
	virtual						~SoftwareSurface() override;

protected:
	/// These functions must be overridden by the hardware buffer
	virtual void				HardwareLock() const override;
	virtual void				HardwareUnLock() const override;

	uint8 *						mPixelData;
	int							mPixelStride;
	int							mPixelLength;
};
