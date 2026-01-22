// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

MOSS_SUPRESS_WARNINGS_BEGIN

/// Class that makes another class non-copyable. Usage: Inherit from NonCopyable.
class MOSS_EXPORT NonCopyable
{
public:
			NonCopyable() = default;
			NonCopyable(const NonCopyable &) = delete;
	void	operator = (const NonCopyable &) = delete;
};

MOSS_SUPRESS_WARNINGS_END
