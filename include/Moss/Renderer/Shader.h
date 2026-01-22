#pragma once

#include <Moss/Core/Reference.h>


class PixelShader : public RefTarget<PixelShader>
{
public:
	virtual					~PixelShader() = default;
};

class VertexShader : public RefTarget<VertexShader>
{
public:
	virtual					~VertexShader() = default;
};

class ComputeShader : public RefTarget<ComputeShader>
{
public:
	virtual					~ComputeShader() = default;
};
