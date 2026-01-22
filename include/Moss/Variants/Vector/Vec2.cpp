// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <Moss/Core/Variants/Vec2.h>

MOSS_SUPRESS_WARNINGS_BEGIN

static void sAddVertex(TStaticArray<Vec2, 1026> &ioVertices, Vec2Arg inVertex)
{
	bool found = false;
	for (const Vec3 &v : ioVertices)
		if (v == inVertex)
		{
			found = true;
			break;
		}
	if (!found)
		ioVertices.push_back(inVertex);
}

static void sCreateVertices(TStaticArray<Vec3, 1026> &ioVertices, Vec2Arg inDir1, Vec2Arg inDir2, Vec2Arg inDir3, int inLevel)
{
	Vec3 center1 = (inDir1 + inDir2).Normalized();
	Vec3 center2 = (inDir2 + inDir3).Normalized();
	Vec3 center3 = (inDir3 + inDir1).Normalized();

	sAddVertex(ioVertices, center1);
	sAddVertex(ioVertices, center2);
	sAddVertex(ioVertices, center3);

	if (inLevel > 0)
	{
		int new_level = inLevel - 1;
		sCreateVertices(ioVertices, inDir1, center1, center3, new_level);
		sCreateVertices(ioVertices, center1, center2, center3, new_level);
		sCreateVertices(ioVertices, center1, inDir2, center2, new_level);
		sCreateVertices(ioVertices, center3, center2, inDir3, new_level);
	}
}

const TStaticArray<Vec2, 1026> Vec2::sUnitSphere = []() {

	const int level = 3;

	TStaticArray<Vec2, 1026> verts;

	// Add unit axis
	verts.push_back(Vec2::sAxisX());
	verts.push_back(-Vec2::sAxisX());
	verts.push_back(Vec2::sAxisY());
	verts.push_back(-Vec2::sAxisY());
	verts.push_back(Vec2::sAxisZ());
	verts.push_back(-Vec2::sAxisZ());

	// Subdivide
	sCreateVertices(verts, Vec2::sAxisX(), Vec2::sAxisY(), Vec2::sAxisZ(), level);
	sCreateVertices(verts, -Vec2::sAxisX(), Vec2::sAxisY(), Vec2::sAxisZ(), level);
	sCreateVertices(verts, Vec2::sAxisX(), -Vec2::sAxisY(), Vec2::sAxisZ(), level);
	sCreateVertices(verts, -Vec2::sAxisX(), -Vec2::sAxisY(), Vec2::sAxisZ(), level);
	sCreateVertices(verts, Vec2::sAxisX(), Vec2::sAxisY(), -Vec2::sAxisZ(), level);
	sCreateVertices(verts, -Vec2::sAxisX(), Vec2::sAxisY(), -Vec2::sAxisZ(), level);
	sCreateVertices(verts, Vec2::sAxisX(), -Vec2::sAxisY(), -Vec2::sAxisZ(), level);
	sCreateVertices(verts, -Vec2::sAxisX(), -Vec2::sAxisY(), -Vec2::sAxisZ(), level);

	return verts;
}();

MOSS_SUPRESS_WARNINGS_END
