// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2026 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

/*
#include <Moss/Variants/Vector/Vec3.h>

MOSS_SUPPRESS_WARNINGS_BEGIN
 
struct ClothSettings {
	Vec3		    mGravity { 0, -9.81f, 0 };
	float			mLinearDamping = 0.1f;
	float			mStretchCompliance = 0.0f;
	float			mShearCompliance = 1.0e-5f;
	float			mBendCompliance = 1.0e-3f;
	float			mTwistCompliance = 1.0e-4f;     // <--- New: Twist inverse stiffness
	int				mSubsteps = 4;
	int				mIterations = 1;
	float			mParticleRadius = 0.01f;
};
 
struct ClothSphere { Vec3 mCenter; float mRadius = 0.5f; float mFriction = 0.3f; };
struct ClothPlane { Vec3 mNormal { 0, 1, 0 }; float mOffset = 0.0f; float mFriction = 0.3f; };	// dot(n, p) = offset
 
class Cloth {
public:
	/// Parallel-for hook: run Body(i) for i in [0, Count). Defaults to serial.
	/// Wire this to your job system. Safe because each colour group has disjoint particles.
	using ParallelFor = std::function<void(uint32_t Count, const std::function<void(uint32_t)> &Body)>;
 
	ClothSettings	mSettings;
	ParallelFor		mParallelFor = [](uint32_t Count, const std::function<void(uint32_t)> &Body) { for (uint32_t i = 0; i < Count; ++i) Body(i); };
 
	std::vector<ClothSphere>	mSpheres;
	std::vector<ClothPlane>		mPlanes;
 
	/// Build a W x H particle grid in the XZ plane (y = origin.y) with stretch, shear and bend constraints.
	void CreateGrid(uint32_t inWidth, uint32_t inHeight, float inSpacing, Vec3 inOrigin, float inParticleMass = 0.05f) {
		mWidth = inWidth; mHeight = inHeight;
		uint32_t n = inWidth * inHeight;
		mPos.assign(n, {}); mPrev = mPos; mVel.assign(n, {}); mInvMass.assign(n, 1.0f / inParticleMass);
		for (uint32_t y = 0; y < inHeight; ++y)
			for (uint32_t x = 0; x < inWidth; ++x)
				mPos[y * inWidth + x] = inOrigin + Vec3 { x * inSpacing, 0, y * inSpacing };
		mPrev = mPos;
 
		mConstraints.clear();
		auto idx = [&](uint32_t x, uint32_t y) { return y * inWidth + x; };
		for (uint32_t y = 0; y < inHeight; ++y)
			for (uint32_t x = 0; x < inWidth; ++x)
			{
				if (x + 1 < inWidth)  AddConstraint(idx(x, y), idx(x + 1, y), Stretch);
				if (y + 1 < inHeight) AddConstraint(idx(x, y), idx(x, y + 1), Stretch);
				if (x + 1 < inWidth && y + 1 < inHeight)
				{
					AddConstraint(idx(x, y), idx(x + 1, y + 1), Shear);
					AddConstraint(idx(x + 1, y), idx(x, y + 1), Shear);
				}
				if (x + 2 < inWidth)  AddConstraint(idx(x, y), idx(x + 2, y), Bend);
				if (y + 2 < inHeight) AddConstraint(idx(x, y), idx(x, y + 2), Bend);
			}
		BuildColours();
	}
 
	/// Pin a particle (infinite mass). Pinned particles can be moved with SetParticlePosition.
	void			Pin(uint32_t inIndex)							{ mInvMass[inIndex] = 0.0f; }
	void			Unpin(uint32_t inIndex, float inMass = 0.05f)	{ mInvMass[inIndex] = 1.0f / inMass; }
	void			SetParticlePosition(uint32_t inIndex, Vec3 inP) { mPos[inIndex] = mPrev[inIndex] = inP; }
 
	uint32_t		GetNumParticles() const							{ return (uint32_t)mPos.size(); }
	const std::vector<Vec3> &GetPositions() const				{ return mPos; }
	uint32_t		GetWidth() const								{ return mWidth; }
	uint32_t		GetHeight() const								{ return mHeight; }
 
	void Step(float inDeltaTime) {
		int substeps = std::max(1, mSettings.mSubsteps);
		float h = inDeltaTime / substeps;
		for (int s = 0; s < substeps; ++s)
		{
			Predict(h);
			for (auto &c : mConstraints) c.mLambda = 0.0f;
			for (int it = 0; it < mSettings.mIterations; ++it)
				for (const auto &colour : mColours)
					mParallelFor((uint32_t)colour.size(), [&](uint32_t i) { SolveConstraint(mConstraints[colour[i]], h); });
			Collide();
			UpdateVelocities(h);
		}
	}
 
private:
	enum Kind : uint8_t { Stretch, Shear, Bend, Twist }; 
 
	struct Constraint {
		uint32_t	mA, mB;
		float		mRest;
		float		mLambda = 0.0f;
		Kind		mKind;
	};
 
	void AddConstraint(uint32_t a, uint32_t b, Kind k)
	{
		mConstraints.push_back({ a, b, (mPos[a] - mPos[b]).Length(), 0.0f, k });
	}
 
	// Greedy graph colouring so constraints in one colour share no particles
	void BuildColours()
	{
		mColours.clear();
		std::vector<std::vector<uint8_t>> used(mPos.size());	// colours used per particle
		for (uint32_t i = 0; i < mConstraints.size(); ++i)
		{
			const auto &c = mConstraints[i];
			uint8_t col = 0;
			for (;; ++col)
			{
				bool taken = std::find(used[c.mA].begin(), used[c.mA].end(), col) != used[c.mA].end()
						  || std::find(used[c.mB].begin(), used[c.mB].end(), col) != used[c.mB].end();
				if (!taken) break;
			}
			used[c.mA].push_back(col); used[c.mB].push_back(col);
			if (col >= mColours.size()) mColours.resize(col + 1);
			mColours[col].push_back(i);
		}
	}
 
	void Predict(float h)
	{
		float damp = std::max(0.0f, 1.0f - mSettings.mLinearDamping * h);
		mParallelFor(GetNumParticles(), [&](uint32_t i)
		{
			mPrev[i] = mPos[i];
			if (mInvMass[i] == 0.0f) return;
			mVel[i] = (mVel[i] + mSettings.mGravity * h) * damp;
			mPos[i] = mPos[i] + mVel[i] * h;
		});

        float damp = std::max(0.0f, 1.0f - mSettings.mLinearDamping * h);
        mParallelFor(GetNumParticles(), [&](uint32_t i) {
            mPrev[i] = mPos[i];
            mRotPrev[i] = mRot[i]; // Cache last rotation frame

            if (mInvMass[i] == 0.0f) return;

            // Linear Update
            mVel[i] = (mVel[i] + mSettings.mGravity * h) * damp;
            mPos[i] = mPos[i] + mVel[i] * h;

            // Angular Update (No external gyroscopic torques applied for speed)
            if (mInvInertia[i] > 0.0f) {
                mAngVel[i] = mAngVel[i] * damp;
                mRot[i] = (mRot[i] * Quat::FromAngularVector(mAngVel[i] * h)).Normalized();
            }
        });
	}
 
	void SolveConstraint(Constraint &c, float h) {
        // Route out linear constraints first...
        if (c.mKind != Twist) {
            // ... Keep your existing linear distance math code here ...
            return;
        }

        // --- TWIST SOLVER CALCULATION ---
        float w = mInvInertia[c.mA] + mInvInertia[c.mB];
        if (w == 0.0f) return;

        // 1. Calculate relative orientation
        Quat deltaQ = mRot[c.mA].Conjugated() * mRot[c.mB];

        // 2. Extract twist angle around local forward vector (Assume local X axis)
        // 2 * atan2(qx, qw) extracts the single rotation angle matching the X axis frame
        float twistAngle = 2.0f * std::atan2(deltaQ.x, deltaQ.w);
        float C = twistAngle - c.mRest;

        // 3. XPBD compliance calculation
        float alpha = mSettings.mTwistCompliance / (h * h);
        float dLambda = (-C - alpha * c.mLambda) / (w + alpha);
        c.mLambda += dLambda;

        // 4. Transform torque axis to world coordinates and update orientations
        // For local-X axis constraint tracking, the torque alignment mirrors local frame projection
        if (mInvInertia[c.mA] > 0.0f) {
            Vec3 torqueA = { 1.0f, 0, 0 }; // Axis vector representation
            mRot[c.mA] = (mRot[c.mA] * Quat::FromAngularVector(torqueA * (mInvInertia[c.mA] * dLambda))).Normalized();
        }
        if (mInvInertia[c.mB] > 0.0f) {
            Vec3 torqueB = { -1.0f, 0, 0 };
            mRot[c.mB] = (mRot[c.mB] * Quat::FromAngularVector(torqueB * (mInvInertia[c.mB] * dLambda))).Normalized();
        }
    }
 
	void Collide()
	{
		float r = mSettings.mParticleRadius;
		mParallelFor(GetNumParticles(), [&](uint32_t i)
		{
			if (mInvMass[i] == 0.0f) return;
			Vec3 &p = mPos[i];
			for (const auto &s : mSpheres)
			{
				Vec3 d = p - s.mCenter;
				float len = d.Length(), min_dist = s.mRadius + r;
				if (len < min_dist && len > 1.0e-9f)
				{
					Vec3 n = d * (1.0f / len);
					ApplyFriction(i, n, min_dist - len, s.mFriction);
				}
			}
			for (const auto &pl : mPlanes)
			{
				float pen = pl.mOffset + r - pl.mNormal.Dot(p);
				if (pen > 0.0f) ApplyFriction(i, pl.mNormal, pen, pl.mFriction);
			}
		});
	}
 
	void ApplyFriction(uint32_t i, Vec3 n, float penetration, float friction)
	{
		mPos[i] = mPos[i] + n * penetration;
		// Position-level Coulomb-ish friction: damp tangential motion this substep
		Vec3 delta = mPos[i] - mPrev[i];
		Vec3 tangent = delta - n * delta.Dot(n);
		mPos[i] = mPos[i] - tangent * std::min(1.0f, friction);
	}
 
	void UpdateVelocities(float h) {
        float inv_h = 1.0f / h;
        mParallelFor(GetNumParticles(), [&](uint32_t i) {
            if (mInvMass[i] != 0.0f) 
                mVel[i] = (mPos[i] - mPrev[i]) * inv_h;

            if (mInvInertia[i] != 0.0f) {
                // Extract angular velocity vector from delta quaternion
                Quat diff = mRotPrev[i].Conjugated() * mRot[i];
                mAngVel[i] = Vec3{ diff.x, diff.y, diff.z } * (2.0f * inv_h);
            }
        });
    }
 
	uint32_t		mWidth = 0, mHeight = 0;
	std::vector<Vec3>	mPos, mPrev, mVel;
	std::vector<float>		mInvMass;
	std::vector<Constraint>	mConstraints;
	std::vector<std::vector<uint32_t>> mColours;

	std::vector<Quat> mRot, mRotPrev;
	std::vector<Vec3> mAngVel;
	std::vector<float>     mInvInertia = 0.0f; // 0.0f if rotationally pinned
};

MOSS_SUPPRESS_WARNINGS_END
*/