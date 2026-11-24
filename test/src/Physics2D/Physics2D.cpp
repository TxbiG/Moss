#include <Moss/Physics/Physics2D/Physics2D.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <set>
#include <unordered_map>
#include <utility>
#include <variant>
#include <limits>
#include <array>
#include <chrono>
#include <thread>
#include <atomic>
#include <numeric>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <exception>

Shape2D Shape2D::Polygon(const std::vector<Vec2>& points) {
    Shape2D s; s.type=ShapeType::Polygon;
    if (points.size()<3) return s;
    std::vector<Vec2> p; p.reserve(points.size());
    for (Vec2 v : points) if (std::isfinite(v.x) && std::isfinite(v.y)) p.push_back(v);
    if (p.size()<3) return s;
    std::sort(p.begin(),p.end(),[](Vec2 a,Vec2 b){ return a.x==b.x ? a.y<b.y : a.x<b.x; });
    p.erase(std::unique(p.begin(),p.end(),[](Vec2 a,Vec2 b){ return (a-b).LengthSquared()<1.0e-12f; }),p.end());
    if (p.size()<3) return s;
    std::vector<Vec2> hull;
    for (Vec2 v : p) {
        while (hull.size()>=2 && Cross(hull.back()-hull[hull.size()-2],v-hull.back())<=kEpsilon) hull.pop_back();
        hull.push_back(v);
    }
    const size_t lower=hull.size();
    for (int i=int(p.size())-2;i>=0;--i) {
        Vec2 v=p[size_t(i)];
        while (hull.size()>lower && Cross(hull.back()-hull[hull.size()-2],v-hull.back())<=kEpsilon) hull.pop_back();
        hull.push_back(v);
    }
    if (!hull.empty()) hull.pop_back();
    if (hull.size()<3) return s;
    s.polygon.vertices=hull;
    s.polygon.normals.reserve(hull.size());
    float area2=0.0f; Vec2 centroid{};
    for (size_t i=0;i<hull.size();++i) {
        Vec2 a=hull[i], b=hull[(i+1)%hull.size()];
        const float cr=Cross(a,b); area2+=cr; centroid+=(a+b)*cr;
        Vec2 e=b-a; s.polygon.normals.push_back(NormalizeOr({e.y,-e.x}));
    }
    if (std::abs(area2)>kEpsilon) centroid/= (3.0f*area2);
    s.polygon.centroid=centroid;
    return s;
}

Shape2D Shape2D::Capsule(Vec2 a,Vec2 b,float radius,int arcSegments) {
    radius=std::max(radius,kEpsilon); arcSegments=std::max(2,arcSegments);
    Vec2 axis=NormalizeOr(b-a); float angle=std::atan2(axis.y,axis.x);
    std::vector<Vec2> points; points.reserve(size_t(2*(arcSegments+1)));
    for(int i=0;i<=arcSegments;++i) { float t=-0.5f*kPi+kPi*float(i)/float(arcSegments); float q=angle+t; points.push_back(b+Vec2{std::cos(q),std::sin(q)}*radius); }
    for(int i=0;i<=arcSegments;++i) { float t=0.5f*kPi+kPi*float(i)/float(arcSegments); float q=angle+t; points.push_back(a+Vec2{std::cos(q),std::sin(q)}*radius); }
    return Polygon(points);
}
Shape2D Shape2D::RoundedBox(Vec2 half,float radius,Vec2 center,float angle,int arcSegments) {
    half.x=std::max(half.x,kEpsilon); half.y=std::max(half.y,kEpsilon);
    radius=Clamp(radius,0.0f,std::min(half.x,half.y)); arcSegments=std::max(1,arcSegments);
    if(radius<=kEpsilon) return Box(half,center,angle);
    std::vector<Vec2> points; points.reserve(size_t(4*(arcSegments+1)));
    const Vec2 corners[4]={{half.x-radius,half.y-radius},{-half.x+radius,half.y-radius},{-half.x+radius,-half.y+radius},{half.x-radius,-half.y+radius}};
    const float starts[4]={0.0f,0.5f*kPi,kPi,1.5f*kPi};
    for(int c=0;c<4;++c) for(int i=0;i<=arcSegments;++i) { float a=starts[c]+0.5f*kPi*float(i)/float(arcSegments); points.push_back(center+Rot2(angle).Apply(corners[c]+Vec2{std::cos(a),std::sin(a)}*radius)); }
    return Polygon(points);
}

Shape2D Shape2D::Compound(const std::vector<CompoundChild2D>& children) {
    Shape2D s; s.type=ShapeType::Compound; s.compound=std::make_shared<CompoundShape2D>();
    s.compound->children=children; return s;
}

bool Shape2D::IsValid() const {
    switch(type) {
    case ShapeType::Circle: return std::isfinite(circle.radius) && circle.radius>0.0f && std::isfinite(circle.center.x) && std::isfinite(circle.center.y);
    case ShapeType::Polygon: return polygon.vertices.size()>=3 && polygon.vertices.size()==polygon.normals.size() && std::all_of(polygon.vertices.begin(),polygon.vertices.end(),[](Vec2 p){return std::isfinite(p.x)&&std::isfinite(p.y);});
    case ShapeType::Edge: return std::isfinite(edge.a.x)&&std::isfinite(edge.a.y)&&std::isfinite(edge.b.x)&&std::isfinite(edge.b.y)&&(edge.b-edge.a).LengthSquared()>kEpsilon*kEpsilon;
    case ShapeType::Chain: return chain.vertices.size()>=2 && std::all_of(chain.vertices.begin(),chain.vertices.end(),[](Vec2 p){return std::isfinite(p.x)&&std::isfinite(p.y);});
    case ShapeType::Compound: return compound && !compound->children.empty() && std::all_of(compound->children.begin(),compound->children.end(),[](const CompoundChild2D& c){return c.shape.IsValid()&&std::isfinite(c.transform.position.x)&&std::isfinite(c.transform.position.y)&&std::isfinite(c.transform.angle);});
    }
    return false;
}

MassData ComputeMass(const Shape2D& shape, float density) {
    MassData md; density=std::max(0.0f,density);
    if (shape.type==ShapeType::Circle) {
        const float area=kPi*shape.circle.radius*shape.circle.radius;
        md.mass=density*area; md.center=shape.circle.center;
        md.inertia=md.mass*(0.5f*shape.circle.radius*shape.circle.radius+shape.circle.center.LengthSquared());
    } else if (shape.type==ShapeType::Compound && shape.compound) {
        Vec2 weighted{}; float total=0.0f, inertia=0.0f;
        for (const auto& child : shape.compound->children) {
            const float childDensity=(std::isfinite(child.density)&&child.density>=0.0f)?child.density:density;
            MassData part=ComputeMass(child.shape,childDensity); if(part.mass<=kEpsilon) continue;
            Vec2 centre=child.transform.position+Rot2(child.transform.angle).Apply(part.center);
            total+=part.mass; weighted+=centre*part.mass;
            inertia+=part.inertia+part.mass*(child.transform.position.LengthSquared()+2.0f*Dot(child.transform.position,Rot2(child.transform.angle).Apply(part.center)));
        }
        if(total>kEpsilon){md.mass=total;md.center=weighted/total;md.inertia=inertia;}
    } else if (shape.type==ShapeType::Polygon) {
        float area2=0.0f, inertia6=0.0f; Vec2 c{};
        const auto& v=shape.polygon.vertices;
        for (size_t i=0;i<v.size();++i) {
            const Vec2 a=v[i], b=v[(i+1)%v.size()];
            const float d=Cross(a,b); area2+=d; c+=(a+b)*d;
            const float intx=a.x*a.x+a.x*b.x+b.x*b.x;
            const float inty=a.y*a.y+a.y*b.y+b.y*b.y;
            inertia6+=d*(intx+inty);
        }
        if (std::abs(area2)>kEpsilon) {
            const float area=0.5f*area2;
            md.mass=density*std::abs(area);
            md.center=c/(3.0f*area2);
            md.inertia=density*std::abs(inertia6/12.0f);
        }
    }
    return md;
}

AABB2 ComputeAABB(const Shape2D& shape, const Transform2D& xf) {
    AABB2 aabb;
    if (shape.type==ShapeType::Circle) {
        Vec2 c=xf.Apply(shape.circle.center), r{shape.circle.radius,shape.circle.radius};
        return {c-r,c+r};
    }
    if (shape.type==ShapeType::Compound && shape.compound) {
        for(const auto& child:shape.compound->children){Transform2D cxf{xf.Apply(child.transform.position),WrapAngle(xf.angle+child.transform.angle)};AABB2 part=ComputeAABB(child.shape,cxf);if(part.IsValid()){aabb.Include(part.min);aabb.Include(part.max);}}
        if(!aabb.IsValid()) return {{0,0},{0,0}};
        aabb.Expand(0.002f); return aabb;
    }
    auto include=[&](Vec2 p){ aabb.Include(xf.Apply(p)); };
    if (shape.type==ShapeType::Polygon) for (Vec2 p:shape.polygon.vertices) include(p);
    else if (shape.type==ShapeType::Edge) { include(shape.edge.a); include(shape.edge.b); }
    else if (shape.type==ShapeType::Chain) for (Vec2 p:shape.chain.vertices) include(p);
    if (!aabb.IsValid()) return {{0,0},{0,0}};
    aabb.Expand(0.002f); return aabb;
}

bool TestPoint(const Shape2D& shape, const Transform2D& xf, Vec2 point) {
    Vec2 p=xf.ApplyInverse(point);
    if (shape.type==ShapeType::Circle) return (p-shape.circle.center).LengthSquared()<=shape.circle.radius*shape.circle.radius;
    if (shape.type==ShapeType::Compound && shape.compound) { for(const auto& child:shape.compound->children){Transform2D cxf{xf.Apply(child.transform.position),WrapAngle(xf.angle+child.transform.angle)};if(TestPoint(child.shape,cxf,point))return true;}return false;}
    if (shape.type==ShapeType::Polygon) {
        for (size_t i=0;i<shape.polygon.vertices.size();++i)
            if (Dot(shape.polygon.normals[i],p-shape.polygon.vertices[i])>0.0f) return false;
        return true;
    }
    if (shape.type==ShapeType::Edge) {
        Vec2 d=shape.edge.b-shape.edge.a; float t=Clamp(Dot(p-shape.edge.a,d)/std::max(d.LengthSquared(),kEpsilon),0.0f,1.0f);
        return (p-(shape.edge.a+t*d)).LengthSquared()<0.01f;
    }
    size_t chainCount=shape.chain.vertices.size()-1+(shape.chain.loop?1:0);
    for (size_t i=0;i<chainCount;++i) {
        Vec2 a=shape.chain.vertices[i], d=shape.chain.vertices[(i+1)%shape.chain.vertices.size()]-a;
        float t=Clamp(Dot(p-a,d)/std::max(d.LengthSquared(),kEpsilon),0.0f,1.0f);
        if ((p-(a+t*d)).LengthSquared()<0.01f) return true;
    }
    return false;
}

namespace {
struct BodyInternal {
    BodyState state;
    Shape2D shape;
    Material2D material;
    CollisionFilter filter;
    bool fixedRotation=false, allowSleep=true, oneWayPlatform=false;
    float gravityScale=1.0f, linearDamping=-1.0f, angularDamping=-1.0f;
    Vec2 oneWayNormal{0,1};
    Vec2 previousPosition{};
    float previousAngle=0.0f;
    float sleepTime=0.0f;
    BroadPhaseProxy2D broadPhaseProxy=InvalidBroadPhaseProxy2D;
};
struct JointInternal {
    JointType type=JointType::Distance;
    BodyID a=InvalidBodyID,b=InvalidBodyID;
    Vec2 anchorA{},anchorB{},axisA{1,0};
    float length=0, minLength=-1, maxLength=-1;
    float frequency=0,damping=0.7f;
    bool enableLimit=false,enableMotor=false,collideConnected=false;
    float lower=0,upper=0,motorSpeed=0,maxMotor=0,referenceAngle=0;
    Vec2 impulse{};
    float angularImpulse=0,motorImpulse=0,limitImpulse=0;
    bool enabled=true;
    Vec2 target{};
    float maxForce=1000.0f;
    float breakForce=std::numeric_limits<float>::infinity(), breakTorque=std::numeric_limits<float>::infinity();
    bool mouseAppliedThisStep=false;
    Vec2 groundAnchorA{},groundAnchorB{},linearOffset{};
    float ratio=1.0f,constant=0.0f,angularOffset=0.0f,correctionFactor=0.3f,maxTorque=0.0f;
};
struct ContactCacheKey {
    BodyID bodyA=InvalidBodyID,bodyB=InvalidBodyID;
    uint32_t childA=0xffffffffu,childB=0xffffffffu;
    bool operator<(const ContactCacheKey& other)const {
        if(bodyA!=other.bodyA)return bodyA<other.bodyA;
        if(bodyB!=other.bodyB)return bodyB<other.bodyB;
        if(childA!=other.childA)return childA<other.childA;
        return childB<other.childB;
    }
};
struct ContactConstraint {
    BodyID a=InvalidBodyID,b=InvalidBodyID;
    Manifold2D manifold;
    float friction=0.6f,restitution=0.0f;
    ContactCacheKey cacheKey{};
};
uint64_t PairKey(BodyID a,BodyID b) { if(a>b) std::swap(a,b); return (uint64_t(a)<<32)|uint64_t(b); }
bool ShouldCollide(const CollisionFilter& a,const CollisionFilter& b) {
    if (a.groupIndex!=0 && a.groupIndex==b.groupIndex) return a.groupIndex>0;
    return (a.maskBits&b.categoryBits)!=0 && (b.maskBits&a.categoryBits)!=0;
}
Vec2 WorldPoint(const BodyInternal& b,Vec2 p) { return b.state.transform.Apply(p); }
Vec2 WorldVector(const BodyInternal& b,Vec2 v) { return Rot2(b.state.transform.angle).Apply(v); }
float ShapeRadiusAboutOrigin(const Shape2D& shape,const Transform2D& transform) {
    const AABB2 bounds=ComputeAABB(shape,transform);
    if(!bounds.IsValid())return 0.0f;
    const Vec2 corners[4]={{bounds.min.x,bounds.min.y},{bounds.min.x,bounds.max.y},{bounds.max.x,bounds.min.y},{bounds.max.x,bounds.max.y}};
    float radiusSquared=0.0f;for(Vec2 point:corners)radiusSquared=std::max(radiusSquared,(point-transform.position).LengthSquared());
    return std::sqrt(radiusSquared);
}
AABB2 ComputeShapeSweepBounds(const Shape2D& shape,const Transform2D& start,Vec2 translation,float rotationDelta,float margin) {
    Transform2D end=start;end.position+=translation;end.angle=WrapAngle(end.angle+rotationDelta);
    AABB2 swept=ComputeAABB(shape,start);const AABB2 endBounds=ComputeAABB(shape,end);swept.Include(endBounds.min);swept.Include(endBounds.max);
    if(std::abs(rotationDelta)>1.0e-6f){
        // Endpoint AABBs alone are insufficient for a half/full turn: an elongated
        // shape can sweep through an obstacle while both endpoint boxes miss it.
        const float radius=ShapeRadiusAboutOrigin(shape,start);
        const Vec2 lo=Min(start.position,end.position)-Vec2(radius,radius);
        const Vec2 hi=Max(start.position,end.position)+Vec2(radius,radius);
        swept.Include(lo);swept.Include(hi);
    }
    swept.Expand(std::max(0.0f,margin));return swept;
}
std::vector<Vec2> WorldVertices(const PolygonShape& p,const Transform2D& xf) {
    std::vector<Vec2> v; v.reserve(p.vertices.size()); for(Vec2 x:p.vertices) v.push_back(xf.Apply(x)); return v;
}
Vec2 ClosestPointSegment(Vec2 p,Vec2 a,Vec2 b) {
    Vec2 d=b-a; float t=Clamp(Dot(p-a,d)/std::max(d.LengthSquared(),kEpsilon),0.0f,1.0f); return a+t*d;
}
bool CollideCircleCircle(const CircleShape& a,const Transform2D& xa,const CircleShape& b,const Transform2D& xb,Manifold2D& m) {
    Vec2 ca=xa.Apply(a.center), cb=xb.Apply(b.center), d=cb-ca;
    const float r=a.radius+b.radius, d2=d.LengthSquared(); if(d2>r*r) return false;
    float dist=std::sqrt(std::max(d2,0.0f)); m.normal=dist>kEpsilon?d/dist:Vec2{1,0};
    m.penetration=r-dist; m.pointCount=1; m.points[0].point=ca+m.normal*(a.radius-0.5f*m.penetration); m.points[0].separation=-m.penetration; m.points[0].featureId=0x10000001u; return true;
}
bool CollideCirclePolygon(const CircleShape& circle,const Transform2D& xc,const PolygonShape& poly,const Transform2D& xp,Manifold2D& m) {
    Vec2 c=xc.Apply(circle.center); auto v=WorldVertices(poly,xp); bool inside=true; float maxSep=-std::numeric_limits<float>::max(); size_t face=0;
    for(size_t i=0;i<v.size();++i) {
        Vec2 n=Rot2(xp.angle).Apply(poly.normals[i]); float s=Dot(n,c-v[i]);
        if(s>0) inside=false;
        if(s>maxSep) {maxSep=s;face=i;}
    }
    Vec2 closest{}; float best=std::numeric_limits<float>::max(); size_t closestFace=0;
    for(size_t i=0;i<v.size();++i) {
        Vec2 q=ClosestPointSegment(c,v[i],v[(i+1)%v.size()]); float d=(c-q).LengthSquared();
        if(d<best) {best=d;closest=q;closestFace=i;}
    }
    const float dist=std::sqrt(best);
    if(!inside && dist>circle.radius) return false;
    Vec2 n;
    if(inside) n=-Rot2(xp.angle).Apply(poly.normals[face]);
    else n=dist>kEpsilon?(closest-c)/dist:Rot2(xp.angle).Apply(poly.normals[face]);
    m.normal=n; m.penetration=inside?circle.radius+std::max(0.0f,-maxSep):circle.radius-dist;
    m.pointCount=1; m.points[0].point=closest; m.points[0].separation=-m.penetration; m.points[0].featureId=0x20000000u|uint32_t(closestFace&0xffffu); return true;
}
bool FindPolySeparation(const std::vector<Vec2>& a,const PolygonShape& pa,const Transform2D& xa,
                        const std::vector<Vec2>& b,float& bestSep,Vec2& bestNormal) {
    bestSep=-std::numeric_limits<float>::max();
    for(size_t i=0;i<pa.vertices.size();++i) {
        Vec2 n=Rot2(xa.angle).Apply(pa.normals[i]);
        Vec2 va=a[i]; float minB=std::numeric_limits<float>::max();
        for(Vec2 p:b) minB=std::min(minB,Dot(n,p-va));
        if(minB>bestSep) {bestSep=minB;bestNormal=n;}
        if(minB>0) return false;
    }
    return true;
}
struct ClipVertex2D { Vec2 point{}; uint16_t feature=0; };
int ClipSegmentToLine(const ClipVertex2D input[2], ClipVertex2D output[2], Vec2 normal, float offset, uint16_t planeFeature) {
    int count=0; float d0=Dot(normal,input[0].point)-offset, d1=Dot(normal,input[1].point)-offset;
    if(d0<=0.0f)output[count++]=input[0];
    if(d1<=0.0f)output[count++]=input[1];
    if(d0*d1<0.0f){float t=d0/(d0-d1);output[count++]={input[0].point+(input[1].point-input[0].point)*t,uint16_t(0x8000u|(planeFeature<<12)|(input[0].feature&0x0fffu))};}
    return count;
}

bool CollidePolygonPolygon(const PolygonShape& pa,const Transform2D& xa,const PolygonShape& pb,const Transform2D& xb,Manifold2D& m) {
    auto a=WorldVertices(pa,xa), b=WorldVertices(pb,xb); float sa,sb; Vec2 na,nb;
    if(!FindPolySeparation(a,pa,xa,b,sa,na) || !FindPolySeparation(b,pb,xb,a,sb,nb)) return false;
    const bool referenceA=sa>=sb;
    const auto& refPoly=referenceA?pa:pb; const auto& incPoly=referenceA?pb:pa;
    const auto& refVerts=referenceA?a:b; const auto& incVerts=referenceA?b:a;
    Vec2 refNormal=referenceA?na:nb;
    if(!referenceA)refNormal=-refNormal;
    Vec2 centreA=xa.Apply(pa.centroid),centreB=xb.Apply(pb.centroid);
    if(Dot(refNormal,referenceA?(centreB-centreA):(centreA-centreB))<0)refNormal=-refNormal;
    size_t refFace=0;float best=-std::numeric_limits<float>::max();
    for(size_t i=0;i<refPoly.vertices.size();++i){Vec2 n=Rot2(referenceA?xa.angle:xb.angle).Apply(refPoly.normals[i]);float d=Dot(n,refNormal);if(d>best){best=d;refFace=i;}}
    Vec2 v1=refVerts[refFace],v2=refVerts[(refFace+1)%refVerts.size()];
    Vec2 tangent=NormalizeOr(v2-v1); refNormal=NormalizeOr(Vec2{tangent.y,-tangent.x});
    // Respect the SAT-selected outward direction when polygon winding is reversed by transforms.
    if(Dot(refNormal,referenceA?na:nb)<0.0f)refNormal=-refNormal;
    size_t incFace=0;float minDot=std::numeric_limits<float>::max();
    for(size_t i=0;i<incPoly.vertices.size();++i){Vec2 n=Rot2(referenceA?xb.angle:xa.angle).Apply(incPoly.normals[i]);float d=Dot(n,refNormal);if(d<minDot){minDot=d;incFace=i;}}
    ClipVertex2D incident[2]={{incVerts[incFace],uint16_t(incFace&0x7fffu)},{incVerts[(incFace+1)%incVerts.size()],uint16_t((incFace+1)%incVerts.size())}};
    ClipVertex2D clip1[2],clip2[2];
    if(ClipSegmentToLine(incident,clip1,-tangent,-Dot(tangent,v1),1)<2)return false;
    if(ClipSegmentToLine(clip1,clip2,tangent,Dot(tangent,v2),2)<2)return false;
    m.normal=referenceA?refNormal:-refNormal;m.pointCount=0;m.penetration=0;
    for(int i=0;i<2;++i){float separation=Dot(refNormal,clip2[i].point-v1);if(separation<=0.01f){ContactPoint2D&cp=m.points[m.pointCount++];cp.separation=separation;cp.point=clip2[i].point-refNormal*(0.5f*separation);const uint32_t fa=uint32_t(referenceA?refFace:incFace);const uint32_t fb=uint32_t(referenceA?incFace:refFace);uint32_t feature=0x30000000u;feature=(feature^((fa+1u)*0x9e3779b9u))*16777619u;feature=(feature^((fb+1u)*0x85ebca6bu))*16777619u;feature=(feature^uint32_t(clip2[i].feature))*16777619u;cp.featureId=feature;m.penetration=std::max(m.penetration,std::max(0.0f,-separation));}}
    return m.pointCount>0;
}
Vec2 SmoothChainNormal(const EdgeShape& edge,const Transform2D& xf,Vec2 queryPoint,float t,Vec2 front) {
    const Vec2 a=xf.Apply(edge.a),b=xf.Apply(edge.b);
    Vec2 normal=front;
    if(t<=0.015f&&edge.hasGhostA) {
        const Vec2 previous=xf.Apply(edge.ghostA);
        const Vec2 previousTangent=NormalizeOr(a-previous);
        const Vec2 previousFront={previousTangent.y,-previousTangent.x};
        const Vec2 candidate=NormalizeOr(queryPoint-a,front);
        // Keep the contact normal inside the adjacent-face normal cone. This
        // prevents a polygon/circle from reacting to the hidden back-side of an
        // internal chain vertex while still preserving convex-corner response.
        if(Dot(candidate,front)>=0.0f&&Dot(candidate,previousFront)>=0.0f) normal=candidate;
        else normal=Dot(candidate,front)>=Dot(candidate,previousFront)?front:previousFront;
    } else if(t>=0.985f&&edge.hasGhostB) {
        const Vec2 next=xf.Apply(edge.ghostB);
        const Vec2 nextTangent=NormalizeOr(next-b);
        const Vec2 nextFront={nextTangent.y,-nextTangent.x};
        const Vec2 candidate=NormalizeOr(queryPoint-b,front);
        if(Dot(candidate,front)>=0.0f&&Dot(candidate,nextFront)>=0.0f) normal=candidate;
        else normal=Dot(candidate,front)>=Dot(candidate,nextFront)?front:nextFront;
    }
    return NormalizeOr(normal,front);
}
bool CollideEdgeCircle(const EdgeShape& edge,const Transform2D& xe,const CircleShape& circle,const Transform2D& xc,Manifold2D& m) {
    Vec2 a=xe.Apply(edge.a), b=xe.Apply(edge.b), c=xc.Apply(circle.center); Vec2 segment=b-a;
    const float length2=segment.LengthSquared();
    const float t=Clamp(Dot(c-a,segment)/std::max(length2,kEpsilon),0.0f,1.0f);
    Vec2 q=a+segment*t,d=c-q;
    float ds=d.LengthSquared(); if(ds>circle.radius*circle.radius) return false;
    Vec2 tangent=NormalizeOr(segment),front=Vec2{tangent.y,-tangent.x};
    if(edge.oneSided && Dot(front,c-a)<-0.005f) return false;
    float dist=std::sqrt(std::max(ds,0.0f));
    Vec2 n=dist>kEpsilon?d/dist:front;
    if(edge.oneSided)n=SmoothChainNormal(edge,xe,c,t,front);
    else if(dist<=kEpsilon)n=front;
    if(edge.oneSided && Dot(n,front)<0.0f) n=front;
    m.normal=n; m.penetration=circle.radius-dist; m.pointCount=1; m.points[0].point=q; m.points[0].separation=-m.penetration;
    const uint32_t feature=t<=0.015f?0u:(t>=0.985f?1u:2u);
    m.points[0].featureId=0x40000000u|feature; return true;
}
bool CollideEdgePolygon(const EdgeShape& edge,const Transform2D& xe,const PolygonShape& poly,const Transform2D& xp,Manifold2D& m) {
    Vec2 a=xe.Apply(edge.a), b=xe.Apply(edge.b), tangent=NormalizeOr(b-a), front={tangent.y,-tangent.x};
    auto v=WorldVertices(poly,xp); Vec2 center{}; for(Vec2 p:v) center+=p; center/=float(v.size());
    const float side=Dot(front,center-a);
    if(edge.oneSided && side < -0.005f) return false;
    Vec2 n=(edge.oneSided || side>=0.0f)?front:-front;
    float minD=std::numeric_limits<float>::max(); Vec2 point{};size_t pointIndex=0;
    for(size_t i=0;i<v.size();++i) { float d=Dot(n,v[i]-a); if(d<minD) {minD=d;point=v[i];pointIndex=i;} }
    if(minD>0.01f) return false;
    const float edgeLength=(b-a).Length();
    const float along=Dot(point-a,tangent);
    if(along < -0.05f || along > edgeLength+0.05f) return false;
    const float t=Clamp(along/std::max(edgeLength,kEpsilon),0.0f,1.0f);
    if(edge.oneSided)n=SmoothChainNormal(edge,xe,center,t,front);
    m.normal=n; m.penetration=std::max(0.0f,-minD); m.pointCount=1;
    m.points[0].point=point-n*minD; m.points[0].separation=-m.penetration;
    const uint32_t feature=t<=0.015f?0u:(t>=0.985f?1u:uint32_t(2u+(pointIndex&0xffffu)));
    m.points[0].featureId=0x50000000u|feature; return true;
}

bool Collide(const Shape2D& a,const Transform2D& xa,const Shape2D& b,const Transform2D& xb,Manifold2D& m) {
    m={};
    if(a.type==ShapeType::Compound && a.compound){bool hit=false;for(const auto&child:a.compound->children){Transform2D cxf{xa.Apply(child.transform.position),WrapAngle(xa.angle+child.transform.angle)};Manifold2D t;if(Collide(child.shape,cxf,b,xb,t)&&(!hit||t.penetration>m.penetration)){m=t;hit=true;}}return hit;}
    if(b.type==ShapeType::Compound && b.compound){bool hit=false;for(const auto&child:b.compound->children){Transform2D cxf{xb.Apply(child.transform.position),WrapAngle(xb.angle+child.transform.angle)};Manifold2D t;if(Collide(a,xa,child.shape,cxf,t)&&(!hit||t.penetration>m.penetration)){m=t;hit=true;}}return hit;}
    if(a.type==ShapeType::Chain) {
        const auto& ch=a.chain; int count=int(ch.vertices.size())-1+(ch.loop?1:0); bool hit=false;
        for(int i=0;i<count;++i) {
            EdgeShape e; e.a=ch.vertices[size_t(i)]; e.b=ch.vertices[size_t((i+1)%ch.vertices.size())]; e.oneSided=ch.oneSided;
            if(ch.loop || i>0) {e.ghostA=ch.vertices[size_t((i+int(ch.vertices.size())-1)%int(ch.vertices.size()))];e.hasGhostA=true;}
            if(ch.loop || i+1<count) {e.ghostB=ch.vertices[size_t((i+2)%ch.vertices.size())];e.hasGhostB=true;}
            Shape2D es=Shape2D::Edge(e.a,e.b); es.edge=e; Manifold2D t;
            if(Collide(es,xa,b,xb,t) && (!hit || t.penetration>m.penetration)) {m=t;hit=true;}
        } return hit;
    }
    if(b.type==ShapeType::Chain) { bool h=Collide(b,xb,a,xa,m); m.normal=-m.normal; return h; }
    if(a.type==ShapeType::Circle && b.type==ShapeType::Circle) return CollideCircleCircle(a.circle,xa,b.circle,xb,m);
    if(a.type==ShapeType::Circle && b.type==ShapeType::Polygon) return CollideCirclePolygon(a.circle,xa,b.polygon,xb,m);
    if(a.type==ShapeType::Polygon && b.type==ShapeType::Circle) { bool h=CollideCirclePolygon(b.circle,xb,a.polygon,xa,m); m.normal=-m.normal; return h; }
    if(a.type==ShapeType::Polygon && b.type==ShapeType::Polygon) return CollidePolygonPolygon(a.polygon,xa,b.polygon,xb,m);
    if(a.type==ShapeType::Edge && b.type==ShapeType::Circle) return CollideEdgeCircle(a.edge,xa,b.circle,xb,m);
    if(a.type==ShapeType::Circle && b.type==ShapeType::Edge) { bool h=CollideEdgeCircle(b.edge,xb,a.circle,xa,m); m.normal=-m.normal; return h; }
    if(a.type==ShapeType::Edge && b.type==ShapeType::Polygon) return CollideEdgePolygon(a.edge,xa,b.polygon,xb,m);
    if(a.type==ShapeType::Polygon && b.type==ShapeType::Edge) { bool h=CollideEdgePolygon(b.edge,xb,a.polygon,xa,m); m.normal=-m.normal; return h; }
    return false;
}

void CollideAll(const Shape2D& a,const Transform2D& xa,const Shape2D& b,const Transform2D& xb,
                std::vector<Manifold2D>& out,SubShapePath2D pathA={},SubShapePath2D pathB={},
                uint16_t rootIndexA=0xffffu,uint16_t rootIndexB=0xffffu,
                void* userDataA=nullptr,void* userDataB=nullptr) {
    if(a.type==ShapeType::Compound && a.compound) {
        for(size_t i=0;i<a.compound->children.size();++i) {
            const auto& child=a.compound->children[i];
            Transform2D cxf{xa.Apply(child.transform.position),WrapAngle(xa.angle+child.transform.angle)};
            const uint32_t cid=child.id==0xffffffffu?uint32_t(i):child.id;
            SubShapePath2D next=pathA; if(!next.Push(cid)) continue;
            const uint16_t root=rootIndexA==0xffffu?uint16_t(std::min<size_t>(i,0xfffeu)):rootIndexA;
            CollideAll(child.shape,cxf,b,xb,out,next,pathB,root,rootIndexB,child.userData,userDataB);
        }
        return;
    }
    if(b.type==ShapeType::Compound && b.compound) {
        for(size_t i=0;i<b.compound->children.size();++i) {
            const auto& child=b.compound->children[i];
            Transform2D cxf{xb.Apply(child.transform.position),WrapAngle(xb.angle+child.transform.angle)};
            const uint32_t cid=child.id==0xffffffffu?uint32_t(i):child.id;
            SubShapePath2D next=pathB; if(!next.Push(cid)) continue;
            const uint16_t root=rootIndexB==0xffffu?uint16_t(std::min<size_t>(i,0xfffeu)):rootIndexB;
            CollideAll(a,xa,child.shape,cxf,out,pathA,next,rootIndexA,root,userDataA,child.userData);
        }
        return;
    }
    auto makeEdge=[&](const ChainShape& chain,int i) {
        const int count=int(chain.vertices.size())-1+(chain.loop?1:0);
        EdgeShape edge; edge.a=chain.vertices[size_t(i)]; edge.b=chain.vertices[size_t((i+1)%chain.vertices.size())]; edge.oneSided=chain.oneSided;
        if(chain.loop || i>0) { edge.ghostA=chain.vertices[size_t((i+int(chain.vertices.size())-1)%int(chain.vertices.size()))]; edge.hasGhostA=true; }
        if(chain.loop || i+1<count) { edge.ghostB=chain.vertices[size_t((i+2)%chain.vertices.size())]; edge.hasGhostB=true; }
        return edge;
    };
    if(a.type==ShapeType::Chain) {
        const auto& ch=a.chain; const int count=int(ch.vertices.size())-1+(ch.loop?1:0);
        for(int i=0;i<count;++i) {
            EdgeShape edge=makeEdge(ch,i); Shape2D segment=Shape2D::Edge(edge.a,edge.b);segment.edge=edge;
            SubShapePath2D next=pathA; if(!next.Push(uint32_t(i)))continue;
            CollideAll(segment,xa,b,xb,out,next,pathB,rootIndexA,rootIndexB,userDataA,userDataB);
        }
        return;
    }
    if(b.type==ShapeType::Chain) {
        const auto& ch=b.chain; const int count=int(ch.vertices.size())-1+(ch.loop?1:0);
        for(int i=0;i<count;++i) {
            EdgeShape edge=makeEdge(ch,i); Shape2D segment=Shape2D::Edge(edge.a,edge.b);segment.edge=edge;
            SubShapePath2D next=pathB; if(!next.Push(uint32_t(i)))continue;
            CollideAll(a,xa,segment,xb,out,pathA,next,rootIndexA,rootIndexB,userDataA,userDataB);
        }
        return;
    }
    Manifold2D manifold;
    if(Collide(a,xa,b,xb,manifold)) {
        manifold.childIndexA=rootIndexA; manifold.childIndexB=rootIndexB;
        manifold.childPathA=pathA; manifold.childPathB=pathB;
        manifold.childIdA=pathA.depth?(pathA.depth==1?pathA.ids[0]:pathA.Hash()):0xffffffffu;
        manifold.childIdB=pathB.depth?(pathB.depth==1?pathB.ids[0]:pathB.Hash()):0xffffffffu;
        manifold.childUserDataA=userDataA; manifold.childUserDataB=userDataB;
        for(uint8_t i=0;i<manifold.pointCount;++i) {
            uint32_t h=2166136261u;
            h=(h^manifold.points[i].featureId)*16777619u;
            h=(h^manifold.childIdA)*16777619u;
            h=(h^manifold.childIdB)*16777619u;
            manifold.points[i].featureId=h;
        }
        out.push_back(manifold);
    }
}

struct ResolvedChildMetadata {
    CollisionFilter filter{};
    bool isSensor=false;
    float friction=-1.0f,restitution=-1.0f,density=-1.0f;
    void* userData=nullptr;
};
ResolvedChildMetadata ResolveChildMetadata(const Shape2D& root,const SubShapePath2D& path,const CollisionFilter& bodyFilter) {
    ResolvedChildMetadata out; out.filter=bodyFilter;
    const Shape2D* shape=&root;
    for(uint8_t depth=0;depth<path.depth;++depth) {
        if(shape->type!=ShapeType::Compound||!shape->compound)break;
        const auto& children=shape->compound->children;
        const CompoundChild2D* selected=nullptr;
        for(size_t i=0;i<children.size();++i) {
            const uint32_t childID=children[i].id==0xffffffffu?uint32_t(i):children[i].id;
            if(childID==path.ids[depth]){selected=&children[i];break;}
        }
        if(!selected)break;
        out.filter.categoryBits=uint16_t(out.filter.categoryBits&selected->categoryBits);
        out.filter.maskBits=uint16_t(out.filter.maskBits&selected->maskBits);
        if(selected->groupIndex!=0)out.filter.groupIndex=selected->groupIndex;
        out.isSensor=out.isSensor||selected->isSensor;
        if(selected->friction>=0.0f)out.friction=selected->friction;
        if(selected->restitution>=0.0f)out.restitution=selected->restitution;
        if(selected->density>=0.0f)out.density=selected->density;
        out.userData=selected->userData;
        shape=&selected->shape;
    }
    return out;
}

float InvMass(const BodyInternal& b) { return b.state.awake && b.state.type==BodyType::Dynamic ? b.state.inverseMass : 0.0f; }
float InvI(const BodyInternal& b) { return b.state.awake && b.state.type==BodyType::Dynamic ? b.state.inverseInertia : 0.0f; }
void ApplyImpulse(BodyInternal& b,Vec2 impulse,Vec2 r,float sign) {
    if(b.state.type!=BodyType::Dynamic || !b.state.awake) return;
    b.state.linearVelocity += impulse*(sign*b.state.inverseMass);
    b.state.angularVelocity += sign*b.state.inverseInertia*Cross(r,impulse);
}
struct CachedImpulse { uint32_t feature[2]{0,0}; float normal[2]{0,0}; float tangent[2]{0,0}; };

bool RayCircle(Vec2 p1,Vec2 p2,Vec2 center,float radius,float& fraction,Vec2& normal) {
    Vec2 d=p2-p1, m=p1-center; float a=Dot(d,d); if(a<kEpsilon) return false;
    float b=Dot(m,d), c=Dot(m,m)-radius*radius; float disc=b*b-a*c; if(disc<0) return false;
    float t=(-b-std::sqrt(disc))/a; if(t<0 || t>1) { t=(-b+std::sqrt(disc))/a; if(t<0||t>1) return false; }
    fraction=t; normal=NormalizeOr(p1+d*t-center); return true;
}
bool RaySegment(Vec2 p1,Vec2 p2,Vec2 a,Vec2 b,float& fraction,Vec2& normal) {
    Vec2 r=p2-p1,s=b-a; float den=Cross(r,s); if(std::abs(den)<kEpsilon) return false;
    float t=Cross(a-p1,s)/den, u=Cross(a-p1,r)/den; if(t<0||t>1||u<0||u>1) return false;
    fraction=t; Vec2 edge=NormalizeOr(s); normal={edge.y,-edge.x}; if(Dot(normal,r)>0) normal=-normal; return true;
}
bool RayShape(const Shape2D& shape,const Transform2D& xf,Vec2 p1,Vec2 p2,float& fraction,Vec2& normal) {
    if(shape.type==ShapeType::Compound && shape.compound){bool hit=false;float best=1.0f;Vec2 bestNormal{};for(const auto&child:shape.compound->children){Transform2D cxf{xf.Apply(child.transform.position),WrapAngle(xf.angle+child.transform.angle)};float t;Vec2 n;if(RayShape(child.shape,cxf,p1,p2,t,n)&&t<best){hit=true;best=t;bestNormal=n;}}if(hit){fraction=best;normal=bestNormal;}return hit;}
    if(shape.type==ShapeType::Circle) return RayCircle(p1,p2,xf.Apply(shape.circle.center),shape.circle.radius,fraction,normal);
    if(shape.type==ShapeType::Edge) return RaySegment(p1,p2,xf.Apply(shape.edge.a),xf.Apply(shape.edge.b),fraction,normal);
    if(shape.type==ShapeType::Chain) {
        bool hit=false; float best=1.0f; Vec2 n;
        int count=int(shape.chain.vertices.size())-1+(shape.chain.loop?1:0);
        for(int i=0;i<count;++i) { float t; Vec2 nn; if(RaySegment(p1,p2,xf.Apply(shape.chain.vertices[size_t(i)]),xf.Apply(shape.chain.vertices[size_t((i+1)%shape.chain.vertices.size())]),t,nn)&&t<best) {hit=true;best=t;n=nn;} }
        if(hit){fraction=best;normal=n;} return hit;
    }
    auto v=WorldVertices(shape.polygon,xf); float tmin=0,tmax=1; Vec2 hitNormal{}; Vec2 d=p2-p1;
    for(size_t i=0;i<v.size();++i) { Vec2 n=Rot2(xf.angle).Apply(shape.polygon.normals[i]); float numerator=Dot(n,v[i]-p1), denominator=Dot(n,d);
        if(std::abs(denominator)<kEpsilon) { if(numerator<0) return false; continue; }
        float t=numerator/denominator;
        if(denominator<0) { if(t>tmin){tmin=t;hitNormal=n;} }
        else tmax=std::min(tmax,t);
        if(tmin>tmax) return false;
    }
    if(tmin<0||tmin>1) return false;
    fraction=tmin; normal=hitNormal.LengthSquared()>kEpsilon?hitNormal:NormalizeOr(-d,{0,1}); return true;
}
}

class PersistentWorkerPool2D {
public:
    PersistentWorkerPool2D() = default;
    ~PersistentWorkerPool2D() { Stop(); }
    PersistentWorkerPool2D(const PersistentWorkerPool2D&) = delete;
    PersistentWorkerPool2D& operator=(const PersistentWorkerPool2D&) = delete;

    void Configure(size_t workerCount) {
        workerCount = workerCount > 0 ? workerCount : 0;
        if (workerCount == workers_.size()) return;
        Stop();
        stopping_ = false;
        workers_.reserve(workerCount);
        for (size_t i = 0; i < workerCount; ++i) {
            workers_.emplace_back([this] {
                for (;;) {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock(mutex_);
                        wake_.wait(lock, [this] { return stopping_ || !tasks_.empty(); });
                        if (stopping_ && tasks_.empty()) return;
                        task = std::move(tasks_.front());
                        tasks_.pop_front();
                    }
                    task();
                }
            });
        }
    }

    size_t WorkerCount() const { return workers_.size(); }

    template<class Function>
    void ParallelFor(size_t count, Function&& function) {
        if (count == 0) return;
        if (workers_.empty() || count < 2) {
            for (size_t i = 0; i < count; ++i) function(i);
            return;
        }
        struct Batch {
            std::atomic<size_t> next{0};
            size_t count = 0;
            std::function<void(size_t)> function;
            std::mutex mutex;
            std::condition_variable done;
            size_t pending = 0;
            std::exception_ptr error;
        };
        auto batch = std::make_shared<Batch>();
        batch->count = count;
        batch->function = std::forward<Function>(function);
        const size_t tasks = std::min(workers_.size(), count);
        batch->pending = tasks;
        auto run = [batch] {
            try {
                for (;;) {
                    const size_t i = batch->next.fetch_add(1, std::memory_order_relaxed);
                    if (i >= batch->count) break;
                    batch->function(i);
                }
            } catch (...) {
                std::lock_guard<std::mutex> lock(batch->mutex);
                if (!batch->error) batch->error = std::current_exception();
                batch->next.store(batch->count, std::memory_order_relaxed);
            }
            {
                std::lock_guard<std::mutex> lock(batch->mutex);
                --batch->pending;
            }
            batch->done.notify_one();
        };
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (size_t i = 0; i < tasks; ++i) tasks_.emplace_back(run);
        }
        wake_.notify_all();
        {
            std::unique_lock<std::mutex> lock(batch->mutex);
            batch->done.wait(lock, [&] { return batch->pending == 0; });
        }
        if (batch->error) std::rethrow_exception(batch->error);
    }

private:
    void Stop() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        wake_.notify_all();
        for (auto& worker : workers_) if (worker.joinable()) worker.join();
        workers_.clear();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            tasks_.clear();
            stopping_ = false;
        }
    }
    std::vector<std::thread> workers_;
    std::deque<std::function<void()>> tasks_;
    std::mutex mutex_;
    std::condition_variable wake_;
    bool stopping_ = false;
};

struct PhysicsWorld2D::Impl {
    WorldSettings settings;
    std::vector<BodyInternal> bodies;
    std::vector<JointInternal> joints;
    DynamicAABBTree2D broadPhaseTree{0.08f, 2.0f};
    PersistentWorkerPool2D workerPool;
    float accumulator=0.0f, alpha=0.0f;
    uint64_t stepCount=0;
    std::vector<ContactEvent> beginEvents,endEvents,activeContactEvents;
    std::function<void(const ContactEvent&,bool)> listener;
    std::function<bool(BodyID,BodyID,const Manifold2D&)> preSolve;
    std::map<ContactCacheKey,CachedImpulse> impulseCache;
    std::set<uint64_t> previousPairs;
    std::set<uint64_t> previousSensorPairs;
    std::vector<ContactConstraint> contacts;
    std::vector<JointBreakEvent2D> jointBreakEvents;
    uint32_t lastCandidateCount=0,lastCCDSubSteps=1,lastIslandCount=0,lastCCDImpactCount=0,lastCCDOverflowCount=0;
    std::atomic<uint64_t> simdBatchCounter{0};
    PhysicsProfile2D profile{};
    bool stepping=false;

    BodyInternal* Body(BodyID id) { return id<bodies.size() && bodies[id].state.id==id ? &bodies[id] : nullptr; }
    const BodyInternal* Body(BodyID id) const { return id<bodies.size() && bodies[id].state.id==id ? &bodies[id] : nullptr; }
    void RecomputeMass(BodyInternal& b,float density) {
        MassData md=ComputeMass(b.shape,density);
        b.state.localCenter=md.center;
        b.state.worldCenter=b.state.transform.Apply(md.center);
        if(b.state.type!=BodyType::Dynamic) { b.state.mass=b.state.inverseMass=b.state.inertia=b.state.inverseInertia=0; return; }
        b.state.mass=std::max(md.mass,kEpsilon); b.state.inverseMass=1.0f/b.state.mass;
        const float centerInertia=std::max(0.0f, md.inertia-md.mass*md.center.LengthSquared());
        b.state.inertia=b.fixedRotation?0.0f:std::max(centerInertia,kEpsilon);
        b.state.inverseInertia=(b.fixedRotation||centerInertia<=kEpsilon)?0.0f:1.0f/centerInertia;
    }
    void Wake(BodyInternal& b) { if(b.state.type==BodyType::Dynamic) { b.state.awake=true; b.sleepTime=0.0f; } }
    static Vec2 WorldCenter(const BodyInternal& b) { return b.state.transform.Apply(b.state.localCenter); }
    void SyncProxy(BodyInternal& b) {
        if(b.state.id==InvalidBodyID || !b.state.enabled) {
            if(b.broadPhaseProxy!=InvalidBroadPhaseProxy2D) { broadPhaseTree.DestroyProxy(b.broadPhaseProxy); b.broadPhaseProxy=InvalidBroadPhaseProxy2D; }
            return;
        }
        const AABB2 bounds=ComputeAABB(b.shape,b.state.transform);
        const BroadPhaseLayer2D layer=b.state.type==BodyType::Static?BroadPhaseLayer2D::Static:BroadPhaseLayer2D::Moving;
        if(b.broadPhaseProxy==InvalidBroadPhaseProxy2D)
            b.broadPhaseProxy=broadPhaseTree.CreateProxy(bounds,b.state.id,layer);
        else {
            broadPhaseTree.UpdateProxyLayer(b.broadPhaseProxy,layer);
            broadPhaseTree.MoveProxy(b.broadPhaseProxy,bounds,b.state.linearVelocity*settings.fixedTimeStep);
        }
        b.state.worldCenter=WorldCenter(b);
    }
    void ApplyPositionImpulse(BodyInternal& b,Vec2 impulse,Vec2 r,float sign) {
        if(b.state.type!=BodyType::Dynamic || b.state.inverseMass<=0.0f) return;
        Vec2 center=WorldCenter(b)+impulse*(sign*b.state.inverseMass);
        const float angle=WrapAngle(b.state.transform.angle+sign*b.state.inverseInertia*Cross(r,impulse));
        b.state.transform.angle=angle;
        b.state.transform.position=center-Rot2(angle).Apply(b.state.localCenter);
        b.state.worldCenter=center;
    }
    void ApplyAngularPositionImpulse(BodyInternal& b,float impulse,float sign) {
        if(b.state.type!=BodyType::Dynamic || b.state.inverseInertia<=0.0f) return;
        Vec2 center=WorldCenter(b);
        b.state.transform.angle=WrapAngle(b.state.transform.angle+sign*impulse*b.state.inverseInertia);
        b.state.transform.position=center-Rot2(b.state.transform.angle).Apply(b.state.localCenter);
        b.state.worldCenter=center;
    }
    void BuildContacts() {
        using ProfileClock = std::chrono::steady_clock;
        const auto broadStart=ProfileClock::now();
        contacts.clear(); beginEvents.clear(); endEvents.clear(); activeContactEvents.clear();
        std::vector<AABB2> tightBounds(bodies.size());
        for(BodyID id=0;id<bodies.size();++id) {
            auto& body=bodies[id];
            if(body.state.id!=id || !body.state.enabled) { SyncProxy(body); continue; }
            tightBounds[id]=ComputeAABB(body.shape,body.state.transform);
            SyncProxy(body);
        }
        const auto pairStart=ProfileClock::now();
        std::vector<std::vector<uint64_t>> localCandidates(bodies.size());
        auto generatePairs=[&](size_t index){
            const BodyID id=BodyID(index);const auto& body=bodies[index];
            if(body.state.id!=id || !body.state.enabled || !tightBounds[index].IsValid())return;
            broadPhaseTree.Query(tightBounds[index],[&](uint32_t other){
                if(other<=id||other>=bodies.size())return true;
                const auto& candidate=bodies[other];
                if(candidate.state.id!=other||!candidate.state.enabled)return true;
                if(tightBounds[index].Overlaps(tightBounds[other]))localCandidates[index].push_back(PairKey(id,other));
                return true;
            });
        };
        if(settings.enableMultithreading&&!settings.deterministicMode&&bodies.size()>32)workerPool.ParallelFor(bodies.size(),generatePairs);
        else for(size_t i=0;i<bodies.size();++i)generatePairs(i);
        std::vector<uint64_t> candidateVector;
        size_t candidateCapacity=0;for(const auto& v:localCandidates)candidateCapacity+=v.size();candidateVector.reserve(candidateCapacity);
        for(auto& v:localCandidates)candidateVector.insert(candidateVector.end(),v.begin(),v.end());
        std::sort(candidateVector.begin(),candidateVector.end());candidateVector.erase(std::unique(candidateVector.begin(),candidateVector.end()),candidateVector.end());
        lastCandidateCount=uint32_t(std::min<size_t>(candidateVector.size(),std::numeric_limits<uint32_t>::max()));
        const auto narrowStart=ProfileClock::now();
        profile.broadPhasePairGenerationMilliseconds+=std::chrono::duration<double,std::milli>(narrowStart-pairStart).count();
        profile.broadPhaseMilliseconds+=std::chrono::duration<double,std::milli>(narrowStart-broadStart).count();
        profile.broadPhaseQueries+=bodies.size();
        std::set<uint64_t> currentPairs,currentSensorPairs;
        for(uint64_t key:candidateVector) {
            BodyID a=BodyID(key>>32), b=BodyID(key&0xffffffffu); BodyInternal* A=Body(a); BodyInternal* B=Body(b);
            if(!A||!B||!A->state.enabled||!B->state.enabled) continue;
            if(A->state.type==BodyType::Static&&B->state.type==BodyType::Static) continue;
            if(!ShouldCollide(A->filter,B->filter)) continue;
            bool blocked=false;
            for(const auto& j:joints) if(j.enabled&&!j.collideConnected&&((j.a==a&&j.b==b)||(j.a==b&&j.b==a))) { blocked=true;break; }
            if(blocked) continue;
            if(!ComputeAABB(A->shape,A->state.transform).Overlaps(ComputeAABB(B->shape,B->state.transform))) continue;
            std::vector<Manifold2D> manifolds; CollideAll(A->shape,A->state.transform,B->shape,B->state.transform,manifolds);
            if(manifolds.empty()) continue;
            std::vector<Manifold2D> accepted;accepted.reserve(manifolds.size());bool pairHasSensor=A->state.isSensor||B->state.isSensor;
            for(auto manifold:manifolds){
                const auto childA=ResolveChildMetadata(A->shape,manifold.childPathA,A->filter);
                const auto childB=ResolveChildMetadata(B->shape,manifold.childPathB,B->filter);
                if(!ShouldCollide(childA.filter,childB.filter))continue;
                manifold.childUserDataA=childA.userData?childA.userData:manifold.childUserDataA;
                manifold.childUserDataB=childB.userData?childB.userData:manifold.childUserDataB;
                manifold.isSensor=A->state.isSensor||B->state.isSensor||childA.isSensor||childB.isSensor;
                pairHasSensor=pairHasSensor||manifold.isSensor;accepted.push_back(manifold);
            }
            manifolds=std::move(accepted);if(manifolds.empty())continue;
            ContactEvent activeEvent;activeEvent.bodyA=a;activeEvent.bodyB=b;activeEvent.manifold=manifolds.front();activeEvent.sensor=pairHasSensor;activeEvent.manifolds=manifolds;activeContactEvents.push_back(std::move(activeEvent));
            if(A->oneWayPlatform || B->oneWayPlatform) {
                BodyInternal* platform=A->oneWayPlatform?A:B; BodyInternal* other=A->oneWayPlatform?B:A;
                Vec2 n=NormalizeOr(WorldVector(*platform,platform->oneWayNormal));
                Vec2 relative=other->state.linearVelocity-platform->state.linearVelocity;
                float side=Dot(other->state.worldCenter-platform->state.worldCenter,n);
                if(side < -0.05f || Dot(relative,n)>0.1f) continue;
            }
            currentPairs.insert(key);if(pairHasSensor)currentSensorPairs.insert(key);
            const bool activeA=(A->state.type==BodyType::Dynamic&&A->state.awake) || (A->state.type==BodyType::Kinematic&&(A->state.linearVelocity.LengthSquared()>kEpsilon||std::abs(A->state.angularVelocity)>kEpsilon));
            const bool activeB=(B->state.type==BodyType::Dynamic&&B->state.awake) || (B->state.type==BodyType::Kinematic&&(B->state.linearVelocity.LengthSquared()>kEpsilon||std::abs(B->state.angularVelocity)>kEpsilon));
            if(activeA||activeB){Wake(*A);Wake(*B);}
            for(const Manifold2D& manifold:manifolds) {
                if(manifold.isSensor) continue;
                if(preSolve && !preSolve(a,b,manifold)) continue;
                const auto childA=ResolveChildMetadata(A->shape,manifold.childPathA,A->filter);
                const auto childB=ResolveChildMetadata(B->shape,manifold.childPathB,B->filter);
                const float frictionA=childA.friction>=0.0f?childA.friction:A->material.friction;
                const float frictionB=childB.friction>=0.0f?childB.friction:B->material.friction;
                const float restitutionA=childA.restitution>=0.0f?childA.restitution:A->material.restitution;
                const float restitutionB=childB.restitution>=0.0f?childB.restitution:B->material.restitution;
                ContactConstraint c; c.a=a;c.b=b;c.manifold=manifold;
                c.friction=std::sqrt(std::max(0.0f,frictionA*frictionB));
                c.restitution=std::max(restitutionA,restitutionB);
                c.cacheKey={a,b,manifold.childIdA,manifold.childIdB};
                auto cached=impulseCache.find(c.cacheKey);
                if(cached!=impulseCache.end()) for(uint8_t i=0;i<c.manifold.pointCount;++i) for(int j=0;j<2;++j)
                    if(c.manifold.points[i].featureId==cached->second.feature[j]) { c.manifold.points[i].normalImpulse=cached->second.normal[j]; c.manifold.points[i].tangentImpulse=cached->second.tangent[j]; }
                for(uint8_t i=0;i<c.manifold.pointCount;++i) {
                    const auto& cp=c.manifold.points[i]; Vec2 ra=cp.point-WorldCenter(*A),rb=cp.point-WorldCenter(*B);
                    Vec2 rv=(B->state.linearVelocity+Cross(B->state.angularVelocity,rb))-(A->state.linearVelocity+Cross(A->state.angularVelocity,ra));
                    const float vn=Dot(rv,c.manifold.normal); c.manifold.points[i].velocityBias=vn < -1.0f ? c.restitution*vn : 0.0f;
                }
                contacts.push_back(c);
            }
        }
        for(uint64_t key:currentPairs) if(previousPairs.find(key)==previousPairs.end()) {
            BodyID a=BodyID(key>>32),b=BodyID(key&0xffffffffu); BodyInternal* A=Body(a);BodyInternal* B=Body(b); if(!A||!B)continue;
            auto active=std::find_if(activeContactEvents.begin(),activeContactEvents.end(),[&](const ContactEvent& e){return e.bodyA==a&&e.bodyB==b;});
            ContactEvent e;
            if(active!=activeContactEvents.end())e=*active;
            else {e.bodyA=a;e.bodyB=b;e.sensor=currentSensorPairs.find(key)!=currentSensorPairs.end();}
            e.sensor=currentSensorPairs.find(key)!=currentSensorPairs.end();
            if(!e.manifolds.empty())e.manifold=e.manifolds.front();
            beginEvents.push_back(e); if(listener) listener(e,true);
            Wake(*A);Wake(*B);
        }
        for(uint64_t key:previousPairs) if(currentPairs.find(key)==currentPairs.end()) {
            BodyID a=BodyID(key>>32),b=BodyID(key&0xffffffffu); endEvents.push_back(ContactEvent{a,b,{},previousSensorPairs.find(key)!=previousSensorPairs.end(),{}}); if(listener) listener(endEvents.back(),false);
        }
        previousPairs=std::move(currentPairs);previousSensorPairs=std::move(currentSensorPairs);
        profile.narrowPhaseMilliseconds+=std::chrono::duration<double,std::milli>(ProfileClock::now()-narrowStart).count();
    }
    void SolveContactVelocity(ContactConstraint& c) {
        BodyInternal* A=Body(c.a);BodyInternal* B=Body(c.b);if(!A||!B)return;
        Vec2 n=c.manifold.normal,t=n.Perpendicular();const uint8_t count=c.manifold.pointCount;
        Vec2 rA[2]{},rB[2]{};float k11=0.0f,k22=0.0f,k12=0.0f;float vn[2]{};
        for(uint8_t pi=0;pi<count;++pi){
            auto& cp=c.manifold.points[pi];rA[pi]=cp.point-WorldCenter(*A);rB[pi]=cp.point-WorldCenter(*B);
            Vec2 rv=(B->state.linearVelocity+Cross(B->state.angularVelocity,rB[pi]))-(A->state.linearVelocity+Cross(A->state.angularVelocity,rA[pi]));vn[pi]=Dot(rv,n);
            float rnA=Cross(rA[pi],n),rnB=Cross(rB[pi],n);float k=InvMass(*A)+InvMass(*B)+InvI(*A)*rnA*rnA+InvI(*B)*rnB*rnB;
            if(pi==0)k11=k;else k22=k;
        }
        if(count==2){
            k12=InvMass(*A)+InvMass(*B)+InvI(*A)*Cross(rA[0],n)*Cross(rA[1],n)+InvI(*B)*Cross(rB[0],n)*Cross(rB[1],n);
            const float det=k11*k22-k12*k12;const float old1=c.manifold.points[0].normalImpulse,old2=c.manifold.points[1].normalImpulse;
            bool solved=false;float x1=0.0f,x2=0.0f;
            if(det>kEpsilon*kEpsilon){const float invDet=1.0f/det;const float b1=vn[0]+c.manifold.points[0].velocityBias-k11*old1-k12*old2;const float b2=vn[1]+c.manifold.points[1].velocityBias-k12*old1-k22*old2;
                x1=-(k22*b1-k12*b2)*invDet;x2=-(-k12*b1+k11*b2)*invDet;
                if(x1>=0.0f&&x2>=0.0f)solved=true;
                if(!solved&&k22>kEpsilon){x1=0.0f;x2=-b2/k22;if(x2>=0.0f&&k12*x2+b1>=0.0f)solved=true;}
                if(!solved&&k11>kEpsilon){x2=0.0f;x1=-b1/k11;if(x1>=0.0f&&k12*x1+b2>=0.0f)solved=true;}
                if(!solved&&b1>=0.0f&&b2>=0.0f){x1=x2=0.0f;solved=true;}
            }
            if(solved){c.manifold.points[0].normalImpulse=x1;c.manifold.points[1].normalImpulse=x2;const float d1=x1-old1,d2=x2-old2;ApplyImpulse(*A,n*d1,rA[0],-1);ApplyImpulse(*B,n*d1,rB[0],1);ApplyImpulse(*A,n*d2,rA[1],-1);ApplyImpulse(*B,n*d2,rB[1],1);}
            else for(uint8_t pi=0;pi<2;++pi){auto&cp=c.manifold.points[pi];float k=pi==0?k11:k22;if(k<=kEpsilon)continue;float lambda=-(vn[pi]+cp.velocityBias)/k;float old=cp.normalImpulse;cp.normalImpulse=std::max(old+lambda,0.0f);lambda=cp.normalImpulse-old;ApplyImpulse(*A,n*lambda,rA[pi],-1);ApplyImpulse(*B,n*lambda,rB[pi],1);}
        } else if(count==1&&k11>kEpsilon){auto&cp=c.manifold.points[0];float lambda=-(vn[0]+cp.velocityBias)/k11;float old=cp.normalImpulse;cp.normalImpulse=std::max(old+lambda,0.0f);lambda=cp.normalImpulse-old;ApplyImpulse(*A,n*lambda,rA[0],-1);ApplyImpulse(*B,n*lambda,rB[0],1);}
        for(uint8_t pi=0;pi<count;++pi){auto&cp=c.manifold.points[pi];Vec2 rv=(B->state.linearVelocity+Cross(B->state.angularVelocity,rB[pi]))-(A->state.linearVelocity+Cross(A->state.angularVelocity,rA[pi]));float rtA=Cross(rA[pi],t),rtB=Cross(rB[pi],t);float kt=InvMass(*A)+InvMass(*B)+InvI(*A)*rtA*rtA+InvI(*B)*rtB*rtB;
            if(kt>kEpsilon){float lambda=-Dot(rv,t)/kt;float maxFriction=c.friction*cp.normalImpulse;float old=cp.tangentImpulse;cp.tangentImpulse=Clamp(old+lambda,-maxFriction,maxFriction);lambda=cp.tangentImpulse-old;Vec2 impulse=t*lambda;ApplyImpulse(*A,impulse,rA[pi],-1);ApplyImpulse(*B,impulse,rB[pi],1);}}
    }
    void SolveContactVelocityBatch4(ContactConstraint* batch[4]) {
        BodyInternal* A[4]{};BodyInternal* B[4]{};Vec2 rA[4]{},rB[4]{},n[4]{},t[4]{};
        Float4x2D vn{},effective{},oldImpulse{},bias{};
        for(int i=0;i<4;++i){auto&c=*batch[i];A[i]=Body(c.a);B[i]=Body(c.b);n[i]=NormalizeOr(c.manifold.normal,{0,1});t[i]=n[i].Perpendicular();
            auto&cp=c.manifold.points[0];rA[i]=cp.point-WorldCenter(*A[i]);rB[i]=cp.point-WorldCenter(*B[i]);
            const Vec2 va=A[i]->state.linearVelocity+Cross(A[i]->state.angularVelocity,rA[i]);const Vec2 vb=B[i]->state.linearVelocity+Cross(B[i]->state.angularVelocity,rB[i]);
            vn.lane[i]=Dot(vb-va,n[i]);const float ca=Cross(rA[i],n[i]),cb=Cross(rB[i],n[i]);
            effective.lane[i]=InvMass(*A[i])+InvMass(*B[i])+InvI(*A[i])*ca*ca+InvI(*B[i])*cb*cb;
            oldImpulse.lane[i]=cp.normalImpulse;bias.lane[i]=cp.velocityBias;
        }
        const Float4x2D lambda=BatchDivSafe(BatchSub(Float4x2D{},BatchAdd(vn,bias)),effective);
        const Float4x2D proposed=BatchMax(BatchAdd(oldImpulse,lambda),Float4x2D{});
        const Float4x2D delta=BatchSub(proposed,oldImpulse);
        for(int i=0;i<4;++i){auto&cp=batch[i]->manifold.points[0];cp.normalImpulse=proposed.lane[i];const Vec2 impulse=n[i]*delta.lane[i];ApplyImpulse(*A[i],impulse,rA[i],-1);ApplyImpulse(*B[i],impulse,rB[i],1);}
        // Tangential friction is also solved in four lanes for independent
        // one-point contacts, with scalar impulse application after the batch.
        Float4x2D vt{},effectiveT{},oldT{},maxT{};
        for(int i=0;i<4;++i){auto&c=*batch[i];auto&cp=c.manifold.points[0];
            const Vec2 rv=(B[i]->state.linearVelocity+Cross(B[i]->state.angularVelocity,rB[i]))-(A[i]->state.linearVelocity+Cross(A[i]->state.angularVelocity,rA[i]));
            const float ra=Cross(rA[i],t[i]),rb=Cross(rB[i],t[i]);
            vt.lane[i]=Dot(rv,t[i]);effectiveT.lane[i]=InvMass(*A[i])+InvMass(*B[i])+InvI(*A[i])*ra*ra+InvI(*B[i])*rb*rb;
            oldT.lane[i]=cp.tangentImpulse;maxT.lane[i]=c.friction*cp.normalImpulse;
        }
        const Float4x2D deltaT=BatchDivSafe(BatchSub(Float4x2D{},vt),effectiveT);
        Float4x2D negativeMaxT{};for(int i=0;i<4;++i)negativeMaxT.lane[i]=-maxT.lane[i];
        const Float4x2D newT=BatchMin(BatchMax(BatchAdd(oldT,deltaT),negativeMaxT),maxT);
        const Float4x2D appliedT=BatchSub(newT,oldT);
        for(int i=0;i<4;++i){auto&cp=batch[i]->manifold.points[0];cp.tangentImpulse=newT.lane[i];
            const Vec2 impulse=t[i]*appliedT.lane[i];ApplyImpulse(*A[i],impulse,rA[i],-1);ApplyImpulse(*B[i],impulse,rB[i],1);}
        if(BatchSIMDEnabled2D())simdBatchCounter.fetch_add(1,std::memory_order_relaxed);
    }
    void SolveContactVelocitySet(const std::vector<size_t>& indices) {
        for(size_t i=0;i<indices.size();){
            if(i+4<=indices.size()){
                ContactConstraint* batch[4]={&contacts[indices[i]],&contacts[indices[i+1]],&contacts[indices[i+2]],&contacts[indices[i+3]]};
                bool eligible=true;std::set<BodyID> dynamicBodies;
                for(int k=0;k<4&&eligible;++k){auto*c=batch[k];if(c->manifold.pointCount!=1){eligible=false;break;}BodyInternal*a=Body(c->a),*b=Body(c->b);if(!a||!b){eligible=false;break;}
                    if(a->state.type==BodyType::Dynamic&&!dynamicBodies.insert(c->a).second)eligible=false;
                    if(b->state.type==BodyType::Dynamic&&!dynamicBodies.insert(c->b).second)eligible=false;
                }
                if(eligible){SolveContactVelocityBatch4(batch);i+=4;continue;}
            }
            SolveContactVelocity(contacts[indices[i]]);++i;
        }
    }

    void SolveContactPosition(ContactConstraint& c) {
        BodyInternal* A=Body(c.a);BodyInternal* B=Body(c.b);if(!A||!B)return;
        std::vector<Manifold2D> manifolds;CollideAll(A->shape,A->state.transform,B->shape,B->state.transform,manifolds);
        auto found=std::find_if(manifolds.begin(),manifolds.end(),[&](const Manifold2D& m){return m.childIndexA==c.manifold.childIndexA&&m.childIndexB==c.manifold.childIndexB;});
        if(found==manifolds.end()) return;
        const Manifold2D& m=*found;Vec2 n=m.normal;
        for(uint8_t pi=0;pi<m.pointCount;++pi){Vec2 p=m.points[pi].point,rA=p-WorldCenter(*A),rB=p-WorldCenter(*B);float rnA=Cross(rA,n),rnB=Cross(rB,n);float k=InvMass(*A)+InvMass(*B)+InvI(*A)*rnA*rnA+InvI(*B)*rnB*rnB;float penetration=std::max(-m.points[pi].separation-settings.contactSlop,0.0f);if(k>kEpsilon&&penetration>0){float impulse=Clamp(settings.baumgarte*penetration/k,0.0f,0.2f);Vec2 P=n*impulse;ApplyPositionImpulse(*A,P,rA,-1);ApplyPositionImpulse(*B,P,rB,1);}}
    }

    void SolveJointVelocity(JointInternal& j,float dt) {
        if(!j.enabled)return;
        if(j.type==JointType::Mouse){if(j.mouseAppliedThisStep)return;j.mouseAppliedThisStep=true;BodyInternal*B=Body(j.b);if(!B||B->state.type!=BodyType::Dynamic)return;float im=InvMass(*B),ii=InvI(*B);Vec2 point=B->state.transform.position+WorldVector(*B,j.anchorB),r=point-WorldCenter(*B);Vec2 velocity=B->state.linearVelocity+Cross(B->state.angularVelocity,r);float effective=1.0f/std::max(im+ii*r.LengthSquared(),kEpsilon);float omega=2.0f*kPi*std::max(0.0f,j.frequency);float stiffness=effective*omega*omega,damping=2.0f*effective*std::max(0.0f,j.damping)*omega;Vec2 force=(j.target-point)*stiffness-velocity*damping;float mag=force.Length();if(mag>j.maxForce&&mag>kEpsilon)force*=j.maxForce/mag;Vec2 impulse=force*dt;ApplyImpulse(*B,impulse,r,1);j.impulse+=impulse;return;}
        BodyInternal* A=Body(j.a); BodyInternal* B=Body(j.b); if(!A||!B) return;
        float imA=InvMass(*A), imB=InvMass(*B), iiA=InvI(*A), iiB=InvI(*B); if(imA+imB+iiA+iiB<=0) return;
        Vec2 pA=A->state.transform.position+WorldVector(*A,j.anchorA),pB=B->state.transform.position+WorldVector(*B,j.anchorB);
        Vec2 rA=pA-WorldCenter(*A),rB=pB-WorldCenter(*B);
        Vec2 va=A->state.linearVelocity+Cross(A->state.angularVelocity,rA), vb=B->state.linearVelocity+Cross(B->state.angularVelocity,rB);
        if(j.type==JointType::Gear) {
            const float ratio=j.ratio;
            const float error=WrapAngle(B->state.transform.angle+ratio*A->state.transform.angle-j.constant);
            const float relative=B->state.angularVelocity+ratio*A->state.angularVelocity;
            const float k=iiB+ratio*ratio*iiA;
            if(k>kEpsilon) {
                const float constraintImpulse=-(relative+0.2f*error/dt)/k;
                A->state.angularVelocity+=ratio*constraintImpulse*iiA;
                B->state.angularVelocity+=constraintImpulse*iiB;
                j.angularImpulse+=constraintImpulse;
                if(j.enableMotor) {
                    const float afterConstraint=B->state.angularVelocity+ratio*A->state.angularVelocity;
                    float lambda=(j.motorSpeed-afterConstraint)/k;const float old=j.motorImpulse,max=j.maxMotor*dt;
                    j.motorImpulse=Clamp(old+lambda,-max,max);lambda=j.motorImpulse-old;
                    A->state.angularVelocity+=ratio*lambda*iiA;B->state.angularVelocity+=lambda*iiB;
                }
            }
            return;
        }
        if(j.type==JointType::Pulley) {
            Vec2 uA=NormalizeOr(pA-j.groundAnchorA),uB=NormalizeOr(pB-j.groundAnchorB);
            const float lenA=(pA-j.groundAnchorA).Length(),lenB=(pB-j.groundAnchorB).Length();
            const float C=lenA+j.ratio*lenB-j.constant;
            const float crA=Cross(rA,uA),crB=Cross(rB,uB);
            const float k=imA+iiA*crA*crA+j.ratio*j.ratio*(imB+iiB*crB*crB);
            if(k>kEpsilon) {
                const float cdot=Dot(va,uA)+j.ratio*Dot(vb,uB);
                const float lambda=-(cdot+0.15f*C/dt)/k;
                ApplyImpulse(*A,uA*lambda,rA,1.0f);
                ApplyImpulse(*B,uB*(j.ratio*lambda),rB,1.0f);
                j.impulse+=uA*lambda;
            }
            return;
        }
        if(j.type==JointType::Friction) {
            const float k11=imA+imB+iiA*rA.y*rA.y+iiB*rB.y*rB.y;
            const float k12=-iiA*rA.x*rA.y-iiB*rB.x*rB.y;
            const float k22=imA+imB+iiA*rA.x*rA.x+iiB*rB.x*rB.x;
            Vec2 delta=Mat22{k11,k12,k12,k22}.Inverse()*(-(vb-va));
            Vec2 old=j.impulse, proposed=old+delta; const float maxImpulse=j.maxForce*dt;
            if(proposed.Length()>maxImpulse&&proposed.Length()>kEpsilon) proposed*=maxImpulse/proposed.Length();
            delta=proposed-old; j.impulse=proposed;
            ApplyImpulse(*A,delta,rA,-1.0f); ApplyImpulse(*B,delta,rB,1.0f);
            const float angularMass=iiA+iiB;
            if(angularMass>kEpsilon) {
                float lambda=-(B->state.angularVelocity-A->state.angularVelocity)/angularMass;
                const float oldTorque=j.motorImpulse; const float maxTorque=j.maxTorque*dt;
                j.motorImpulse=Clamp(oldTorque+lambda,-maxTorque,maxTorque); lambda=j.motorImpulse-oldTorque;
                A->state.angularVelocity-=lambda*iiA; B->state.angularVelocity+=lambda*iiB;
            }
            return;
        }
        if(j.type==JointType::Motor) {
            const Vec2 desired=A->state.transform.Apply(j.linearOffset);
            const Vec2 error=(B->state.transform.position-desired);
            const Vec2 targetVelocity=error*(-j.correctionFactor/dt);
            const Vec2 ra=A->state.transform.position-WorldCenter(*A),rb=B->state.transform.position-WorldCenter(*B);
            const Vec2 vA=A->state.linearVelocity+Cross(A->state.angularVelocity,ra);
            const Vec2 vB=B->state.linearVelocity+Cross(B->state.angularVelocity,rb);
            const float k11=imA+imB+iiA*ra.y*ra.y+iiB*rb.y*rb.y;
            const float k12=-iiA*ra.x*ra.y-iiB*rb.x*rb.y;
            const float k22=imA+imB+iiA*ra.x*ra.x+iiB*rb.x*rb.x;
            Vec2 delta=Mat22{k11,k12,k12,k22}.Inverse()*(targetVelocity-(vB-vA));
            Vec2 old=j.impulse,proposed=old+delta; const float maxImpulse=j.maxForce*dt;
            if(proposed.Length()>maxImpulse&&proposed.Length()>kEpsilon) proposed*=maxImpulse/proposed.Length();
            delta=proposed-old;j.impulse=proposed;ApplyImpulse(*A,delta,ra,-1);ApplyImpulse(*B,delta,rb,1);
            const float angleError=WrapAngle(B->state.transform.angle-A->state.transform.angle-j.angularOffset);
            const float targetOmega=-j.correctionFactor*angleError/dt;
            const float angularMass=iiA+iiB;
            if(angularMass>kEpsilon){float lambda=(targetOmega-(B->state.angularVelocity-A->state.angularVelocity))/angularMass;float oldT=j.motorImpulse;float maxT=j.maxTorque*dt;j.motorImpulse=Clamp(oldT+lambda,-maxT,maxT);lambda=j.motorImpulse-oldT;A->state.angularVelocity-=lambda*iiA;B->state.angularVelocity+=lambda*iiB;}
            return;
        }
        if(j.type==JointType::Distance || j.type==JointType::Rope) {
            Vec2 d=pB-pA; float len=d.Length(); if(len<kEpsilon)return; Vec2 n=d/len;
            float k=imA+imB+iiA*Cross(rA,n)*Cross(rA,n)+iiB*Cross(rB,n)*Cross(rB,n); if(k<kEpsilon)return;
            float error=0.0f;
            if(j.type==JointType::Rope){if(len<=j.maxLength)return;error=len-j.maxLength;}
            else if(j.minLength<0 && j.maxLength<0) error=len-j.length;
            else if(j.minLength>=0 && len<j.minLength) error=len-j.minLength;
            else if(j.maxLength>=0 && len>j.maxLength) error=len-j.maxLength;
            else return;
            float bias=0.0f;
            if(j.frequency>0) { float omega=2*kPi*j.frequency, mass=1.0f/k, damp=2*mass*j.damping*omega, stiff=mass*omega*omega; float gamma=dt*(damp+dt*stiff); gamma=gamma>kEpsilon?1.0f/gamma:0; bias=error*dt*stiff*gamma; k+=gamma; }
            else bias=0.15f*error/dt;
            float lambda=-(Dot(vb-va,n)+bias)/k;
            if(j.type==JointType::Rope)lambda=std::min(0.0f,lambda);
            if(j.minLength>=0 && len<j.minLength) lambda=std::max(0.0f,lambda);
            if(j.maxLength>=0 && len>j.maxLength) lambda=std::min(0.0f,lambda);
            Vec2 impulse=n*lambda; ApplyImpulse(*A,impulse,rA,-1);ApplyImpulse(*B,impulse,rB,1); j.impulse+=impulse;
            return;
        }
        if(j.type==JointType::Revolute||j.type==JointType::Weld) {
            Vec2 rv=vb-va;
            float k11=imA+imB+iiA*rA.y*rA.y+iiB*rB.y*rB.y;
            float k12=-iiA*rA.x*rA.y-iiB*rB.x*rB.y;
            float k22=imA+imB+iiA*rA.x*rA.x+iiB*rB.x*rB.x;
            Mat22 K{k11,k12,k12,k22}; Vec2 impulse=K.Inverse()*(-rv);
            ApplyImpulse(*A,impulse,rA,-1);ApplyImpulse(*B,impulse,rB,1); j.impulse+=impulse;
            float rel=B->state.angularVelocity-A->state.angularVelocity;
            if(j.type==JointType::Weld) {
                float target=WrapAngle(B->state.transform.angle-A->state.transform.angle-j.referenceAngle);
                float k=iiA+iiB; if(k>kEpsilon) {float lambda=-(rel+0.2f*target/dt)/k; ApplyImpulse(*A,{}, {},0); A->state.angularVelocity-=lambda*iiA;B->state.angularVelocity+=lambda*iiB; j.angularImpulse+=lambda;}
            } else if(j.enableLimit) {
                float angle=WrapAngle(B->state.transform.angle-A->state.transform.angle-j.referenceAngle);
                float lambda=0; if(angle<j.lower) lambda=-(rel+0.2f*(angle-j.lower)/dt)/std::max(iiA+iiB,kEpsilon);
                else if(angle>j.upper) lambda=-(rel+0.2f*(angle-j.upper)/dt)/std::max(iiA+iiB,kEpsilon);
                if(lambda!=0){ A->state.angularVelocity-=lambda*iiA;B->state.angularVelocity+=lambda*iiB;j.limitImpulse+=lambda; }
            }
            if(j.type==JointType::Revolute&&j.enableMotor) {
                float k=iiA+iiB; if(k>kEpsilon) {float lambda=(j.motorSpeed-rel)/k; float old=j.motorImpulse; float max=j.maxMotor*dt; j.motorImpulse=Clamp(old+lambda,-max,max); lambda=j.motorImpulse-old; A->state.angularVelocity-=lambda*iiA;B->state.angularVelocity+=lambda*iiB;}
            }
            return;
        }
        Vec2 axis=NormalizeOr(WorldVector(*A,j.axisA)); Vec2 perp=axis.Perpendicular(); Vec2 d=pB-pA;
        float cdot=Dot(vb-va,perp);
        float k=imA+imB+iiA*Cross(rA,perp)*Cross(rA,perp)+iiB*Cross(rB,perp)*Cross(rB,perp);
        if(k>kEpsilon) {float lambda=-cdot/k; Vec2 imp=perp*lambda;ApplyImpulse(*A,imp,rA,-1);ApplyImpulse(*B,imp,rB,1);j.impulse+=imp;}
        float translation=Dot(d,axis), axisSpeed=Dot(vb-va,axis);
        if(j.type==JointType::Wheel && j.frequency>0) {
            float omega=2*kPi*j.frequency, invMass=std::max(imA+imB,kEpsilon), mass=1.0f/invMass;
            float stiffness=mass*omega*omega, damping=2*mass*j.damping*omega;
            float lambda=-(axisSpeed+dt*stiffness*translation)/(invMass+dt*damping+dt*dt*stiffness);
            Vec2 imp=axis*lambda;ApplyImpulse(*A,imp,rA,-1);ApplyImpulse(*B,imp,rB,1);
        }
        if(j.type==JointType::Prismatic) {
            // A prismatic pair may slide on its axis but must preserve relative angle.
            float angularMass=iiA+iiB;
            if(angularMass>kEpsilon) {float lambda=-(B->state.angularVelocity-A->state.angularVelocity)/angularMass;A->state.angularVelocity-=lambda*iiA;B->state.angularVelocity+=lambda*iiB;}
            if(j.enableLimit) {
                float lambda=0; if(translation<j.lower) lambda=-(axisSpeed+0.2f*(translation-j.lower)/dt)/std::max(imA+imB,kEpsilon);
                else if(translation>j.upper) lambda=-(axisSpeed+0.2f*(translation-j.upper)/dt)/std::max(imA+imB,kEpsilon);
                Vec2 imp=axis*lambda;ApplyImpulse(*A,imp,rA,-1);ApplyImpulse(*B,imp,rB,1);
            }
            if(j.enableMotor) {
                float effective=imA+imB; if(effective>kEpsilon) {float lambda=(j.motorSpeed-axisSpeed)/effective;float old=j.motorImpulse;float max=j.maxMotor*dt;j.motorImpulse=Clamp(old+lambda,-max,max);lambda=j.motorImpulse-old;Vec2 imp=axis*lambda;ApplyImpulse(*A,imp,rA,-1);ApplyImpulse(*B,imp,rB,1);}
            }
        } else if(j.type==JointType::Wheel && j.enableMotor) {
            // Wheel motor drives relative rotation, not suspension-axis translation.
            float effective=iiA+iiB; if(effective>kEpsilon) {
                float relative=B->state.angularVelocity-A->state.angularVelocity;
                float lambda=(j.motorSpeed-relative)/effective, old=j.motorImpulse, max=j.maxMotor*dt;
                j.motorImpulse=Clamp(old+lambda,-max,max); lambda=j.motorImpulse-old;
                A->state.angularVelocity-=lambda*iiA; B->state.angularVelocity+=lambda*iiB;
            }
        }
    }
    void SolveJointPosition(JointInternal& j) {
        BodyInternal* A=Body(j.a);BodyInternal* B=Body(j.b);if(!A||!B||!j.enabled)return;
        float imA=InvMass(*A),imB=InvMass(*B),iiA=InvI(*A),iiB=InvI(*B);
        Vec2 pA=A->state.transform.position+WorldVector(*A,j.anchorA),pB=B->state.transform.position+WorldVector(*B,j.anchorB);Vec2 rA=pA-WorldCenter(*A),rB=pB-WorldCenter(*B),d=pB-pA;
        if(j.type==JointType::Mouse || j.type==JointType::Friction)return;
        if(j.type==JointType::Gear) {
            const float error=WrapAngle(B->state.transform.angle+j.ratio*A->state.transform.angle-j.constant);
            const float k=iiB+j.ratio*j.ratio*iiA;
            if(k>kEpsilon&&std::abs(error)>1.0e-4f){const float lambda=-Clamp(error,-0.2f,0.2f)*0.2f/k;ApplyAngularPositionImpulse(*A,j.ratio*lambda,-1);ApplyAngularPositionImpulse(*B,lambda,1);}
            return;
        }
        if(j.type==JointType::Pulley) {
            Vec2 uA=NormalizeOr(pA-j.groundAnchorA),uB=NormalizeOr(pB-j.groundAnchorB);
            const float lenA=(pA-j.groundAnchorA).Length(),lenB=(pB-j.groundAnchorB).Length();
            const float error=lenA+j.ratio*lenB-j.constant;
            const float crA=Cross(rA,uA),crB=Cross(rB,uB);
            const float k=imA+iiA*crA*crA+j.ratio*j.ratio*(imB+iiB*crB*crB);
            if(k>kEpsilon&&std::abs(error)>1.0e-4f){const float lambda=-Clamp(error,-0.2f,0.2f)*0.2f/k;ApplyPositionImpulse(*A,uA*lambda,rA,1);ApplyPositionImpulse(*B,uB*(j.ratio*lambda),rB,1);}
            return;
        }
        if(j.type==JointType::Motor) {
            const Vec2 desired=A->state.transform.Apply(j.linearOffset);
            const Vec2 error=B->state.transform.position-desired;
            const Vec2 ra=A->state.transform.position-WorldCenter(*A),rb=B->state.transform.position-WorldCenter(*B);
            const float k11=imA+imB+iiA*ra.y*ra.y+iiB*rb.y*rb.y,k12=-iiA*ra.x*ra.y-iiB*rb.x*rb.y,k22=imA+imB+iiA*ra.x*ra.x+iiB*rb.x*rb.x;
            Vec2 correction=Mat22{k11,k12,k12,k22}.Inverse()*(-error*0.2f);
            ApplyPositionImpulse(*A,correction,ra,-1);ApplyPositionImpulse(*B,correction,rb,1);
            const float angleError=WrapAngle(B->state.transform.angle-A->state.transform.angle-j.angularOffset);
            const float inv=iiA+iiB;if(inv>kEpsilon){const float lambda=-Clamp(angleError,-0.2f,0.2f)*0.2f/inv;ApplyAngularPositionImpulse(*A,lambda,-1);ApplyAngularPositionImpulse(*B,lambda,1);}
            return;
        }
        if(j.type==JointType::Distance || j.type==JointType::Rope) {
            float len=d.Length(); if(len<kEpsilon || (j.type==JointType::Distance&&j.frequency>0))return; float error=0.0f;
            if(j.type==JointType::Rope){if(len<=j.maxLength)return;error=len-j.maxLength;}
            else if(j.minLength<0 && j.maxLength<0) error=len-j.length;
            else if(j.minLength>=0 && len<j.minLength) error=len-j.minLength;
            else if(j.maxLength>=0 && len>j.maxLength) error=len-j.maxLength;
            else return;
            Vec2 n=NormalizeOr(d);float crA=Cross(rA,n),crB=Cross(rB,n);float k=imA+imB+iiA*crA*crA+iiB*crB*crB;if(k>kEpsilon){float lambda=-Clamp(error,-0.2f,0.2f)*0.2f/k;Vec2 P=n*lambda;ApplyPositionImpulse(*A,P,rA,-1);ApplyPositionImpulse(*B,P,rB,1);}
        } else if(j.type==JointType::Prismatic || j.type==JointType::Wheel) {
            Vec2 axis=NormalizeOr(WorldVector(*A,j.axisA)); Vec2 perp=axis.Perpendicular();
            float error=Dot(d,perp);
            float crA=Cross(rA,perp),crB=Cross(rB,perp);
            float k=imA+imB+iiA*crA*crA+iiB*crB*crB;
            if(k>kEpsilon) {float lambda=-Clamp(error,-0.2f,0.2f)*0.2f/k;Vec2 P=perp*lambda;ApplyPositionImpulse(*A,P,rA,-1);ApplyPositionImpulse(*B,P,rB,1);}
            if(j.type==JointType::Prismatic) {
                float angleError=WrapAngle(B->state.transform.angle-A->state.transform.angle-j.referenceAngle);
                float inv=iiA+iiB;if(inv>kEpsilon){float lambda=-Clamp(angleError,-0.2f,0.2f)*0.2f/inv;ApplyAngularPositionImpulse(*A,lambda,-1);ApplyAngularPositionImpulse(*B,lambda,1);}
                if(j.enableLimit){float tr=Dot(d,axis),limitError=tr<j.lower?tr-j.lower:(tr>j.upper?tr-j.upper:0.0f);if(limitError!=0&&imA+imB>kEpsilon){Vec2 P=axis*(-Clamp(limitError,-0.2f,0.2f)*0.2f/(imA+imB));ApplyPositionImpulse(*A,P,{},-1);ApplyPositionImpulse(*B,P,{},1);}}
            }
        } else {
            float k11=imA+imB+iiA*rA.y*rA.y+iiB*rB.y*rB.y,k12=-iiA*rA.x*rA.y-iiB*rB.x*rB.y,k22=imA+imB+iiA*rA.x*rA.x+iiB*rB.x*rB.x;
            Vec2 correction=Mat22{k11,k12,k12,k22}.Inverse()*(-d*0.2f);
            ApplyPositionImpulse(*A,correction,rA,-1); ApplyPositionImpulse(*B,correction,rB,1);
            float angleError=WrapAngle(B->state.transform.angle-A->state.transform.angle-j.referenceAngle);
            if(j.type==JointType::Weld || (j.type==JointType::Revolute&&j.enableLimit)) {
                if(j.type==JointType::Revolute) angleError=angleError<j.lower?angleError-j.lower:(angleError>j.upper?angleError-j.upper:0.0f);
                float inv=iiA+iiB;if(inv>kEpsilon&&std::abs(angleError)>1.0e-4f){float lambda=-Clamp(angleError,-0.2f,0.2f)*0.2f/inv;ApplyAngularPositionImpulse(*A,lambda,-1);ApplyAngularPositionImpulse(*B,lambda,1);}
            }
        }
    }
    void AdvanceBodies(float dt) {
        if(dt<=0.0f)return;
        for(auto& b:bodies){
            if(b.state.id==InvalidBodyID||!b.state.enabled||b.state.type==BodyType::Static)continue;
            if(b.state.type==BodyType::Dynamic&&!b.state.awake)continue;
            const Vec2 center=WorldCenter(b)+b.state.linearVelocity*dt;
            const float angle=WrapAngle(b.state.transform.angle+b.state.angularVelocity*dt);
            b.state.transform.angle=angle;
            b.state.transform.position=center-Rot2(angle).Apply(b.state.localCenter);
            b.state.worldCenter=center;
        }
        for(auto& b:bodies)if(b.state.id!=InvalidBodyID&&b.state.enabled)SyncProxy(b);
    }

    // Chronological event-based CCD. Every event advances the whole world to the
    // earliest pair impact, applies the impact impulse, then searches the remaining
    // interval again. This prevents the old one-hit-per-bullet tunnelling path.
    void AdvanceBodiesWithCCD(float dt) {
        lastCCDImpactCount=0;lastCCDOverflowCount=0;
        float remaining=dt;
        const uint32_t maxEvents=std::max<uint32_t>(1,settings.maxCCDImpactsPerStep);
        constexpr float targetDistance=0.001f;
        constexpr float minTimeAdvance=1.0e-6f;
        while(remaining>minTimeAdvance && lastCCDImpactCount<maxEvents){
            bool found=false;float earliest=1.0f;BodyID eventA=InvalidBodyID,eventB=InvalidBodyID;ShapeCastResult2D eventHit{};
            for(BodyID id=0;id<bodies.size();++id){
                BodyInternal& A=bodies[id];
                if(A.state.id!=id||!A.state.enabled||A.state.type!=BodyType::Dynamic||!A.state.bullet||!A.state.awake)continue;
                const Vec2 displacement=A.state.linearVelocity*remaining;
                const float rotation=A.state.angularVelocity*remaining;
                float maximumTargetMotion=0.0f;
                for(const auto& target:bodies){
                    if(target.state.id==InvalidBodyID||!target.state.enabled||target.state.type==BodyType::Static||target.state.id==id)continue;
                    const float angular=std::abs(target.state.angularVelocity*remaining)*ShapeRadiusAboutOrigin(target.shape,target.state.transform);
                    maximumTargetMotion=std::max(maximumTargetMotion,(target.state.linearVelocity*remaining).Length()+angular);
                }
                const float estimatedMotion=displacement.Length()+std::abs(rotation)*ShapeRadiusAboutOrigin(A.shape,A.state.transform)+maximumTargetMotion;
                if(estimatedMotion<settings.ccdMotionThreshold)continue;
                AABB2 swept=ComputeShapeSweepBounds(A.shape,A.state.transform,displacement,rotation,0.01f);
                swept.Expand(maximumTargetMotion);
                broadPhaseTree.Query(swept,[&](uint32_t otherId){
                    if(otherId==id||otherId>=bodies.size())return true;
                    BodyInternal* B=Body(otherId);
                    if(!B||!B->state.enabled||B->state.isSensor||!ShouldCollide(A.filter,B->filter))return true;
                    if(B->state.type==BodyType::Static&&A.state.type==BodyType::Static)return true;
                    if(B->state.bullet&&otherId<id)return true; // one canonical query for bullet pairs
                    bool blocked=false;for(const auto& j:joints)if(j.enabled&&!j.collideConnected&&((j.a==id&&j.b==otherId)||(j.a==otherId&&j.b==id))){blocked=true;break;}
                    if(blocked)return true;
                    const Vec2 targetMove=B->state.type==BodyType::Static?Vec2{}:B->state.linearVelocity*remaining;
                    const float targetRotation=B->state.type==BodyType::Static?0.0f:B->state.angularVelocity*remaining;
                    const auto hit=CastShapeAgainstShape2D(A.shape,A.state.transform,displacement,rotation,B->shape,B->state.transform,targetDistance,56,targetMove,targetRotation);
                    ++profile.shapeCastCount;profile.gjkIterations+=hit.iterations;
                    if(!hit.hit||hit.fraction>earliest)return true;
                    // Reject a time-zero overlap when bodies are already separating.
                    const Vec2 n=NormalizeOr(hit.normal,{1,0});
                    const Vec2 ca=WorldCenter(A),cb=WorldCenter(*B);
                    const Vec2 va=A.state.linearVelocity+Cross(A.state.angularVelocity,hit.point-ca);
                    const Vec2 vb=B->state.linearVelocity+Cross(B->state.angularVelocity,hit.point-cb);
                    if(hit.fraction<=1.0e-5f&&Dot(vb-va,n)>=-1.0e-5f)return true;
                    if(!found||hit.fraction<earliest-1.0e-6f||(std::abs(hit.fraction-earliest)<=1.0e-6f&&std::pair<BodyID,BodyID>{id,otherId}<std::pair<BodyID,BodyID>{eventA,eventB})){
                        found=true;earliest=hit.fraction;eventA=id;eventB=otherId;eventHit=hit;
                    }
                    return true;
                });
            }
            if(!found){AdvanceBodies(remaining);remaining=0.0f;break;}
            const float eventDt=Clamp(remaining*earliest,0.0f,remaining);
            if(eventDt>0.0f){AdvanceBodies(eventDt);remaining-=eventDt;}
            BodyInternal* A=Body(eventA);BodyInternal* B=Body(eventB);
            if(!A||!B){if(eventDt<=minTimeAdvance){AdvanceBodies(std::min(remaining,minTimeAdvance));remaining-=std::min(remaining,minTimeAdvance);}continue;}
            const Vec2 point=eventHit.point;
            const Vec2 ra=point-WorldCenter(*A),rb=point-WorldCenter(*B);
            const Vec2 va=A->state.linearVelocity+Cross(A->state.angularVelocity,ra);
            const Vec2 vb=B->state.linearVelocity+Cross(B->state.angularVelocity,rb);
            const Vec2 n=NormalizeOr(eventHit.normal,{1,0});
            const float vn=Dot(vb-va,n);
            const float rnA=Cross(ra,n),rnB=Cross(rb,n);
            const float effective=InvMass(*A)+InvMass(*B)+InvI(*A)*rnA*rnA+InvI(*B)*rnB*rnB;
            bool applied=false;
            if(vn< -1.0e-5f&&effective>kEpsilon){
                const float restitution=std::max(A->material.restitution,B->material.restitution);
                const float impulse=-(1.0f+restitution)*vn/effective;
                const Vec2 P=n*impulse;
                ApplyImpulse(*A,P,ra,-1.0f);ApplyImpulse(*B,P,rb,1.0f);
                Wake(*A);Wake(*B);applied=true;
            }
            ++lastCCDImpactCount;++profile.ccdImpactCount;
            // Repeated zero-time contacts should be impossible after an impulse;
            // still consume a tiny amount if numerical degeneracy prevents change.
            if(!applied&&eventDt<=minTimeAdvance){const float consume=std::min(remaining,minTimeAdvance);AdvanceBodies(consume);remaining-=consume;}
        }
        if(remaining>0.0f){
            if(lastCCDImpactCount>=maxEvents){++lastCCDOverflowCount;}
            AdvanceBodies(remaining);
        }
    }
    void Simulate(float dt,bool clearForces=true,const PhysicsStepHooks2D* hooks=nullptr) {
        using ProfileClock = std::chrono::steady_clock;
        const auto totalStart=ProfileClock::now();
        stepping=true;
        ++stepCount; jointBreakEvents.clear(); for(auto&j:joints){j.impulse={};j.angularImpulse=j.motorImpulse=j.limitImpulse=0;j.mouseAppliedThisStep=false;}
        for(auto& b:bodies) if(b.state.id!=InvalidBodyID && b.state.enabled) {
            b.previousPosition=b.state.transform.position; b.previousAngle=b.state.transform.angle;
            if(b.state.type==BodyType::Dynamic && b.state.awake) {
                b.state.linearVelocity+=(settings.gravity*b.gravityScale+b.state.force*b.state.inverseMass)*dt;
                b.state.angularVelocity+=b.state.torque*b.state.inverseInertia*dt;
                float linearDamping=b.linearDamping>=0.0f?b.linearDamping:settings.linearDamping;float angularDamping=b.angularDamping>=0.0f?b.angularDamping:settings.angularDamping;b.state.linearVelocity*=1.0f/(1.0f+linearDamping*dt);
                b.state.angularVelocity*=1.0f/(1.0f+angularDamping*dt);
            }
        }
        if(hooks && hooks->prepare) hooks->prepare(dt);
        BuildContacts();
        struct SolverIsland { std::vector<size_t> contactIndices,jointIndices;std::vector<BodyID> bodyIds; };
        std::vector<size_t> parent(bodies.size());std::iota(parent.begin(),parent.end(),size_t(0));
        auto findRoot=[&](size_t x){size_t r=x;while(parent[r]!=r)r=parent[r];while(parent[x]!=x){size_t n=parent[x];parent[x]=r;x=n;}return r;};
        auto unite=[&](BodyID a,BodyID b){const BodyInternal*A=Body(a);const BodyInternal*B=Body(b);if(!A||!B||A->state.type!=BodyType::Dynamic||B->state.type!=BodyType::Dynamic)return;size_t ra=findRoot(a),rb=findRoot(b);if(ra!=rb)parent[rb]=ra;};
        for(const auto& c:contacts)unite(c.a,c.b);
        for(const auto& j:joints)if(j.enabled&&j.type!=JointType::Mouse)unite(j.a,j.b);
        if(hooks&&hooks->gatherIslandLinks){std::vector<std::pair<BodyID,BodyID>> links;hooks->gatherIslandLinks(links);for(const auto& link:links)unite(link.first,link.second);}
        std::map<size_t,size_t> islandByRoot;std::vector<SolverIsland> islands;
        for(BodyID id=0;id<bodies.size();++id){const auto*b=Body(id);if(!b||!b->state.enabled||b->state.type!=BodyType::Dynamic)continue;size_t root=findRoot(id);auto it=islandByRoot.find(root);if(it==islandByRoot.end()){islandByRoot[root]=islands.size();islands.emplace_back();it=islandByRoot.find(root);}islands[it->second].bodyIds.push_back(id);}
        auto islandForBody=[&](BodyID id)->size_t{const auto*b=Body(id);if(!b||b->state.type!=BodyType::Dynamic)return std::numeric_limits<size_t>::max();auto it=islandByRoot.find(findRoot(id));return it==islandByRoot.end()?std::numeric_limits<size_t>::max():it->second;};
        for(size_t i=0;i<contacts.size();++i){size_t island=islandForBody(contacts[i].a);if(island==std::numeric_limits<size_t>::max())island=islandForBody(contacts[i].b);if(island!=std::numeric_limits<size_t>::max())islands[island].contactIndices.push_back(i);}
        for(size_t i=0;i<joints.size();++i){const auto&j=joints[i];if(!j.enabled)continue;size_t island=j.type==JointType::Mouse?islandForBody(j.b):islandForBody(j.a);if(island==std::numeric_limits<size_t>::max())island=islandForBody(j.b);if(island!=std::numeric_limits<size_t>::max())islands[island].jointIndices.push_back(i);}
        lastIslandCount=uint32_t(islands.size());
        auto parallelIslands=[&](auto&& function){
            if(!settings.enableMultithreading||islands.size()<2){for(auto& island:islands)function(island);return;}
            workerPool.ParallelFor(islands.size(),[&](size_t index){function(islands[index]);});
        };
        const auto velocityStart=ProfileClock::now();
        // Warm start once per island, then advance all rigid and coupled soft
        // constraints in the same global iteration schedule. The callback runs
        // only after every rigid island has completed that iteration.
        parallelIslands([&](SolverIsland& island){
            for(size_t ci:island.contactIndices){auto&c=contacts[ci];BodyInternal*A=Body(c.a);BodyInternal*B=Body(c.b);if(!A||!B)continue;Vec2 n=c.manifold.normal,t=n.Perpendicular();for(uint8_t i=0;i<c.manifold.pointCount;++i){auto&cp=c.manifold.points[i];Vec2 impulse=n*cp.normalImpulse+t*cp.tangentImpulse;ApplyImpulse(*A,impulse,cp.point-WorldCenter(*A),-1);ApplyImpulse(*B,impulse,cp.point-WorldCenter(*B),1);}}
            for(size_t ji:island.jointIndices)SolveJointVelocity(joints[ji],dt);
        });
        for(int iter=0;iter<settings.velocityIterations;++iter){
            parallelIslands([&](SolverIsland& island){
                SolveContactVelocitySet(island.contactIndices);
                for(size_t ji:island.jointIndices)SolveJointVelocity(joints[ji],dt);
                if(hooks&&hooks->solveVelocityIsland)hooks->solveVelocityIsland(dt,iter,island.bodyIds);
            });
            if(hooks&&hooks->solveVelocityDetached)hooks->solveVelocityDetached(dt,iter);
            if(hooks&&hooks->solveVelocityIteration)hooks->solveVelocityIteration(dt,iter);
        }
        const auto velocityEnd=ProfileClock::now();
        profile.velocitySolveMilliseconds += std::chrono::duration<double,std::milli>(velocityEnd-velocityStart).count();
        profile.simdBatchCount=simdBatchCounter.load(std::memory_order_relaxed);
        const auto ccdStart=ProfileClock::now();
        AdvanceBodiesWithCCD(dt);
        const auto ccdEnd=ProfileClock::now();
        profile.ccdMilliseconds += std::chrono::duration<double,std::milli>(ccdEnd-ccdStart).count();
        const auto positionStart=ProfileClock::now();
        for(int iter=0;iter<settings.positionIterations;++iter){
            parallelIslands([&](SolverIsland& island){
                for(size_t ci:island.contactIndices)SolveContactPosition(contacts[ci]);
                for(size_t ji:island.jointIndices)SolveJointPosition(joints[ji]);
                if(hooks&&hooks->solvePositionIsland)hooks->solvePositionIsland(dt,iter,island.bodyIds);
            });
            if(hooks&&hooks->solvePositionDetached)hooks->solvePositionDetached(dt,iter);
            if(hooks&&hooks->solvePositionIteration)hooks->solvePositionIteration(dt,iter);
        }
        if(hooks && hooks->finish) hooks->finish(dt);
        const auto positionEnd=ProfileClock::now();
        profile.positionSolveMilliseconds += std::chrono::duration<double,std::milli>(positionEnd-positionStart).count();
        impulseCache.clear(); for(const auto& c:contacts){CachedImpulse cached;for(uint8_t i=0;i<c.manifold.pointCount;++i){cached.feature[i]=c.manifold.points[i].featureId;cached.normal[i]=c.manifold.points[i].normalImpulse;cached.tangent[i]=c.manifold.points[i].tangentImpulse;}impulseCache[c.cacheKey]=cached;}
        for(JointID id=0;id<joints.size();++id){auto&j=joints[id];if(!j.enabled)continue;float invDt=1.0f/std::max(dt,kEpsilon);Vec2 force=j.impulse*invDt;float torque=std::abs(j.angularImpulse+j.motorImpulse+j.limitImpulse)*invDt;if(force.Length()>j.breakForce||torque>j.breakTorque){jointBreakEvents.push_back({id,j.a,j.b,force,torque});j.enabled=false;}}
        for(auto& b:bodies) if(b.state.id!=InvalidBodyID) {
            if(clearForces){b.state.force={};b.state.torque=0;}
            if(b.state.type==BodyType::Dynamic&&b.allowSleep) {
                if(b.state.linearVelocity.LengthSquared()<settings.sleepLinearThreshold*settings.sleepLinearThreshold && std::abs(b.state.angularVelocity)<settings.sleepAngularThreshold) {
                    b.sleepTime+=dt; if(b.sleepTime>settings.timeToSleep){b.state.awake=false;b.state.linearVelocity={};b.state.angularVelocity=0;}
                } else b.sleepTime=0;
            }
            b.state.worldCenter=WorldCenter(b);
            SyncProxy(b);
        }
        profile.totalStepMilliseconds += std::chrono::duration<double,std::milli>(ProfileClock::now()-totalStart).count();
        stepping=false;
    }
};

PhysicsWorld2D::PhysicsWorld2D(const WorldSettings& s):impl_(new Impl){SetSettings(s);}
PhysicsWorld2D::~PhysicsWorld2D()=default;
PhysicsWorld2D::PhysicsWorld2D(PhysicsWorld2D&&) noexcept=default;
PhysicsWorld2D& PhysicsWorld2D::operator=(PhysicsWorld2D&&) noexcept=default;
BodyID PhysicsWorld2D::CreateBody(const BodyDef& def) {
    if(!def.shape.IsValid()||!std::isfinite(def.position.x)||!std::isfinite(def.position.y)||!std::isfinite(def.angle)||!std::isfinite(def.linearVelocity.x)||!std::isfinite(def.linearVelocity.y)||!std::isfinite(def.angularVelocity)||!std::isfinite(def.gravityScale)||!std::isfinite(def.linearDamping)||!std::isfinite(def.angularDamping)||def.linearDamping < -1.0f||def.angularDamping < -1.0f||!std::isfinite(def.material.density)||def.material.density<0||(def.type==BodyType::Dynamic&&def.material.density<=0)||!std::isfinite(def.material.friction)||def.material.friction<0||!std::isfinite(def.material.restitution)||def.material.restitution<0) return InvalidBodyID;
    BodyInternal b; b.state.id=BodyID(impl_->bodies.size()); b.state.type=def.type;b.state.transform={def.position,def.angle};b.state.linearVelocity=def.linearVelocity;b.state.angularVelocity=def.angularVelocity;
    b.state.awake=def.initiallyAwake;b.state.enabled=def.enabled;b.state.isSensor=def.isSensor;b.state.bullet=def.bullet;b.state.userData=def.userData;b.state.name=def.name;
    b.shape=def.shape;b.material=def.material;b.filter=def.filter;b.fixedRotation=def.fixedRotation;b.allowSleep=def.allowSleep;b.oneWayPlatform=def.oneWayPlatform;b.oneWayNormal=NormalizeOr(def.oneWayNormal);b.gravityScale=def.gravityScale;b.linearDamping=def.linearDamping;b.angularDamping=def.angularDamping;b.previousPosition=def.position;b.previousAngle=def.angle;
    impl_->RecomputeMass(b,def.material.density); if(def.type==BodyType::Dynamic&&b.state.mass<=kEpsilon)return InvalidBodyID;
    impl_->bodies.push_back(std::move(b));
    impl_->SyncProxy(impl_->bodies.back());
    return BodyID(impl_->bodies.size()-1);
}
bool PhysicsWorld2D::DestroyBody(BodyID id) {
    auto* b=impl_->Body(id); if(!b)return false;
    if(b->broadPhaseProxy!=InvalidBroadPhaseProxy2D) { impl_->broadPhaseTree.DestroyProxy(b->broadPhaseProxy); b->broadPhaseProxy=InvalidBroadPhaseProxy2D; }
    b->state.id=InvalidBodyID;b->state.enabled=false;
    for(auto& j:impl_->joints)if(j.a==id||j.b==id)j.enabled=false;
    for(auto it=impl_->previousPairs.begin();it!=impl_->previousPairs.end();) {if(BodyID(*it>>32)==id||BodyID(*it&0xffffffffu)==id)it=impl_->previousPairs.erase(it);else ++it;}for(auto it=impl_->previousSensorPairs.begin();it!=impl_->previousSensorPairs.end();) {if(BodyID(*it>>32)==id||BodyID(*it&0xffffffffu)==id)it=impl_->previousSensorPairs.erase(it);else ++it;}
    return true;
}
BodyState* PhysicsWorld2D::GetBody(BodyID id){auto*b=impl_->Body(id);return b?&b->state:nullptr;}
const BodyState* PhysicsWorld2D::GetBody(BodyID id)const{auto*b=impl_->Body(id);return b?&b->state:nullptr;}
const Shape2D* PhysicsWorld2D::GetBodyShape(BodyID id)const{auto*b=impl_->Body(id);return b?&b->shape:nullptr;}
AABB2 PhysicsWorld2D::GetBodyAABB(BodyID id)const{auto*b=impl_->Body(id);return b?ComputeAABB(b->shape,b->state.transform):AABB2{};}
AABB2 PhysicsWorld2D::GetBroadPhaseFatAABB(BodyID id)const{const auto*b=impl_->Body(id);return b&&b->broadPhaseProxy!=InvalidBroadPhaseProxy2D?impl_->broadPhaseTree.GetFatBounds(b->broadPhaseProxy):AABB2{};}
Transform2D PhysicsWorld2D::GetInterpolatedTransform(BodyID id)const{auto*b=impl_->Body(id);if(!b)return {};float a=Clamp(impl_->alpha,0.0f,1.0f);return {b->previousPosition+(b->state.transform.position-b->previousPosition)*a,WrapAngle(b->previousAngle+WrapAngle(b->state.transform.angle-b->previousAngle)*a)};}
bool PhysicsWorld2D::SetTransform(BodyID id,Vec2 p,float a,bool wake){auto*b=impl_->Body(id);if(!b||!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(a))return false;b->state.transform={p,a};b->state.worldCenter=impl_->WorldCenter(*b);b->previousPosition=p;b->previousAngle=a;impl_->SyncProxy(*b);if(wake)impl_->Wake(*b);return true;}
bool PhysicsWorld2D::SetLinearVelocity(BodyID id,Vec2 v,bool wake){auto*b=impl_->Body(id);if(!b||!std::isfinite(v.x)||!std::isfinite(v.y))return false;b->state.linearVelocity=v;if(wake)impl_->Wake(*b);return true;}
bool PhysicsWorld2D::SetAngularVelocity(BodyID id,float v,bool wake){auto*b=impl_->Body(id);if(!b||!std::isfinite(v))return false;b->state.angularVelocity=v;if(wake)impl_->Wake(*b);return true;}
bool PhysicsWorld2D::ApplyForce(BodyID id,Vec2 f,Vec2 p,bool wake){auto*b=impl_->Body(id);if(!b||!std::isfinite(f.x)||!std::isfinite(f.y)||!std::isfinite(p.x)||!std::isfinite(p.y))return false;b->state.force+=f;b->state.torque+=Cross(p-impl_->WorldCenter(*b),f);if(wake)impl_->Wake(*b);return true;}
bool PhysicsWorld2D::ApplyForceToCenter(BodyID id,Vec2 f,bool wake){auto*b=impl_->Body(id);if(!b||!std::isfinite(f.x)||!std::isfinite(f.y))return false;b->state.force+=f;if(wake)impl_->Wake(*b);return true;}
bool PhysicsWorld2D::ApplyTorque(BodyID id,float t,bool wake){auto*b=impl_->Body(id);if(!b||!std::isfinite(t))return false;b->state.torque+=t;if(wake)impl_->Wake(*b);return true;}
bool PhysicsWorld2D::ApplyLinearImpulse(BodyID id,Vec2 impulse,Vec2 p,bool wake){auto*b=impl_->Body(id);if(!b||!std::isfinite(impulse.x)||!std::isfinite(impulse.y)||!std::isfinite(p.x)||!std::isfinite(p.y))return false;if(b->state.type!=BodyType::Dynamic)return true;b->state.linearVelocity+=impulse*b->state.inverseMass;b->state.angularVelocity+=b->state.inverseInertia*Cross(p-impl_->WorldCenter(*b),impulse);if(wake)impl_->Wake(*b);return true;}
bool PhysicsWorld2D::ApplyPositionImpulse(BodyID id,Vec2 impulse,Vec2 worldPoint,bool wake){auto*b=impl_->Body(id);if(!b||!std::isfinite(impulse.x)||!std::isfinite(impulse.y)||!std::isfinite(worldPoint.x)||!std::isfinite(worldPoint.y))return false;if(b->state.type!=BodyType::Dynamic)return true;impl_->ApplyPositionImpulse(*b,impulse,worldPoint-impl_->WorldCenter(*b),1.0f);b->state.worldCenter=impl_->WorldCenter(*b);if(!impl_->stepping)impl_->SyncProxy(*b);if(wake)impl_->Wake(*b);return true;}
bool PhysicsWorld2D::ApplyAngularImpulse(BodyID id,float impulse,bool wake){auto*b=impl_->Body(id);if(!b||!std::isfinite(impulse))return false;if(b->state.type==BodyType::Dynamic)b->state.angularVelocity+=impulse*b->state.inverseInertia;if(wake)impl_->Wake(*b);return true;}
bool PhysicsWorld2D::SetBodyEnabled(BodyID id,bool e){auto*b=impl_->Body(id);if(!b)return false;b->state.enabled=e;impl_->SyncProxy(*b);if(e)impl_->Wake(*b);return true;}
bool PhysicsWorld2D::SetBodyGravityScale(BodyID id,float scale){auto*b=impl_->Body(id);if(!b||!std::isfinite(scale))return false;b->gravityScale=scale;impl_->Wake(*b);return true;}
bool PhysicsWorld2D::SetBodyDamping(BodyID id,float linear,float angular){auto*b=impl_->Body(id);if(!b||!std::isfinite(linear)||!std::isfinite(angular)||linear<0||angular<0)return false;b->linearDamping=linear;b->angularDamping=angular;impl_->Wake(*b);return true;}
bool PhysicsWorld2D::SetBodyShape(BodyID id,const Shape2D&s,float density){auto*b=impl_->Body(id);if(!b||!s.IsValid()||!std::isfinite(density)||density<0||(b->state.type==BodyType::Dynamic&&(density<=0||ComputeMass(s,density).mass<=kEpsilon)))return false;b->shape=s;impl_->RecomputeMass(*b,density);impl_->SyncProxy(*b);impl_->Wake(*b);return true;}
bool PhysicsWorld2D::SetBodyMaterial(BodyID id,const Material2D&m){auto*b=impl_->Body(id);if(!b||!std::isfinite(m.density)||m.density<0||(b->state.type==BodyType::Dynamic&&m.density<=0)||!std::isfinite(m.friction)||m.friction<0||!std::isfinite(m.restitution)||m.restitution<0)return false;b->material=m;impl_->RecomputeMass(*b,m.density);impl_->SyncProxy(*b);return true;}
bool PhysicsWorld2D::SetBodyFilter(BodyID id,const CollisionFilter&f){auto*b=impl_->Body(id);if(!b)return false;b->filter=f;return true;}
void PhysicsWorld2D::SetGravity(Vec2 g){if(std::isfinite(g.x)&&std::isfinite(g.y))impl_->settings.gravity=g;}
Vec2 PhysicsWorld2D::GetGravity()const{return impl_->settings.gravity;}
const WorldSettings& PhysicsWorld2D::GetSettings()const{return impl_->settings;}
void PhysicsWorld2D::SetSettings(const WorldSettings&s){WorldSettings v=s;if(!std::isfinite(v.gravity.x)||!std::isfinite(v.gravity.y))v.gravity={0,-9.81f};v.velocityIterations=std::max(1,v.velocityIterations);v.positionIterations=std::max(1,v.positionIterations);v.fixedTimeStep=std::isfinite(v.fixedTimeStep)?Clamp(v.fixedTimeStep,0.001f,0.1f):1.0f/60.0f;v.maxSubSteps=std::max(1,v.maxSubSteps);v.maxCCDSubSteps=std::max(1,v.maxCCDSubSteps);v.maxCCDImpactsPerStep=std::max(1,v.maxCCDImpactsPerStep);v.ccdMotionThreshold=std::isfinite(v.ccdMotionThreshold)?Clamp(v.ccdMotionThreshold,0.05f,2.0f):0.5f;v.broadPhaseCellSize=std::isfinite(v.broadPhaseCellSize)?std::max(0.25f,v.broadPhaseCellSize):1.0f;v.maxFrameTime=std::isfinite(v.maxFrameTime)?std::max(v.fixedTimeStep,v.maxFrameTime):0.25f;v.linearDamping=std::isfinite(v.linearDamping)?std::max(0.0f,v.linearDamping):0.01f;v.angularDamping=std::isfinite(v.angularDamping)?std::max(0.0f,v.angularDamping):0.01f;v.contactSlop=std::isfinite(v.contactSlop)?std::max(0.0f,v.contactSlop):0.005f;v.baumgarte=std::isfinite(v.baumgarte)?Clamp(v.baumgarte,0.0f,1.0f):0.2f;v.sleepLinearThreshold=std::isfinite(v.sleepLinearThreshold)?std::max(0.0f,v.sleepLinearThreshold):0.03f;v.sleepAngularThreshold=std::isfinite(v.sleepAngularThreshold)?std::max(0.0f,v.sleepAngularThreshold):0.03f;v.timeToSleep=std::isfinite(v.timeToSleep)?std::max(0.0f,v.timeToSleep):0.5f;impl_->settings=v;uint32_t requested=v.workerCount?v.workerCount:std::max(1u,std::thread::hardware_concurrency());impl_->workerPool.Configure(v.enableMultithreading&&!v.deterministicMode&&requested>1?size_t(requested-1):0);}
void PhysicsWorld2D::Step(float frameDelta){if(!std::isfinite(frameDelta)||frameDelta<=0)return;frameDelta=std::min(frameDelta,impl_->settings.maxFrameTime);impl_->accumulator+=frameDelta;int steps=0;const float h=std::max(impl_->settings.fixedTimeStep,1.0f/1000.0f);while(impl_->accumulator>=h&&steps<impl_->settings.maxSubSteps){StepFixed(h);impl_->accumulator-=h;++steps;}if(steps==impl_->settings.maxSubSteps&&impl_->accumulator>h)impl_->accumulator=std::fmod(impl_->accumulator,h);impl_->alpha=impl_->accumulator/h;}
void PhysicsWorld2D::StepFixed(float dt,const PhysicsStepHooks2D* hooks){
    if(!std::isfinite(dt)||dt<=0)return;
    int subSteps=1;
    for(const auto&b:impl_->bodies){if(b.state.id==InvalidBodyID||!b.state.enabled||b.state.type!=BodyType::Dynamic||!b.state.bullet||!b.state.awake)continue;AABB2 a=ComputeAABB(b.shape,b.state.transform);Vec2 ext=a.Extents();float minExtent=std::max(0.01f,2.0f*std::min(ext.x,ext.y));float radius=std::max(ext.x,ext.y);Vec2 acceleration=impl_->settings.gravity+b.state.force*b.state.inverseMass;float travel=b.state.linearVelocity.Length()*dt+0.5f*acceleration.Length()*dt*dt+std::abs(b.state.angularVelocity)*radius*dt;float allowed=std::max(1.0e-4f,minExtent*impl_->settings.ccdMotionThreshold);subSteps=std::max(subSteps,int(std::ceil(travel/allowed)));}
    subSteps=Clamp(subSteps,1,impl_->settings.maxCCDSubSteps);impl_->lastCCDSubSteps=uint32_t(subSteps);
    std::vector<Vec2> previousPositions(impl_->bodies.size());std::vector<float> previousAngles(impl_->bodies.size());for(size_t i=0;i<impl_->bodies.size();++i){previousPositions[i]=impl_->bodies[i].previousPosition;previousAngles[i]=impl_->bodies[i].previousAngle;}
    std::vector<ContactEvent> begins,ends;std::vector<JointBreakEvent2D> breaks;for(int i=0;i<subSteps;++i){impl_->Simulate(dt/float(subSteps),i==subSteps-1,hooks);begins.insert(begins.end(),impl_->beginEvents.begin(),impl_->beginEvents.end());ends.insert(ends.end(),impl_->endEvents.begin(),impl_->endEvents.end());breaks.insert(breaks.end(),impl_->jointBreakEvents.begin(),impl_->jointBreakEvents.end());}
    for(size_t i=0;i<impl_->bodies.size();++i){impl_->bodies[i].previousPosition=previousPositions[i];impl_->bodies[i].previousAngle=previousAngles[i];}
    impl_->beginEvents=std::move(begins);impl_->endEvents=std::move(ends);impl_->jointBreakEvents=std::move(breaks);impl_->alpha=1.0f;
}
void PhysicsWorld2D::ClearForces(){for(auto&b:impl_->bodies){b.state.force={};b.state.torque=0;}}

JointID PhysicsWorld2D::CreateDistanceJoint(const DistanceJointDef& d){if(!impl_->Body(d.bodyA)||!impl_->Body(d.bodyB))return InvalidJointID;JointInternal j;j.type=JointType::Distance;j.a=d.bodyA;j.b=d.bodyB;j.anchorA=d.localAnchorA;j.anchorB=d.localAnchorB;j.length=d.length>=0?d.length:Distance(WorldPoint(*impl_->Body(d.bodyA),d.localAnchorA),WorldPoint(*impl_->Body(d.bodyB),d.localAnchorB));j.minLength=d.minLength;j.maxLength=d.maxLength;j.frequency=d.frequencyHz;j.damping=d.dampingRatio;j.collideConnected=d.collideConnected;impl_->joints.push_back(j);return JointID(impl_->joints.size()-1);}
JointID PhysicsWorld2D::CreateRevoluteJoint(const RevoluteJointDef& d){if(!impl_->Body(d.bodyA)||!impl_->Body(d.bodyB))return InvalidJointID;JointInternal j;j.type=JointType::Revolute;j.a=d.bodyA;j.b=d.bodyB;j.anchorA=d.localAnchorA;j.anchorB=d.localAnchorB;j.enableLimit=d.enableLimit;j.lower=d.lowerAngle;j.upper=d.upperAngle;j.enableMotor=d.enableMotor;j.motorSpeed=d.motorSpeed;j.maxMotor=d.maxMotorTorque;j.collideConnected=d.collideConnected;j.referenceAngle=impl_->Body(d.bodyB)->state.transform.angle-impl_->Body(d.bodyA)->state.transform.angle;impl_->joints.push_back(j);return JointID(impl_->joints.size()-1);}
JointID PhysicsWorld2D::CreateWeldJoint(const WeldJointDef& d){if(!impl_->Body(d.bodyA)||!impl_->Body(d.bodyB))return InvalidJointID;JointInternal j;j.type=JointType::Weld;j.a=d.bodyA;j.b=d.bodyB;j.anchorA=d.localAnchorA;j.anchorB=d.localAnchorB;j.referenceAngle=std::isfinite(d.referenceAngle)?d.referenceAngle:impl_->Body(d.bodyB)->state.transform.angle-impl_->Body(d.bodyA)->state.transform.angle;j.frequency=d.frequencyHz;j.damping=d.dampingRatio;j.collideConnected=d.collideConnected;impl_->joints.push_back(j);return JointID(impl_->joints.size()-1);}
JointID PhysicsWorld2D::CreatePrismaticJoint(const PrismaticJointDef& d){if(!impl_->Body(d.bodyA)||!impl_->Body(d.bodyB))return InvalidJointID;JointInternal j;j.type=JointType::Prismatic;j.a=d.bodyA;j.b=d.bodyB;j.anchorA=d.localAnchorA;j.anchorB=d.localAnchorB;j.axisA=NormalizeOr(d.localAxisA);j.enableLimit=d.enableLimit;j.lower=d.lowerTranslation;j.upper=d.upperTranslation;j.enableMotor=d.enableMotor;j.motorSpeed=d.motorSpeed;j.maxMotor=d.maxMotorForce;j.collideConnected=d.collideConnected;j.referenceAngle=impl_->Body(d.bodyB)->state.transform.angle-impl_->Body(d.bodyA)->state.transform.angle;impl_->joints.push_back(j);return JointID(impl_->joints.size()-1);}
JointID PhysicsWorld2D::CreateWheelJoint(const WheelJointDef& d){if(!impl_->Body(d.bodyA)||!impl_->Body(d.bodyB))return InvalidJointID;JointInternal j;j.type=JointType::Wheel;j.a=d.bodyA;j.b=d.bodyB;j.anchorA=d.localAnchorA;j.anchorB=d.localAnchorB;j.axisA=NormalizeOr(d.localAxisA);j.frequency=d.frequencyHz;j.damping=d.dampingRatio;j.enableMotor=d.enableMotor;j.motorSpeed=d.motorSpeed;j.maxMotor=d.maxMotorTorque;j.collideConnected=d.collideConnected;j.referenceAngle=impl_->Body(d.bodyB)->state.transform.angle-impl_->Body(d.bodyA)->state.transform.angle;impl_->joints.push_back(j);return JointID(impl_->joints.size()-1);}

JointID PhysicsWorld2D::CreateRopeJoint(const RopeJointDef&d){if(!impl_->Body(d.bodyA)||!impl_->Body(d.bodyB)||!std::isfinite(d.maxLength)||d.maxLength<0)return InvalidJointID;JointInternal j;j.type=JointType::Rope;j.a=d.bodyA;j.b=d.bodyB;j.anchorA=d.localAnchorA;j.anchorB=d.localAnchorB;j.maxLength=d.maxLength;j.collideConnected=d.collideConnected;impl_->joints.push_back(j);return JointID(impl_->joints.size()-1);}
JointID PhysicsWorld2D::CreateMouseJoint(const MouseJointDef&d){BodyInternal*b=impl_->Body(d.body);if(!b||b->state.type!=BodyType::Dynamic||!std::isfinite(d.target.x)||!std::isfinite(d.target.y)||!std::isfinite(d.maxForce)||d.maxForce<=0||!std::isfinite(d.frequencyHz)||d.frequencyHz<0||!std::isfinite(d.dampingRatio)||d.dampingRatio<0)return InvalidJointID;JointInternal j;j.type=JointType::Mouse;j.a=InvalidBodyID;j.b=d.body;j.anchorB=d.localAnchor;j.target=d.target;j.maxForce=d.maxForce;j.frequency=d.frequencyHz;j.damping=d.dampingRatio;impl_->joints.push_back(j);impl_->Wake(*b);return JointID(impl_->joints.size()-1);}
JointID PhysicsWorld2D::CreateFrictionJoint(const FrictionJointDef& d) {
    if(!impl_->Body(d.bodyA)||!impl_->Body(d.bodyB)||d.bodyA==d.bodyB||!std::isfinite(d.maxForce)||!std::isfinite(d.maxTorque)||d.maxForce<0||d.maxTorque<0)return InvalidJointID;
    JointInternal j;j.type=JointType::Friction;j.a=d.bodyA;j.b=d.bodyB;j.anchorA=d.localAnchorA;j.anchorB=d.localAnchorB;j.maxForce=d.maxForce;j.maxTorque=d.maxTorque;j.collideConnected=d.collideConnected;impl_->joints.push_back(j);return JointID(impl_->joints.size()-1);
}
JointID PhysicsWorld2D::CreateMotorJoint(const MotorJointDef& d) {
    if(!impl_->Body(d.bodyA)||!impl_->Body(d.bodyB)||d.bodyA==d.bodyB||!std::isfinite(d.linearOffset.x)||!std::isfinite(d.linearOffset.y)||!std::isfinite(d.angularOffset)||!std::isfinite(d.maxForce)||!std::isfinite(d.maxTorque)||!std::isfinite(d.correctionFactor)||d.maxForce<0||d.maxTorque<0||d.correctionFactor<0||d.correctionFactor>1)return InvalidJointID;
    JointInternal j;j.type=JointType::Motor;j.a=d.bodyA;j.b=d.bodyB;j.linearOffset=d.linearOffset;j.angularOffset=d.angularOffset;j.maxForce=d.maxForce;j.maxTorque=d.maxTorque;j.correctionFactor=d.correctionFactor;j.collideConnected=d.collideConnected;impl_->joints.push_back(j);return JointID(impl_->joints.size()-1);
}
JointID PhysicsWorld2D::CreatePulleyJoint(const PulleyJointDef& d) {
    BodyInternal* a=impl_->Body(d.bodyA);BodyInternal* b=impl_->Body(d.bodyB);
    if(!a||!b||d.bodyA==d.bodyB||!std::isfinite(d.ratio)||d.ratio<=0||!std::isfinite(d.groundAnchorA.x)||!std::isfinite(d.groundAnchorA.y)||!std::isfinite(d.groundAnchorB.x)||!std::isfinite(d.groundAnchorB.y))return InvalidJointID;
    JointInternal j;j.type=JointType::Pulley;j.a=d.bodyA;j.b=d.bodyB;j.anchorA=d.localAnchorA;j.anchorB=d.localAnchorB;j.groundAnchorA=d.groundAnchorA;j.groundAnchorB=d.groundAnchorB;j.ratio=d.ratio;
    const Vec2 pa=WorldPoint(*a,d.localAnchorA),pb=WorldPoint(*b,d.localAnchorB);
    const float initial=(pa-d.groundAnchorA).Length()+d.ratio*(pb-d.groundAnchorB).Length();
    if(std::isfinite(d.constantLength)&&d.constantLength>=0)j.constant=d.constantLength;else j.constant=initial;
    j.collideConnected=d.collideConnected;impl_->joints.push_back(j);return JointID(impl_->joints.size()-1);
}
JointID PhysicsWorld2D::CreateGearJoint(const GearJointDef& d) {
    BodyInternal* a=impl_->Body(d.bodyA);BodyInternal* b=impl_->Body(d.bodyB);
    if(!a||!b||d.bodyA==d.bodyB||!std::isfinite(d.ratio)||std::abs(d.ratio)<kEpsilon||!std::isfinite(d.motorSpeed)||!std::isfinite(d.maxMotorTorque)||d.maxMotorTorque<0)return InvalidJointID;
    JointInternal j;j.type=JointType::Gear;j.a=d.bodyA;j.b=d.bodyB;j.ratio=d.ratio;j.constant=std::isfinite(d.constant)?d.constant:b->state.transform.angle+d.ratio*a->state.transform.angle;j.motorSpeed=d.motorSpeed;j.maxMotor=d.maxMotorTorque;j.enableMotor=d.enableMotor;j.collideConnected=d.collideConnected;impl_->joints.push_back(j);return JointID(impl_->joints.size()-1);
}
bool PhysicsWorld2D::SetJointMotor(JointID id,float speed,float maxForce,bool enabled) {
    if(id>=impl_->joints.size()||!impl_->joints[id].enabled||!std::isfinite(speed)||!std::isfinite(maxForce)||maxForce<0)return false;
    auto& j=impl_->joints[id];
    if(j.type!=JointType::Revolute&&j.type!=JointType::Prismatic&&j.type!=JointType::Wheel&&j.type!=JointType::Gear)return false;
    j.motorSpeed=speed;j.maxMotor=maxForce;j.enableMotor=enabled;if(auto*a=impl_->Body(j.a))impl_->Wake(*a);if(auto*b=impl_->Body(j.b))impl_->Wake(*b);return true;
}

bool PhysicsWorld2D::SetMouseJointTarget(JointID id,Vec2 target,bool wake){if(id>=impl_->joints.size()||!impl_->joints[id].enabled||impl_->joints[id].type!=JointType::Mouse||!std::isfinite(target.x)||!std::isfinite(target.y))return false;auto&j=impl_->joints[id];j.target=target;if(wake)if(auto*b=impl_->Body(j.b))impl_->Wake(*b);return true;}
bool PhysicsWorld2D::SetJointBreakThreshold(JointID id,float force,float torque){if(id>=impl_->joints.size()||!impl_->joints[id].enabled||std::isnan(force)||std::isnan(torque)||force<0||torque<0)return false;impl_->joints[id].breakForce=force;impl_->joints[id].breakTorque=torque;return true;}
Vec2 PhysicsWorld2D::GetJointReactionForce(JointID id,float inverseDt)const{if(id>=impl_->joints.size()||!std::isfinite(inverseDt))return {};return impl_->joints[id].impulse*inverseDt;}
float PhysicsWorld2D::GetJointReactionTorque(JointID id,float inverseDt)const{if(id>=impl_->joints.size()||!std::isfinite(inverseDt))return 0;const auto&j=impl_->joints[id];return (j.angularImpulse+j.motorImpulse+j.limitImpulse)*inverseDt;}
const std::vector<JointBreakEvent2D>& PhysicsWorld2D::GetJointBreakEvents()const{return impl_->jointBreakEvents;}

bool PhysicsWorld2D::DestroyJoint(JointID id){if(id>=impl_->joints.size()||!impl_->joints[id].enabled)return false;impl_->joints[id].enabled=false;return true;}

std::vector<BodyID> PhysicsWorld2D::QueryAABB(const AABB2& box,const CollisionFilter& f)const{std::vector<BodyID> out;impl_->broadPhaseTree.Query(box,[&](uint32_t id){auto*b=impl_->Body(id);if(b&&b->state.enabled&&ShouldCollide(b->filter,f)&&ComputeAABB(b->shape,b->state.transform).Overlaps(box))out.push_back(id);return true;});std::sort(out.begin(),out.end());return out;}
BroadPhaseTreeStats2D PhysicsWorld2D::GetBroadPhaseStats()const{return impl_->broadPhaseTree.GetStats();}
bool PhysicsWorld2D::ValidateBroadPhase()const{return impl_->broadPhaseTree.Validate();}
std::vector<BodyID> PhysicsWorld2D::QueryPoint(Vec2 p,const CollisionFilter& f)const{std::vector<BodyID> out;for(BodyID id=0;id<impl_->bodies.size();++id){auto*b=impl_->Body(id);if(b&&b->state.enabled&&ShouldCollide(b->filter,f)&&TestPoint(b->shape,b->state.transform,p))out.push_back(id);}return out;}
std::vector<BodyID> PhysicsWorld2D::QueryShape(const Shape2D&shape,const Transform2D&transform,const CollisionFilter&f)const{std::vector<BodyID>out;if(!shape.IsValid())return out;AABB2 bounds=ComputeAABB(shape,transform);for(BodyID id=0;id<impl_->bodies.size();++id){auto*b=impl_->Body(id);if(!b||!b->state.enabled||!ShouldCollide(b->filter,f)||!bounds.Overlaps(ComputeAABB(b->shape,b->state.transform)))continue;Manifold2D manifold;if(Collide(shape,transform,b->shape,b->state.transform,manifold))out.push_back(id);}return out;}
std::vector<RayCastHit> PhysicsWorld2D::RayCastAll(Vec2 from,Vec2 to,const CollisionFilter& f,BodyID ignore)const{std::vector<RayCastHit> hits;impl_->broadPhaseTree.RayQuery(from,to,[&](uint32_t id){auto*b=impl_->Body(id);if(!b||!b->state.enabled||id==ignore||!ShouldCollide(b->filter,f))return true;float t;Vec2 n;if(RayShape(b->shape,b->state.transform,from,to,t,n))hits.push_back({id,from+(to-from)*t,n,t,b->state.userData});return true;});std::sort(hits.begin(),hits.end(),[](const RayCastHit&a,const RayCastHit&b){return a.fraction<b.fraction;});return hits;}
std::optional<RayCastHit> PhysicsWorld2D::RayCast(Vec2 from,Vec2 to,const CollisionFilter& f,BodyID ignore)const{auto hits=RayCastAll(from,to,f,ignore);if(hits.empty())return std::nullopt;return hits.front();}
std::optional<ShapeCastHit2D> PhysicsWorld2D::ShapeCast(const Shape2D& shape,const Transform2D& start,Vec2 translation,float rotationDelta,const CollisionFilter& f,BodyID ignore,float targetDistance)const {
    if(!shape.IsValid()||!std::isfinite(translation.x)||!std::isfinite(translation.y)||!std::isfinite(rotationDelta))return std::nullopt;
    AABB2 swept=ComputeShapeSweepBounds(shape,start,translation,rotationDelta,std::max(0.0f,targetDistance)+0.01f);
    std::optional<ShapeCastHit2D> best;
    ++impl_->profile.shapeCastCount;
    impl_->broadPhaseTree.Query(swept,[&](uint32_t id){
        const BodyInternal* body=impl_->Body(id);if(!body||!body->state.enabled||body->state.isSensor||id==ignore||!ShouldCollide(body->filter,f))return true;
        if(!swept.Overlaps(ComputeAABB(body->shape,body->state.transform)))return true;
        ShapeCastResult2D cast=CastShapeAgainstShape2D(shape,start,translation,rotationDelta,body->shape,body->state.transform,targetDistance,40);
        if(cast.hit&&(!best||cast.fraction<best->fraction))best=ShapeCastHit2D{id,cast.point,cast.normal,cast.fraction,cast.iterations,body->state.userData};
        return true;
    });
    return best;
}
TimeOfImpact2D PhysicsWorld2D::ComputeTimeOfImpact(BodyID a,BodyID b,float dt)const {
    TimeOfImpact2D out;const BodyInternal*A=impl_->Body(a),*B=impl_->Body(b);if(!A||!B||!A->state.enabled||!B->state.enabled||!std::isfinite(dt)||dt<=0)return out;
    ShapeCastResult2D cast=CastShapeAgainstShape2D(A->shape,A->state.transform,A->state.linearVelocity*dt,A->state.angularVelocity*dt,
        B->shape,B->state.transform,0.001f,48,B->state.linearVelocity*dt,B->state.angularVelocity*dt);
    out.hit=cast.hit;out.fraction=cast.fraction;out.point=cast.point;out.normal=cast.normal;out.iterations=cast.iterations;return out;
}
const std::vector<ContactEvent>& PhysicsWorld2D::GetBeginContactEvents()const{return impl_->beginEvents;}
const std::vector<ContactEvent>& PhysicsWorld2D::GetEndContactEvents()const{return impl_->endEvents;}
const std::vector<ContactEvent>& PhysicsWorld2D::GetActiveContactEvents()const{return impl_->activeContactEvents;}
std::vector<JointDebugInfo2D> PhysicsWorld2D::GetJointDebugInfo()const{
    std::vector<JointDebugInfo2D> out;out.reserve(impl_->joints.size());
    for(JointID id=0;id<impl_->joints.size();++id){const auto&j=impl_->joints[id];if(!j.enabled)continue;JointDebugInfo2D d;d.id=id;d.type=j.type;d.bodyA=j.a;d.bodyB=j.b;d.enabled=true;
        if(const auto*a=impl_->Body(j.a))d.anchorA=a->state.transform.Apply(j.anchorA);
        if(const auto*b=impl_->Body(j.b))d.anchorB=b->state.transform.Apply(j.anchorB);
        if(j.type==JointType::Mouse)d.anchorA=j.target;
        out.push_back(d);
    }return out;
}
void PhysicsWorld2D::SetContactListener(std::function<void(const ContactEvent&,bool)> l){impl_->listener=std::move(l);}
void PhysicsWorld2D::SetPreSolveFilter(std::function<bool(BodyID,BodyID,const Manifold2D&)> f){impl_->preSolve=std::move(f);}
float PhysicsWorld2D::GetInterpolationAlpha()const{return impl_->alpha;}
uint64_t PhysicsWorld2D::GetStepCount()const{return impl_->stepCount;}
std::vector<BodyID> PhysicsWorld2D::GetBodies()const{std::vector<BodyID> v;for(BodyID id=0;id<impl_->bodies.size();++id)if(impl_->Body(id))v.push_back(id);return v;}
PhysicsStats2D PhysicsWorld2D::GetStats()const{PhysicsStats2D s;s.stepCount=impl_->stepCount;s.broadPhaseCandidateCount=impl_->lastCandidateCount;s.ccdSubStepCount=impl_->lastCCDSubSteps;s.ccdImpactCount=impl_->lastCCDImpactCount;s.ccdEventOverflowCount=impl_->lastCCDOverflowCount;s.contactCount=uint32_t(impl_->contacts.size());s.broadPhaseTree=impl_->broadPhaseTree.GetStats();s.islandCount=impl_->lastIslandCount;for(const auto&b:impl_->bodies)if(b.state.id!=InvalidBodyID){++s.bodyCount;if(b.state.enabled)++s.enabledBodyCount;}return s;}
PhysicsProfile2D PhysicsWorld2D::GetProfile()const{return impl_->profile;}
void PhysicsWorld2D::ResetProfile(){impl_->profile={};}
WorldSnapshot2D PhysicsWorld2D::CaptureSnapshot()const{WorldSnapshot2D s;s.stepCount=impl_->stepCount;for(const auto&b:impl_->bodies)if(b.state.id!=InvalidBodyID)s.bodies.push_back({b.state.id,b.state.transform,b.state.linearVelocity,b.state.angularVelocity,b.state.force,b.state.torque,b.state.awake,b.state.enabled});return s;}
bool PhysicsWorld2D::RestoreSnapshot(const WorldSnapshot2D&s,bool exact){size_t active=0;for(const auto&b:impl_->bodies)if(b.state.id!=InvalidBodyID)++active;if(exact&&active!=s.bodies.size())return false;for(const auto&v:s.bodies){auto*b=impl_->Body(v.id);if(!b||!std::isfinite(v.transform.position.x)||!std::isfinite(v.transform.position.y)||!std::isfinite(v.transform.angle)||!std::isfinite(v.linearVelocity.x)||!std::isfinite(v.linearVelocity.y)||!std::isfinite(v.angularVelocity)||!std::isfinite(v.force.x)||!std::isfinite(v.force.y)||!std::isfinite(v.torque))return false;}if(exact){for(const auto&b:impl_->bodies)if(b.state.id!=InvalidBodyID&&std::none_of(s.bodies.begin(),s.bodies.end(),[&](const BodySnapshot2D&v){return v.id==b.state.id;}))return false;}for(const auto&v:s.bodies){auto*b=impl_->Body(v.id);b->state.transform=v.transform;b->state.worldCenter=impl_->WorldCenter(*b);impl_->SyncProxy(*b);b->previousPosition=v.transform.position;b->previousAngle=v.transform.angle;b->state.linearVelocity=v.linearVelocity;b->state.angularVelocity=v.angularVelocity;b->state.force=v.force;b->state.torque=v.torque;b->state.awake=v.awake;b->state.enabled=v.enabled;}impl_->stepCount=s.stepCount;impl_->accumulator=0;impl_->alpha=1;impl_->impulseCache.clear();impl_->previousPairs.clear();impl_->previousSensorPairs.clear();impl_->contacts.clear();impl_->beginEvents.clear();impl_->endEvents.clear();return true;}

WorldArchive2D PhysicsWorld2D::CaptureArchive() const {
    WorldArchive2D archive;archive.settings=impl_->settings;archive.stepCount=impl_->stepCount;
    archive.bodies.reserve(impl_->bodies.size());
    for(const auto& b:impl_->bodies){if(b.state.id==InvalidBodyID)continue;ArchivedBody2D item;item.sourceId=b.state.id;
        item.definition.type=b.state.type;item.definition.shape=b.shape;item.definition.position=b.state.transform.position;item.definition.angle=b.state.transform.angle;
        item.definition.linearVelocity=b.state.linearVelocity;item.definition.angularVelocity=b.state.angularVelocity;item.definition.gravityScale=b.gravityScale;
        item.definition.linearDamping=b.linearDamping;item.definition.angularDamping=b.angularDamping;item.definition.fixedRotation=b.fixedRotation;
        item.definition.bullet=b.state.bullet;item.definition.isSensor=b.state.isSensor;item.definition.allowSleep=b.allowSleep;item.definition.initiallyAwake=b.state.awake;
        item.definition.enabled=b.state.enabled;item.definition.oneWayPlatform=b.oneWayPlatform;item.definition.oneWayNormal=b.oneWayNormal;
        item.definition.filter=b.filter;item.definition.material=b.material;item.definition.userData=nullptr;item.definition.name=b.state.name;
        item.state={b.state.id,b.state.transform,b.state.linearVelocity,b.state.angularVelocity,b.state.force,b.state.torque,b.state.awake,b.state.enabled};
        archive.bodies.push_back(std::move(item));
    }
    archive.joints.reserve(impl_->joints.size());
    for(JointID id=0;id<impl_->joints.size();++id){const auto&j=impl_->joints[id];ArchivedJoint2D a;a.sourceId=id;a.type=j.type;a.bodyA=j.a;a.bodyB=j.b;a.anchorA=j.anchorA;a.anchorB=j.anchorB;a.axisA=j.axisA;
        a.length=j.length;a.minLength=j.minLength;a.maxLength=j.maxLength;a.frequency=j.frequency;a.damping=j.damping;a.enableLimit=j.enableLimit;a.enableMotor=j.enableMotor;
        a.lower=j.lower;a.upper=j.upper;a.motorSpeed=j.motorSpeed;a.maxMotor=j.maxMotor;a.referenceAngle=j.referenceAngle;a.target=j.target;a.maxForce=j.maxForce;
        a.groundAnchorA=j.groundAnchorA;a.groundAnchorB=j.groundAnchorB;a.linearOffset=j.linearOffset;a.ratio=j.ratio;a.constant=j.constant;
        a.angularOffset=j.angularOffset;a.correctionFactor=j.correctionFactor;a.maxTorque=j.maxTorque;a.breakForce=j.breakForce;a.breakTorque=j.breakTorque;
        a.collideConnected=j.collideConnected;a.enabled=j.enabled;archive.joints.push_back(a);
    }
    return archive;
}

bool PhysicsWorld2D::RestoreArchive(const WorldArchive2D& archive,std::vector<std::pair<BodyID,BodyID>>* bodyIdRemap,
                                    std::vector<std::pair<JointID,JointID>>* jointIdRemap) {
    if(archive.formatVersion!=1)return false;
    std::set<BodyID> sourceIds;
    for(const auto& b:archive.bodies){if(b.sourceId==InvalidBodyID||!sourceIds.insert(b.sourceId).second||!b.definition.shape.IsValid()||
        !std::isfinite(b.state.transform.position.x)||!std::isfinite(b.state.transform.position.y)||!std::isfinite(b.state.transform.angle)||
        !std::isfinite(b.state.linearVelocity.x)||!std::isfinite(b.state.linearVelocity.y)||!std::isfinite(b.state.angularVelocity))return false;}
    for(const auto& j:archive.joints){
        if(!j.enabled)continue;
        if(j.type==JointType::Mouse){if(!sourceIds.count(j.bodyB))return false;}
        else if(!sourceIds.count(j.bodyA)||!sourceIds.count(j.bodyB))return false;
    }
    if(bodyIdRemap) bodyIdRemap->clear();
    if(jointIdRemap) jointIdRemap->clear();
    // Clear old proxies and world-owned simulation state only after validation.
    impl_->bodies.clear();impl_->joints.clear();impl_->broadPhaseTree.Clear();impl_->impulseCache.clear();impl_->previousPairs.clear();impl_->previousSensorPairs.clear();
    impl_->contacts.clear();impl_->beginEvents.clear();impl_->endEvents.clear();impl_->activeContactEvents.clear();impl_->jointBreakEvents.clear();
    SetSettings(archive.settings);impl_->stepCount=0;impl_->accumulator=0;impl_->alpha=0;
    std::unordered_map<BodyID,BodyID> remap;
    for(const auto& item:archive.bodies){BodyDef def=item.definition;def.position=item.state.transform.position;def.angle=item.state.transform.angle;def.linearVelocity=item.state.linearVelocity;def.angularVelocity=item.state.angularVelocity;def.initiallyAwake=item.state.awake;def.enabled=item.state.enabled;def.userData=nullptr;
        BodyID created=CreateBody(def);if(created==InvalidBodyID)return false;remap[item.sourceId]=created;
        if(auto* state=GetBody(created)){state->force=item.state.force;state->torque=item.state.torque;state->awake=item.state.awake;state->enabled=item.state.enabled;impl_->SyncProxy(impl_->bodies[created]);}
        if(bodyIdRemap)bodyIdRemap->push_back({item.sourceId,created});
    }
    auto mapped=[&](BodyID source)->BodyID{auto it=remap.find(source);return it==remap.end()?InvalidBodyID:it->second;};
    for(const auto& a:archive.joints){
        if(!a.enabled){if(jointIdRemap)jointIdRemap->push_back({a.sourceId,InvalidJointID});continue;}
        JointID created=InvalidJointID;const BodyID A=mapped(a.bodyA),B=mapped(a.bodyB);
        switch(a.type){
        case JointType::Distance:{DistanceJointDef d;d.bodyA=A;d.bodyB=B;d.localAnchorA=a.anchorA;d.localAnchorB=a.anchorB;d.length=a.length;d.minLength=a.minLength;d.maxLength=a.maxLength;d.frequencyHz=a.frequency;d.dampingRatio=a.damping;d.collideConnected=a.collideConnected;created=CreateDistanceJoint(d);break;}
        case JointType::Revolute:{RevoluteJointDef d;d.bodyA=A;d.bodyB=B;d.localAnchorA=a.anchorA;d.localAnchorB=a.anchorB;d.enableLimit=a.enableLimit;d.lowerAngle=a.lower;d.upperAngle=a.upper;d.enableMotor=a.enableMotor;d.motorSpeed=a.motorSpeed;d.maxMotorTorque=a.maxMotor;d.collideConnected=a.collideConnected;created=CreateRevoluteJoint(d);break;}
        case JointType::Weld:{WeldJointDef d;d.bodyA=A;d.bodyB=B;d.localAnchorA=a.anchorA;d.localAnchorB=a.anchorB;d.referenceAngle=a.referenceAngle;d.frequencyHz=a.frequency;d.dampingRatio=a.damping;d.collideConnected=a.collideConnected;created=CreateWeldJoint(d);break;}
        case JointType::Prismatic:{PrismaticJointDef d;d.bodyA=A;d.bodyB=B;d.localAnchorA=a.anchorA;d.localAnchorB=a.anchorB;d.localAxisA=a.axisA;d.enableLimit=a.enableLimit;d.lowerTranslation=a.lower;d.upperTranslation=a.upper;d.enableMotor=a.enableMotor;d.motorSpeed=a.motorSpeed;d.maxMotorForce=a.maxMotor;d.collideConnected=a.collideConnected;created=CreatePrismaticJoint(d);break;}
        case JointType::Wheel:{WheelJointDef d;d.bodyA=A;d.bodyB=B;d.localAnchorA=a.anchorA;d.localAnchorB=a.anchorB;d.localAxisA=a.axisA;d.frequencyHz=a.frequency;d.dampingRatio=a.damping;d.enableMotor=a.enableMotor;d.motorSpeed=a.motorSpeed;d.maxMotorTorque=a.maxMotor;d.collideConnected=a.collideConnected;created=CreateWheelJoint(d);break;}
        case JointType::Rope:{RopeJointDef d;d.bodyA=A;d.bodyB=B;d.localAnchorA=a.anchorA;d.localAnchorB=a.anchorB;d.maxLength=a.maxLength;d.collideConnected=a.collideConnected;created=CreateRopeJoint(d);break;}
        case JointType::Mouse:{MouseJointDef d;d.body=B;d.localAnchor=a.anchorB;d.target=a.target;d.frequencyHz=a.frequency;d.dampingRatio=a.damping;d.maxForce=a.maxForce;created=CreateMouseJoint(d);break;}
        case JointType::Friction:{FrictionJointDef d;d.bodyA=A;d.bodyB=B;d.localAnchorA=a.anchorA;d.localAnchorB=a.anchorB;d.maxForce=a.maxForce;d.maxTorque=a.maxTorque;d.collideConnected=a.collideConnected;created=CreateFrictionJoint(d);break;}
        case JointType::Motor:{MotorJointDef d;d.bodyA=A;d.bodyB=B;d.linearOffset=a.linearOffset;d.angularOffset=a.angularOffset;d.maxForce=a.maxForce;d.maxTorque=a.maxTorque;d.correctionFactor=a.correctionFactor;d.collideConnected=a.collideConnected;created=CreateMotorJoint(d);break;}
        case JointType::Pulley:{PulleyJointDef d;d.bodyA=A;d.bodyB=B;d.groundAnchorA=a.groundAnchorA;d.groundAnchorB=a.groundAnchorB;d.localAnchorA=a.anchorA;d.localAnchorB=a.anchorB;d.ratio=a.ratio;d.constantLength=a.constant;d.collideConnected=a.collideConnected;created=CreatePulleyJoint(d);break;}
        case JointType::Gear:{GearJointDef d;d.bodyA=A;d.bodyB=B;d.ratio=a.ratio;d.constant=a.constant;d.motorSpeed=a.motorSpeed;d.maxMotorTorque=a.maxMotor;d.enableMotor=a.enableMotor;d.collideConnected=a.collideConnected;created=CreateGearJoint(d);break;}
        }
        if(created==InvalidJointID)return false;
        SetJointBreakThreshold(created,a.breakForce,a.breakTorque);
        if(jointIdRemap)jointIdRemap->push_back({a.sourceId,created});
    }
    impl_->stepCount=archive.stepCount;impl_->accumulator=0;impl_->alpha=1.0f;return true;
}

void Ragdoll2D::Destroy(PhysicsWorld2D& world){for(JointID j:joints)world.DestroyJoint(j);for(BodyID b:bones)world.DestroyBody(b);bones.clear();joints.clear();jointParentBone.clear();jointChildBone.clear();boneNames.clear();valid=false;}
Ragdoll2D CreateRagdoll2D(PhysicsWorld2D& world,const RagdollDef& def){
    Ragdoll2D r;if(def.bones.empty())return r;
    static std::atomic<uint32_t> nextRagdollGroup{1};
    const int16_t ragdollGroup=def.collideSelf?def.filter.groupIndex:static_cast<int16_t>(-1-int(nextRagdollGroup.fetch_add(1,std::memory_order_relaxed)%32767u));
    r.bones.resize(def.bones.size(),InvalidBodyID);r.boneNames.reserve(def.bones.size());for(const auto& bone:def.bones)r.boneNames.push_back(bone.name);
    for(size_t i=0;i<def.bones.size();++i){const auto& bd=def.bones[i];Transform2D xf{def.position,def.angle};if(bd.parent>=0){if(size_t(bd.parent)>=i||r.bones[size_t(bd.parent)]==InvalidBodyID){r.Destroy(world);return r;}auto* parent=world.GetBody(r.bones[size_t(bd.parent)]);xf.position=parent->transform.Apply(bd.localPosition);xf.angle=parent->transform.angle;}else{xf.position=def.position+Rot2(def.angle).Apply(bd.localPosition);}
        BodyDef body;body.type=BodyType::Dynamic;body.shape=bd.shape;body.position=xf.position;body.angle=xf.angle;body.material={def.friction,def.restitution,1.0f};body.filter=def.filter;if(!def.collideSelf)body.filter.groupIndex=ragdollGroup;body.name=bd.name;body.fixedRotation=false;
        MassData md=ComputeMass(body.shape,1.0f);if(md.mass>kEpsilon)body.material.density=std::max(0.001f,bd.mass/md.mass);
        BodyID id=world.CreateBody(body);if(id==InvalidBodyID){r.Destroy(world);return r;}r.bones[i]=id;
        if(bd.parent>=0){RevoluteJointDef j;j.bodyA=r.bones[size_t(bd.parent)];j.bodyB=id;j.localAnchorA=bd.parentAnchor;j.localAnchorB=bd.childAnchor;j.enableLimit=bd.enableLimit;j.lowerAngle=bd.lowerAngle;j.upperAngle=bd.upperAngle;JointID jid=world.CreateRevoluteJoint(j);if(jid==InvalidJointID){r.Destroy(world);return r;}r.joints.push_back(jid);r.jointParentBone.push_back(uint32_t(bd.parent));r.jointChildBone.push_back(uint32_t(i));}
    }
    r.valid=true;return r;
}
RagdollDef MakeHumanoidRagdoll(Vec2 p,float s){RagdollDef d;d.position=p;d.bones={
    {"pelvis",-1,Shape2D::Box({0.18f*s,0.20f*s}),4.0f,{0,0},{},{},-0.35f,0.35f,true},
    {"torso",0,Shape2D::Box({0.22f*s,0.30f*s}),5.0f,{0,0.45f*s},{0,0.18f*s},{0,-0.22f*s},-0.45f,0.45f,true},
    {"head",1,Shape2D::Circle(0.16f*s,{0,0.02f*s}),2.0f,{0,0.36f*s},{0,0.25f*s},{0,-0.11f*s},-0.55f,0.55f,true},
    {"upper_arm_l",1,Shape2D::Box({0.10f*s,0.23f*s}, {-0.04f*s,-0.20f*s},0.25f),1.5f,{-0.18f*s,-0.06f*s},{-0.18f*s,0.12f*s},{0,0.18f*s},-1.8f,1.1f,true},
    {"lower_arm_l",3,Shape2D::Box({0.085f*s,0.22f*s}),1.0f,{0,-0.35f*s},{0,-0.18f*s},{0,0.17f*s},-0.1f,2.4f,true},
    {"upper_arm_r",1,Shape2D::Box({0.10f*s,0.23f*s}, {0.04f*s,-0.20f*s},-0.25f),1.5f,{0.18f*s,-0.06f*s},{0.18f*s,0.12f*s},{0,0.18f*s},-1.1f,1.8f,true},
    {"lower_arm_r",5,Shape2D::Box({0.085f*s,0.22f*s}),1.0f,{0,-0.35f*s},{0,-0.18f*s},{0,0.17f*s},-2.4f,0.1f,true},
    {"upper_leg_l",0,Shape2D::Box({0.12f*s,0.27f*s}),3.0f,{-0.10f*s,-0.39f*s},{-0.10f*s,-0.16f*s},{0,0.23f*s},-1.2f,0.8f,true},
    {"lower_leg_l",7,Shape2D::Box({0.095f*s,0.26f*s}),2.0f,{0,-0.44f*s},{0,-0.22f*s},{0,0.22f*s},-2.5f,0.1f,true},
    {"upper_leg_r",0,Shape2D::Box({0.12f*s,0.27f*s}),3.0f,{0.10f*s,-0.39f*s},{0.10f*s,-0.16f*s},{0,0.23f*s},-0.8f,1.2f,true},
    {"lower_leg_r",9,Shape2D::Box({0.095f*s,0.26f*s}),2.0f,{0,-0.44f*s},{0,-0.22f*s},{0,0.22f*s},-0.1f,2.5f,true}
};return d;}

void DriveRagdollPose(PhysicsWorld2D& world,const Ragdoll2D& rag,const std::vector<float>& targets,float stiffness,float damping,float maxTorque) {
    ActiveRagdollSettings2D settings;settings.stiffness=stiffness;settings.damping=damping;settings.maxTorque=maxTorque;
    DriveRagdollPoseAdvanced(world,rag,targets,settings);
}

void DriveRagdollPoseAdvanced(PhysicsWorld2D& world,const Ragdoll2D& rag,const std::vector<float>& targets,
                              const ActiveRagdollSettings2D& input) {
    if(!rag.valid||targets.size()!=rag.joints.size()||rag.jointParentBone.size()!=rag.joints.size()||rag.jointChildBone.size()!=rag.joints.size())return;
    const float stiffness=std::max(0.0f,std::isfinite(input.stiffness)?input.stiffness:0.0f);
    const float damping=std::max(0.0f,std::isfinite(input.damping)?input.damping:0.0f);
    const float maxTorque=std::max(0.0f,std::isfinite(input.maxTorque)?input.maxTorque:0.0f);
    const float maxSpeed=std::max(0.0f,std::isfinite(input.maxAngularSpeed)?input.maxAngularSpeed:0.0f);
    const float blend=Clamp(std::isfinite(input.blend)?input.blend:0.0f,0.0f,1.0f);
    for(size_t i=0;i<rag.joints.size();++i){
        const BodyState* parent=world.GetBody(rag.bones[rag.jointParentBone[i]]);
        const BodyState* child=world.GetBody(rag.bones[rag.jointChildBone[i]]);
        if(!parent||!child||!std::isfinite(targets[i]))continue;
        const float error=WrapAngle((child->transform.angle-parent->transform.angle)-targets[i]);
        const float relative=child->angularVelocity-parent->angularVelocity;
        const float torqueScale=i<input.perJointTorqueScale.size()&&std::isfinite(input.perJointTorqueScale[i])
            ?Clamp(input.perJointTorqueScale[i],0.0f,4.0f):1.0f;
        const float targetSpeed=Clamp((-stiffness*error-damping*relative)*blend,-maxSpeed,maxSpeed);
        world.SetJointMotor(rag.joints[i],targetSpeed,maxTorque*torqueScale,true);
    }
}

void ApplyRagdollPoseToSkeleton(const PhysicsWorld2D& world,const Ragdoll2D& rag,Skeleton2D& skeleton,float blend) {
    if(!rag.valid||rag.bones.size()!=rag.boneNames.size())return;
    blend=Clamp(blend,0.0f,1.0f);
    std::vector<int> ragIndexBySkeletonBone(skeleton.GetBoneCount(),-1);
    for(size_t i=0;i<rag.boneNames.size();++i) {
        int index=skeleton.FindBone(rag.boneNames[i]);
        if(index>=0) ragIndexBySkeletonBone[size_t(index)]=int(i);
    }
    // Skeleton2D guarantees parent-before-child ordering, so each parent's
    // world transform is current before its child is converted back to local.
    for(size_t boneIndex=0;boneIndex<skeleton.GetBoneCount();++boneIndex) {
        int ragIndex=ragIndexBySkeletonBone[boneIndex]; if(ragIndex<0)continue;
        const BodyState* body=world.GetBody(rag.bones[size_t(ragIndex)]);
        SkeletonBone2D* bone=skeleton.GetBone(boneIndex); if(!body||!bone)continue;
        Transform2D target;
        if(bone->parent>=0) {
            const SkeletonBone2D* parent=skeleton.GetBone(size_t(bone->parent)); if(!parent)continue;
            target.position=parent->world.ApplyInverse(body->transform.position);
            target.angle=WrapAngle(body->transform.angle-parent->world.angle);
        } else target=body->transform;
        Transform2D local=bone->local;
        local.position=local.position*(1.0f-blend)+target.position*blend;
        local.angle=WrapAngle(local.angle+WrapAngle(target.angle-local.angle)*blend);
        skeleton.SetLocalTransform(boneIndex,local);
        skeleton.UpdateWorldTransforms();
    }
}

PlatformerController2D::PlatformerController2D(PhysicsWorld2D&w,BodyID b,PlatformerControllerSettings s):world_(&w),body_(b),settings_(s){}
void PlatformerController2D::Reset(){grounded_=false;wasGrounded_=false;groundNormal_={0,1};coyoteTimer_=0;jumpBufferTimer_=0;jumpHeld_=false;}
void PlatformerController2D::Update(float input,bool jump,float dt){
    if(!world_||!std::isfinite(dt)||dt<=0) return;
    dt=std::min(dt,0.1f); BodyState*b=world_->GetBody(body_); if(!b||b->type!=BodyType::Dynamic) return;
    input=Clamp(input,-1.0f,1.0f);if(jump)jumpBufferTimer_=std::max(0.0f,settings_.jumpBufferTime);else jumpBufferTimer_=std::max(0.0f,jumpBufferTimer_-dt);
    AABB2 box=world_->GetBodyAABB(body_);Vec2 start{box.Center().x,box.min.y+0.015f};float probe=std::max(settings_.groundProbe,settings_.groundSnapDistance+0.02f);auto hit=world_->RayCast(start,start+Vec2{0,-probe},{},body_);grounded_=false;
    if(hit&&Dot(hit->normal,Vec2{0,1})>=settings_.groundNormalThreshold&&b->linearVelocity.y<=0.5f){grounded_=true;groundNormal_=hit->normal;}
    if(grounded_)coyoteTimer_=std::max(0.0f,settings_.coyoteTime);else coyoteTimer_=std::max(0.0f,coyoteTimer_-dt);
    Vec2 v=b->linearVelocity;float target=input*settings_.moveSpeed;float acceleration=std::abs(input)>0.001f?(grounded_?settings_.groundAcceleration:settings_.airAcceleration*settings_.airControl):(grounded_?settings_.groundDeceleration:settings_.airAcceleration*0.35f);float maxDelta=std::max(0.0f,acceleration)*dt;v.x+=Clamp(target-v.x,-maxDelta,maxDelta);
    if(jumpBufferTimer_>0&&coyoteTimer_>0){v.y=settings_.jumpSpeed;jumpBufferTimer_=0;coyoteTimer_=0;grounded_=false;jumpCutApplied_=false;}
    else if(!jumpHeld_&&!jumpCutApplied_&&v.y>0.0f){v.y*=Clamp(settings_.jumpCutMultiplier,0.0f,1.0f);jumpCutApplied_=true;}
    v.y=std::max(v.y,-std::max(0.0f,settings_.maxFallSpeed));world_->SetLinearVelocity(body_,v,true);wasGrounded_=grounded_;
}

void DrawDebug(const PhysicsWorld2D& world,const DebugDraw2D& draw,const PhysicsDebugOptions2D& options){
    for(BodyID id:world.GetBodies()){
        const BodyState*b=world.GetBody(id);const Shape2D*shape=world.GetBodyShape(id);if(!b||!shape)continue;
        const uint32_t color=b->type==BodyType::Static?0xff8a8a8au:(b->type==BodyType::Kinematic?0xff6da9d8u:0xff72c78au);
        std::function<void(const Shape2D&,const Transform2D&)> render=[&](const Shape2D&sh,const Transform2D&xf){
            if(sh.type==ShapeType::Compound&&sh.compound){for(const auto&child:sh.compound->children)render(child.shape,{xf.Apply(child.transform.position),WrapAngle(xf.angle+child.transform.angle)});return;}
            if(sh.type==ShapeType::Circle&&draw.circle)draw.circle(xf.Apply(sh.circle.center),sh.circle.radius,color);
            else if(sh.type==ShapeType::Polygon&&draw.polygon){std::vector<Vec2>v;for(Vec2 p:sh.polygon.vertices)v.push_back(xf.Apply(p));draw.polygon(v,color);}
            else if(sh.type==ShapeType::Edge&&draw.line)draw.line(xf.Apply(sh.edge.a),xf.Apply(sh.edge.b),color);
            else if(sh.type==ShapeType::Chain&&draw.line){int n=int(sh.chain.vertices.size())-1+(sh.chain.loop?1:0);for(int i=0;i<n;++i)draw.line(xf.Apply(sh.chain.vertices[size_t(i)]),xf.Apply(sh.chain.vertices[size_t((i+1)%sh.chain.vertices.size())]),color);}
        };render(*shape,b->transform);
        if(options.drawAABBs&&draw.polygon){AABB2 a=world.GetBodyAABB(id);draw.polygon({a.min,{a.max.x,a.min.y},a.max,{a.min.x,a.max.y}},0x668888ffu);}
        if(options.drawFatAABBs&&draw.polygon){AABB2 a=world.GetBroadPhaseFatAABB(id);if(a.IsValid())draw.polygon({a.min,{a.max.x,a.min.y},a.max,{a.min.x,a.max.y}},0x4466ddffu);}
        if(options.drawCentersOfMass&&draw.circle)draw.circle(b->worldCenter,0.035f,0xffffcc33u);
        if(options.drawVelocities&&draw.line)draw.line(b->worldCenter,b->worldCenter+b->linearVelocity*options.velocityScale,0xff33ccffu);
        if(options.drawBodyNames&&draw.text&&!b->name.empty())draw.text(b->transform.position,b->name,0xffffffffu);
    }
    if(options.drawContacts&&draw.circle&&draw.line){for(const auto&e:world.GetActiveContactEvents())for(const auto&m:e.manifolds)if(!m.isSensor)for(uint8_t i=0;i<m.pointCount;++i){Vec2 p=m.points[i].point;draw.circle(p,0.025f,0xffff5533u);draw.line(p,p+m.normal*options.contactNormalScale,0xffffff33u);}}
    if(options.drawJoints&&draw.line){for(const auto&j:world.GetJointDebugInfo()){draw.line(j.anchorA,j.anchorB,0xffffaa33u);if(draw.circle){draw.circle(j.anchorA,0.04f,0xffffaa33u);draw.circle(j.anchorB,0.04f,0xffffaa33u);}}}
}
void DrawDebug(const PhysicsWorld2D& world,const DebugDraw2D& draw,bool drawAABBs){PhysicsDebugOptions2D options;options.drawAABBs=drawAABBs;DrawDebug(world,draw,options);}

std::vector<Shape2D> BuildTilemapEdges(const std::vector<uint8_t>& solid,int width,int height,float cell,Vec2 origin,bool merge) {
    std::vector<Shape2D> out;
    if(width<=0||height<=0||!std::isfinite(cell)||cell<=0||int64_t(width)*int64_t(height)>int64_t(solid.size()))return out;
    struct GridPoint { int x=0,y=0; bool operator==(const GridPoint&b)const{return x==b.x&&y==b.y;} };
    struct GridEdge { GridPoint a,b; bool used=false; };
    auto key=[](GridPoint p){return (uint64_t(uint32_t(p.x))<<32)|uint32_t(p.y);};
    auto isSolid=[&](int x,int y){return x>=0&&x<width&&y>=0&&y<height&&solid[size_t(y*width+x)]!=0;};
    std::vector<GridEdge> edges;
    edges.reserve(size_t(width)*size_t(height));
    for(int y=0;y<height;++y)for(int x=0;x<width;++x)if(isSolid(x,y)){
        if(!isSolid(x,y-1))edges.push_back({{x,y},{x+1,y},false});
        if(!isSolid(x+1,y))edges.push_back({{x+1,y},{x+1,y+1},false});
        if(!isSolid(x,y+1))edges.push_back({{x+1,y+1},{x,y+1},false});
        if(!isSolid(x-1,y))edges.push_back({{x,y+1},{x,y},false});
    }
    if(edges.empty())return out;
    std::unordered_map<uint64_t,std::vector<size_t>> outgoing;
    for(size_t i=0;i<edges.size();++i)outgoing[key(edges[i].a)].push_back(i);
    auto toWorld=[&](GridPoint p){return origin+Vec2{float(p.x)*cell,float(p.y)*cell};};
    auto chooseNext=[&](size_t current)->size_t{
        const auto& e=edges[current];auto it=outgoing.find(key(e.b));if(it==outgoing.end())return edges.size();
        const Vec2 incoming{float(e.b.x-e.a.x),float(e.b.y-e.a.y)};
        size_t best=edges.size();float bestTurn=-100.0f;
        for(size_t candidate:it->second){if(edges[candidate].used)continue;
            const auto& next=edges[candidate];Vec2 d{float(next.b.x-next.a.x),float(next.b.y-next.a.y)};
            float angle=std::atan2(Cross(incoming,d),Dot(incoming,d));
            // At a diagonal-touch vertex choose the left-most continuation.
            // This keeps touching tile islands as separate contours instead of
            // splicing their boundary loops together.
            if(angle>bestTurn){bestTurn=angle;best=candidate;}
        }
        return best;
    };
    std::vector<std::vector<GridPoint>> contours;
    for(size_t seed=0;seed<edges.size();++seed){
        if(edges[seed].used)continue;
        std::vector<GridPoint> contour;size_t current=seed;const GridPoint first=edges[seed].a;
        for(size_t guard=0;guard<=edges.size();++guard){
            if(edges[current].used)break;
            edges[current].used=true;contour.push_back(edges[current].a);
            if(edges[current].b==first)break;
            size_t next=chooseNext(current);if(next==edges.size()) {contour.push_back(edges[current].b);break;}
            current=next;
        }
        if(contour.size()>=2)contours.push_back(std::move(contour));
    }
    if(merge){
        out.reserve(contours.size());
        for(auto& contour:contours){
            // Remove collinear intermediate vertices while preserving winding
            // and every convex/concave corner used by ghost-edge filtering.
            bool changed=true;
            while(changed&&contour.size()>3){changed=false;
                for(size_t i=0;i<contour.size();++i){
                    const auto prev=contour[(i+contour.size()-1)%contour.size()],cur=contour[i],next=contour[(i+1)%contour.size()];
                    Vec2 a{float(cur.x-prev.x),float(cur.y-prev.y)},b{float(next.x-cur.x),float(next.y-cur.y)};
                    if(std::abs(Cross(a,b))<1.0e-6f&&Dot(a,b)>0.0f){contour.erase(contour.begin()+std::ptrdiff_t(i));changed=true;break;}
                }
            }
            std::vector<Vec2> points;points.reserve(contour.size());for(auto p:contour)points.push_back(toWorld(p));
            if(points.size()>=3)out.push_back(Shape2D::Chain(points,true,true));
            else if(points.size()==2)out.push_back(Shape2D::Edge(points[0],points[1],true));
        }
    } else {
        // Expose one edge per boundary segment but retain both adjacent ghost
        // vertices so downstream narrow-phase tests can suppress seam bumps.
        std::vector<EdgeShape> segmentData;segmentData.reserve(edges.size());
        std::unordered_map<uint64_t,std::vector<size_t>> incoming;
        for(size_t i=0;i<edges.size();++i){
            EdgeShape edge;edge.a=toWorld(edges[i].a);edge.b=toWorld(edges[i].b);edge.oneSided=true;segmentData.push_back(edge);
            incoming[key(edges[i].b)].push_back(i);
        }
        for(size_t i=0;i<edges.size();++i){
            // A boundary vertex can have multiple incident contours when tiles
            // touch diagonally. Select the continuation with the same left-most
            // turn rule used by contour tracing, without scanning every edge.
            auto prevIt=incoming.find(key(edges[i].a));
            if(prevIt!=incoming.end()&&!prevIt->second.empty()){
                size_t best=prevIt->second.front();float bestTurn=-100.0f;
                const Vec2 incomingDir{float(edges[i].b.x-edges[i].a.x),float(edges[i].b.y-edges[i].a.y)};
                for(size_t candidate:prevIt->second){
                    Vec2 d{float(edges[candidate].b.x-edges[candidate].a.x),float(edges[candidate].b.y-edges[candidate].a.y)};
                    const float turn=std::atan2(Cross(d,incomingDir),Dot(d,incomingDir));
                    if(turn>bestTurn){bestTurn=turn;best=candidate;}
                }
                segmentData[i].ghostA=toWorld(edges[best].a);segmentData[i].hasGhostA=true;
            }
            auto nextIt=outgoing.find(key(edges[i].b));
            if(nextIt!=outgoing.end()&&!nextIt->second.empty()){
                size_t best=nextIt->second.front();float bestTurn=-100.0f;
                const Vec2 incomingDir{float(edges[i].b.x-edges[i].a.x),float(edges[i].b.y-edges[i].a.y)};
                for(size_t candidate:nextIt->second){
                    Vec2 d{float(edges[candidate].b.x-edges[candidate].a.x),float(edges[candidate].b.y-edges[candidate].a.y)};
                    const float turn=std::atan2(Cross(incomingDir,d),Dot(incomingDir,d));
                    if(turn>bestTurn){bestTurn=turn;best=candidate;}
                }
                segmentData[i].ghostB=toWorld(edges[best].b);segmentData[i].hasGhostB=true;
            }
            Shape2D edge=Shape2D::Edge(segmentData[i].a,segmentData[i].b,true);edge.edge=segmentData[i];out.push_back(std::move(edge));
        }
    }
    return out;
}

KinematicCharacterController2D::KinematicCharacterController2D(PhysicsWorld2D& world,BodyID body,KinematicCharacterSettings2D settings)
    :world_(&world),body_(body),settings_(settings) {}

bool KinematicCharacterController2D::TryStep(Vec2 remaining,Vec2& position,CharacterMoveResult2D& result) {
    if(!settings_.enableStepClimbing||settings_.stepHeight<=0.0f||remaining.LengthSquared()<kEpsilon*kEpsilon)return false;
    const BodyState* body=world_?world_->GetBody(body_):nullptr;const Shape2D* shape=world_?world_->GetBodyShape(body_):nullptr;if(!body||!shape)return false;
    const Vec2 up=NormalizeOr(settings_.upDirection,{0,1});Transform2D xf=body->transform;xf.position=position;
    auto upHit=world_->ShapeCast(*shape,xf,up*settings_.stepHeight,0.0f,{},body_,settings_.skinWidth);
    if(upHit&&upHit->fraction<0.999f)return false;
    position+=up*settings_.stepHeight;xf.position=position;
    auto forwardHit=world_->ShapeCast(*shape,xf,remaining,0.0f,{},body_,settings_.skinWidth);
    float moved=forwardHit?std::max(0.0f,forwardHit->fraction-settings_.skinWidth/std::max(remaining.Length(),kEpsilon)):1.0f;
    if(moved<=0.05f)return false;
    position+=remaining*moved;xf.position=position;
    const float downDistance=settings_.stepHeight+settings_.snapDistance;
    auto downHit=world_->ShapeCast(*shape,xf,-up*downDistance,0.0f,{},body_,settings_.skinWidth);
    if(!downHit)return false;
    Vec2 normal=-downHit->normal;
    if(Dot(normal,up)<std::cos(settings_.maxSlopeAngle))return false;
    float downFraction=std::max(0.0f,downHit->fraction-settings_.skinWidth/std::max(downDistance,kEpsilon));
    position-=up*(downDistance*downFraction);result.grounded=true;result.groundNormal=normal;result.groundBody=downHit->body;result.stepped=true;result.sweepCount+=3;return true;
}

CharacterMoveResult2D KinematicCharacterController2D::Move(Vec2 displacement) {
    CharacterMoveResult2D result;const BodyState* body=world_?world_->GetBody(body_):nullptr;const Shape2D* shape=world_?world_->GetBodyShape(body_):nullptr;
    if(!body||!shape||!std::isfinite(displacement.x)||!std::isfinite(displacement.y)){lastResult_=result;return result;}
    const Vec2 start=body->transform.position;Vec2 position=start;const Vec2 up=NormalizeOr(settings_.upDirection,{0,1});
    Vec2 carry{};
    if(settings_.carryMovingPlatforms&&hasGroundAnchor_&&lastGroundBody_!=InvalidBodyID){
        if(const BodyState* platform=world_->GetBody(lastGroundBody_);platform&&platform->enabled){
            const Vec2 previousPlayerPosition=lastGroundTransform_.Apply(lastGroundLocalAnchor_);
            const Vec2 currentPlayerPosition=platform->transform.Apply(lastGroundLocalAnchor_);
            carry=(currentPlayerPosition-previousPlayerPosition)*Clamp(settings_.platformCarryScale,0.0f,2.0f);
        }else hasGroundAnchor_=false;
    }
    Vec2 remaining=carry+displacement;
    const int iterations=std::max(1,settings_.maxSlideIterations);
    for(int i=0;i<iterations&&remaining.LengthSquared()>1.0e-10f;++i){
        Transform2D xf=body->transform;xf.position=position;
        auto hit=world_->ShapeCast(*shape,xf,remaining,0.0f,{},body_,settings_.skinWidth);++result.sweepCount;
        if(!hit){position+=remaining;remaining={};break;}
        const float length=remaining.Length();const float safeFraction=Clamp(hit->fraction-settings_.skinWidth/std::max(length,kEpsilon),0.0f,1.0f);
        position+=remaining*safeFraction;remaining*=1.0f-safeFraction;
        Vec2 surfaceNormal=-NormalizeOr(hit->normal,up);
        if(Dot(surfaceNormal,up)>=std::cos(settings_.maxSlopeAngle)){
            result.grounded=true;result.groundNormal=surfaceNormal;result.groundBody=hit->body;
        } else if(Dot(surfaceNormal,up)<-0.5f)result.hitCeiling=true;
        else result.hitWall=true;
        if(result.hitWall&&TryStep(remaining,position,result)){remaining={};break;}
        const float into=Dot(remaining,surfaceNormal);
        if(into<0.0f)remaining-=surfaceNormal*into;
        else if(into>0.0f&&result.hitWall)remaining-=surfaceNormal*into;
    }
    if(settings_.enableGroundSnap&&!result.grounded&&Dot(displacement,up)<=0.0f&&settings_.snapDistance>0.0f){
        Transform2D xf=body->transform;xf.position=position;
        auto snap=world_->ShapeCast(*shape,xf,-up*settings_.snapDistance,0.0f,{},body_,settings_.skinWidth);++result.sweepCount;
        if(snap){Vec2 n=-snap->normal;if(Dot(n,up)>=std::cos(settings_.maxSlopeAngle)){float f=std::max(0.0f,snap->fraction-settings_.skinWidth/std::max(settings_.snapDistance,kEpsilon));position-=up*(settings_.snapDistance*f);result.grounded=true;result.groundNormal=n;result.groundBody=snap->body;}}
    }
    world_->SetTransform(body_,position,body->transform.angle,true);
    if(result.grounded&&result.groundBody!=InvalidBodyID){
        if(const BodyState* platform=world_->GetBody(result.groundBody)){lastGroundBody_=result.groundBody;lastGroundLocalAnchor_=platform->transform.ApplyInverse(position);lastGroundTransform_=platform->transform;hasGroundAnchor_=true;}
    }else{lastGroundBody_=InvalidBodyID;hasGroundAnchor_=false;}
    result.position=position;result.actualDisplacement=position-start;lastResult_=result;return result;
}
CharacterMoveResult2D KinematicCharacterController2D::MoveAndSlide(Vec2 desiredVelocity,float dt) {
    if(!std::isfinite(dt)||dt<=0.0f)return lastResult_;
    velocity_=desiredVelocity;CharacterMoveResult2D result=Move(velocity_*dt);velocity_=result.actualDisplacement/dt;
    const Vec2 up=NormalizeOr(settings_.upDirection,{0,1});
    const float vertical=Dot(velocity_,up);if(result.grounded&&vertical<0.0f)velocity_-=up*vertical;
    lastResult_=result;return result;
}

bool PhysicsWorld2D::SetBodyType(BodyID id, BodyType type) {
    auto* b = impl_->Body(id);
    if (!b) return false;
    if (b->state.type == type) return true;
    if (type == BodyType::Dynamic) {
        const float m = b->massOverride ? b->massOverrideData.mass
                                        : ComputeMass(b->shape, b->material.density).mass;
        if (m <= kEpsilon) return false;
    }
    b->state.type = type;
    impl_->RecomputeMass(*b, b->material.density);   // zeroes mass/inertia for non-dynamic
    b->state.force = {};
    b->state.torque = 0.0f;
    if (type == BodyType::Static) { b->state.linearVelocity = {}; b->state.angularVelocity = 0.0f; }
    b->state.awake = true;
    b->sleepTime = 0.0f;
    impl_->SyncProxy(*b);                            // moves the proxy to the Static/Moving layer
    // Whatever was resting on / near this body needs to re-evaluate.
    if (b->broadPhaseProxy != InvalidBroadPhaseProxy2D) {
        for (uint32_t other : impl_->broadPhaseTree.Query(impl_->broadPhaseTree.GetFatBounds(b->broadPhaseProxy)))
            if (auto* o = impl_->Body(other)) impl_->Wake(*o);
    }
    return true;
}

bool PhysicsWorld2D::SetBodyAwake(BodyID id, bool awake) {
    auto* b = impl_->Body(id);
    if (!b || b->state.type != BodyType::Dynamic) return false;
    if (awake) { impl_->Wake(*b); return true; }
    b->state.awake = false;
    b->state.linearVelocity = {};
    b->state.angularVelocity = 0.0f;
    b->state.force = {};
    b->state.torque = 0.0f;
    b->sleepTime = 0.0f;
    return true;
}

bool PhysicsWorld2D::SetBodyBullet(BodyID id, bool bullet) {
    auto* b = impl_->Body(id);
    if (!b) return false;
    b->state.bullet = bullet;
    return true;
}

bool PhysicsWorld2D::SetBodyFixedRotation(BodyID id, bool fixedRotation) {
    auto* b = impl_->Body(id);
    if (!b) return false;
    b->fixedRotation = fixedRotation;
    if (fixedRotation) b->state.angularVelocity = 0.0f;
    impl_->RecomputeMass(*b, b->material.density);
    impl_->Wake(*b);
    return true;
}

bool PhysicsWorld2D::SetBodySleepingAllowed(BodyID id, bool allow) {
    auto* b = impl_->Body(id);
    if (!b) return false;
    b->allowSleep = allow;
    if (!allow) impl_->Wake(*b);
    return true;
}

bool PhysicsWorld2D::SetBodyMassData(BodyID id, const MassData& mass) {
    auto* b = impl_->Body(id);
    if (!b || b->state.type != BodyType::Dynamic) return false;
    if (!std::isfinite(mass.mass) || !std::isfinite(mass.inertia) ||
        !std::isfinite(mass.center.x) || !std::isfinite(mass.center.y) ||
        mass.mass <= kEpsilon || mass.inertia < 0.0f) return false;
    const Vec2 oldCenter = b->state.worldCenter;
    b->massOverride = true;
    b->massOverrideData = mass;
    impl_->RecomputeMass(*b, b->material.density);
    // Keep the body's motion consistent when the centre of mass moves (same as Box2D).
    b->state.linearVelocity += Cross(b->state.angularVelocity, b->state.worldCenter - oldCenter);
    impl_->Wake(*b);
    return true;
}

bool PhysicsWorld2D::ResetBodyMassData(BodyID id) {
    auto* b = impl_->Body(id);
    if (!b) return false;
    const Vec2 oldCenter = b->state.worldCenter;
    b->massOverride = false;
    b->massOverrideData = {};
    impl_->RecomputeMass(*b, b->material.density);
    b->state.linearVelocity += Cross(b->state.angularVelocity, b->state.worldCenter - oldCenter);
    impl_->Wake(*b);
    return true;
}

// ---------------- Joint state ----------------

bool PhysicsWorld2D::IsJointValid(JointID id) const { return impl_->LiveJoint(id) != nullptr; }

std::optional<JointType> PhysicsWorld2D::GetJointType(JointID id) const {
    const auto* j = impl_->LiveJoint(id);
    if (!j) return std::nullopt;
    return j->type;
}

std::optional<float> PhysicsWorld2D::GetJointAngle(JointID id) const {
    const auto* j = impl_->LiveJoint(id);
    if (!j || (j->type != JointType::Revolute && j->type != JointType::Weld)) return std::nullopt;
    const auto* A = impl_->Body(j->a);
    const auto* B = impl_->Body(j->b);
    // Same expression the solver uses for limits.
    return WrapAngle(B->state.transform.angle - A->state.transform.angle - j->referenceAngle);
}

std::optional<float> PhysicsWorld2D::GetJointAngularSpeed(JointID id) const {
    const auto* j = impl_->LiveJoint(id);
    if (!j || (j->type != JointType::Revolute && j->type != JointType::Wheel)) return std::nullopt;
    const auto* A = impl_->Body(j->a);
    const auto* B = impl_->Body(j->b);
    return B->state.angularVelocity - A->state.angularVelocity;
}

std::optional<float> PhysicsWorld2D::GetJointTranslation(JointID id) const {
    const auto* j = impl_->LiveJoint(id);
    if (!j || (j->type != JointType::Prismatic && j->type != JointType::Wheel)) return std::nullopt;
    const auto* A = impl_->Body(j->a);
    const auto* B = impl_->Body(j->b);
    const Vec2 pA = A->state.transform.position + WorldVector(*A, j->anchorA);
    const Vec2 pB = B->state.transform.position + WorldVector(*B, j->anchorB);
    const Vec2 axis = NormalizeOr(WorldVector(*A, j->axisA));
    return Dot(pB - pA, axis);
}

std::optional<float> PhysicsWorld2D::GetJointLinearSpeed(JointID id) const {
    const auto* j = impl_->LiveJoint(id);
    if (!j || (j->type != JointType::Prismatic && j->type != JointType::Wheel)) return std::nullopt;
    const auto* A = impl_->Body(j->a);
    const auto* B = impl_->Body(j->b);
    const Vec2 pA = A->state.transform.position + WorldVector(*A, j->anchorA);
    const Vec2 pB = B->state.transform.position + WorldVector(*B, j->anchorB);
    const Vec2 rA = pA - impl_->WorldCenter(*A), rB = pB - impl_->WorldCenter(*B);
    const Vec2 vA = A->state.linearVelocity + Cross(A->state.angularVelocity, rA);
    const Vec2 vB = B->state.linearVelocity + Cross(B->state.angularVelocity, rB);
    const Vec2 axis = NormalizeOr(WorldVector(*A, j->axisA));
    return Dot(pB - pA, Cross(A->state.angularVelocity, axis)) + Dot(axis, vB - vA);
}

bool PhysicsWorld2D::SetJointLimits(JointID id, bool enable, float lower, float upper) {
    auto* j = impl_->LiveJoint(id);
    if (!j || (j->type != JointType::Revolute && j->type != JointType::Prismatic)) return false;
    if (!std::isfinite(lower) || !std::isfinite(upper) || lower > upper) return false;
    j->enableLimit = enable;
    j->lower = lower;
    j->upper = upper;
    if (auto* a = impl_->Body(j->a)) impl_->Wake(*a);
    if (auto* b = impl_->Body(j->b)) impl_->Wake(*b);
    return true;
}

bool PhysicsWorld2D::SetJointSpring(JointID id, float frequencyHz, float dampingRatio) {
    auto* j = impl_->LiveJoint(id);
    if (!j) return false;
    if (j->type != JointType::Distance && j->type != JointType::Weld &&
        j->type != JointType::Wheel && j->type != JointType::Mouse) return false;
    if (!std::isfinite(frequencyHz) || !std::isfinite(dampingRatio) || frequencyHz < 0.0f || dampingRatio < 0.0f) return false;
    j->frequency = frequencyHz;
    j->damping = dampingRatio;
    if (auto* a = impl_->Body(j->a)) impl_->Wake(*a);
    if (auto* b = impl_->Body(j->b)) impl_->Wake(*b);
    return true;
}

bool PhysicsWorld2D::SetJointEnabled(JointID id, bool enabled) {
    auto* j = impl_->LiveJoint(id);          // refuses destroyed joints and joints whose bodies were destroyed
    if (!j) return false;
    j->enabled = enabled;
    if (auto* a = impl_->Body(j->a)) impl_->Wake(*a);
    if (auto* b = impl_->Body(j->b)) impl_->Wake(*b);
    return true;
}

bool PhysicsWorld2D::SetDistanceJointLength(JointID id, float length, float minLength, float maxLength) {
    auto* j = impl_->LiveJoint(id);
    if (!j || j->type != JointType::Distance) return false;
    if (!std::isfinite(length) || length <= kEpsilon || !std::isfinite(minLength) || !std::isfinite(maxLength)) return false;
    j->length = length;
    j->minLength = minLength;                // negative = unset, same as DistanceJointDef
    j->maxLength = maxLength;
    if (auto* a = impl_->Body(j->a)) impl_->Wake(*a);
    if (auto* b = impl_->Body(j->b)) impl_->Wake(*b);
    return true;
}
