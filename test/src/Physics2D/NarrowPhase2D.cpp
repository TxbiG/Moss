#include <Moss/Physics/Physics2D/NarrowPhase2D.hpp>
#include <Moss/Physics/Physics2D/BatchMath2D.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

struct ConvexProxy { Shape2D shape; Transform2D transform; };
struct SupportPoint { Vec2 w{}, a{}, b{}; float weight=0.0f; };

void CollectConvex(const Shape2D& shape,const Transform2D& xf,std::vector<ConvexProxy>& out) {
    if(shape.type==ShapeType::Compound && shape.compound) {
        for(const auto& child:shape.compound->children) {
            Transform2D childXf{xf.Apply(child.transform.position),WrapAngle(xf.angle+child.transform.angle)};
            CollectConvex(child.shape,childXf,out);
        }
    } else if(shape.type==ShapeType::Chain) {
        int count=int(shape.chain.vertices.size())-1+(shape.chain.loop?1:0);
        for(int i=0;i<count;++i) {
            EdgeShape edge;edge.a=shape.chain.vertices[size_t(i)];edge.b=shape.chain.vertices[size_t((i+1)%shape.chain.vertices.size())];edge.oneSided=shape.chain.oneSided;
            if(shape.chain.loop||i>0){edge.ghostA=shape.chain.vertices[size_t((i+int(shape.chain.vertices.size())-1)%int(shape.chain.vertices.size()))];edge.hasGhostA=true;}
            if(shape.chain.loop||i+1<count){edge.ghostB=shape.chain.vertices[size_t((i+2)%shape.chain.vertices.size())];edge.hasGhostB=true;}
            Shape2D proxy=Shape2D::Edge(edge.a,edge.b,edge.oneSided);proxy.edge=edge;
            out.push_back({std::move(proxy),xf});
        }
    } else if(shape.type==ShapeType::Circle || shape.type==ShapeType::Polygon || shape.type==ShapeType::Edge) {
        out.push_back({shape,xf});
    }
}

Vec2 Support(const Shape2D& shape,const Transform2D& xf,Vec2 direction) {
    Vec2 localDir=Rot2(xf.angle).ApplyInverse(direction);
    if(shape.type==ShapeType::Circle) {
        return xf.Apply(shape.circle.center+NormalizeOr(localDir,{1,0})*shape.circle.radius);
    }
    if(shape.type==ShapeType::Polygon) {
        float best=-std::numeric_limits<float>::max();Vec2 point{};
        const auto& vertices=shape.polygon.vertices;
        size_t i=0;
        for(;i+4<=vertices.size();i+=4){
            Float4x2D xs{{vertices[i].x,vertices[i+1].x,vertices[i+2].x,vertices[i+3].x}};
            Float4x2D ys{{vertices[i].y,vertices[i+1].y,vertices[i+2].y,vertices[i+3].y}};
            const Float4x2D dots=BatchAdd(BatchMul(xs,Float4x2D{{localDir.x,localDir.x,localDir.x,localDir.x}}),
                                           BatchMul(ys,Float4x2D{{localDir.y,localDir.y,localDir.y,localDir.y}}));
            for(uint32_t lane=0;lane<4;++lane)if(dots.lane[lane]>best){best=dots.lane[lane];point=vertices[i+lane];}
        }
        for(;i<vertices.size();++i){const float d=Dot(vertices[i],localDir);if(d>best){best=d;point=vertices[i];}}
        return xf.Apply(point);
    }
    if(shape.type==ShapeType::Edge) {
        Vec2 p=Dot(shape.edge.a,localDir)>Dot(shape.edge.b,localDir)?shape.edge.a:shape.edge.b;
        return xf.Apply(p);
    }
    return xf.position;
}
SupportPoint MinkowskiSupport(const Shape2D& a,const Transform2D& xa,const Shape2D& b,const Transform2D& xb,Vec2 d) {
    SupportPoint p;p.a=Support(a,xa,d);p.b=Support(b,xb,-d);p.w=p.a-p.b;return p;
}

bool ReduceSimplex(std::vector<SupportPoint>& s,Vec2& closest) {
    if(s.size()==1){s[0].weight=1.0f;closest=s[0].w;return closest.LengthSquared()<=1.0e-12f;}
    if(s.size()==2){
        Vec2 a=s[0].w,b=s[1].w,ab=b-a;float den=ab.LengthSquared();float t=den>kEpsilon?Clamp(-Dot(a,ab)/den,0.0f,1.0f):0.0f;
        if(t<=1.0e-6f){SupportPoint p=s[0];p.weight=1.0f;s.assign(1,p);closest=p.w;}
        else if(t>=1.0f-1.0e-6f){SupportPoint p=s[1];p.weight=1.0f;s.assign(1,p);closest=p.w;}
        else{s[0].weight=1.0f-t;s[1].weight=t;closest=a+(b-a)*t;}
        return closest.LengthSquared()<=1.0e-12f;
    }
    if(s.size()>=3){
        const Vec2 a=s[0].w,b=s[1].w,c=s[2].w;
        const float den=Cross(b-a,c-a);
        if(std::abs(den)>1.0e-12f){
            const float wa=Cross(b,c)/den,wb=Cross(c,a)/den,wc=Cross(a,b)/den;
            if(wa>=-1.0e-6f&&wb>=-1.0e-6f&&wc>=-1.0e-6f){s[0].weight=std::max(0.0f,wa);s[1].weight=std::max(0.0f,wb);s[2].weight=std::max(0.0f,wc);float sum=s[0].weight+s[1].weight+s[2].weight;if(sum>0){for(auto&x:s)x.weight/=sum;}closest={};return true;}
        }
        float best=std::numeric_limits<float>::max();std::vector<SupportPoint> bestSimplex;Vec2 bestPoint{};
        const int edges[3][2]={{0,1},{1,2},{2,0}};
        for(const auto& e:edges){
            SupportPoint p0=s[size_t(e[0])],p1=s[size_t(e[1])];Vec2 d=p1.w-p0.w;float d2=d.LengthSquared();float t=d2>kEpsilon?Clamp(-Dot(p0.w,d)/d2,0.0f,1.0f):0.0f;Vec2 q=p0.w+d*t;float q2=q.LengthSquared();
            if(q2<best){best=q2;bestPoint=q;bestSimplex.clear();if(t<=1.0e-6f){p0.weight=1.0f;bestSimplex.push_back(p0);}else if(t>=1.0f-1.0e-6f){p1.weight=1.0f;bestSimplex.push_back(p1);}else{p0.weight=1.0f-t;p1.weight=t;bestSimplex.push_back(p0);bestSimplex.push_back(p1);}}
        }
        s=std::move(bestSimplex);closest=bestPoint;return closest.LengthSquared()<=1.0e-12f;
    }
    return false;
}

ShapeDistance2D GJKDistance(const Shape2D& a,const Transform2D& xa,const Shape2D& b,const Transform2D& xb) {
    ShapeDistance2D result;Vec2 direction=xb.position-xa.position;if(direction.LengthSquared()<kEpsilon*kEpsilon)direction={1,0};
    std::vector<SupportPoint> simplex;simplex.reserve(3);simplex.push_back(MinkowskiSupport(a,xa,b,xb,direction));
    Vec2 closest=simplex[0].w;bool overlap=false;
    for(uint32_t iter=0;iter<40;++iter){
        result.iterations=iter+1;
        if(ReduceSimplex(simplex,closest)){overlap=true;break;}
        const float dist2=closest.LengthSquared();if(dist2<=1.0e-12f){overlap=true;break;}
        direction=-closest;
        SupportPoint p=MinkowskiSupport(a,xa,b,xb,direction);
        const float progress=Dot(p.w,direction)-Dot(closest,direction);
        bool duplicate=false;for(const auto& q:simplex)if((q.w-p.w).LengthSquared()<1.0e-12f){duplicate=true;break;}
        if(duplicate || progress<=1.0e-6f*std::max(1.0f,std::sqrt(dist2)))break;
        simplex.push_back(p);
        if(simplex.size()>3)simplex.erase(simplex.begin());
    }
    ReduceSimplex(simplex,closest);
    Vec2 pointA{},pointB{};float weightSum=0.0f;
    for(const auto& p:simplex){pointA+=p.a*p.weight;pointB+=p.b*p.weight;weightSum+=p.weight;}
    if(weightSum>kEpsilon){pointA/=weightSum;pointB/=weightSum;}
    result.valid=true;result.overlapping=overlap||closest.LengthSquared()<=1.0e-10f;
    result.pointA=pointA;result.pointB=pointB;result.distance=result.overlapping?0.0f:Distance(pointA,pointB);
    result.normal=NormalizeOr(pointB-pointA,NormalizeOr(xb.position-xa.position,{1,0}));
    return result;
}
float ShapeRadius(const Shape2D& shape,const Transform2D& xf) {
    // Conservative rotational radius about the transform origin. The AABB
    // extents alone under-estimate offset geometry (e.g. an offset circle).
    AABB2 b=ComputeAABB(shape,xf); if(!b.IsValid()) return 0.0f;
    const Vec2 corners[4]={{b.min.x,b.min.y},{b.min.x,b.max.y},{b.max.x,b.min.y},{b.max.x,b.max.y}};
    float radius2=0.0f;
    for(Vec2 p:corners) radius2=std::max(radius2,(p-xf.position).LengthSquared());
    return std::sqrt(radius2);
}

} // namespace

ShapeDistance2D DistanceBetweenShapes2D(const Shape2D& a,const Transform2D& xa,const Shape2D& b,const Transform2D& xb) {
    std::vector<ConvexProxy> aa,bb;CollectConvex(a,xa,aa);CollectConvex(b,xb,bb);
    ShapeDistance2D best;best.distance=std::numeric_limits<float>::max();
    for(const auto& pa:aa)for(const auto& pb:bb){ShapeDistance2D d=GJKDistance(pa.shape,pa.transform,pb.shape,pb.transform);if(d.valid&&(d.overlapping||d.distance<best.distance))best=d;}
    if(best.distance==std::numeric_limits<float>::max())best.valid=false;
    return best;
}

ShapeCastResult2D CastShapeAgainstShape2D(const Shape2D& moving,const Transform2D& start,Vec2 translation,float rotationDelta,
                                           const Shape2D& target,const Transform2D& targetXf,float targetDistance,uint32_t maxIterations,
                                           Vec2 targetTranslation,float targetRotationDelta) {
    ShapeCastResult2D best;best.hit=false;
    std::vector<ConvexProxy> movingParts,targetParts;CollectConvex(moving,start,movingParts);CollectConvex(target,targetXf,targetParts);
    if(movingParts.empty()||targetParts.empty())return best;
    targetDistance=std::max(0.0f,targetDistance);maxIterations=std::max<uint32_t>(1,maxIterations);
    // GJK distance is computed in single precision. A small scale-aware tolerance
    // prevents conservative advancement from stopping just outside the requested
    // target distance due to a sub-epsilon final step.
    const float distanceTolerance=std::max(1.0e-5f,targetDistance*1.0e-3f);
    for(const auto& ma:movingParts)for(const auto& tb:targetParts){
        float t=0.0f;ShapeDistance2D d=GJKDistance(ma.shape,ma.transform,tb.shape,tb.transform);
        uint32_t iterations=0;
        if(d.overlapping||d.distance<=targetDistance){
            bool validInitialHit=true;
            if(tb.shape.type==ShapeType::Edge&&tb.shape.edge.oneSided){
                const Vec2 ea=tb.transform.Apply(tb.shape.edge.a),eb=tb.transform.Apply(tb.shape.edge.b);
                const Vec2 tangent=NormalizeOr(eb-ea,{1,0});const Vec2 front{tangent.y,-tangent.x};
                const Vec2 movingCenter=ComputeAABB(ma.shape,ma.transform).Center();
                // A shape already overlapping from behind must not be pulled
                // through the back of a one-sided edge by a zero-time cast.
                validInitialHit=Dot(front,movingCenter-ea)>=-0.005f;
            }
            if(validInitialHit&&(!best.hit||0.0f<best.fraction)){best.hit=true;best.fraction=0.0f;best.point=(d.pointA+d.pointB)*0.5f;best.normal=d.normal;best.iterations=d.iterations;}
            continue;
        }
        const float radius=ShapeRadius(ma.shape,ma.transform);
        bool hit=false;
        for(uint32_t it=0;it<maxIterations;++it){
            ++iterations;
            Transform2D movingXf=ma.transform;movingXf.position+=translation*t;movingXf.angle=WrapAngle(movingXf.angle+rotationDelta*t);
            Transform2D targetAt=tb.transform;targetAt.position+=targetTranslation*t;targetAt.angle=WrapAngle(targetAt.angle+targetRotationDelta*t);
            d=GJKDistance(ma.shape,movingXf,tb.shape,targetAt);
            if(!d.valid)break;
            if(d.overlapping||d.distance<=targetDistance+distanceTolerance){hit=true;break;}
            const float closing=Dot(translation-targetTranslation,d.normal)+std::abs(rotationDelta)*radius+std::abs(targetRotationDelta)*ShapeRadius(tb.shape,tb.transform);
            if(closing<=1.0e-8f)break;
            const float advance=(d.distance-targetDistance)/closing;
            if(!std::isfinite(advance))break;
            if(advance<=1.0e-7f){
                if(d.distance<=targetDistance+distanceTolerance*4.0f)hit=true;
                break;
            }
            t+=advance;
            if(t>1.0f)break;
        }
        if(hit&&t<=1.0f&&tb.shape.type==ShapeType::Edge&&tb.shape.edge.oneSided){
            const Vec2 ea=tb.transform.Apply(tb.shape.edge.a),eb=tb.transform.Apply(tb.shape.edge.b);
            const Vec2 tangent=NormalizeOr(eb-ea,{1,0});const Vec2 front{tangent.y,-tangent.x};
            const Vec2 movingCenter=ComputeAABB(ma.shape,ma.transform).Center();
            const bool startedOnFront=Dot(front,movingCenter-ea)>=-0.005f;
            // GJK's normal points from moving shape to target. A valid one-sided
            // hit therefore approaches opposite the edge's outward/front normal.
            const bool approachesFront=Dot(front,d.normal)<-0.05f;
            if(!startedOnFront||!approachesFront)hit=false;
        }
        if(hit&&t<=1.0f&&(!best.hit||t<best.fraction)){
            best.hit=true;best.fraction=Clamp(t,0.0f,1.0f);best.point=(d.pointA+d.pointB)*0.5f;best.normal=d.normal;best.iterations=iterations;
        }
    }
    return best;
}