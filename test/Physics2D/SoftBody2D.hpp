#pragma once
#include "Physics2D.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>
#include <utility>

struct MOSS_API SoftBodyNode2D { Vec2 position{}, previousPosition{}, velocity{}; float inverseMass=1.0f; };
struct MOSS_API SoftBodyDistanceConstraint2D { uint32_t a=0,b=0; float restLength=0,compliance=0,lambda=0; };
struct MOSS_API SoftBodyAreaConstraint2D { std::vector<uint32_t> nodes; float restArea=0,compliance=0,lambda=0; };
struct MOSS_API SoftBodySettings2D { Vec2 gravity{0,-9.81f}; int solverIterations=8; float damping=0.995f,collisionRadius=0.04f; float contactFriction=0.45f,restitutionThreshold=1.0f,contactSlop=0.002f; };

// XPBD particle deformable. The host supplies collision projection so this
// module remains independent from Moss renderer/ECS and the rigid-body solver.
class MOSS_API SoftBody2D {
public:
    using CollisionProjector=std::function<void(Vec2& position,Vec2& velocity,float radius)>;
    uint32_t AddNode(Vec2 p,float mass=1.0f){SoftBodyNode2D n;n.position=n.previousPosition=p;n.inverseMass=mass>kEpsilon?1.0f/mass:0.0f;nodes_.push_back(n);return uint32_t(nodes_.size()-1);}
    bool PinNode(uint32_t i,Vec2 p){if(i>=nodes_.size())return false;auto&n=nodes_[i];n.position=n.previousPosition=p;n.velocity={};n.inverseMass=0;return true;}
    bool SetNodeMass(uint32_t i,float mass){if(i>=nodes_.size()||!std::isfinite(mass)||mass<0)return false;nodes_[i].inverseMass=mass>kEpsilon?1.0f/mass:0.0f;return true;}
    bool AddDistanceConstraint(uint32_t a,uint32_t b,float compliance=0){if(a>=nodes_.size()||b>=nodes_.size()||a==b||!std::isfinite(compliance)||compliance<0)return false;float l=Distance(nodes_[a].position,nodes_[b].position);if(l<=kEpsilon)return false;distance_.push_back({a,b,l,compliance,0});return true;}
    bool AddAreaConstraint(const std::vector<uint32_t>& ids,float compliance=0){if(ids.size()<3||!std::isfinite(compliance)||compliance<0)return false;for(auto i:ids)if(i>=nodes_.size())return false;SoftBodyAreaConstraint2D c;c.nodes=ids;c.compliance=compliance;c.restArea=Area(ids);if(std::abs(c.restArea)<=kEpsilon)return false;area_.push_back(std::move(c));return true;}
    void Clear(){nodes_.clear();distance_.clear();area_.clear();}
    size_t GetNodeCount()const{return nodes_.size();}
    const std::vector<SoftBodyNode2D>& GetNodes()const{return nodes_;}
    std::vector<SoftBodyNode2D>& GetNodes(){return nodes_;}
    const std::vector<SoftBodyDistanceConstraint2D>& GetDistanceConstraints()const{return distance_;}
    const std::vector<SoftBodyAreaConstraint2D>& GetAreaConstraints()const{return area_;}
    void Step(float dt,const SoftBodySettings2D& settings={},const CollisionProjector& collision={}){
        if(!std::isfinite(dt)||dt<=0||nodes_.empty())return;
        const float invDt2=1.0f/(dt*dt),damping=Clamp(settings.damping,0.0f,1.0f);
        for(auto&n:nodes_){n.previousPosition=n.position;if(n.inverseMass<=0){n.velocity={};continue;}n.velocity=(n.velocity+settings.gravity*dt)*damping;n.position+=n.velocity*dt;}
        for(auto&c:distance_)c.lambda=0;for(auto&c:area_)c.lambda=0;
        for(int it=0;it<std::max(1,settings.solverIterations);++it){
            for(auto&c:distance_){auto&a=nodes_[c.a];auto&b=nodes_[c.b];Vec2 d=b.position-a.position;float len=d.Length();if(len<=kEpsilon)continue;Vec2 n=d/len;float alpha=c.compliance*invDt2,den=a.inverseMass+b.inverseMass+alpha;if(den<=kEpsilon)continue;float dl=(-(len-c.restLength)-alpha*c.lambda)/den;c.lambda+=dl;a.position-=n*(a.inverseMass*dl);b.position+=n*(b.inverseMass*dl);}
            for(auto&c:area_)SolveArea(c,invDt2);
        }
        for(auto&n:nodes_)if(n.inverseMass>0)n.velocity=(n.position-n.previousPosition)/dt;
        if(collision)for(auto&n:nodes_)if(n.inverseMass>0)collision(n.position,n.velocity,std::max(0.0f,settings.collisionRadius));
    }
    // Prepare a deformable prediction and cache its rigid contacts. These methods
    // are called by CoupledSoftBodyWorld2D from PhysicsWorld2D's own solver phases.
    void PrepareCoupledStep(PhysicsWorld2D& world,float dt,const SoftBodySettings2D& settings={},float restitution=0.0f,float skinWidth=0.002f){
        if(!std::isfinite(dt)||dt<=0.0f)return;
        const float damping=Clamp(settings.damping,0.0f,1.0f);
        contacts_.assign(nodes_.size(),RigidContact{});
        for(auto& n:nodes_){
            n.previousPosition=n.position;
            if(n.inverseMass<=0.0f){n.velocity={};continue;}
            n.velocity=(n.velocity+settings.gravity*dt)*damping;
            n.position+=n.velocity*dt;
        }
        for(auto& c:distance_)c.lambda=0.0f;
        for(auto& c:area_)c.lambda=0.0f;
        const float radius=std::max(0.0f,settings.collisionRadius);
        lastCollisionRadius_=radius;
        for(size_t i=0;i<nodes_.size();++i){
            auto& node=nodes_[i];
            if(node.inverseMass<=0.0f)continue;
            const Vec2 displacement=node.position-node.previousPosition;
            if(displacement.LengthSquared()<=kEpsilon*kEpsilon)continue;
            auto hit=world.ShapeCast(Shape2D::Circle(std::max(radius,0.001f)),
                Transform2D{node.previousPosition,0.0f},displacement,0.0f,{},InvalidBodyID,skinWidth);
            if(!hit)continue;
            const BodyState* body=world.GetBody(hit->body);
            if(!body)continue;
            RigidContact& contact=contacts_[i];
            contact.active=true;contact.body=hit->body;
            contact.localPoint=body->transform.ApplyInverse(hit->point);
            const Vec2 worldNormal=-NormalizeOr(hit->normal,{0,1});
            contact.localNormal=Rot2(body->transform.angle).ApplyInverse(worldNormal);
            contact.restitution=Clamp(restitution,0.0f,1.0f);
            contact.friction=std::max(0.0f,settings.contactFriction);
            contact.slop=std::max(0.0f,settings.contactSlop);
            const Vec2 r=hit->point-body->worldCenter;
            const Vec2 bodyVelocity=body->linearVelocity+Cross(body->angularVelocity,r);
            const float initialVn=Dot(node.velocity-bodyVelocity,worldNormal);
            contact.restitutionBias=initialVn < -std::max(0.0f,settings.restitutionThreshold)
                ? -contact.restitution*initialVn : 0.0f;
            // Keep the prediction intact: the common position iterations project
            // the particle and the rigid body together using the cached local anchor.
        }
    }

    void SolveCoupledVelocityIteration(PhysicsWorld2D& world,float dt,int iteration){
        if(dt<=0.0f)return;
        for(size_t i=0;i<nodes_.size()&&i<contacts_.size();++i){
            auto& node=nodes_[i];auto& c=contacts_[i];
            if(!c.active||node.inverseMass<=0.0f)continue;
            const BodyState* body=world.GetBody(c.body);if(!body)continue;
            const Vec2 normal=NormalizeOr(Rot2(body->transform.angle).Apply(c.localNormal),{0,1});
            const Vec2 tangent=normal.Perpendicular();
            const Vec2 point=body->transform.Apply(c.localPoint);
            const Vec2 r=point-body->worldCenter;
            Vec2 bodyVelocity=body->linearVelocity+Cross(body->angularVelocity,r);
            Vec2 relative=node.velocity-bodyVelocity;
            const float rn=Cross(r,normal);
            const float normalMass=node.inverseMass+body->inverseMass+body->inverseInertia*rn*rn;
            if(normalMass>kEpsilon){
                const float vn=Dot(relative,normal);
                const float bias=iteration==0?c.restitutionBias:0.0f;
                const float delta=-(vn-bias)/normalMass;
                const float old=c.normalImpulse;
                c.normalImpulse=std::max(0.0f,old+delta);
                const float applied=c.normalImpulse-old;
                if(std::abs(applied)>kEpsilon){
                    const Vec2 impulse=normal*applied;
                    node.velocity+=impulse*node.inverseMass;
                    node.position+=impulse*(node.inverseMass*dt);
                    world.ApplyLinearImpulse(c.body,-impulse,point,true);
                }
            }
            body=world.GetBody(c.body);if(!body)continue;
            bodyVelocity=body->linearVelocity+Cross(body->angularVelocity,point-body->worldCenter);
            relative=node.velocity-bodyVelocity;
            const float rt=Cross(point-body->worldCenter,tangent);
            const float tangentMass=node.inverseMass+body->inverseMass+body->inverseInertia*rt*rt;
            if(tangentMass>kEpsilon){
                const float delta=-Dot(relative,tangent)/tangentMass;
                const float maxFriction=c.friction*c.normalImpulse;
                const float old=c.tangentImpulse;
                c.tangentImpulse=Clamp(old+delta,-maxFriction,maxFriction);
                const float applied=c.tangentImpulse-old;
                if(std::abs(applied)>kEpsilon){
                    const Vec2 impulse=tangent*applied;
                    node.velocity+=impulse*node.inverseMass;
                    node.position+=impulse*(node.inverseMass*dt);
                    world.ApplyLinearImpulse(c.body,-impulse,point,true);
                }
            }
        }
    }

    void SolveCoupledPositionIteration(PhysicsWorld2D& world,float dt){
        if(dt<=0.0f)return;
        const float invDt2=1.0f/(dt*dt);
        for(auto& c:distance_){
            auto& a=nodes_[c.a];auto& b=nodes_[c.b];
            const Vec2 d=b.position-a.position;const float len=d.Length();if(len<=kEpsilon)continue;
            const Vec2 n=d/len;const float alpha=c.compliance*invDt2;
            const float denominator=a.inverseMass+b.inverseMass+alpha;if(denominator<=kEpsilon)continue;
            const float dl=(-(len-c.restLength)-alpha*c.lambda)/denominator;c.lambda+=dl;
            a.position-=n*(a.inverseMass*dl);b.position+=n*(b.inverseMass*dl);
        }
        for(auto& c:area_)SolveArea(c,invDt2);
        const float radius=std::max(0.0f,lastCollisionRadius_);
        for(size_t i=0;i<nodes_.size()&&i<contacts_.size();++i){
            auto& node=nodes_[i];auto& c=contacts_[i];
            if(!c.active||node.inverseMass<=0.0f)continue;
            const BodyState* body=world.GetBody(c.body);if(!body)continue;
            const Vec2 normal=NormalizeOr(Rot2(body->transform.angle).Apply(c.localNormal),{0,1});
            const Vec2 point=body->transform.Apply(c.localPoint);const Vec2 r=point-body->worldCenter;
            const float separation=Dot(node.position-point,normal)-radius;
            const float penetration=std::max(0.0f,-separation-c.slop);if(penetration<=0.0f)continue;
            const float rn=Cross(r,normal);
            const float effective=node.inverseMass+body->inverseMass+body->inverseInertia*rn*rn;
            if(effective<=kEpsilon)continue;
            const float lambda=0.8f*penetration/effective;
            node.position+=normal*(node.inverseMass*lambda);
            world.ApplyPositionImpulse(c.body,-normal*lambda,point,true);
        }
    }

    void FinishCoupledStep(float dt){
        if(dt<=0.0f)return;
        for(auto& n:nodes_)if(n.inverseMass>0.0f)n.velocity=(n.position-n.previousPosition)/dt;
    }

    void AppendCoupledIslandLinks(const PhysicsWorld2D& world,std::vector<std::pair<BodyID,BodyID>>& links)const{
        std::vector<BodyID> dynamicBodies;
        for(const auto& c:contacts_)if(c.active){const BodyState* body=world.GetBody(c.body);if(body&&body->type==BodyType::Dynamic)dynamicBodies.push_back(c.body);}
        std::sort(dynamicBodies.begin(),dynamicBodies.end());
        dynamicBodies.erase(std::unique(dynamicBodies.begin(),dynamicBodies.end()),dynamicBodies.end());
        if(dynamicBodies.size()>1)for(size_t i=1;i<dynamicBodies.size();++i)links.emplace_back(dynamicBodies[0],dynamicBodies[i]);
    }
    bool HasCoupledDynamicBody(const PhysicsWorld2D& world)const{
        for(const auto& c:contacts_)if(c.active){const BodyState* body=world.GetBody(c.body);if(body&&body->type==BodyType::Dynamic)return true;}
        return false;
    }
    bool BelongsToCoupledIsland(const PhysicsWorld2D& world,const std::vector<BodyID>& islandBodies)const{
        for(const auto& c:contacts_)if(c.active){const BodyState* body=world.GetBody(c.body);if(body&&body->type==BodyType::Dynamic&&std::find(islandBodies.begin(),islandBodies.end(),c.body)!=islandBodies.end())return true;}
        return false;
    }

    // Backwards-compatible convenience call. The coordinator below is preferred
    // when multiple deformables share one rigid world.
    void StepCoupled(PhysicsWorld2D& world,float dt,const SoftBodySettings2D& settings={},float restitution=0.0f,float skinWidth=0.002f){
        PhysicsStepHooks2D hooks;
        hooks.prepare=[&](float h){PrepareCoupledStep(world,h,settings,restitution,skinWidth);};
        hooks.solveVelocityIteration=[&](float h,int i){SolveCoupledVelocityIteration(world,h,i);};
        hooks.solvePositionIteration=[&](float h,int){SolveCoupledPositionIteration(world,h);};
        hooks.finish=[&](float h){FinishCoupledStep(h);};
        world.StepFixed(dt,&hooks);
    }

    static SoftBody2D CreateRope(Vec2 a,Vec2 b,uint32_t count,float mass=1.0f,float compliance=0){SoftBody2D r;count=std::max<uint32_t>(2,count);for(uint32_t i=0;i<count;++i)r.AddNode(a+(b-a)*(float(i)/float(count-1)),mass);for(uint32_t i=1;i<count;++i)r.AddDistanceConstraint(i-1,i,compliance);r.PinNode(0,a);return r;}
    static SoftBody2D CreateClothGrid(uint32_t cols,uint32_t rows,float spacing,Vec2 origin,float mass=1.0f,float struct MOSS_APIural=0,float shear=0.0001f,float bend=0.001f){
        SoftBody2D c;cols=std::max<uint32_t>(2,cols);rows=std::max<uint32_t>(2,rows);if(!std::isfinite(spacing)||spacing<=0)return c;
        for(uint32_t y=0;y<rows;++y)for(uint32_t x=0;x<cols;++x)c.AddNode(origin+Vec2{float(x)*spacing,-float(y)*spacing},mass);
        auto id=[cols](uint32_t x,uint32_t y){return y*cols+x;};
        for(uint32_t y=0;y<rows;++y)for(uint32_t x=0;x<cols;++x){if(x+1<cols)c.AddDistanceConstraint(id(x,y),id(x+1,y),struct MOSS_APIural);if(y+1<rows)c.AddDistanceConstraint(id(x,y),id(x,y+1),struct MOSS_APIural);if(x+1<cols&&y+1<rows){c.AddDistanceConstraint(id(x,y),id(x+1,y+1),shear);c.AddDistanceConstraint(id(x+1,y),id(x,y+1),shear);}if(x+2<cols)c.AddDistanceConstraint(id(x,y),id(x+2,y),bend);if(y+2<rows)c.AddDistanceConstraint(id(x,y),id(x,y+2),bend);}
        for(uint32_t y=0;y+1<rows;++y)for(uint32_t x=0;x+1<cols;++x)c.AddAreaConstraint({id(x,y),id(x+1,y),id(x+1,y+1),id(x,y+1)},struct MOSS_APIural);
        c.PinNode(0,origin);c.PinNode(cols-1,origin+Vec2{float(cols-1)*spacing,0});return c;
    }
    AABB2 GetBounds()const{AABB2 b;for(const auto&n:nodes_)b.Include(n.position);return b.IsValid()?b:AABB2{{0,0},{0,0}};}
private:
    struct MOSS_API RigidContact {
        bool active=false;BodyID body=InvalidBodyID;Vec2 localPoint{},localNormal{0,1};
        float normalImpulse=0.0f,tangentImpulse=0.0f,restitution=0.0f,restitutionBias=0.0f,friction=0.45f,slop=0.002f;
    };
    float lastCollisionRadius_=0.04f;
    std::vector<RigidContact> contacts_;
    float Area(const std::vector<uint32_t>&ids)const{float a=0;for(size_t i=0;i<ids.size();++i)a+=Cross(nodes_[ids[i]].position,nodes_[ids[(i+1)%ids.size()]].position);return 0.5f*a;}
    void SolveArea(SoftBodyAreaConstraint2D&c,float invDt2){size_t n=c.nodes.size();std::vector<Vec2>g(n);float w=0,err=Area(c.nodes)-c.restArea;for(size_t i=0;i<n;++i){Vec2 prev=nodes_[c.nodes[(i+n-1)%n]].position,next=nodes_[c.nodes[(i+1)%n]].position;g[i]=0.5f*Vec2{next.y-prev.y,prev.x-next.x};w+=nodes_[c.nodes[i]].inverseMass*g[i].LengthSquared();}float alpha=c.compliance*invDt2,den=w+alpha;if(den<=kEpsilon)return;float dl=(-err-alpha*c.lambda)/den;c.lambda+=dl;for(size_t i=0;i<n;++i)nodes_[c.nodes[i]].position+=g[i]*(nodes_[c.nodes[i]].inverseMass*dl);}
    std::vector<SoftBodyNode2D>nodes_;std::vector<SoftBodyDistanceConstraint2D>distance_;std::vector<SoftBodyAreaConstraint2D>area_;
};

// Fixed-step coordinator. Soft bodies contribute graph edges between every
// dynamic rigid body they touch. PhysicsWorld2D merges those bodies into one
// mixed island and invokes the soft-body XPBD/contact passes inside that island's
// worker task. Unanchored deformables form independent serial islands.
class MOSS_API CoupledSoftBodyWorld2D {
public:
    bool Add(SoftBody2D& body){if(std::find(bodies_.begin(),bodies_.end(),&body)!=bodies_.end())return false;bodies_.push_back(&body);return true;}
    bool Remove(SoftBody2D& body){auto it=std::find(bodies_.begin(),bodies_.end(),&body);if(it==bodies_.end())return false;bodies_.erase(it);return true;}
    void Clear(){bodies_.clear();accumulator_=0;}
    size_t GetBodyCount()const{return bodies_.size();}
    float GetInterpolationAlpha(const PhysicsWorld2D& world)const{const float h=std::max(world.GetSettings().fixedTimeStep,1.0f/1000.0f);return Clamp(accumulator_/h,0.0f,1.0f);}
    void Step(PhysicsWorld2D& rigidWorld,float frameDt,const SoftBodySettings2D& softSettings={},float restitution=0.0f){
        if(!std::isfinite(frameDt)||frameDt<=0.0f)return;
        const auto settings=rigidWorld.GetSettings();
        accumulator_=std::min(accumulator_+frameDt,settings.maxFrameTime);int steps=0;
        while(accumulator_+kEpsilon>=settings.fixedTimeStep&&steps<settings.maxSubSteps){
            PhysicsStepHooks2D hooks;
            hooks.prepare=[&](float h){for(auto* body:bodies_)if(body)body->PrepareCoupledStep(rigidWorld,h,softSettings,restitution);};
            hooks.gatherIslandLinks=[&](std::vector<std::pair<BodyID,BodyID>>& links){for(const auto* body:bodies_)if(body)body->AppendCoupledIslandLinks(rigidWorld,links);};
            hooks.solveVelocityIsland=[&](float h,int iteration,const std::vector<BodyID>& ids){for(auto* body:bodies_)if(body&&body->BelongsToCoupledIsland(rigidWorld,ids))body->SolveCoupledVelocityIteration(rigidWorld,h,iteration);};
            hooks.solvePositionIsland=[&](float h,int,const std::vector<BodyID>& ids){for(auto* body:bodies_)if(body&&body->BelongsToCoupledIsland(rigidWorld,ids))body->SolveCoupledPositionIteration(rigidWorld,h);};
            hooks.solveVelocityDetached=[&](float h,int iteration){for(auto* body:bodies_)if(body&&!body->HasCoupledDynamicBody(rigidWorld))body->SolveCoupledVelocityIteration(rigidWorld,h,iteration);};
            hooks.solvePositionDetached=[&](float h,int){for(auto* body:bodies_)if(body&&!body->HasCoupledDynamicBody(rigidWorld))body->SolveCoupledPositionIteration(rigidWorld,h);};
            hooks.finish=[&](float h){for(auto* body:bodies_)if(body)body->FinishCoupledStep(h);};
            rigidWorld.StepFixed(settings.fixedTimeStep,&hooks);
            accumulator_-=settings.fixedTimeStep;++steps;
        }
    }
private:
    std::vector<SoftBody2D*> bodies_;float accumulator_=0.0f;
};
