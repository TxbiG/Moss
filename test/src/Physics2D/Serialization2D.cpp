#include <Moss/Physics2D/Serialization2D.hpp>
#include <cstring>
#include <algorithm>
#include <limits>

namespace Moss::Physics2D {
namespace {
struct Writer {
    std::vector<uint8_t>& out;
    void U8(uint8_t v){out.push_back(v);} void U16(uint16_t v){U8(uint8_t(v));U8(uint8_t(v>>8));}
    void U32(uint32_t v){for(int i=0;i<4;++i)U8(uint8_t(v>>(i*8)));}
    void U64(uint64_t v){for(int i=0;i<8;++i)U8(uint8_t(v>>(i*8)));}
    void I16(int16_t v){U16(static_cast<uint16_t>(v));}
    void F32(float v){uint32_t bits=0;std::memcpy(&bits,&v,sizeof(bits));U32(bits);}
    void Bool(bool v){U8(v?1:0);} void Vec(Vec2 v){F32(v.x);F32(v.y);} void Transform(Transform2D v){Vec(v.position);F32(v.angle);}
    void String(const std::string& v){U32(uint32_t(v.size()));out.insert(out.end(),v.begin(),v.end());}
};
struct Reader {
    const std::vector<uint8_t>& in; size_t pos=0; bool ok=true;
    uint8_t U8(){if(pos>=in.size()){ok=false;return 0;}return in[pos++];}
    uint16_t U16(){uint16_t v=U8();v|=uint16_t(U8())<<8;return v;}
    uint32_t U32(){uint32_t v=0;for(int i=0;i<4;++i)v|=uint32_t(U8())<<(i*8);return v;}
    uint64_t U64(){uint64_t v=0;for(int i=0;i<8;++i)v|=uint64_t(U8())<<(i*8);return v;}
    int16_t I16(){return static_cast<int16_t>(U16());}
    float F32(){uint32_t bits=U32();float v=0;std::memcpy(&v,&bits,sizeof(v));return v;}
    bool Bool(){uint8_t v=U8();if(v>1)ok=false;return v!=0;}
    Vec2 Vec(){return {F32(),F32()};} Transform2D Transform(){Transform2D v;v.position=Vec();v.angle=F32();return v;}
    std::string String(){uint32_t n=U32();if(n>1024*1024||n>in.size()-std::min(pos,in.size())){ok=false;return {};}std::string v(reinterpret_cast<const char*>(in.data()+pos),n);pos+=n;return v;}
};
constexpr uint32_t MaxBodies=1000000,MaxJoints=1000000,MaxChildren=65535,MaxVertices=65535,MaxDepth=16;
void WriteShape(Writer&w,const Shape2D&s,uint32_t depth){
    w.U8(uint8_t(s.type));if(depth>MaxDepth){return;}
    switch(s.type){
    case ShapeType::Circle:w.Vec(s.circle.center);w.F32(s.circle.radius);break;
    case ShapeType::Polygon:w.U32(uint32_t(s.polygon.vertices.size()));for(auto v:s.polygon.vertices)w.Vec(v);break;
    case ShapeType::Edge:w.Vec(s.edge.a);w.Vec(s.edge.b);w.Vec(s.edge.ghostA);w.Vec(s.edge.ghostB);w.Bool(s.edge.hasGhostA);w.Bool(s.edge.hasGhostB);w.Bool(s.edge.oneSided);break;
    case ShapeType::Chain:w.Bool(s.chain.loop);w.Bool(s.chain.oneSided);w.U32(uint32_t(s.chain.vertices.size()));for(auto v:s.chain.vertices)w.Vec(v);break;
    case ShapeType::Compound:{const uint32_t count=s.compound?uint32_t(s.compound->children.size()):0;w.U32(count);if(s.compound)for(const auto&c:s.compound->children){w.Transform(c.transform);w.U32(c.id);w.Bool(c.isSensor);w.U16(c.categoryBits);w.U16(c.maskBits);w.I16(c.groupIndex);w.F32(c.friction);w.F32(c.restitution);w.F32(c.density);WriteShape(w,c.shape,depth+1);}break;}
    }
}
bool ReadShape(Reader&r,Shape2D&out,uint32_t depth){
    if(depth>MaxDepth){r.ok=false;return false;}uint8_t type=r.U8();if(type>uint8_t(ShapeType::Compound)){r.ok=false;return false;}
    switch(ShapeType(type)){
    case ShapeType::Circle:{Vec2 center=r.Vec();float radius=r.F32();out=Shape2D::Circle(radius,center);break;}
    case ShapeType::Polygon:{uint32_t n=r.U32();if(n<3||n>MaxVertices){r.ok=false;return false;}std::vector<Vec2> points;points.reserve(n);for(uint32_t i=0;i<n;++i)points.push_back(r.Vec());out=Shape2D::Polygon(points);break;}
    case ShapeType::Edge:{Vec2 a=r.Vec(),b=r.Vec(),ga=r.Vec(),gb=r.Vec();bool ha=r.Bool(),hb=r.Bool(),one=r.Bool();out=Shape2D::Edge(a,b,one);out.edge.ghostA=ga;out.edge.ghostB=gb;out.edge.hasGhostA=ha;out.edge.hasGhostB=hb;break;}
    case ShapeType::Chain:{bool loop=r.Bool(),one=r.Bool();uint32_t n=r.U32();if(n<2||n>MaxVertices){r.ok=false;return false;}std::vector<Vec2> points;points.reserve(n);for(uint32_t i=0;i<n;++i)points.push_back(r.Vec());out=Shape2D::Chain(points,loop,one);break;}
    case ShapeType::Compound:{uint32_t n=r.U32();if(n==0||n>MaxChildren){r.ok=false;return false;}std::vector<CompoundChild2D> children;children.reserve(n);for(uint32_t i=0;i<n;++i){CompoundChild2D c;c.transform=r.Transform();c.id=r.U32();c.isSensor=r.Bool();c.categoryBits=r.U16();c.maskBits=r.U16();c.groupIndex=r.I16();c.friction=r.F32();c.restitution=r.F32();c.density=r.F32();if(!ReadShape(r,c.shape,depth+1))return false;children.push_back(std::move(c));}out=Shape2D::Compound(children);break;}
    }
    if(!r.ok||!out.IsValid()){r.ok=false;return false;}return true;
}
void WriteSettings(Writer&w,const WorldSettings&s){w.Vec(s.gravity);w.U32(uint32_t(s.velocityIterations));w.U32(uint32_t(s.positionIterations));w.F32(s.fixedTimeStep);w.U32(uint32_t(s.maxSubSteps));w.F32(s.linearDamping);w.F32(s.angularDamping);w.F32(s.contactSlop);w.F32(s.baumgarte);w.F32(s.sleepLinearThreshold);w.F32(s.sleepAngularThreshold);w.F32(s.timeToSleep);w.F32(s.broadPhaseCellSize);w.F32(s.maxFrameTime);w.U32(uint32_t(s.maxCCDSubSteps));w.U32(s.maxCCDImpactsPerStep);w.F32(s.ccdMotionThreshold);w.Bool(s.deterministicMode);w.Bool(s.enableMultithreading);w.U32(s.workerCount);}
void ReadSettings(Reader&r,WorldSettings&s){s.gravity=r.Vec();s.velocityIterations=int(r.U32());s.positionIterations=int(r.U32());s.fixedTimeStep=r.F32();s.maxSubSteps=int(r.U32());s.linearDamping=r.F32();s.angularDamping=r.F32();s.contactSlop=r.F32();s.baumgarte=r.F32();s.sleepLinearThreshold=r.F32();s.sleepAngularThreshold=r.F32();s.timeToSleep=r.F32();s.broadPhaseCellSize=r.F32();s.maxFrameTime=r.F32();s.maxCCDSubSteps=int(r.U32());s.maxCCDImpactsPerStep=r.U32();s.ccdMotionThreshold=r.F32();s.deterministicMode=r.Bool();s.enableMultithreading=r.Bool();s.workerCount=r.U32();}
void WriteBody(Writer&w,const ArchivedBody2D&b){const auto&d=b.definition;w.U32(b.sourceId);w.U8(uint8_t(d.type));WriteShape(w,d.shape,0);w.Vec(d.position);w.F32(d.angle);w.Vec(d.linearVelocity);w.F32(d.angularVelocity);w.F32(d.gravityScale);w.F32(d.linearDamping);w.F32(d.angularDamping);w.Bool(d.fixedRotation);w.Bool(d.bullet);w.Bool(d.isSensor);w.Bool(d.allowSleep);w.Bool(d.initiallyAwake);w.Bool(d.enabled);w.Bool(d.oneWayPlatform);w.Vec(d.oneWayNormal);w.U16(d.filter.categoryBits);w.U16(d.filter.maskBits);w.I16(d.filter.groupIndex);w.F32(d.material.friction);w.F32(d.material.restitution);w.F32(d.material.density);w.String(d.name);w.Transform(b.state.transform);w.Vec(b.state.linearVelocity);w.F32(b.state.angularVelocity);w.Vec(b.state.force);w.F32(b.state.torque);w.Bool(b.state.awake);w.Bool(b.state.enabled);}
bool ReadBody(Reader&r,ArchivedBody2D&b){auto&d=b.definition;b.sourceId=r.U32();uint8_t t=r.U8();if(t>uint8_t(BodyType::Dynamic)){r.ok=false;return false;}d.type=BodyType(t);if(!ReadShape(r,d.shape,0))return false;d.position=r.Vec();d.angle=r.F32();d.linearVelocity=r.Vec();d.angularVelocity=r.F32();d.gravityScale=r.F32();d.linearDamping=r.F32();d.angularDamping=r.F32();d.fixedRotation=r.Bool();d.bullet=r.Bool();d.isSensor=r.Bool();d.allowSleep=r.Bool();d.initiallyAwake=r.Bool();d.enabled=r.Bool();d.oneWayPlatform=r.Bool();d.oneWayNormal=r.Vec();d.filter.categoryBits=r.U16();d.filter.maskBits=r.U16();d.filter.groupIndex=r.I16();d.material.friction=r.F32();d.material.restitution=r.F32();d.material.density=r.F32();d.name=r.String();b.state.id=b.sourceId;b.state.transform=r.Transform();b.state.linearVelocity=r.Vec();b.state.angularVelocity=r.F32();b.state.force=r.Vec();b.state.torque=r.F32();b.state.awake=r.Bool();b.state.enabled=r.Bool();d.userData=nullptr;return r.ok;}
void WriteJoint(Writer&w,const ArchivedJoint2D&j){w.U32(j.sourceId);w.U8(uint8_t(j.type));w.U32(j.bodyA);w.U32(j.bodyB);w.Vec(j.anchorA);w.Vec(j.anchorB);w.Vec(j.axisA);w.Vec(j.target);w.Vec(j.groundAnchorA);w.Vec(j.groundAnchorB);w.Vec(j.linearOffset);w.F32(j.length);w.F32(j.minLength);w.F32(j.maxLength);w.F32(j.frequency);w.F32(j.damping);w.F32(j.lower);w.F32(j.upper);w.F32(j.motorSpeed);w.F32(j.maxMotor);w.F32(j.referenceAngle);w.F32(j.ratio);w.F32(j.constant);w.F32(j.angularOffset);w.F32(j.correctionFactor);w.F32(j.maxTorque);w.F32(j.maxForce);w.F32(j.breakForce);w.F32(j.breakTorque);w.Bool(j.enableLimit);w.Bool(j.enableMotor);w.Bool(j.collideConnected);w.Bool(j.enabled);}
bool ReadJoint(Reader&r,ArchivedJoint2D&j){j.sourceId=r.U32();uint8_t t=r.U8();if(t>uint8_t(JointType::Gear)){r.ok=false;return false;}j.type=JointType(t);j.bodyA=r.U32();j.bodyB=r.U32();j.anchorA=r.Vec();j.anchorB=r.Vec();j.axisA=r.Vec();j.target=r.Vec();j.groundAnchorA=r.Vec();j.groundAnchorB=r.Vec();j.linearOffset=r.Vec();j.length=r.F32();j.minLength=r.F32();j.maxLength=r.F32();j.frequency=r.F32();j.damping=r.F32();j.lower=r.F32();j.upper=r.F32();j.motorSpeed=r.F32();j.maxMotor=r.F32();j.referenceAngle=r.F32();j.ratio=r.F32();j.constant=r.F32();j.angularOffset=r.F32();j.correctionFactor=r.F32();j.maxTorque=r.F32();j.maxForce=r.F32();j.breakForce=r.F32();j.breakTorque=r.F32();j.enableLimit=r.Bool();j.enableMotor=r.Bool();j.collideConnected=r.Bool();j.enabled=r.Bool();return r.ok;}
}

bool SerializeWorldArchive2D(const WorldArchive2D&a,std::vector<uint8_t>&bytes,std::string*error){
    auto fail=[&](const char*m){if(error)*error=m;return false;};if(a.formatVersion!=1)return fail("Unsupported world archive version");if(a.bodies.size()>MaxBodies||a.joints.size()>MaxJoints)return fail("Archive exceeds supported object count");
    bytes.clear();bytes.reserve(128+a.bodies.size()*128+a.joints.size()*128);Writer w{bytes};const uint8_t magic[8]={'M','S','P','2','D','A','R','1'};bytes.insert(bytes.end(),magic,magic+8);w.U32(a.formatVersion);w.U64(a.stepCount);WriteSettings(w,a.settings);w.U32(uint32_t(a.bodies.size()));for(const auto&b:a.bodies){if(!b.definition.shape.IsValid())return fail("Archive contains invalid body shape");WriteBody(w,b);}w.U32(uint32_t(a.joints.size()));for(const auto&j:a.joints)WriteJoint(w,j);w.U32(0x4d325f45u);if(error)error->clear();return true;
}
bool DeserializeWorldArchive2D(const std::vector<uint8_t>&bytes,WorldArchive2D&archive,std::string*error){
    auto fail=[&](const char*m){if(error)*error=m;return false;};if(bytes.size()<24)return fail("Archive is truncated");const uint8_t magic[8]={'M','S','P','2','D','A','R','1'};if(!std::equal(magic,magic+8,bytes.begin()))return fail("Invalid archive signature");
    Reader r{bytes};r.pos=8;WorldArchive2D temp;temp.formatVersion=r.U32();temp.stepCount=r.U64();if(temp.formatVersion!=1)return fail("Unsupported world archive version");ReadSettings(r,temp.settings);uint32_t nb=r.U32();if(!r.ok||nb>MaxBodies)return fail("Invalid body count");temp.bodies.resize(nb);for(auto&b:temp.bodies)if(!ReadBody(r,b))return fail("Invalid body or shape data");uint32_t nj=r.U32();if(!r.ok||nj>MaxJoints)return fail("Invalid joint count");temp.joints.resize(nj);for(auto&j:temp.joints)if(!ReadJoint(r,j))return fail("Invalid joint data");uint32_t footer=r.U32();if(!r.ok||footer!=0x4d325f45u||r.pos!=bytes.size())return fail("Archive footer or trailing data is invalid");
    std::sort(temp.bodies.begin(),temp.bodies.end(),[](const ArchivedBody2D&a,const ArchivedBody2D&b){return a.sourceId<b.sourceId;});for(size_t i=1;i<temp.bodies.size();++i)if(temp.bodies[i-1].sourceId==temp.bodies[i].sourceId)return fail("Duplicate body ID");
    for(const auto&j:temp.joints){if(j.enabled){auto has=[&](BodyID id){auto it=std::lower_bound(temp.bodies.begin(),temp.bodies.end(),id,[](const ArchivedBody2D&b,BodyID x){return b.sourceId<x;});return it!=temp.bodies.end()&&it->sourceId==id;};if(j.type==JointType::Mouse){if(!has(j.bodyB))return fail("Mouse joint refers to missing body");}else if(!has(j.bodyA)||!has(j.bodyB))return fail("Joint refers to missing body");}}
    archive=std::move(temp);if(error)error->clear();return true;
}
} // namespace Moss::Physics2D
