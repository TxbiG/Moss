#pragma once
#include "Physics2D.hpp"
#include <algorithm>
#include <cmath>
#include <cmath>
#include <vector>

enum class VehicleDifferentialMode2D : uint8_t { Open, Locked, LimitedSlip };

struct MOSS_API EngineTorquePoint2D { float rpm=0.0f,torque=0.0f; };

struct MOSS_API VehicleWheelDef2D {
    Vec2 localAnchor{};
    Vec2 localSuspensionDirection{0,-1};
    float suspensionLength=0.45f, radius=0.22f;
    float springFrequencyHz=5.0f, dampingRatio=0.8f, friction=1.2f;
    float maxSuspensionForce=2000.0f;
    float wheelInertia=1.2f;
    float longitudinalStiffness=7.0f, corneringStiffness=8.0f;
    float rollingResistance=0.015f;
    float driveWeight=1.0f;
    float longitudinalRelaxationLength=0.25f,lateralRelaxationLength=0.35f;
    float loadSensitivity=0.08f,referenceLoad=350.0f;
    float optimumTemperature=85.0f,temperatureRange=100.0f;
    float tyreHeatCapacity=0.0008f,tyreCoolingRate=0.025f,tyreWearRate=0.0000002f;
    bool steerable=false, driven=true;
};

struct MOSS_API VehicleSettings2D {
    Vec2 localForwardAxis{1,0};
    float maxSteeringAngle=0.45f;
    float maxEngineTorque=360.0f;
    float maxEnginePower=90000.0f;
    float idleRPM=850.0f, redlineRPM=6500.0f;
    float engineInertia=0.28f,engineDragTorque=16.0f,idleGovernorStrength=0.8f;
    float clutchCapacity=520.0f,clutchStiffness=45.0f,clutchEngagement=1.0f;
    std::vector<EngineTorquePoint2D> engineTorqueCurve{};
    float ambientTemperature=20.0f;
    float finalDriveRatio=3.7f, drivetrainEfficiency=0.88f;
    float reverseRatio=3.2f;
    std::vector<float> forwardGearRatios{3.2f,2.1f,1.5f,1.15f,0.9f};
    float maxBrakeTorque=1800.0f;
    float upshiftRPM=5600.0f, downshiftRPM=1900.0f;
    float antiRollStiffness=0.0f;
    VehicleDifferentialMode2D differentialMode=VehicleDifferentialMode2D::Open;
    float limitedSlipStrength=0.65f;
    float limitedSlipMaxBias=3.0f;
    float differentialLockTorque=900.0f;
    bool automaticTransmission=true;
};

struct MOSS_API VehicleWheelState2D {
    bool grounded=false;
    BodyID groundBody=InvalidBodyID;
    Vec2 contactPoint{},contactNormal{0,1};
    float suspensionCompression=0,suspensionForce=0;
    float longitudinalSpeed=0,lateralSpeed=0;
    float slipRatio=0,slipAngle=0;
    float filteredSlipRatio=0,filteredSlipAngle=0;
    float temperature=20.0f,wear=0.0f;
    float angularVelocity=0,rotation=0;
    float engineTorque=0,brakeTorque=0,longitudinalForce=0,lateralForce=0;
};

// 2D raycast-suspension vehicle with wheel rotational inertia, tyre slip and a
// simplified engine/gearbox/final-drive/differential model. It is a gameplay
// vehicle model, not a high-fidelity combustion or tyre-temperature simulator.
class MOSS_API Vehicle2D {
public:
    Vehicle2D(PhysicsWorld2D& world,BodyID chassis,const std::vector<VehicleWheelDef2D>& wheels,const VehicleSettings2D& settings={})
        :world_(&world),chassis_(chassis),defs_(wheels),states_(wheels.size()),settings_(settings) {
            settings_.idleRPM=std::max(0.0f,std::isfinite(settings_.idleRPM)?settings_.idleRPM:850.0f);
            settings_.redlineRPM=std::max(settings_.idleRPM+100.0f,std::isfinite(settings_.redlineRPM)?settings_.redlineRPM:6500.0f);
            settings_.maxEngineTorque=std::max(0.0f,std::isfinite(settings_.maxEngineTorque)?settings_.maxEngineTorque:0.0f);
            settings_.engineInertia=std::max(0.001f,std::isfinite(settings_.engineInertia)?settings_.engineInertia:0.28f);
            settings_.finalDriveRatio=std::max(0.001f,std::isfinite(settings_.finalDriveRatio)?settings_.finalDriveRatio:1.0f);
            settings_.engineTorqueCurve.erase(std::remove_if(settings_.engineTorqueCurve.begin(),settings_.engineTorqueCurve.end(),[](const EngineTorquePoint2D&p){return !std::isfinite(p.rpm)||!std::isfinite(p.torque)||p.rpm<0.0f;}),settings_.engineTorqueCurve.end());
            std::stable_sort(settings_.engineTorqueCurve.begin(),settings_.engineTorqueCurve.end(),[](const EngineTorquePoint2D&a,const EngineTorquePoint2D&b){return a.rpm<b.rpm;});
            engineOmega_=std::max(1.0f,settings_.idleRPM)*2.0f*kPi/60.0f;
            for(auto& state:states_)state.temperature=settings_.ambientTemperature;
        }

    void SetThrottle(float value){throttle_=Clamp(value,-1.0f,1.0f);}
    void SetBrake(float value){brake_=Clamp(value,0.0f,1.0f);}
    void SetSteering(float value){steering_=Clamp(value,-1.0f,1.0f);}
    void SetAutomaticTransmission(bool enabled){settings_.automaticTransmission=enabled;}
    void SetDifferentialMode(VehicleDifferentialMode2D mode){settings_.differentialMode=mode;}
    bool SetWheelAngularVelocity(size_t wheel,float velocity){if(wheel>=states_.size()||!std::isfinite(velocity))return false;states_[wheel].angularVelocity=velocity;return true;}
    bool SetGear(int gear){if(gear < -1 || gear > int(settings_.forwardGearRatios.size()))return false;gear_=gear;return true;}
    void ShiftUp(){gear_=std::min(gear_+1,int(settings_.forwardGearRatios.size()));}
    void ShiftDown(){gear_=std::max(-1,gear_-1);}
    int GetGear()const{return gear_;}
    float GetEngineRPM()const{return engineRPM_;}
    float GetEngineTorque()const{return engineTorque_;}
    float GetSpeed()const{return speed_;}
    const std::vector<VehicleWheelState2D>& GetWheelStates()const{return states_;}

    void Update(float dt){
        if(!world_||!std::isfinite(dt)||dt<=0.0f)return;
        const BodyState* chassis=world_->GetBody(chassis_);if(!chassis||chassis->type!=BodyType::Dynamic)return;
        const Transform2D xf=chassis->transform;
        const Vec2 forwardBase=NormalizeOr(Rot2(xf.angle).Apply(settings_.localForwardAxis));
        speed_=Dot(chassis->linearVelocity,forwardBase);
        float drivenOmega=0.0f,drivenWeight=0.0f,maxDrivenOmega=0.0f;
        size_t drivenCount=0;
        for(size_t i=0;i<defs_.size();++i)if(defs_[i].driven){
            const float weight=std::max(0.0f,defs_[i].driveWeight);
            drivenOmega+=states_[i].angularVelocity*weight;drivenWeight+=weight;
            maxDrivenOmega=std::max(maxDrivenOmega,std::abs(states_[i].angularVelocity));++drivenCount;
        }
        const float wheelOmega=drivenWeight>kEpsilon?drivenOmega/drivenWeight:0.0f;
        float ratio=CurrentGearRatio();
        engineRPM_=std::abs(engineOmega_)*60.0f/(2.0f*kPi);
        if(settings_.automaticTransmission&&gear_>0){
            if(engineRPM_>settings_.upshiftRPM&&gear_<int(settings_.forwardGearRatios.size())){++gear_;ratio=CurrentGearRatio();}
            else if(engineRPM_<settings_.downshiftRPM&&gear_>1){--gear_;ratio=CurrentGearRatio();}
        }
        engineRPM_=Clamp(engineRPM_,settings_.idleRPM,settings_.redlineRPM);
        const float engineOmega=std::max(engineRPM_*2.0f*kPi/60.0f,1.0f);
        engineTorque_=TorqueAtRPM(engineRPM_)*std::abs(throttle_);
        engineTorque_=std::min(engineTorque_,std::max(0.0f,settings_.maxEnginePower)/engineOmega);
        const float targetEngineOmega=wheelOmega*ratio*settings_.finalDriveRatio;
        const float clutchSlip=engineOmega_-targetEngineOmega;
        const float clutchTorque=gear_==0?0.0f:Clamp(clutchSlip*std::max(0.0f,settings_.clutchStiffness),-std::max(0.0f,settings_.clutchCapacity),std::max(0.0f,settings_.clutchCapacity))*Clamp(settings_.clutchEngagement,0.0f,1.0f);
        const float idleGovernor=(settings_.idleRPM*2.0f*kPi/60.0f-engineOmega_)*std::max(0.0f,settings_.idleGovernorStrength)*std::max(0.0f,settings_.engineInertia);
        const float drag=settings_.engineDragTorque*(engineOmega_>0.0f?1.0f:-1.0f);
        engineOmega_=Clamp(engineOmega_+(engineTorque_+idleGovernor-clutchTorque-drag)/std::max(settings_.engineInertia,0.001f)*dt,0.0f,settings_.redlineRPM*2.0f*kPi/60.0f);
        engineRPM_=Clamp(engineOmega_*60.0f/(2.0f*kPi),0.0f,settings_.redlineRPM);
        // Clutch slip is solved with finite capacity and engine inertia, rather
        // than deriving RPM algebraically from wheel speed. This supports launch,
        // throttle blips, clutch slip and engine braking.
        float totalDriveTorque=clutchTorque*ratio*settings_.finalDriveRatio*Clamp(settings_.drivetrainEfficiency,0.0f,1.0f);
        // Differential torque weights are computed from wheel speed. An open
        // differential shares torque evenly; an LSD biases torque toward the
        // slower wheel; a locked differential additionally couples wheel speeds.
        std::vector<float> differentialWeights(defs_.size(),0.0f);
        float differentialWeightSum=0.0f;
        for(size_t i=0;i<defs_.size();++i)if(defs_[i].driven){
            float weight=1.0f;
            if(settings_.differentialMode==VehicleDifferentialMode2D::LimitedSlip){
                const float speedRange=std::max(maxDrivenOmega,1.0f);
                const float slowFactor=1.0f-std::abs(states_[i].angularVelocity)/speedRange;
                const float strength=Clamp(settings_.limitedSlipStrength,0.0f,1.0f);
                const float bias=1.0f+strength*std::max(0.0f,slowFactor)*(std::max(1.0f,settings_.limitedSlipMaxBias)-1.0f);
                weight=bias*std::max(0.0f,defs_[i].driveWeight);
            } else if(settings_.differentialMode==VehicleDifferentialMode2D::Open){
                weight=std::max(0.0f,defs_[i].driveWeight);
            }
            differentialWeights[i]=weight;differentialWeightSum+=weight;
        }

        float compressionLeft=0.0f,compressionRight=0.0f;int leftCount=0,rightCount=0;
        for(size_t i=0;i<defs_.size();++i){
            const auto& def=defs_[i];auto& state=states_[i];state.grounded=false;state.groundBody=InvalidBodyID;state.suspensionForce=0;state.longitudinalForce=state.lateralForce=0;
            state.engineTorque=def.driven&&differentialWeightSum>kEpsilon?totalDriveTorque*differentialWeights[i]/differentialWeightSum:0.0f;
            state.brakeTorque=std::max(0.0f,settings_.maxBrakeTorque)*brake_;
            const Vec2 anchor=xf.Apply(def.localAnchor);const Vec2 suspension=NormalizeOr(Rot2(xf.angle).Apply(def.localSuspensionDirection),{0,-1});
            const float castLength=std::max(0.0f,def.suspensionLength)+std::max(0.0f,def.radius);
            auto hit=world_->RayCast(anchor,anchor+suspension*castLength,{},chassis_);
            if(!hit){
                const float brakeSign=std::abs(state.angularVelocity)>0.05f?(state.angularVelocity>0?1.0f:-1.0f):(state.engineTorque>=0?1.0f:-1.0f);
                const float netTorque=state.engineTorque-state.brakeTorque*brakeSign;
                state.angularVelocity+=netTorque/std::max(def.wheelInertia,0.001f)*dt;
                state.temperature+=(-std::max(0.0f,def.tyreCoolingRate)*(state.temperature-settings_.ambientTemperature))*dt;
                if(brake_>0.0f&&std::abs(state.angularVelocity)<0.1f&&std::abs(state.engineTorque)<state.brakeTorque)state.angularVelocity=0.0f;
                continue;
            }
            const float along=castLength*hit->fraction;
            const float compression=Clamp(def.suspensionLength-(along-def.radius),0.0f,std::max(0.0f,def.suspensionLength));
            state.grounded=true;state.groundBody=hit->body;state.contactPoint=hit->point;state.contactNormal=NormalizeOr(hit->normal,{0,1});state.suspensionCompression=compression;
            if(def.localAnchor.x<0){compressionLeft+=compression;++leftCount;}else{compressionRight+=compression;++rightCount;}
            const BodyState* ground=world_->GetBody(hit->body);Vec2 groundVelocity{};
            if(ground)groundVelocity=ground->linearVelocity+Cross(ground->angularVelocity,hit->point-ground->worldCenter);
            const Vec2 chassisVelocity=chassis->linearVelocity+Cross(chassis->angularVelocity,hit->point-chassis->worldCenter);
            const Vec2 relativeVelocity=chassisVelocity-groundVelocity;
            const float normalSpeed=Dot(relativeVelocity,state.contactNormal);
            const float omega=2.0f*kPi*std::max(0.0f,def.springFrequencyHz);
            const float wheelMass=chassis->mass/float(std::max<size_t>(1,defs_.size()));
            const float springK=wheelMass*omega*omega,damper=2.0f*wheelMass*std::max(0.0f,def.dampingRatio)*omega;
            const float normalForce=Clamp(springK*compression-damper*normalSpeed,0.0f,std::max(0.0f,def.maxSuspensionForce));
            state.suspensionForce=normalForce;const Vec2 spring=state.contactNormal*normalForce;
            world_->ApplyForce(chassis_,spring,hit->point,true);if(ground&&ground->type==BodyType::Dynamic)world_->ApplyForce(hit->body,-spring,hit->point,true);

            const float steerAngle=def.steerable?steering_*settings_.maxSteeringAngle:0.0f;
            const Vec2 forward=NormalizeOr(Rot2(xf.angle+steerAngle).Apply(settings_.localForwardAxis));const Vec2 side=forward.Perpendicular();
            state.longitudinalSpeed=Dot(relativeVelocity,forward);state.lateralSpeed=Dot(relativeVelocity,side);
            const float rawSlipRatio=(state.angularVelocity*def.radius-state.longitudinalSpeed)/std::max(std::abs(state.longitudinalSpeed),1.0f);
            const float rawSlipAngle=std::atan2(state.lateralSpeed,std::max(std::abs(state.longitudinalSpeed),0.5f));
            const float longAlpha=1.0f-std::exp(-std::abs(state.longitudinalSpeed)*dt/std::max(def.longitudinalRelaxationLength,0.001f));
            const float latAlpha=1.0f-std::exp(-std::abs(state.longitudinalSpeed)*dt/std::max(def.lateralRelaxationLength,0.001f));
            state.filteredSlipRatio+=Clamp(longAlpha,0.0f,1.0f)*(rawSlipRatio-state.filteredSlipRatio);
            state.filteredSlipAngle+=Clamp(latAlpha,0.0f,1.0f)*(rawSlipAngle-state.filteredSlipAngle);
            state.slipRatio=state.filteredSlipRatio;state.slipAngle=state.filteredSlipAngle;
            float brakeSign=std::abs(state.angularVelocity)>0.05f?(state.angularVelocity>0?1.0f:-1.0f):((state.engineTorque>=0)?1.0f:-1.0f);
            const float longitudinalDemand=def.longitudinalStiffness*state.slipRatio;
            const float lateralDemand=-def.corneringStiffness*state.slipAngle;
            const float loadFactor=std::pow(std::max(normalForce/std::max(def.referenceLoad,1.0f),0.05f),-std::max(0.0f,def.loadSensitivity));
            const float temperatureFactor=Clamp(1.0f-std::abs(state.temperature-def.optimumTemperature)/std::max(def.temperatureRange,1.0f)*0.55f,0.4f,1.0f);
            const float wearFactor=Clamp(1.0f-state.wear*0.45f,0.45f,1.0f);
            const float maxForce=std::max(0.0f,def.friction)*normalForce*loadFactor*temperatureFactor*wearFactor;
            const float demand=std::sqrt(longitudinalDemand*longitudinalDemand+lateralDemand*lateralDemand);
            const float scale=demand>maxForce&&demand>kEpsilon?maxForce/demand:1.0f;
            state.longitudinalForce=longitudinalDemand*scale;state.lateralForce=lateralDemand*scale;
            const Vec2 tyreForce=forward*state.longitudinalForce+side*state.lateralForce;
            world_->ApplyForce(chassis_,tyreForce,hit->point,true);if(ground&&ground->type==BodyType::Dynamic)world_->ApplyForce(hit->body,-tyreForce,hit->point,true);
            const float longitudinalSlipSpeed=state.angularVelocity*def.radius-state.longitudinalSpeed;
            const float slipPower=std::abs(state.longitudinalForce*longitudinalSlipSpeed)+std::abs(state.lateralForce*state.lateralSpeed);
            state.temperature+=((slipPower*std::max(0.0f,def.tyreHeatCapacity))-std::max(0.0f,def.tyreCoolingRate)*(state.temperature-settings_.ambientTemperature))*dt;
            state.temperature=Clamp(state.temperature,settings_.ambientTemperature,settings_.ambientTemperature+std::max(40.0f,def.temperatureRange*2.0f));
            state.wear=Clamp(state.wear+slipPower*std::max(0.0f,def.tyreWearRate)*dt,0.0f,1.0f);
            const float rolling=def.rollingResistance*state.suspensionForce*def.radius*brakeSign;
            const float tyreReaction=state.longitudinalForce*def.radius;
            const float netTorque=state.engineTorque-tyreReaction-state.brakeTorque*brakeSign-rolling;
            state.angularVelocity+=netTorque/std::max(def.wheelInertia,0.001f)*dt;
            if(brake_>0.0f&&std::abs(state.angularVelocity)<0.1f&&std::abs(state.engineTorque)<state.brakeTorque)state.angularVelocity=0.0f;
        }
        if(settings_.differentialMode==VehicleDifferentialMode2D::Locked&&drivenCount>1){
            float meanOmega=0.0f,totalInertia=0.0f;
            for(size_t i=0;i<defs_.size();++i)if(defs_[i].driven){const float inertia=std::max(defs_[i].wheelInertia,0.001f);meanOmega+=states_[i].angularVelocity*inertia;totalInertia+=inertia;}
            if(totalInertia>kEpsilon){meanOmega/=totalInertia;const float lockTorque=std::max(0.0f,settings_.differentialLockTorque);
                for(size_t i=0;i<defs_.size();++i)if(defs_[i].driven){const float inertia=std::max(defs_[i].wheelInertia,0.001f);const float maxDelta=lockTorque*dt/inertia;states_[i].angularVelocity+=Clamp(meanOmega-states_[i].angularVelocity,-maxDelta,maxDelta);}}
        }
        for(auto& state:states_)state.rotation=WrapAngle(state.rotation+state.angularVelocity*dt);
        if(settings_.antiRollStiffness>0.0f&&leftCount>0&&rightCount>0&&states_.size()>=2){
            const float left=compressionLeft/float(leftCount),right=compressionRight/float(rightCount);
            const float anti=(left-right)*settings_.antiRollStiffness;
            const Vec2 up=NormalizeOr(-Rot2(xf.angle).Apply({0,-1}),{0,1});
            const Vec2 impulse=up*(anti*dt);
            world_->ApplyLinearImpulse(chassis_,impulse,chassis->worldCenter,true);
        }
    }

private:
    float TorqueAtRPM(float rpm)const{
        if(settings_.engineTorqueCurve.empty())return std::max(0.0f,settings_.maxEngineTorque);
        const auto& curve=settings_.engineTorqueCurve;if(rpm<=curve.front().rpm)return std::max(0.0f,curve.front().torque);
        for(size_t i=1;i<curve.size();++i)if(rpm<=curve[i].rpm){const float span=std::max(curve[i].rpm-curve[i-1].rpm,0.001f);const float t=Clamp((rpm-curve[i-1].rpm)/span,0.0f,1.0f);return std::max(0.0f,curve[i-1].torque+(curve[i].torque-curve[i-1].torque)*t);}
        return std::max(0.0f,curve.back().torque);
    }
    float CurrentGearRatio()const{
        if(gear_==0)return 0.0f;if(gear_==-1)return -std::abs(settings_.reverseRatio);
        const size_t index=size_t(gear_-1);return index<settings_.forwardGearRatios.size()?settings_.forwardGearRatios[index]:0.0f;
    }
    PhysicsWorld2D* world_=nullptr;BodyID chassis_=InvalidBodyID;
    std::vector<VehicleWheelDef2D> defs_;std::vector<VehicleWheelState2D> states_;VehicleSettings2D settings_{};
    float throttle_=0.0f,brake_=0.0f,steering_=0.0f,engineRPM_=0.0f,engineTorque_=0.0f,speed_=0.0f;
    int gear_=1;
    float engineOmega_=0.0f;
};
