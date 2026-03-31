#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

namespace VCX::Labs::RigidBody {

    enum class ShapeType {
        Box,
        Sphere,
        Cylinder,
    };

    struct BodyState {
        Eigen::Vector3f    X;
        Eigen::Quaternionf Q;
    };

    struct RigidBody {
        ShapeType Shape { ShapeType::Box };
        bool      Fixed { false };
        float     Mass { 1.f };
        float     InvMass { 1.f };
        float     Restitution { 0.5f };
        float     Friction { 0.4f };

        Eigen::Vector3f    X { 0.f, 0.f, 0.f };
        Eigen::Quaternionf Q { 1.f, 0.f, 0.f, 0.f };
        Eigen::Vector3f    V { 0.f, 0.f, 0.f };
        Eigen::Vector3f    W { 0.f, 0.f, 0.f };

        Eigen::Vector3f Force { 0.f, 0.f, 0.f };
        Eigen::Vector3f Torque { 0.f, 0.f, 0.f };

        Eigen::Vector3f Dim { 1.f, 1.f, 1.f }; // for box
        float           Radius { 0.5f };       // for sphere/cylinder
        float           Height { 1.f };        // for cylinder
        bool            Sleeping { false };
        float           SleepTimer { 0.f };

        Eigen::Matrix3f InertiaBodyInv { Eigen::Matrix3f::Identity() };

        Eigen::Matrix3f GetWorldInvInertia() const;
        void            UpdateMass(float mass, bool fixed);
        void            UpdateInertiaFromShape();

        void ClearAccumulators();
        void ApplyImpulse(Eigen::Vector3f const & worldPoint, Eigen::Vector3f const & impulse);
    };

    struct Contact {
        int             A { -1 };
        int             B { -1 };
        Eigen::Vector3f Point { 0.f, 0.f, 0.f };
        Eigen::Vector3f Normal { 0.f, 1.f, 0.f }; // points from A to B
        float           Penetration { 0.f };
        float           Restitution { 0.5f };
    };

    struct PointJoint {
        int             A { -1 };
        int             B { -1 };
        Eigen::Vector3f LocalAnchorA { 0.f, 0.f, 0.f };
        Eigen::Vector3f LocalAnchorB { 0.f, 0.f, 0.f };
    };

    class RigidBodySystem {
    public:
        float Gravity { 0.f };
        int   VelocityIterations { 12 };
        int   PositionIterations { 3 };
        float BaumgarteBeta { 0.2f };
        float PenetrationSlop { 1e-3f };
        float RestitutionVelocityThreshold { 0.6f };
        float RestingLinearThreshold { 0.08f };
        float RestingAngularThreshold { 0.12f };
        float SleepTimeThreshold { 0.45f };

        bool EnableCCD { true };
        bool EnableWarmStart { true };
        bool EnableSchurComplement { false };

        std::vector<RigidBody>  Bodies;
        std::vector<Contact>    Contacts;
        std::vector<PointJoint> Joints;

        int  AddBox(Eigen::Vector3f const & dim, Eigen::Vector3f const & x, Eigen::Quaternionf const & q, float mass, bool fixed = false);
        int  AddSphere(float radius, Eigen::Vector3f const & x, Eigen::Quaternionf const & q, float mass, bool fixed = false);
        int  AddCylinder(float radius, float height, Eigen::Vector3f const & x, Eigen::Quaternionf const & q, float mass, bool fixed = false);
        void AddPointJoint(int a, int b, Eigen::Vector3f const & worldAnchor);

        void Clear();
        void Step(float dt);

        std::vector<BodyState> const & GetPrevStates() const { return _prevStates; }

    private:
        std::vector<BodyState>                   _prevStates;
        std::unordered_map<std::uint64_t, float> _cachedNormalImpulse;

        void integrateBodies(float dt);
        void continuousCollisionPass(float dt);
        void detectCollisions();

        void solveContactsSequential(float dt);
        void solveContactsSchur(float dt);
        void solvePositionCorrection();
        void solveJointsSequential(float dt);
        void stabilizeRestingContacts(float dt);

        float getNormalImpulseCache(int a, int b) const;
        void  setNormalImpulseCache(int a, int b, float impulse);

        static std::uint64_t makePairKey(int a, int b);
    };

} // namespace VCX::Labs::RigidBody
