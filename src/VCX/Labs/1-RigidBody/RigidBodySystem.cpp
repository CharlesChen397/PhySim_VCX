#include "Labs/1-RigidBody/RigidBodySystem.h"

#include <algorithm>
#include <cmath>
#include <memory>

#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/cylinder.h>
#include <fcl/geometry/shape/sphere.h>
#include <fcl/narrowphase/collision.h>

namespace VCX::Labs::RigidBody {

    namespace {
        using GeometryPtr = std::shared_ptr<fcl::CollisionGeometry<float>>;

        GeometryPtr createGeometry(RigidBody const & body) {
            switch (body.Shape) {
            case ShapeType::Box:
                return std::make_shared<fcl::Box<float>>(body.Dim.x(), body.Dim.y(), body.Dim.z());
            case ShapeType::Sphere:
                return std::make_shared<fcl::Sphere<float>>(body.Radius);
            case ShapeType::Cylinder:
                return std::make_shared<fcl::Cylinder<float>>(body.Radius, body.Height);
            }
            return std::make_shared<fcl::Box<float>>(1.f, 1.f, 1.f);
        }

        /// FCL 圆柱局部对称轴为 +Z（见 fcl::Cylinder::lz）；惯量与 PrimitiveMesh 以局部 +Y 为轴。
        /// 右乘此旋转将 FCL 局部坐标映射到物体体轴系，使碰撞与网格、惯量一致。
        Eigen::Matrix3f cylinderFclToBodyRotation() {
            return Eigen::AngleAxisf(-float(M_PI) / 2.f, Eigen::Vector3f::UnitX()).toRotationMatrix();
        }

        fcl::Transform3<float> createTransform(Eigen::Vector3f const & x, Eigen::Quaternionf const & q, ShapeType shape) {
            fcl::Transform3<float> tf = fcl::Transform3<float>::Identity();
            Eigen::Matrix3f        R  = q.normalized().toRotationMatrix();
            if (shape == ShapeType::Cylinder) {
                R = R * cylinderFclToBodyRotation();
            }
            tf.linear()      = R;
            tf.translation() = x;
            return tf;
        }

        bool collidePair(
            RigidBody const &             a,
            RigidBody const &             b,
            Eigen::Vector3f const &       xa,
            Eigen::Quaternionf const &    qa,
            Eigen::Vector3f const &       xb,
            Eigen::Quaternionf const &    qb,
            fcl::CollisionResult<float> & result,
            int                           maxContact = 4) {
            auto                         geomA = createGeometry(a);
            auto                         geomB = createGeometry(b);
            fcl::CollisionObject<float>  objA(geomA, createTransform(xa, qa, a.Shape));
            fcl::CollisionObject<float>  objB(geomB, createTransform(xb, qb, b.Shape));
            fcl::CollisionRequest<float> request(maxContact, true);
            result.clear();
            fcl::collide(&objA, &objB, request, result);
            return result.isCollision();
        }

        Eigen::Matrix3f skew(Eigen::Vector3f const & v) {
            Eigen::Matrix3f s;
            s << 0.f, -v.z(), v.y(),
                v.z(), 0.f, -v.x(),
                -v.y(), v.x(), 0.f;
            return s;
        }

        float safeInv(float x) {
            constexpr float eps = 1e-8f;
            return std::abs(x) < eps ? 0.f : 1.f / x;
        }

        /// 将同一物体对上、法向几乎一致的多个 FCL 接触合并为一点，避免面–面时角点顺序冲量产生虚假角速度。
        void appendContactsForPair(
            std::vector<Contact> &           out,
            int                              ia,
            int                              ib,
            RigidBody const &                bodyA,
            RigidBody const &                bodyB,
            std::vector<fcl::Contact<float>> const & fclContacts) {
            std::vector<Contact> batch;
            batch.reserve(fclContacts.size());
            for (auto const & c : fclContacts) {
                Eigen::Vector3f n = c.normal;
                if (n.squaredNorm() < 1e-12f) {
                    continue;
                }
                n.normalize();
                Eigen::Vector3f const ab = bodyB.X - bodyA.X;
                if (ab.dot(n) < 0.f) {
                    n = -n;
                }
                Contact contact;
                contact.A           = ia;
                contact.B           = ib;
                contact.Point       = c.pos;
                contact.Normal      = n;
                contact.Penetration = std::max(0.f, c.penetration_depth);
                contact.Restitution = std::min(bodyA.Restitution, bodyB.Restitution);
                batch.push_back(contact);
            }
            if (batch.empty()) {
                return;
            }
            if (batch.size() == 1) {
                out.push_back(batch[0]);
                return;
            }
            Eigen::Vector3f const nRef = batch[0].Normal;
            for (std::size_t k = 1; k < batch.size(); ++k) {
                if (nRef.dot(batch[k].Normal) < 0.985f) {
                    out.insert(out.end(), batch.begin(), batch.end());
                    return;
                }
            }
            Eigen::Vector3f nSum   = Eigen::Vector3f::Zero();
            Eigen::Vector3f pSum   = Eigen::Vector3f::Zero();
            float             penM = 0.f;
            for (auto const & c : batch) {
                nSum += c.Normal;
                pSum += c.Point;
                penM = std::max(penM, c.Penetration);
            }
            Contact merged;
            merged.A           = ia;
            merged.B           = ib;
            merged.Normal      = nSum.normalized();
            merged.Point       = pSum / static_cast<float>(batch.size());
            merged.Penetration = penM;
            merged.Restitution = batch[0].Restitution;
            out.push_back(merged);
        }
    } // namespace

    Eigen::Matrix3f RigidBody::GetWorldInvInertia() const {
        Eigen::Matrix3f const R = Q.normalized().toRotationMatrix();
        return R * InertiaBodyInv * R.transpose();
    }

    void RigidBody::UpdateMass(float mass, bool fixed) {
        Fixed      = fixed;
        Mass       = fixed ? 0.f : std::max(1e-6f, mass);
        InvMass    = fixed ? 0.f : 1.f / Mass;
        Sleeping   = false;
        SleepTimer = 0.f;
    }

    void RigidBody::UpdateInertiaFromShape() {
        InertiaBodyInv = Eigen::Matrix3f::Zero();
        if (Fixed || InvMass == 0.f) {
            return;
        }

        Eigen::Vector3f diag(1.f, 1.f, 1.f);
        if (Shape == ShapeType::Box) {
            float const wx = Dim.x();
            float const wy = Dim.y();
            float const wz = Dim.z();
            diag.x()       = (Mass / 12.f) * (wy * wy + wz * wz);
            diag.y()       = (Mass / 12.f) * (wx * wx + wz * wz);
            diag.z()       = (Mass / 12.f) * (wx * wx + wy * wy);
        } else if (Shape == ShapeType::Sphere) {
            float const i = 0.4f * Mass * Radius * Radius;
            diag          = Eigen::Vector3f(i, i, i);
        } else {
            float const r2 = Radius * Radius;
            float const h2 = Height * Height;
            float const ix = (Mass / 12.f) * (3.f * r2 + h2);
            float const iy = 0.5f * Mass * r2;
            float const iz = ix;
            diag           = Eigen::Vector3f(ix, iy, iz);
        }

        InertiaBodyInv(0, 0) = safeInv(diag.x());
        InertiaBodyInv(1, 1) = safeInv(diag.y());
        InertiaBodyInv(2, 2) = safeInv(diag.z());
    }

    void RigidBody::ClearAccumulators() {
        Force.setZero();
        Torque.setZero();
    }

    void RigidBody::ApplyImpulse(Eigen::Vector3f const & worldPoint, Eigen::Vector3f const & impulse) {
        if (Fixed) {
            return;
        }
        Sleeping   = false;
        SleepTimer = 0.f;
        V += impulse * InvMass;
        Eigen::Vector3f const r = worldPoint - X;
        W += GetWorldInvInertia() * r.cross(impulse);
    }

    int RigidBodySystem::AddBox(Eigen::Vector3f const & dim, Eigen::Vector3f const & x, Eigen::Quaternionf const & q, float mass, bool fixed) {
        RigidBody body;
        body.Shape = ShapeType::Box;
        body.Dim   = dim;
        body.X     = x;
        body.Q     = q.normalized();
        body.UpdateMass(mass, fixed);
        body.UpdateInertiaFromShape();
        Bodies.push_back(body);
        return static_cast<int>(Bodies.size() - 1);
    }

    int RigidBodySystem::AddSphere(float radius, Eigen::Vector3f const & x, Eigen::Quaternionf const & q, float mass, bool fixed) {
        RigidBody body;
        body.Shape  = ShapeType::Sphere;
        body.Radius = radius;
        body.X      = x;
        body.Q      = q.normalized();
        body.UpdateMass(mass, fixed);
        body.UpdateInertiaFromShape();
        Bodies.push_back(body);
        return static_cast<int>(Bodies.size() - 1);
    }

    int RigidBodySystem::AddCylinder(float radius, float height, Eigen::Vector3f const & x, Eigen::Quaternionf const & q, float mass, bool fixed) {
        RigidBody body;
        body.Shape  = ShapeType::Cylinder;
        body.Radius = radius;
        body.Height = height;
        body.X      = x;
        body.Q      = q.normalized();
        body.UpdateMass(mass, fixed);
        body.UpdateInertiaFromShape();
        Bodies.push_back(body);
        return static_cast<int>(Bodies.size() - 1);
    }

    void RigidBodySystem::AddPointJoint(int a, int b, Eigen::Vector3f const & worldAnchor) {
        if (a < 0 || b < 0 || a >= static_cast<int>(Bodies.size()) || b >= static_cast<int>(Bodies.size())) {
            return;
        }
        PointJoint joint;
        joint.A            = a;
        joint.B            = b;
        joint.LocalAnchorA = Bodies[a].Q.conjugate() * (worldAnchor - Bodies[a].X);
        joint.LocalAnchorB = Bodies[b].Q.conjugate() * (worldAnchor - Bodies[b].X);
        Joints.push_back(joint);
    }

    void RigidBodySystem::Clear() {
        Bodies.clear();
        Contacts.clear();
        Joints.clear();
        _prevStates.clear();
        _cachedNormalImpulse.clear();
    }

    void RigidBodySystem::Step(float dt) {
        if (dt <= 0.f || Bodies.empty()) {
            return;
        }

        if (RelaxOverlapsBeforeIntegrate) {
            relaxOverlappingPositions();
        }

        _prevStates.resize(Bodies.size());
        for (std::size_t i = 0; i < Bodies.size(); ++i) {
            _prevStates[i] = BodyState { .X = Bodies[i].X, .Q = Bodies[i].Q };
        }

        integrateBodies(dt);
        if (EnableCCD) {
            continuousCollisionPass(dt);
        }
        detectCollisions();

        if (EnableSchurComplement) {
            solveContactsSchur(dt);
        } else {
            solveContactsSequential(dt);
            solveJointsSequential(dt);
        }
        solvePositionCorrection();
        stabilizeRestingContacts(dt);

        for (auto & body : Bodies) {
            body.ClearAccumulators();
        }
    }

    void RigidBodySystem::integrateBodies(float dt) {
        for (auto & body : Bodies) {
            if (body.Fixed || body.Sleeping) {
                continue;
            }
            // Translational: explicit Euler (课件 §2, §6)
            Eigen::Vector3f const gravityAccel(0.f, -Gravity, 0.f);
            Eigen::Vector3f const linearAccel = gravityAccel + body.Force * body.InvMass;
            body.V += linearAccel * dt;
            body.X += body.V * dt;

            // Rotational: ω ← ω + Δt · I⁻¹(τ − ω×(Iω)) （课件 §5 欧拉方程）
            Eigen::Matrix3f const R       = body.Q.normalized().toRotationMatrix();
            Eigen::Matrix3f       Ibody = Eigen::Matrix3f::Zero();
            for (int i = 0; i < 3; ++i) {
                float const invIi = body.InertiaBodyInv(i, i);
                if (std::abs(invIi) > 1e-12f) {
                    Ibody(i, i) = 1.f / invIi;
                }
            }
            Eigen::Matrix3f const Iworld = R * Ibody * R.transpose();
            Eigen::Vector3f const L        = Iworld * body.W;
            Eigen::Vector3f const tauNet   = body.Torque - body.W.cross(L);
            Eigen::Vector3f const angularAccel = body.GetWorldInvInertia() * tauNet;
            body.W += angularAccel * dt;

            // Quaternion: Q ← Q + (Δt/2)[0,ω]×Q，再归一化（课件 §4）
            Eigen::Quaternionf const wq(0.f, body.W.x(), body.W.y(), body.W.z());
            body.Q.coeffs() += (0.5f * dt * (wq * body.Q).coeffs());
            body.Q.normalize();
        }
    }

    void RigidBodySystem::relaxOverlappingPositions() {
        constexpr float relaxFactor = 0.48f;
        for (int p = 0; p < RelaxOverlapPasses; ++p) {
            detectCollisions();
            if (Contacts.empty()) {
                break;
            }
            bool moved = false;
            for (auto const & c : Contacts) {
                auto & a = Bodies[c.A];
                auto & b = Bodies[c.B];
                float  depth = std::max(0.f, c.Penetration - PenetrationSlop);
                if (depth <= 0.f) {
                    continue;
                }
                depth = std::min(depth, MaxDepthPerPositionPass);
                float const invMassSum = a.InvMass + b.InvMass;
                if (invMassSum <= 1e-8f) {
                    continue;
                }
                Eigen::Vector3f const corr = (relaxFactor * depth / invMassSum) * c.Normal;
                moved = true;
                if (! a.Fixed) {
                    a.X -= a.InvMass * corr;
                }
                if (! b.Fixed) {
                    b.X += b.InvMass * corr;
                }
            }
            if (! moved) {
                break;
            }
        }
        Contacts.clear();
    }

    void RigidBodySystem::continuousCollisionPass(float dt) {
        // 用「沿相对速度方向的投影厚度」判断是否可跳过 CCD；薄板沿弹道方向很薄时，旧版 min(Dim) 与 0.5 系数仍可能让大步长跳过 CCD 而穿模。
        constexpr float speedThresholdScale = 0.22f;

        auto extentAlongDirection = [](RigidBody const & body, Eigen::Vector3f const & dirUnit) {
            if (body.Shape == ShapeType::Sphere) {
                return body.Radius * 2.f;
            }
            if (body.Shape == ShapeType::Cylinder) {
                return std::min(body.Radius * 2.f, body.Height);
            }
            Eigen::Matrix3f const R  = body.Q.normalized().toRotationMatrix();
            Eigen::Vector3f const ex = R.col(0) * (0.5f * body.Dim.x());
            Eigen::Vector3f const ey = R.col(1) * (0.5f * body.Dim.y());
            Eigen::Vector3f const ez = R.col(2) * (0.5f * body.Dim.z());
            return 2.f * (std::abs(dirUnit.dot(ex)) + std::abs(dirUnit.dot(ey)) + std::abs(dirUnit.dot(ez)));
        };

        for (int i = 0; i < static_cast<int>(Bodies.size()); ++i) {
            for (int j = i + 1; j < static_cast<int>(Bodies.size()); ++j) {
                if (Bodies[i].Fixed && Bodies[j].Fixed) {
                    continue;
                }

                Eigen::Vector3f const relV     = Bodies[i].V - Bodies[j].V;
                float const           relSpeed = relV.norm();
                if (relSpeed <= 1e-8f) {
                    continue;
                }
                Eigen::Vector3f const u        = relV / relSpeed;
                float const           minExtent = std::min(extentAlongDirection(Bodies[i], u), extentAlongDirection(Bodies[j], u));
                if (relSpeed * dt <= speedThresholdScale * minExtent) {
                    continue;
                }

                fcl::CollisionResult<float> prevResult;
                fcl::CollisionResult<float> curResult;
                bool const                  prevCollide = collidePair(Bodies[i], Bodies[j], _prevStates[i].X, _prevStates[i].Q, _prevStates[j].X, _prevStates[j].Q, prevResult, 4);
                bool const                  curCollide  = collidePair(Bodies[i], Bodies[j], Bodies[i].X, Bodies[i].Q, Bodies[j].X, Bodies[j].Q, curResult, 4);
                if (prevCollide || curCollide) {
                    continue;
                }

                bool  found = false;
                float lo    = 0.f;
                float hi    = 1.f;
                for (int k = 0; k < 22; ++k) {
                    float const              mid = 0.5f * (lo + hi);
                    Eigen::Vector3f const    xi  = _prevStates[i].X + mid * (Bodies[i].X - _prevStates[i].X);
                    Eigen::Vector3f const    xj  = _prevStates[j].X + mid * (Bodies[j].X - _prevStates[j].X);
                    Eigen::Quaternionf const qi  = _prevStates[i].Q.slerp(mid, Bodies[i].Q).normalized();
                    Eigen::Quaternionf const qj  = _prevStates[j].Q.slerp(mid, Bodies[j].Q).normalized();

                    fcl::CollisionResult<float> midResult;
                    bool const                  midCollide = collidePair(Bodies[i], Bodies[j], xi, qi, xj, qj, midResult, 4);
                    if (midCollide) {
                        found = true;
                        hi    = mid;
                    } else {
                        lo = mid;
                    }
                }

                if (! found) {
                    float const disp = (Bodies[i].X - _prevStates[i].X).norm() + (Bodies[j].X - _prevStates[j].X).norm();
                    if (disp > 0.15f * minExtent) {
                        for (int s = 1; s <= 32; ++s) {
                            float const              t   = static_cast<float>(s) / 33.f;
                            Eigen::Vector3f const    xi  = _prevStates[i].X + t * (Bodies[i].X - _prevStates[i].X);
                            Eigen::Vector3f const    xj  = _prevStates[j].X + t * (Bodies[j].X - _prevStates[j].X);
                            Eigen::Quaternionf const qi  = _prevStates[i].Q.slerp(t, Bodies[i].Q).normalized();
                            Eigen::Quaternionf const qj  = _prevStates[j].Q.slerp(t, Bodies[j].Q).normalized();
                            fcl::CollisionResult<float> midResult;
                            if (collidePair(Bodies[i], Bodies[j], xi, qi, xj, qj, midResult, 4)) {
                                found = true;
                                hi    = t;
                                break;
                            }
                        }
                    }
                }

                if (found) {
                    float const arcLen = (Bodies[i].X - _prevStates[i].X).norm() + (Bodies[j].X - _prevStates[j].X).norm();
                    float const margin = std::max(1e-4f, std::min(2e-3f, 2e-4f * std::max(1.f, arcLen)));
                    float const toi    = std::max(0.f, hi - margin);
                    Bodies[i].X        = _prevStates[i].X + toi * (Bodies[i].X - _prevStates[i].X);
                    Bodies[j].X        = _prevStates[j].X + toi * (Bodies[j].X - _prevStates[j].X);
                    Bodies[i].Q        = _prevStates[i].Q.slerp(toi, Bodies[i].Q).normalized();
                    Bodies[j].Q        = _prevStates[j].Q.slerp(toi, Bodies[j].Q).normalized();
                }
            }
        }
    }

    void RigidBodySystem::detectCollisions() {
        Contacts.clear();

        for (int i = 0; i < static_cast<int>(Bodies.size()); ++i) {
            for (int j = i + 1; j < static_cast<int>(Bodies.size()); ++j) {
                if (Bodies[i].Fixed && Bodies[j].Fixed) {
                    continue;
                }

                fcl::CollisionResult<float> result;
                if (! collidePair(Bodies[i], Bodies[j], Bodies[i].X, Bodies[i].Q, Bodies[j].X, Bodies[j].Q, result, 8)) {
                    continue;
                }

                std::vector<fcl::Contact<float>> fclContacts;
                result.getContacts(fclContacts);
                appendContactsForPair(Contacts, i, j, Bodies[i], Bodies[j], fclContacts);
            }
        }
    }

    void RigidBodySystem::solveContactsSequential(float dt) {
        if (Contacts.empty()) {
            return;
        }

        for (int iter = 0; iter < VelocityIterations; ++iter) {
            for (auto const & c : Contacts) {
                auto & a = Bodies[c.A];
                auto & b = Bodies[c.B];

                Eigen::Vector3f const ra = c.Point - a.X;
                Eigen::Vector3f const rb = c.Point - b.X;
                Eigen::Vector3f const va = a.V + a.W.cross(ra);
                Eigen::Vector3f const vb = b.V + b.W.cross(rb);
                Eigen::Vector3f const rv = vb - va;

                float const vn = rv.dot(c.Normal);
                float const rawPen           = std::max(0.f, c.Penetration - PenetrationSlop);
                float const penetrationError = std::min(rawPen, BiasPenetrationCap);
                float const biasVel          = std::min(MaxBiasVelocity, BaumgarteBeta * penetrationError / std::max(dt, 1e-6f));
                float const restitution      = (vn < -RestitutionVelocityThreshold) ? c.Restitution : 0.f;

                // If the pair is already separating and there is no meaningful penetration,
                // skip this contact row to avoid injecting energy.
                if (vn > 0.f && rawPen <= 1e-5f) {
                    continue;
                }

                Eigen::Vector3f const iRaN    = a.GetWorldInvInertia() * ra.cross(c.Normal);
                Eigen::Vector3f const iRbN    = b.GetWorldInvInertia() * rb.cross(c.Normal);
                float const           effMass = a.InvMass + b.InvMass + c.Normal.dot(iRaN.cross(ra) + iRbN.cross(rb));
                if (effMass < 1e-8f) {
                    continue;
                }

                float jn = (-(1.f + restitution) * vn + biasVel) / effMass;
                jn       = std::max(0.f, jn);

                // Friction should not be scaled by the penetration-bias part of normal impulse,
                // otherwise deep contacts can receive unrealistically large tangential kicks.
                float const biasImpulsePart = biasVel / effMass;
                float const frictionNormal  = std::max(0.f, jn - biasImpulsePart);

                Eigen::Vector3f const normalImpulse = jn * c.Normal;
                a.ApplyImpulse(c.Point, -normalImpulse);
                b.ApplyImpulse(c.Point, normalImpulse);

                Eigen::Vector3f const va2    = a.V + a.W.cross(ra);
                Eigen::Vector3f const vb2    = b.V + b.W.cross(rb);
                Eigen::Vector3f const rv2    = vb2 - va2;
                Eigen::Vector3f       vt     = rv2 - rv2.dot(c.Normal) * c.Normal;
                float const           vtNorm = vt.norm();
                if (vtNorm > 1e-6f) {
                    Eigen::Vector3f const tangent  = vt / vtNorm;
                    Eigen::Vector3f const iRaT     = a.GetWorldInvInertia() * ra.cross(tangent);
                    Eigen::Vector3f const iRbT     = b.GetWorldInvInertia() * rb.cross(tangent);
                    float const           effMassT = a.InvMass + b.InvMass + tangent.dot(iRaT.cross(ra) + iRbT.cross(rb));
                    if (effMassT > 1e-8f) {
                        float       jt                       = -rv2.dot(tangent) / effMassT;
                        float const mu                       = std::sqrt(std::max(0.f, a.Friction * b.Friction));
                        float const maxFrictionImpulse       = mu * frictionNormal;
                        jt                                   = std::clamp(jt, -maxFrictionImpulse, maxFrictionImpulse);
                        Eigen::Vector3f const tangentImpulse = jt * tangent;
                        a.ApplyImpulse(c.Point, -tangentImpulse);
                        b.ApplyImpulse(c.Point, tangentImpulse);
                    }
                }

                setNormalImpulseCache(c.A, c.B, jn);
            }
        }
    }

    void RigidBodySystem::solveContactsSchur(float dt) {
        struct ConstraintRow {
            int             A { -1 };
            int             B { -1 };
            Eigen::Vector3f N { 0.f, 1.f, 0.f };
            Eigen::Vector3f RA { 0.f, 0.f, 0.f };
            Eigen::Vector3f RB { 0.f, 0.f, 0.f };
            float           Bias { 0.f };
            bool            Unilateral { false };
            Eigen::Vector3f Point { 0.f, 0.f, 0.f };
            float           RowVn { 0.f };
        };

        std::vector<ConstraintRow> rows;
        rows.reserve(Contacts.size() + Joints.size() * 3);

        for (auto const & c : Contacts) {
            auto const &          a                = Bodies[c.A];
            auto const &          b0               = Bodies[c.B];
            Eigen::Vector3f const ra               = c.Point - a.X;
            Eigen::Vector3f const rb               = c.Point - b0.X;
            Eigen::Vector3f const va               = a.V + a.W.cross(ra);
            Eigen::Vector3f const vb               = b0.V + b0.W.cross(rb);
            float const           vn        = (vb - va).dot(c.Normal);
            float const           rawPen    = std::max(0.f, c.Penetration - PenetrationSlop);
            float const           penetrationError = std::min(rawPen, BiasPenetrationCap);
            float const           restitution = (vn < -RestitutionVelocityThreshold) ? c.Restitution : 0.f;
            float                 biasVel     = std::min(MaxBiasVelocity, BaumgarteBeta * penetrationError / std::max(dt, 1e-6f));
            if (restitution < 1e-6f && std::abs(vn) < SchurRestingNormalThreshold && rawPen < 0.05f) {
                biasVel *= SchurRestingBiasScale;
            }
            if (vn > 0.f && rawPen <= 1e-5f) {
                continue;
            }
            float const bias = (1.f + restitution) * vn - biasVel;
            rows.push_back(ConstraintRow {
                .A          = c.A,
                .B          = c.B,
                .N          = c.Normal,
                .RA         = ra,
                .RB         = rb,
                .Bias       = bias,
                .Unilateral = true,
                .Point      = c.Point,
                .RowVn      = vn,
            });
        }

        for (auto const & joint : Joints) {
            auto const &          a   = Bodies[joint.A];
            auto const &          b   = Bodies[joint.B];
            Eigen::Vector3f const ra  = a.Q * joint.LocalAnchorA;
            Eigen::Vector3f const rb  = b.Q * joint.LocalAnchorB;
            Eigen::Vector3f const pa  = a.X + ra;
            Eigen::Vector3f const pb  = b.X + rb;
            Eigen::Vector3f const err = pb - pa;
            Eigen::Vector3f const va  = a.V + a.W.cross(ra);
            Eigen::Vector3f const vb  = b.V + b.W.cross(rb);
            Eigen::Vector3f const rv  = vb - va;

            for (int axis = 0; axis < 3; ++axis) {
                Eigen::Vector3f n = Eigen::Vector3f::Zero();
                n[axis]           = 1.f;
                float const bias  = rv.dot(n) + 0.2f * err.dot(n) / std::max(dt, 1e-6f);
                rows.push_back(ConstraintRow {
                    .A          = joint.A,
                    .B          = joint.B,
                    .N          = n,
                    .RA         = ra,
                    .RB         = rb,
                    .Bias       = bias,
                    .Unilateral = false,
                    .Point      = 0.5f * (pa + pb),
                    .RowVn      = 0.f,
                });
            }
        }

        int const m = static_cast<int>(rows.size());
        if (m == 0) {
            return;
        }

        Eigen::MatrixXf A = Eigen::MatrixXf::Zero(m, m);
        Eigen::VectorXf b = Eigen::VectorXf::Zero(m);

        for (int i = 0; i < m; ++i) {
            b(i) = rows[i].Bias;
        }

        for (int i = 0; i < m; ++i) {
            auto const & ri = rows[i];
            auto const & ai = Bodies[ri.A];
            auto const & bi = Bodies[ri.B];

            Eigen::Vector3f const wiA = ai.GetWorldInvInertia() * ri.RA.cross(ri.N);
            Eigen::Vector3f const wiB = bi.GetWorldInvInertia() * ri.RB.cross(ri.N);

            for (int j = 0; j < m; ++j) {
                auto const & rj = rows[j];
                auto const & aj = Bodies[rj.A];
                auto const & bj = Bodies[rj.B];

                float val = 0.f;
                if (ri.A == rj.A) {
                    val += ai.InvMass * ri.N.dot(rj.N) + wiA.dot(rj.RA.cross(rj.N));
                }
                if (ri.B == rj.B) {
                    val += bi.InvMass * ri.N.dot(rj.N) + wiB.dot(rj.RB.cross(rj.N));
                }
                if (ri.A == rj.B) {
                    val -= ai.InvMass * ri.N.dot(rj.N) + wiA.dot(rj.RB.cross(rj.N));
                }
                if (ri.B == rj.A) {
                    val -= bi.InvMass * ri.N.dot(rj.N) + wiB.dot(rj.RA.cross(rj.N));
                }
                A(i, j) = val;
            }
        }

        A += 2e-4f * Eigen::MatrixXf::Identity(m, m);

        Eigen::VectorXf lambda = A.ldlt().solve(-b);
        float const     gEff = std::max(Gravity, 0.5f);
        for (int i = 0; i < m; ++i) {
            float li = lambda(i);
            if (rows[i].Unilateral) {
                li = std::max(0.f, li);
                auto const & ba = Bodies[rows[i].A];
                auto const & bb = Bodies[rows[i].B];
                float const  mDyn =
                    (ba.InvMass < 1e-8f) ? bb.Mass : ((bb.InvMass < 1e-8f) ? ba.Mass : std::min(ba.Mass, bb.Mass));
                float const capRest = mDyn * gEff * std::max(dt, 1e-5f) * SchurImpulseGravityMult;
                float const vnAbs   = std::abs(rows[i].RowVn);
                float       cap     = std::max(SchurMinNormalImpulseCap, capRest);
                if (vnAbs > SchurRestingNormalThreshold * 1.35f) {
                    float const capColl = mDyn * (vnAbs * 1.25f + 0.8f);
                    cap                 = std::max(cap, std::max(SchurMinNormalImpulseCap * 5.f, capColl));
                }
                li = std::min(li, cap);
            }
            Eigen::Vector3f const impulse = li * rows[i].N;
            Bodies[rows[i].A].ApplyImpulse(rows[i].Point, -impulse);
            Bodies[rows[i].B].ApplyImpulse(rows[i].Point, impulse);
            if (rows[i].Unilateral) {
                setNormalImpulseCache(rows[i].A, rows[i].B, li);
            }
        }
    }

    void RigidBodySystem::solveJointsSequential(float dt) {
        if (Joints.empty()) {
            return;
        }

        float const beta = JointBaumgarteBeta;

        for (int iter = 0; iter < VelocityIterations; ++iter) {
            for (auto const & joint : Joints) {
                auto & a = Bodies[joint.A];
                auto & b = Bodies[joint.B];

                Eigen::Vector3f const ra = a.Q * joint.LocalAnchorA;
                Eigen::Vector3f const rb = b.Q * joint.LocalAnchorB;
                Eigen::Vector3f const pa = a.X + ra;
                Eigen::Vector3f const pb = b.X + rb;

                Eigen::Vector3f const err = pb - pa;
                Eigen::Vector3f const va  = a.V + a.W.cross(ra);
                Eigen::Vector3f const vb  = b.V + b.W.cross(rb);
                Eigen::Vector3f const rv  = vb - va;

                Eigen::Matrix3f const invIA = a.GetWorldInvInertia();
                Eigen::Matrix3f const invIB = b.GetWorldInvInertia();

                Eigen::Matrix3f K = (a.InvMass + b.InvMass) * Eigen::Matrix3f::Identity();
                K -= skew(ra) * invIA * skew(ra);
                K -= skew(rb) * invIB * skew(rb);
                K += 1e-5f * Eigen::Matrix3f::Identity();

                Eigen::Vector3f const rhs     = -(rv + beta * err / std::max(dt, 1e-6f));
                Eigen::Vector3f const impulse = K.ldlt().solve(rhs);

                a.ApplyImpulse(pa, -impulse);
                b.ApplyImpulse(pb, impulse);
            }
        }
    }

    void RigidBodySystem::solvePositionCorrection() {
        for (int iter = 0; iter < PositionIterations; ++iter) {
            for (auto const & c : Contacts) {
                auto &      a          = Bodies[c.A];
                auto &      b          = Bodies[c.B];
                float const invMassSum = a.InvMass + b.InvMass;
                if (invMassSum <= 1e-8f) {
                    continue;
                }
                float depth = std::max(0.f, c.Penetration - PenetrationSlop);
                if (depth <= 0.f) {
                    continue;
                }
                depth                    = std::min(depth, MaxDepthPerPositionPass);
                Eigen::Vector3f const corr = (0.6f * depth / invMassSum) * c.Normal;
                if (! a.Fixed) {
                    a.X -= a.InvMass * corr;
                }
                if (! b.Fixed) {
                    b.X += b.InvMass * corr;
                }
            }
        }
    }

    void RigidBodySystem::stabilizeRestingContacts(float dt) {
        std::vector<bool> supportContact(Bodies.size(), false);

        auto isStableSupport = [this](RigidBody const & body) {
            if (body.Fixed || body.Sleeping) {
                return true;
            }
            float const v2 = body.V.squaredNorm();
            float const w2 = body.W.squaredNorm();
            return v2 < std::pow(4.f * RestingLinearThreshold, 2.f) && w2 < std::pow(4.f * RestingAngularThreshold, 2.f);
        };

        for (auto const & c : Contacts) {
            auto const & a = Bodies[c.A];
            auto const & b = Bodies[c.B];

            if (! a.Fixed && c.Normal.y() < -0.5f && isStableSupport(b)) {
                supportContact[c.A] = true;
            }
            if (! b.Fixed && c.Normal.y() > 0.5f && isStableSupport(a)) {
                supportContact[c.B] = true;
            }
        }

        for (std::size_t i = 0; i < Bodies.size(); ++i) {
            auto & body = Bodies[i];
            if (body.Fixed) {
                continue;
            }

            float const linearSpeed  = body.V.norm();
            float const angularSpeed = body.W.norm();
            bool const  nearResting  = linearSpeed < RestingLinearThreshold && angularSpeed < RestingAngularThreshold;

            if (supportContact[i] && linearSpeed < 2.f * RestingLinearThreshold && angularSpeed < 2.f * RestingAngularThreshold) {
                if (body.V.y() > 0.f || std::abs(body.V.y()) < RestitutionVelocityThreshold) {
                    body.V.y() = 0.f;
                }
                if (std::abs(body.V.x()) < RestingLinearThreshold) {
                    body.V.x() = 0.f;
                } else {
                    body.V.x() *= 0.6f;
                }
                if (std::abs(body.V.z()) < RestingLinearThreshold) {
                    body.V.z() = 0.f;
                } else {
                    body.V.z() *= 0.6f;
                }
                body.W *= 0.65f;
                if (nearResting) {
                    body.V.setZero();
                    body.W.setZero();
                }
            }

            if (supportContact[i] && nearResting) {
                body.SleepTimer += dt;
                if (body.SleepTimer >= SleepTimeThreshold) {
                    body.Sleeping = true;
                    body.V.setZero();
                    body.W.setZero();
                }
            } else {
                body.SleepTimer = 0.f;
                if (linearSpeed > 2.f * RestingLinearThreshold || angularSpeed > 2.f * RestingAngularThreshold) {
                    body.Sleeping = false;
                }
            }
        }
    }

    float RigidBodySystem::getNormalImpulseCache(int a, int b) const {
        auto const key = makePairKey(a, b);
        auto       it  = _cachedNormalImpulse.find(key);
        return it == _cachedNormalImpulse.end() ? 0.f : it->second;
    }

    void RigidBodySystem::setNormalImpulseCache(int a, int b, float impulse) {
        _cachedNormalImpulse[makePairKey(a, b)] = impulse;
    }

    std::uint64_t RigidBodySystem::makePairKey(int a, int b) {
        if (a > b) {
            std::swap(a, b);
        }
        return (std::uint64_t(std::uint32_t(a)) << 32U) | std::uint64_t(std::uint32_t(b));
    }

} // namespace VCX::Labs::RigidBody
