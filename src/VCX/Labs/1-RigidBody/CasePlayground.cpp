#include "Labs/1-RigidBody/CasePlayground.h"

#include <algorithm>
#include <cmath>

#include "Engine/app.h"
#include "Labs/1-RigidBody/PrimitiveMesh.h"

namespace VCX::Labs::RigidBody {

    namespace {
        glm::vec3 eigen2glm(Eigen::Vector3f const & v) {
            return glm::vec3(v.x(), v.y(), v.z());
        }

        Eigen::Vector3f glm2eigen(glm::vec3 const & v) {
            return Eigen::Vector3f(v.x, v.y, v.z);
        }
    }

    CasePlayground::CasePlayground():
        _program(Engine::GL::UniqueProgram({
            Engine::GL::SharedShader("assets/shaders/flat.vert"),
            Engine::GL::SharedShader("assets/shaders/flat.frag"),
        })),
        _solidItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
        _wireItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines),
        _impulseItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines) {
        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);
        resetPreset();
    }

    void CasePlayground::OnSetupPropsUI() {
        int preset = static_cast<int>(_preset);
        if (ImGui::Combo("Scenario", &preset, _presetNames.data(), static_cast<int>(_presetNames.size()))) {
            _preset      = static_cast<Preset>(preset);
            _presetDirty = true;
        }

        if (ImGui::Button("Reset Preset")) {
            _presetDirty = true;
        }
        ImGui::SameLine();
        if (ImGui::Button(_stopped ? "Start Simulation" : "Stop Simulation")) {
            _stopped = ! _stopped;
        }

        ImGui::Separator();

        int const maxBody = std::max(0, static_cast<int>(_system.Bodies.size()) - 1);
        _selectedBody     = std::clamp(_selectedBody, 0, maxBody);
        ImGui::SliderInt("Selected Body", &_selectedBody, 0, maxBody);
        ImGui::SliderFloat("Impulse Strength", &_userImpulse, 0.f, 20.f);

        if (_selectedBody < static_cast<int>(_system.Bodies.size())) {
            auto & b      = _system.Bodies[_selectedBody];
            bool   edited = false;
            edited |= ImGui::SliderFloat("Linear Vx", &b.V.x(), -12.f, 12.f);
            edited |= ImGui::SliderFloat("Linear Vy", &b.V.y(), -12.f, 12.f);
            edited |= ImGui::SliderFloat("Linear Vz", &b.V.z(), -12.f, 12.f);
            edited |= ImGui::SliderFloat("Angular Wx", &b.W.x(), -20.f, 20.f);
            edited |= ImGui::SliderFloat("Angular Wy", &b.W.y(), -20.f, 20.f);
            edited |= ImGui::SliderFloat("Angular Wz", &b.W.z(), -20.f, 20.f);
            if (edited) {
                b.Sleeping   = false;
                b.SleepTimer = 0.f;
            }
        }

        auto applySelectedImpulse = [&](Eigen::Vector3f const & dir) {
            if (_selectedBody >= static_cast<int>(_system.Bodies.size())) {
                return;
            }
            auto & b = _system.Bodies[_selectedBody];
            applyImpulseWithViz(_selectedBody, b.X, _userImpulse * dir);
        };

        if (ImGui::Button("Impulse +X")) applySelectedImpulse(Eigen::Vector3f(1.f, 0.f, 0.f));
        ImGui::SameLine();
        if (ImGui::Button("Impulse -X")) applySelectedImpulse(Eigen::Vector3f(-1.f, 0.f, 0.f));

        if (ImGui::Button("Impulse +Y")) applySelectedImpulse(Eigen::Vector3f(0.f, 1.f, 0.f));
        ImGui::SameLine();
        if (ImGui::Button("Impulse -Y")) applySelectedImpulse(Eigen::Vector3f(0.f, -1.f, 0.f));

        if (ImGui::Button("Impulse +Z")) applySelectedImpulse(Eigen::Vector3f(0.f, 0.f, 1.f));
        ImGui::SameLine();
        if (ImGui::Button("Impulse -Z")) applySelectedImpulse(Eigen::Vector3f(0.f, 0.f, -1.f));

        ImGui::Separator();
        ImGui::SliderFloat("Gravity", &_system.Gravity, 0.f, 20.f);
    }

    Common::CaseRenderResult CasePlayground::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (_presetDirty) {
            resetPreset();
        }

        applyUserInteraction(Engine::GetDeltaTime());
        if (! _stopped) {
            stepSimulation(Engine::GetDeltaTime());
        }

        std::vector<glm::vec3>     positions;
        std::vector<std::uint32_t> triangles;
        std::vector<std::uint32_t> lines;

        for (auto const & body : _system.Bodies) {
            MeshData            mesh = BuildBodyMesh(body);
            std::uint32_t const base = static_cast<std::uint32_t>(positions.size());

            for (auto const & v : mesh.Vertices) {
                positions.push_back(eigen2glm(v));
            }
            for (auto idx : mesh.Triangles) {
                triangles.push_back(base + idx);
            }
            for (auto idx : mesh.Lines) {
                lines.push_back(base + idx);
            }
        }

        if (! positions.empty()) {
            _solidItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(positions));
            _solidItem.UpdateElementBuffer(triangles);
            _wireItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(positions));
            _wireItem.UpdateElementBuffer(lines);
        }

        float const frameDt = Engine::GetDeltaTime();
        if (! _stopped && _impulseVizTimer > 0.f) {
            _impulseVizTimer = std::max(0.f, _impulseVizTimer - frameDt);
        }
        if (_impulseVizTimer <= 0.f) {
            _hasImpulseViz = false;
        }

        if (_hasImpulseViz && _showImpulseViz) {
            std::vector<glm::vec3>     impulseVerts;
            std::vector<std::uint32_t> impulseLines;
            impulseVerts.reserve(7);
            impulseLines.reserve(12);

            Eigen::Vector3f const j  = _lastImpulseVector;
            float const           jn = j.norm();
            if (jn > 1e-6f) {
                Eigen::Vector3f const dir = j / jn;
                float const           len = std::clamp(jn * _impulseVizScale, 0.25f, 3.0f);

                Eigen::Vector3f const p0 = _lastImpulsePoint;
                Eigen::Vector3f const p1 = p0 + dir * len;

                Eigen::Vector3f axis = (std::abs(dir.y()) < 0.95f) ? Eigen::Vector3f::UnitY() : Eigen::Vector3f::UnitX();
                Eigen::Vector3f side = dir.cross(axis);
                if (side.squaredNorm() < 1e-8f) {
                    axis = Eigen::Vector3f::UnitZ();
                    side = dir.cross(axis);
                }
                side.normalize();
                Eigen::Vector3f up = side.cross(dir).normalized();

                float const           headLen = 0.22f * len;
                float const           headWid = 0.14f * len;
                Eigen::Vector3f const h1      = p1 - dir * headLen + side * headWid;
                Eigen::Vector3f const h2      = p1 - dir * headLen - side * headWid;
                Eigen::Vector3f const h3      = p1 - dir * headLen + up * headWid;
                Eigen::Vector3f const h4      = p1 - dir * headLen - up * headWid;

                impulseVerts.push_back(eigen2glm(p0));
                impulseVerts.push_back(eigen2glm(p1));
                impulseVerts.push_back(eigen2glm(h1));
                impulseVerts.push_back(eigen2glm(h2));
                impulseVerts.push_back(eigen2glm(h3));
                impulseVerts.push_back(eigen2glm(h4));

                impulseLines = {
                    0,
                    1,
                    1,
                    2,
                    1,
                    3,
                    1,
                    4,
                    1,
                    5,
                    0,
                    2,
                };

                _impulseItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(impulseVerts));
                _impulseItem.UpdateElementBuffer(impulseLines);
            }
        }

        _frame.Resize(desiredSize);
        _cameraManager.Update(_camera);

        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix(float(desiredSize.first) / float(desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LINE_SMOOTH);

        _program.GetUniforms().SetByName("u_Color", glm::vec3(0.68f, 0.85f, 0.78f));
        _solidItem.Draw({ _program.Use() });

        if (_showWireframe) {
            glLineWidth(1.f);
            _program.GetUniforms().SetByName("u_Color", glm::vec3(0.1f, 0.1f, 0.1f));
            _wireItem.Draw({ _program.Use() });
        }

        if (_hasImpulseViz && _showImpulseViz) {
            glDisable(GL_DEPTH_TEST);
            glLineWidth(2.5f);
            _program.GetUniforms().SetByName("u_Color", glm::vec3(0.95f, 0.25f, 0.2f));
            _impulseItem.Draw({ _program.Use() });
            glLineWidth(1.f);
            glEnable(GL_DEPTH_TEST);
        }

        glDisable(GL_LINE_SMOOTH);
        glDisable(GL_DEPTH_TEST);

        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

    void CasePlayground::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);

        if (_selectedBody < static_cast<int>(_system.Bodies.size()) && ImGui::IsItemFocused()) {
            auto & body = _system.Bodies[_selectedBody];
            if (ImGui::IsKeyPressed(ImGuiKey_J)) applyImpulseWithViz(_selectedBody, body.X, Eigen::Vector3f(-_userImpulse, 0.f, 0.f));
            if (ImGui::IsKeyPressed(ImGuiKey_L)) applyImpulseWithViz(_selectedBody, body.X, Eigen::Vector3f(_userImpulse, 0.f, 0.f));
            if (ImGui::IsKeyPressed(ImGuiKey_I)) applyImpulseWithViz(_selectedBody, body.X, Eigen::Vector3f(0.f, 0.f, -_userImpulse));
            if (ImGui::IsKeyPressed(ImGuiKey_K)) applyImpulseWithViz(_selectedBody, body.X, Eigen::Vector3f(0.f, 0.f, _userImpulse));
            if (ImGui::IsKeyPressed(ImGuiKey_U)) applyImpulseWithViz(_selectedBody, body.X, Eigen::Vector3f(0.f, _userImpulse, 0.f));
            if (ImGui::IsKeyPressed(ImGuiKey_O)) applyImpulseWithViz(_selectedBody, body.X, Eigen::Vector3f(0.f, -_userImpulse, 0.f));
        }
    }

    void CasePlayground::stepSimulation(float dt) {
        float const scaledDt = dt * _timeScale;
        int const   substeps = std::max(1, _substeps);
        float const h        = scaledDt / static_cast<float>(substeps);
        for (int i = 0; i < substeps; ++i) {
            _system.Step(h);
        }
    }

    void CasePlayground::applyUserInteraction(float dt) {
        (void) dt;
        glm::vec3 move = _cameraManager.getMouseMove();
        if (_selectedBody < 0 || _selectedBody >= static_cast<int>(_system.Bodies.size())) {
            return;
        }
        auto & body = _system.Bodies[_selectedBody];
        if (body.Fixed) {
            return;
        }
        Eigen::Vector3f impulse = glm2eigen(move * 120.f);
        if (impulse.squaredNorm() > 0.f) {
            applyImpulseWithViz(_selectedBody, body.X, impulse);
        }
    }

    void CasePlayground::applyImpulseWithViz(int bodyId, Eigen::Vector3f const & worldPoint, Eigen::Vector3f const & impulse) {
        if (bodyId < 0 || bodyId >= static_cast<int>(_system.Bodies.size())) {
            return;
        }
        if (impulse.squaredNorm() <= 1e-12f) {
            return;
        }

        auto & body = _system.Bodies[bodyId];
        body.ApplyImpulse(worldPoint, impulse);

        _hasImpulseViz     = true;
        _impulseVizTimer   = _impulseVizDuration;
        _lastImpulsePoint  = worldPoint;
        _lastImpulseVector = impulse;
    }

    void CasePlayground::resetPreset() {
        _presetDirty = false;
        _system.Clear();
        _stopped   = true;
        _timeScale = 1.f;
        _substeps  = 1;
        // Reset solver knobs to a deterministic baseline before each preset.
        _system.Gravity                      = 0.f;
        _system.EnableCCD                    = true;
        _system.EnableSchurComplement        = false;
        _system.EnableWarmStart              = true;
        _system.VelocityIterations           = 12;
        _system.PositionIterations           = 3;
        _system.BaumgarteBeta                = 0.2f;
        _system.JointBaumgarteBeta           = 0.2f;
        _system.MaxBiasVelocity              = 1.5f;
        _system.PenetrationSlop              = 1e-3f;
        _system.RestitutionVelocityThreshold = 0.6f;
        _system.RestingLinearThreshold       = 0.08f;
        _system.RestingAngularThreshold      = 0.12f;
        _system.SleepTimeThreshold           = 0.45f;

        switch (_preset) {
        case Preset::SingleBody:
            initSingleBody();
            break;
        case Preset::DoubleEdge:
            initDoubleEdge();
            break;
        case Preset::DoubleVertexFace:
            initDoubleVertexFace();
            break;
        case Preset::DoubleFaceFace:
            initDoubleFaceFace();
            break;
        case Preset::ComplexScene:
            initComplexScene();
            break;
        case Preset::BonusNewtonCradle:
            initBonusNewtonCradle();
            break;
        case Preset::BonusStacking:
            initBonusStacking();
            break;
        case Preset::BonusMixedPrimitives:
            initBonusMixedPrimitives();
            break;
        case Preset::BonusConstrainedContacts:
            initBonusConstrainedContacts();
            break;
        case Preset::BonusArticulatedBodies:
            initBonusArticulatedBodies();
            break;
        case Preset::BonusCCD:
            initBonusCCD();
            break;
        }

        _selectedBody = std::clamp(_selectedBody, 0, std::max(0, static_cast<int>(_system.Bodies.size()) - 1));
    }

    void CasePlayground::initSingleBody() {
        _system.Gravity               = 0.f;
        _system.EnableCCD             = false;
        _system.EnableSchurComplement = false;

        int b               = _system.AddBox(Eigen::Vector3f(1.2f, 0.8f, 1.6f), Eigen::Vector3f(0.f, 0.f, 0.f), Eigen::Quaternionf::Identity(), 1.5f, false);
        _system.Bodies[b].V = Eigen::Vector3f(0.5f, 0.1f, -0.2f);
        _system.Bodies[b].W = Eigen::Vector3f(1.0f, 1.6f, 0.6f);
    }

    void CasePlayground::initDoubleEdge() {
        _system.Gravity               = 0.f;
        _system.EnableCCD             = false;
        _system.EnableSchurComplement = false;

        constexpr float    kDegToRad = 0.01745329252f;
        Eigen::Quaternionf qa        = Eigen::AngleAxisf(35.f * kDegToRad, Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(45.f * kDegToRad, Eigen::Vector3f::UnitZ())
            * Eigen::AngleAxisf(10.f * kDegToRad, Eigen::Vector3f::UnitX());
        Eigen::Quaternionf qb = Eigen::AngleAxisf(-35.f * kDegToRad, Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(-45.f * kDegToRad, Eigen::Vector3f::UnitZ())
            * Eigen::AngleAxisf(-10.f * kDegToRad, Eigen::Vector3f::UnitX());

        int a               = _system.AddBox(Eigen::Vector3f(1.2f, 1.0f, 1.0f), Eigen::Vector3f(-1.8f, 0.0f, 0.35f), qa.normalized(), 1.0f, false);
        int b               = _system.AddBox(Eigen::Vector3f(1.2f, 1.0f, 1.0f), Eigen::Vector3f(1.8f, 0.0f, -0.35f), qb.normalized(), 1.0f, false);
        _system.Bodies[a].V = Eigen::Vector3f(1.8f, 0.f, 0.f);
        _system.Bodies[b].V = Eigen::Vector3f(-1.8f, 0.f, 0.f);
        _system.Bodies[a].W = Eigen::Vector3f(0.f, 0.f, 0.f);
        _system.Bodies[b].W = Eigen::Vector3f(0.f, 0.f, 0.f);
    }

    void CasePlayground::initDoubleVertexFace() {
        _system.Gravity               = 0.f;
        _system.EnableCCD             = false;
        _system.EnableSchurComplement = false;

        Eigen::Quaternionf const qLead =
            Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f(1.f, 1.f, 1.f).normalized(), Eigen::Vector3f(1.f, 0.f, 0.f)).normalized();

        int a               = _system.AddBox(Eigen::Vector3f(1.f, 1.f, 1.f), Eigen::Vector3f(-1.22f, 0.f, 0.f), qLead, 1.2f, false);
        int b               = _system.AddBox(Eigen::Vector3f(1.f, 1.f, 1.f), Eigen::Vector3f(1.38f, 0.f, 0.f), Eigen::Quaternionf::Identity(), 1.2f, false);
        _system.Bodies[a].V = Eigen::Vector3f(2.0f, 0.f, 0.f);
        _system.Bodies[b].V = Eigen::Vector3f(-1.5f, 0.f, 0.f);
        _system.Bodies[a].W.setZero();
        _system.Bodies[b].W.setZero();
    }

    void CasePlayground::initDoubleFaceFace() {
        _system.Gravity               = 0.f;
        _system.EnableCCD             = false;
        _system.EnableSchurComplement = false;

        int a = _system.AddBox(Eigen::Vector3f(1.2f, 1.2f, 1.2f), Eigen::Vector3f(-1.8f, 0.f, 0.f), Eigen::Quaternionf::Identity(), 1.0f, false);
        int b = _system.AddBox(Eigen::Vector3f(1.2f, 1.2f, 1.2f), Eigen::Vector3f(1.8f, 0.f, 0.f), Eigen::Quaternionf::Identity(), 1.0f, false);
        _system.Bodies[a].V = Eigen::Vector3f(1.4f, 0.f, 0.f);
        _system.Bodies[b].V = Eigen::Vector3f(-1.4f, 0.f, 0.f);
        _system.Bodies[a].W.setZero();
        _system.Bodies[b].W.setZero();
        _system.Bodies[a].Friction = 0.f;
        _system.Bodies[b].Friction = 0.f;
    }

    void CasePlayground::initComplexScene() {
        _system.Gravity               = 9.8f;
        _system.EnableCCD             = true;
        _system.EnableSchurComplement = false;
        _system.VelocityIterations    = 24;
        _system.PositionIterations    = 6;
        _system.BaumgarteBeta         = 0.14f;
        _system.MaxBiasVelocity       = 1.1f;
        _system.PenetrationSlop       = 2.0e-3f;

        _timeScale = 0.9f;
        _substeps  = 2;

        int const ground = _system.AddBox(Eigen::Vector3f(20.f, 0.4f, 20.f), Eigen::Vector3f(0.f, -2.2f, 0.f), Eigen::Quaternionf::Identity(), 1.f, true);
        int const wallL  = _system.AddBox(Eigen::Vector3f(0.45f, 6.f, 18.f), Eigen::Vector3f(-7.2f, 1.f, 0.f), Eigen::Quaternionf::Identity(), 1.f, true);
        int const wallR  = _system.AddBox(Eigen::Vector3f(0.45f, 6.f, 18.f), Eigen::Vector3f(7.2f, 1.f, 0.f), Eigen::Quaternionf::Identity(), 1.f, true);

        _system.Bodies[ground].Restitution = 0.02f;
        _system.Bodies[ground].Friction    = 0.55f;
        _system.Bodies[wallL].Restitution  = 0.02f;
        _system.Bodies[wallL].Friction     = 0.50f;
        _system.Bodies[wallR].Restitution  = 0.02f;
        _system.Bodies[wallR].Friction     = 0.50f;

        float const hx   = 0.4f;
        float const step = 2.f * hx + 0.07f;
        for (int i = 0; i < 5; ++i) {
            int id                         = _system.AddBox(Eigen::Vector3f(0.8f, 0.8f, 0.8f), Eigen::Vector3f(-1.5f + step * i, 2.5f + 1.02f * i, (i % 2 == 0) ? 0.5f : -0.5f), Eigen::Quaternionf::Identity(), 1.0f, false);
            _system.Bodies[id].Restitution = 0.04f;
            _system.Bodies[id].Friction    = 0.45f;
            _system.Bodies[id].W           = Eigen::Vector3f(0.08f * i, 0.03f, -0.05f * i);
        }

        float const row1Right = -1.5f + step * 4.f + hx;
        float const row2X0    = row1Right + hx + 0.08f;
        for (int i = 0; i < 5; ++i) {
            int id                         = _system.AddBox(Eigen::Vector3f(0.8f, 0.8f, 0.8f), Eigen::Vector3f(row2X0 + step * i, 2.35f + 0.95f * i, 2.45f), Eigen::Quaternionf::Identity(), 1.0f, false);
            _system.Bodies[id].Restitution = 0.04f;
            _system.Bodies[id].Friction    = 0.45f;
            _system.Bodies[id].W           = Eigen::Vector3f(-0.07f * i, 0.035f, -0.04f * i);
        }

        float const h3 = 0.375f;
        float const s3 = 2.f * h3 + 0.08f;
        for (int i = 0; i < 4; ++i) {
            int id                         = _system.AddBox(Eigen::Vector3f(0.75f, 0.75f, 0.75f), Eigen::Vector3f(-0.55f + s3 * i, 4.15f + 0.58f * i, -2.55f), Eigen::Quaternionf::Identity(), 1.0f, false);
            _system.Bodies[id].Restitution = 0.04f;
            _system.Bodies[id].Friction    = 0.45f;
            _system.Bodies[id].W           = Eigen::Vector3f(0.05f, -0.02f, 0.06f);
        }
    }

    void CasePlayground::initBonusNewtonCradle() {
        _system.Gravity                      = 0.f;
        _system.EnableCCD                    = false;
        _system.EnableSchurComplement        = false;
        _system.RelaxOverlapsBeforeIntegrate = true;
        _system.VelocityIterations           = 48;
        _system.PositionIterations           = 8;
        _system.BaumgarteBeta                = 0.08f;
        _system.MaxBiasVelocity              = 0.45f;
        _system.PenetrationSlop              = 4e-4f;
        _system.RestitutionVelocityThreshold = 0.f;
        _system.RestingLinearThreshold       = 0.02f;
        _system.RestingAngularThreshold      = 0.04f;
        _system.SleepTimeThreshold           = 1.2f;

        _timeScale          = 0.42f;
        _substeps           = 3;
        _showImpulseViz     = true;
        _impulseVizScale    = std::max(_impulseVizScale, 0.28f);
        _impulseVizDuration = std::max(_impulseVizDuration, 1.2f);

        Eigen::Vector3f const boxDim(0.9f, 0.55f, 0.55f);
        float const           hx = 0.5f * boxDim.x();

        int ids[5];
        ids[1] = _system.AddBox(boxDim, Eigen::Vector3f(0.f, 0.f, 0.f), Eigen::Quaternionf::Identity(), 1.f, false);
        ids[2]           = _system.AddBox(boxDim, Eigen::Vector3f(hx * 2.f, 0.f, 0.f), Eigen::Quaternionf::Identity(), 1.f, false);
        ids[3]           = _system.AddBox(boxDim, Eigen::Vector3f(4.f * hx, 0.f, 0.f), Eigen::Quaternionf::Identity(), 1.f, false);
        ids[4]           = _system.AddBox(boxDim, Eigen::Vector3f(6.f * hx, 0.f, 0.f), Eigen::Quaternionf::Identity(), 1.f, false);

        float const strikerGap = 0.88f;
        float const strikerX   = -2.f * hx - strikerGap;
        ids[0]                 = _system.AddBox(boxDim, Eigen::Vector3f(strikerX, 0.f, 0.f), Eigen::Quaternionf::Identity(), 1.f, false);

        for (int i = 0; i < 5; ++i) {
            auto & b      = _system.Bodies[ids[i]];
            b.Restitution = 0.992f;
            b.Friction    = 0.f;
            b.W.setZero();
            b.V.setZero();
        }
        _system.Bodies[ids[0]].V = Eigen::Vector3f(5.0f, 0.f, 0.f);

        _selectedBody = ids[0];
    }

    void CasePlayground::initBonusStacking() {
        _system.Gravity                      = 9.8f;
        _system.EnableCCD                    = true;
        _system.EnableSchurComplement        = false;
        _system.VelocityIterations           = 32;
        _system.PositionIterations           = 10;
        _system.BaumgarteBeta                = 0.08f;
        _system.MaxBiasVelocity              = 0.55f;
        _system.PenetrationSlop              = 3.0e-3f;
        _system.RestitutionVelocityThreshold = 0.2f;
        _system.RestingLinearThreshold       = 0.05f;
        _system.RestingAngularThreshold      = 0.08f;
        _system.SleepTimeThreshold           = 0.25f;

        _timeScale = 0.75f;
        _substeps  = 5;

        int const ground                   = _system.AddBox(Eigen::Vector3f(10.f, 0.4f, 10.f), Eigen::Vector3f(0.f, -2.2f, 0.f), Eigen::Quaternionf::Identity(), 1.f, true);
        _system.Bodies[ground].Restitution = 0.0f;
        _system.Bodies[ground].Friction    = 0.65f;
        for (int i = 0; i < 6; ++i) {
            float const y                  = -1.5f + 1.035f * i;
            int         id                 = _system.AddBox(Eigen::Vector3f(1.f, 1.f, 1.f), Eigen::Vector3f(0.f, y, 0.f), Eigen::Quaternionf::Identity(), 1.f, false);
            _system.Bodies[id].Restitution = 0.0f;
            _system.Bodies[id].Friction    = 0.62f;
            _system.Bodies[id].W.setZero();
        }
    }

    void CasePlayground::initBonusMixedPrimitives() {
        _system.Gravity                      = 9.8f;
        _system.EnableCCD                    = true;
        _system.EnableSchurComplement        = false;
        _system.VelocityIterations           = 24;
        _system.PositionIterations           = 6;
        _system.BaumgarteBeta                = 0.10f;
        _system.MaxBiasVelocity              = 0.70f;
        _system.PenetrationSlop              = 2.0e-3f;
        _system.RestitutionVelocityThreshold = 0.2f;

        _timeScale = 0.9f;
        _substeps  = 3;

        int const ground                   = _system.AddBox(Eigen::Vector3f(14.f, 0.4f, 14.f), Eigen::Vector3f(0.f, -2.2f, 0.f), Eigen::Quaternionf::Identity(), 1.f, true);
        _system.Bodies[ground].Restitution = 0.02f;
        _system.Bodies[ground].Friction    = 0.55f;

        int box                         = _system.AddBox(Eigen::Vector3f(1.4f, 1.0f, 1.2f), Eigen::Vector3f(-2.2f, 1.05f, 0.f), Eigen::Quaternionf::Identity(), 1.2f, false);
        int sph                         = _system.AddSphere(0.55f, Eigen::Vector3f(0.f, 2.75f, 0.f), Eigen::Quaternionf::Identity(), 1.0f, false);
        int c                           = _system.AddCylinder(0.45f, 1.3f, Eigen::Vector3f(2.35f, 3.15f, 0.f), Eigen::Quaternionf::Identity(), 1.3f, false);
        _system.Bodies[box].Restitution = 0.10f;
        _system.Bodies[box].Friction    = 0.45f;
        _system.Bodies[sph].Restitution = 0.28f;
        _system.Bodies[sph].Friction    = 0.25f;
        _system.Bodies[c].Restitution   = 0.16f;
        _system.Bodies[c].Friction      = 0.35f;
        _system.Bodies[c].W             = Eigen::Vector3f(0.f, 1.4f, 0.f);
    }

    void CasePlayground::initBonusConstrainedContacts() {
        _system.Gravity               = 9.8f;
        _system.EnableCCD             = true;
        _system.EnableSchurComplement = true;
        _system.VelocityIterations    = 10;
        _system.PositionIterations    = 3;
        _system.BaumgarteBeta         = 0.07f;
        _system.MaxBiasVelocity       = 0.5f;

        _system.AddBox(Eigen::Vector3f(12.f, 0.4f, 12.f), Eigen::Vector3f(0.f, -2.2f, 0.f), Eigen::Quaternionf::Identity(), 1.f, true);
        for (int i = 0; i < 6; ++i) {
            _system.AddBox(Eigen::Vector3f(1.0f, 1.0f, 1.0f), Eigen::Vector3f(-2.25f + 1.06f * i, -1.55f + 1.08f * i, 0.f), Eigen::Quaternionf::Identity(), 1.f, false);
        }
    }

    void CasePlayground::initBonusArticulatedBodies() {
        _system.Gravity               = 9.8f;
        _system.EnableCCD             = true;
        _system.EnableSchurComplement = false;
        _system.VelocityIterations    = 40;
        _system.JointBaumgarteBeta    = 0.055f;

        _system.AddBox(Eigen::Vector3f(8.f, 0.4f, 8.f), Eigen::Vector3f(0.f, -3.f, 0.f), Eigen::Quaternionf::Identity(), 1.f, true);

        float const           rootHalf = 0.25f;
        float const           cy       = 2.2f;
        float const           cz       = 0.f;
        int const             root     = _system.AddBox(Eigen::Vector3f(2.f * rootHalf, 2.f * rootHalf, 2.f * rootHalf), Eigen::Vector3f(0.f, cy, cz), Eigen::Quaternionf::Identity(), 1.f, true);
        int                   prev       = root;
        float const           gap        = 0.05f;
        Eigen::Vector3f const linkDim(0.9f, 0.35f, 0.35f);
        Eigen::Vector3f const linkHalf(0.45f, 0.175f, 0.175f);
        // 上一节 +x 端面位置（根块中心在原点）
        float prevRightX = rootHalf;

        for (int i = 0; i < 5; ++i) {
            // 端面对端面排成一条直线长条：质心共线 (cy,cz)，沿 x 每隔 2·半长 + 间隙
            float const             cx   = prevRightX + gap + linkHalf.x();
            Eigen::Vector3f const   newC(cx, cy, cz);
            float const           xMid    = prevRightX + 0.5f * gap;
            Eigen::Vector3f const jointAt(xMid, cy, cz);
            // 仅用一处球铰：双点顺序冲量会在两锚间「抢误差」导致抖动；几何上仍端面对齐成长条，铰链取间隙中面中心
            int const id = _system.AddBox(linkDim, newC, Eigen::Quaternionf::Identity(), 0.8f, false);
            _system.AddPointJoint(prev, id, jointAt);
            prev       = id;
            prevRightX = cx + linkHalf.x();
        }
    }

    void CasePlayground::initBonusCCD() {
        _system.Gravity               = 0.f;
        _system.EnableCCD             = true;
        _system.EnableSchurComplement = false;
        _system.VelocityIterations    = 20;
        _substeps                     = 4;

        _system.AddBox(Eigen::Vector3f(0.25f, 5.f, 6.f), Eigen::Vector3f(0.f, 0.f, 0.f), Eigen::Quaternionf::Identity(), 1.f, true);

        int bullet                         = _system.AddSphere(0.2f, Eigen::Vector3f(-5.f, 0.f, 0.f), Eigen::Quaternionf::Identity(), 0.2f, false);
        _system.Bodies[bullet].V           = Eigen::Vector3f(35.f, 0.f, 0.f);
        _system.Bodies[bullet].Restitution = 0.8f;

        int target                         = _system.AddBox(Eigen::Vector3f(0.6f, 0.6f, 0.6f), Eigen::Vector3f(2.5f, 0.f, 0.f), Eigen::Quaternionf::Identity(), 1.0f, false);
        _system.Bodies[target].Restitution = 0.4f;
    }

} // namespace VCX::Labs::RigidBody
