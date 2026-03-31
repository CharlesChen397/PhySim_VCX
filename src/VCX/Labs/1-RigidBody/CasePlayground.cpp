#include "Labs/1-RigidBody/CasePlayground.h"

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
        _wireItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines) {
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

        if (ImGui::CollapsingHeader("Global Physics", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Checkbox("Enable CCD", &_system.EnableCCD);
            ImGui::Checkbox("Enable Schur Solve (B4)", &_system.EnableSchurComplement);
            ImGui::Checkbox("Enable Warm Start", &_system.EnableWarmStart);
            ImGui::SliderFloat("Gravity", &_system.Gravity, 0.f, 20.f);
            ImGui::SliderInt("Velocity Iterations", &_system.VelocityIterations, 2, 40);
            ImGui::SliderInt("Position Iterations", &_system.PositionIterations, 1, 10);
            ImGui::SliderFloat("Baumgarte Beta", &_system.BaumgarteBeta, 0.f, 0.8f);
            ImGui::SliderFloat("Penetration Slop", &_system.PenetrationSlop, 1e-4f, 2e-2f, "%.4f", ImGuiSliderFlags_Logarithmic);
        }

        if (ImGui::CollapsingHeader("Interaction", ImGuiTreeNodeFlags_DefaultOpen)) {
            int maxBody   = std::max(0, static_cast<int>(_system.Bodies.size()) - 1);
            _selectedBody = std::clamp(_selectedBody, 0, maxBody);
            ImGui::SliderInt("Selected Body", &_selectedBody, 0, maxBody);
            ImGui::SliderFloat("Impulse Strength", &_userImpulse, 0.f, 20.f);
            ImGui::Checkbox("Auto Kick On Reset", &_autoApplyKick);
            ImGui::Checkbox("Show Wireframe", &_showWireframe);

            if (ImGui::Button("Impulse +X") && _selectedBody < static_cast<int>(_system.Bodies.size())) {
                auto & b = _system.Bodies[_selectedBody];
                b.ApplyImpulse(b.X, Eigen::Vector3f(_userImpulse, 0.f, 0.f));
            }
            ImGui::SameLine();
            if (ImGui::Button("Impulse -X") && _selectedBody < static_cast<int>(_system.Bodies.size())) {
                auto & b = _system.Bodies[_selectedBody];
                b.ApplyImpulse(b.X, Eigen::Vector3f(-_userImpulse, 0.f, 0.f));
            }
            if (ImGui::Button("Impulse +Y") && _selectedBody < static_cast<int>(_system.Bodies.size())) {
                auto & b = _system.Bodies[_selectedBody];
                b.ApplyImpulse(b.X, Eigen::Vector3f(0.f, _userImpulse, 0.f));
            }
            ImGui::SameLine();
            if (ImGui::Button("Impulse +Z") && _selectedBody < static_cast<int>(_system.Bodies.size())) {
                auto & b = _system.Bodies[_selectedBody];
                b.ApplyImpulse(b.X, Eigen::Vector3f(0.f, 0.f, _userImpulse));
            }
        }

        ImGui::Text("Bodies: %d", static_cast<int>(_system.Bodies.size()));
        ImGui::Text("Contacts: %d", static_cast<int>(_system.Contacts.size()));
        ImGui::Text("Joints: %d", static_cast<int>(_system.Joints.size()));
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
            if (ImGui::IsKeyPressed(ImGuiKey_J)) body.ApplyImpulse(body.X, Eigen::Vector3f(-_userImpulse, 0.f, 0.f));
            if (ImGui::IsKeyPressed(ImGuiKey_L)) body.ApplyImpulse(body.X, Eigen::Vector3f(_userImpulse, 0.f, 0.f));
            if (ImGui::IsKeyPressed(ImGuiKey_I)) body.ApplyImpulse(body.X, Eigen::Vector3f(0.f, 0.f, -_userImpulse));
            if (ImGui::IsKeyPressed(ImGuiKey_K)) body.ApplyImpulse(body.X, Eigen::Vector3f(0.f, 0.f, _userImpulse));
            if (ImGui::IsKeyPressed(ImGuiKey_U)) body.ApplyImpulse(body.X, Eigen::Vector3f(0.f, _userImpulse, 0.f));
        }
    }

    void CasePlayground::stepSimulation(float dt) {
        _system.Step(dt);
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
            body.ApplyImpulse(body.X, impulse);
        }
    }

    void CasePlayground::resetPreset() {
        _presetDirty = false;
        _system.Clear();
        _stopped = false;

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

        int a               = _system.AddBox(Eigen::Vector3f(0.7f, 0.7f, 1.6f), Eigen::Vector3f(-1.8f, 0.f, 0.f), Eigen::Quaternionf::Identity(), 1.0f, false);
        int b               = _system.AddBox(Eigen::Vector3f(0.7f, 1.6f, 0.7f), Eigen::Vector3f(1.8f, -0.4f, 0.f), Eigen::Quaternionf(0.94f, 0.12f, 0.28f, 0.15f).normalized(), 1.0f, false);
        _system.Bodies[a].V = Eigen::Vector3f(1.3f, 0.f, 0.f);
        _system.Bodies[b].V = Eigen::Vector3f(-1.3f, 0.f, 0.f);
    }

    void CasePlayground::initDoubleVertexFace() {
        _system.Gravity               = 0.f;
        _system.EnableCCD             = false;
        _system.EnableSchurComplement = false;

        int a               = _system.AddBox(Eigen::Vector3f(1.f, 1.f, 1.f), Eigen::Vector3f(-1.7f, 0.1f, 0.f), Eigen::Quaternionf::Identity(), 1.2f, false);
        int b               = _system.AddBox(Eigen::Vector3f(1.f, 1.f, 1.f), Eigen::Vector3f(1.6f, 0.0f, 0.f), Eigen::Quaternionf(0.91f, 0.2f, 0.35f, 0.1f).normalized(), 1.2f, false);
        _system.Bodies[a].V = Eigen::Vector3f(1.5f, 0.f, 0.f);
        _system.Bodies[b].V = Eigen::Vector3f(-1.2f, 0.f, 0.f);
    }

    void CasePlayground::initDoubleFaceFace() {
        _system.Gravity               = 0.f;
        _system.EnableCCD             = false;
        _system.EnableSchurComplement = false;

        int a               = _system.AddBox(Eigen::Vector3f(1.2f, 1.2f, 1.2f), Eigen::Vector3f(-1.8f, 0.3f, 0.4f), Eigen::Quaternionf::Identity(), 1.0f, false);
        int b               = _system.AddBox(Eigen::Vector3f(1.2f, 1.2f, 1.2f), Eigen::Vector3f(1.8f, 0.3f, -0.4f), Eigen::Quaternionf::Identity(), 1.0f, false);
        _system.Bodies[a].V = Eigen::Vector3f(1.4f, 0.f, -0.1f);
        _system.Bodies[b].V = Eigen::Vector3f(-1.4f, 0.f, 0.1f);
    }

    void CasePlayground::initComplexScene() {
        _system.Gravity               = 9.8f;
        _system.EnableCCD             = true;
        _system.EnableSchurComplement = false;
        _system.VelocityIterations    = 18;

        _system.AddBox(Eigen::Vector3f(14.f, 0.4f, 14.f), Eigen::Vector3f(0.f, -2.2f, 0.f), Eigen::Quaternionf::Identity(), 1.f, true);
        _system.AddBox(Eigen::Vector3f(0.3f, 6.f, 14.f), Eigen::Vector3f(-5.f, 1.f, 0.f), Eigen::Quaternionf::Identity(), 1.f, true);
        _system.AddBox(Eigen::Vector3f(0.3f, 6.f, 14.f), Eigen::Vector3f(5.f, 1.f, 0.f), Eigen::Quaternionf::Identity(), 1.f, true);

        for (int i = 0; i < 5; ++i) {
            int id               = _system.AddBox(Eigen::Vector3f(0.8f, 0.8f, 0.8f), Eigen::Vector3f(-1.5f + 0.8f * i, 2.5f + 1.0f * i, (i % 2 == 0) ? 0.5f : -0.5f), Eigen::Quaternionf::Identity(), 1.0f, false);
            _system.Bodies[id].W = Eigen::Vector3f(0.2f * i, 0.1f, -0.1f * i);
        }
    }

    void CasePlayground::initBonusNewtonCradle() {
        _system.Gravity               = 9.8f;
        _system.EnableCCD             = false;
        _system.EnableSchurComplement = false;
        _system.VelocityIterations    = 26;

        _system.AddBox(Eigen::Vector3f(12.f, 0.4f, 6.f), Eigen::Vector3f(0.f, -2.2f, 0.f), Eigen::Quaternionf::Identity(), 1.f, true);

        for (int i = 0; i < 5; ++i) {
            float x                        = -2.f + float(i) * 1.0f;
            int   id                       = _system.AddBox(Eigen::Vector3f(0.9f, 0.9f, 0.9f), Eigen::Vector3f(x, -0.6f, 0.f), Eigen::Quaternionf::Identity(), 1.f, false);
            _system.Bodies[id].Restitution = 0.95f;
        }
        if (_autoApplyKick && _system.Bodies.size() > 1) {
            _system.Bodies[1].ApplyImpulse(_system.Bodies[1].X, Eigen::Vector3f(12.f, 0.f, 0.f));
        }
    }

    void CasePlayground::initBonusStacking() {
        _system.Gravity               = 9.8f;
        _system.EnableCCD             = true;
        _system.EnableSchurComplement = false;
        _system.VelocityIterations    = 32;
        _system.PositionIterations    = 8;
        _system.BaumgarteBeta         = 0.28f;

        _system.AddBox(Eigen::Vector3f(10.f, 0.4f, 10.f), Eigen::Vector3f(0.f, -2.2f, 0.f), Eigen::Quaternionf::Identity(), 1.f, true);
        for (int i = 0; i < 6; ++i) {
            int id                         = _system.AddBox(Eigen::Vector3f(1.f, 1.f, 1.f), Eigen::Vector3f(0.f, -1.6f + 1.02f * i, 0.f), Eigen::Quaternionf::Identity(), 1.f, false);
            _system.Bodies[id].Restitution = 0.05f;
            _system.Bodies[id].Friction    = 0.8f;
        }
    }

    void CasePlayground::initBonusMixedPrimitives() {
        _system.Gravity               = 9.8f;
        _system.EnableCCD             = true;
        _system.EnableSchurComplement = false;

        _system.AddBox(Eigen::Vector3f(14.f, 0.4f, 14.f), Eigen::Vector3f(0.f, -2.2f, 0.f), Eigen::Quaternionf::Identity(), 1.f, true);

        _system.AddBox(Eigen::Vector3f(1.4f, 1.0f, 1.2f), Eigen::Vector3f(-2.f, 1.2f, 0.f), Eigen::Quaternionf::Identity(), 1.2f, false);
        _system.AddSphere(0.55f, Eigen::Vector3f(0.f, 2.5f, 0.f), Eigen::Quaternionf::Identity(), 1.0f, false);
        int c               = _system.AddCylinder(0.45f, 1.3f, Eigen::Vector3f(2.f, 3.0f, 0.f), Eigen::Quaternionf::Identity(), 1.3f, false);
        _system.Bodies[c].W = Eigen::Vector3f(0.f, 2.f, 0.f);
    }

    void CasePlayground::initBonusConstrainedContacts() {
        _system.Gravity               = 9.8f;
        _system.EnableCCD             = true;
        _system.EnableSchurComplement = true;
        _system.VelocityIterations    = 10;
        _system.PositionIterations    = 3;

        _system.AddBox(Eigen::Vector3f(10.f, 0.4f, 10.f), Eigen::Vector3f(0.f, -2.2f, 0.f), Eigen::Quaternionf::Identity(), 1.f, true);
        for (int i = 0; i < 6; ++i) {
            _system.AddBox(Eigen::Vector3f(1.0f, 1.0f, 1.0f), Eigen::Vector3f(-2.f + 0.8f * i, -1.6f + 1.02f * i, 0.f), Eigen::Quaternionf::Identity(), 1.f, false);
        }
    }

    void CasePlayground::initBonusArticulatedBodies() {
        _system.Gravity               = 9.8f;
        _system.EnableCCD             = true;
        _system.EnableSchurComplement = false;
        _system.VelocityIterations    = 30;

        _system.AddBox(Eigen::Vector3f(8.f, 0.4f, 8.f), Eigen::Vector3f(0.f, -3.f, 0.f), Eigen::Quaternionf::Identity(), 1.f, true);

        int root = _system.AddBox(Eigen::Vector3f(0.5f, 0.5f, 0.5f), Eigen::Vector3f(0.f, 2.2f, 0.f), Eigen::Quaternionf::Identity(), 1.f, true);
        int prev = root;
        for (int i = 0; i < 5; ++i) {
            int             id = _system.AddBox(Eigen::Vector3f(0.9f, 0.35f, 0.35f), Eigen::Vector3f(0.9f * (i + 1), 2.2f, 0.f), Eigen::Quaternionf::Identity(), 0.8f, false);
            Eigen::Vector3f jointPos(0.9f * i + 0.45f, 2.2f, 0.f);
            _system.AddPointJoint(prev, id, jointPos);
            prev = id;
        }
    }

    void CasePlayground::initBonusCCD() {
        _system.Gravity               = 0.f;
        _system.EnableCCD             = true;
        _system.EnableSchurComplement = false;
        _system.VelocityIterations    = 20;

        _system.AddBox(Eigen::Vector3f(0.25f, 5.f, 6.f), Eigen::Vector3f(0.f, 0.f, 0.f), Eigen::Quaternionf::Identity(), 1.f, true);

        int bullet                         = _system.AddSphere(0.2f, Eigen::Vector3f(-5.f, 0.f, 0.f), Eigen::Quaternionf::Identity(), 0.2f, false);
        _system.Bodies[bullet].V           = Eigen::Vector3f(35.f, 0.f, 0.f);
        _system.Bodies[bullet].Restitution = 0.8f;

        int target                         = _system.AddBox(Eigen::Vector3f(0.6f, 0.6f, 0.6f), Eigen::Vector3f(2.5f, 0.f, 0.f), Eigen::Quaternionf::Identity(), 1.0f, false);
        _system.Bodies[target].Restitution = 0.4f;
    }

} // namespace VCX::Labs::RigidBody
