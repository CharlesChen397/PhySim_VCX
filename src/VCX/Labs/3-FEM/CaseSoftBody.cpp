#include "Labs/3-FEM/CaseSoftBody.h"

#include <algorithm>
#include <cmath>

#include <glm/gtc/type_ptr.hpp>
#include <imgui.h>

#include "Engine/app.h"
#include "Labs/Common/ImGuiHelper.h"

namespace VCX::Labs::FEM {

    namespace {
        glm::vec3 clampForce(glm::vec3 const & f, float maxLen) {
            float const len = glm::length(f);
            if (len <= maxLen || len <= 1e-8f) return f;
            return f * (maxLen / len);
        }

        float length2(glm::vec3 const & v) {
            return glm::dot(v, v);
        }
    } // namespace

    CaseSoftBody::CaseSoftBody():
        _program(Engine::GL::UniqueProgram({
            Engine::GL::SharedShader("assets/shaders/flat.vert"),
            Engine::GL::SharedShader("assets/shaders/flat.frag"),
        })),
        _solidItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
        _wireItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines),
        _particleItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Points),
        _fixedItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Points),
        _selectedItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Points),
        _forceItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines) {
        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);
        ResetSystem();
    }

    void CaseSoftBody::OnSetupPropsUI() {
        if (ImGui::Button("Reset System")) {
            ResetSystem();
        }
        ImGui::SameLine();
        if (ImGui::Button(_stopped ? "Start Simulation" : "Stop Simulation")) {
            _stopped = !_stopped;
        }

        ImGui::Separator();
        ImGui::Text("Explicit FEM");
        if (ImGui::Checkbox("Bonus1: compare 3 materials", &_compareMaterials)) {
            UpdateStaticBuffers();
        }
        if (!_compareMaterials) {
            char const * names[] = { "StVK", "Neo-Hookean", "Corotated" };
            if (ImGui::Combo("Material Model", &_activeModel, names, 3)) {
                _systems[0].Model = static_cast<MaterialModel>(_activeModel);
            }
        } else {
            ImGui::Text("Left/Mid/Right: StVK, Neo-Hookean, Corotated");
        }
        ImGui::SliderFloat("Time Step", &_timeStep, 0.0002f, 0.003f, "%.4f");
        ImGui::SliderInt("Steps / Frame", &_stepsPerFrame, 1, 32);
        for (auto & system : _systems) {
            system.Model = _compareMaterials ? system.Model : static_cast<MaterialModel>(_activeModel);
        }
        if (ImGui::SliderFloat("Young Modulus", &_systems[0].YoungModulus, 1000.f, 80000.f, "%.0f")) {
            for (auto & system : _systems) system.YoungModulus = _systems[0].YoungModulus;
        }
        if (ImGui::SliderFloat("Poisson Ratio", &_systems[0].PoissonRatio, 0.0f, 0.45f, "%.2f")) {
            for (auto & system : _systems) system.PoissonRatio = _systems[0].PoissonRatio;
        }
        if (ImGui::SliderFloat("Density", &_systems[0].Density, 50.f, 1000.f, "%.0f")) {
            for (auto & system : _systems) {
                system.Density = _systems[0].Density;
                system.RecomputeMasses();
            }
        }
        if (ImGui::SliderFloat("Gravity", &_systems[0].Gravity, 0.f, 9.8f, "%.3f")) {
            for (auto & system : _systems) system.Gravity = _systems[0].Gravity;
        }
        if (ImGui::SliderFloat("Velocity Damping", &_systems[0].VelocityDamping, 0.f, 8.f, "%.2f")) {
            for (auto & system : _systems) system.VelocityDamping = _systems[0].VelocityDamping;
        }

        ImGui::Separator();
        ImGui::Text("Mesh");
        bool meshEdited = false;
        int  wx = static_cast<int>(_resolution.X);
        int  wy = static_cast<int>(_resolution.Y);
        int  wz = static_cast<int>(_resolution.Z);
        meshEdited |= ImGui::SliderInt("Cells X", &wx, 2, 14);
        meshEdited |= ImGui::SliderInt("Cells Y", &wy, 1, 5);
        meshEdited |= ImGui::SliderInt("Cells Z", &wz, 1, 5);
        meshEdited |= ImGui::SliderFloat3("Body Size", glm::value_ptr(_size), 0.8f, 10.f, "%.2f");
        if (meshEdited) {
            _resolution.X = static_cast<std::size_t>(std::max(wx, 2));
            _resolution.Y = static_cast<std::size_t>(std::max(wy, 1));
            _resolution.Z = static_cast<std::size_t>(std::max(wz, 1));
            _size = glm::max(_size, glm::vec3(0.5f));
            ResetSystem();
        }

        ImGui::Separator();
        ImGui::Text("Interaction");
        int maxParticle = std::max(0, static_cast<int>(_systems[0].Positions.size()) - 1);
        int selected = static_cast<int>(std::min<std::size_t>(_selectedParticle, std::size_t(maxParticle)));
        if (ImGui::SliderInt("Control Particle", &selected, 0, maxParticle)) {
            _selectedParticle = static_cast<std::size_t>(selected);
        }
        if (ImGui::Button("Select Free End")) {
            SetSelectedToFreeEnd();
        }
        ImGui::SliderFloat("Control Force", &_controlForceMagnitude, 0.f, 5000.f, "%.0f");
        ImGui::SliderFloat("Mouse Force Scale", &_mouseForceScale, 0.f, 5000.f, "%.0f");

        auto forceButton = [&](char const * label, glm::vec3 dir) {
            if (ImGui::Button(label)) {
                _buttonForce += dir * _controlForceMagnitude;
            }
        };
        forceButton("+X", glm::vec3(1.f, 0.f, 0.f));
        ImGui::SameLine();
        forceButton("-X", glm::vec3(-1.f, 0.f, 0.f));
        forceButton("+Y", glm::vec3(0.f, 1.f, 0.f));
        ImGui::SameLine();
        forceButton("-Y", glm::vec3(0.f, -1.f, 0.f));
        forceButton("+Z", glm::vec3(0.f, 0.f, 1.f));
        ImGui::SameLine();
        forceButton("-Z", glm::vec3(0.f, 0.f, -1.f));

        ImGui::Text("Hotkeys: J/L X, U/O Y, I/K Z");
        ImGui::Text("Alt + Left Drag: pull selected vertex in view plane");

        if (_selectedParticle < _systems[0].Positions.size()) {
            glm::vec3 const p = _systems[0].Positions[_selectedParticle];
            ImGui::Text("Selected: %zu  (%.2f, %.2f, %.2f)", _selectedParticle, p.x, p.y, p.z);
        }

        ImGui::Separator();
        ImGui::Checkbox("Surface", &_showSurface);
        ImGui::Checkbox("Wireframe", &_showWireframe);
        ImGui::Checkbox("Particles", &_showParticles);
        ImGui::Checkbox("Fixed Points", &_showFixed);
    }

    Common::CaseRenderResult CaseSoftBody::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        glm::vec3 const frameControlForce = clampForce(_keyboardForce + _mouseForce + _buttonForce, 8000.f);

        if (!_stopped) {
            int const steps = std::max(1, _stepsPerFrame);
            for (int i = 0; i < steps; ++i) {
                int const count = _compareMaterials ? 3 : 1;
                for (int s = 0; s < count; ++s) {
                    _systems[s].Step(_timeStep, _selectedParticle, frameControlForce);
                }
            }
        }
        _buttonForce = glm::vec3(0.f);

        UpdateDynamicBuffers(frameControlForce);

        _frame.Resize(desiredSize);
        _cameraManager.Update(_camera);

        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix(float(desiredSize.first) / float(desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LINE_SMOOTH);

        if (_showSurface) {
            _program.GetUniforms().SetByName("u_Color", glm::vec3(0.45f, 0.70f, 0.90f));
            _solidItem.Draw({ _program.Use() });
        }

        if (_showWireframe) {
            glLineWidth(1.2f);
            _program.GetUniforms().SetByName("u_Color", glm::vec3(0.04f, 0.08f, 0.10f));
            _wireItem.Draw({ _program.Use() });
            glLineWidth(1.f);
        }

        if (_showParticles) {
            glPointSize(3.f);
            _program.GetUniforms().SetByName("u_Color", glm::vec3(0.98f, 0.95f, 0.65f));
            _particleItem.Draw({ _program.Use() });
            glPointSize(1.f);
        }

        if (_showFixed && !_fixedPositions.empty()) {
            glPointSize(7.f);
            _program.GetUniforms().SetByName("u_Color", glm::vec3(0.85f, 0.20f, 0.14f));
            _fixedItem.Draw({ _program.Use() });
            glPointSize(1.f);
        }

        if (!_selectedPosition.empty()) {
            glDisable(GL_DEPTH_TEST);
            glPointSize(11.f);
            _program.GetUniforms().SetByName("u_Color", glm::vec3(1.0f, 0.86f, 0.18f));
            _selectedItem.Draw({ _program.Use() });
            glPointSize(1.f);

            if (length2(frameControlForce) > 1e-8f) {
                glLineWidth(4.f);
                _program.GetUniforms().SetByName("u_Color", glm::vec3(1.0f, 0.34f, 0.12f));
                _forceItem.Draw({ _program.Use() });
                glLineWidth(1.f);
            }
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

    void CaseSoftBody::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);

        _keyboardForce = glm::vec3(0.f);
        _mouseForce    = glm::vec3(0.f);

        ImGuiIO const & io = ImGui::GetIO();
        bool const focused = ImGui::IsItemFocused();
        bool const hovered = ImGui::IsItemHovered();

        if (focused) {
            if (ImGui::IsKeyDown(ImGuiKey_J)) _keyboardForce.x -= _controlForceMagnitude;
            if (ImGui::IsKeyDown(ImGuiKey_L)) _keyboardForce.x += _controlForceMagnitude;
            if (ImGui::IsKeyDown(ImGuiKey_U)) _keyboardForce.y += _controlForceMagnitude;
            if (ImGui::IsKeyDown(ImGuiKey_O)) _keyboardForce.y -= _controlForceMagnitude;
            if (ImGui::IsKeyDown(ImGuiKey_I)) _keyboardForce.z -= _controlForceMagnitude;
            if (ImGui::IsKeyDown(ImGuiKey_K)) _keyboardForce.z += _controlForceMagnitude;
        }

        bool const dragging = hovered && io.KeyAlt && ImGui::IsMouseDown(ImGuiMouseButton_Left) && (io.MouseDelta.x != 0.f || io.MouseDelta.y != 0.f);
        if (dragging) {
            auto const * window = ImGui::GetCurrentWindow();
            glm::vec3 const front = glm::normalize(_camera.Target - _camera.Eye);
            glm::vec3       right = glm::cross(front, _camera.Up);
            if (length2(right) < 1e-8f) right = glm::vec3(1.f, 0.f, 0.f);
            right = glm::normalize(right);
            glm::vec3 const up = glm::normalize(glm::cross(right, front));

            float const heightNorm = 1.f / std::max(window->Rect().GetHeight(), 1.f);
            float const targetDistance = glm::length(_camera.Target - _camera.Eye);
            float const panScale = 2.f * targetDistance * std::tan(glm::radians(_camera.Fovy) * 0.5f);
            glm::vec3 const deltaWorld =
                right * (io.MouseDelta.x * heightNorm * panScale) +
                up * (-io.MouseDelta.y * heightNorm * panScale);
            _mouseForce = deltaWorld * _mouseForceScale;
        }

        (void) pos;
    }

    void CaseSoftBody::ResetSystem() {
        _stopped        = true;
        glm::vec3 const offsets[3] = {
            glm::vec3(-_size.x * 0.75f - 1.0f, 0.f, 0.f),
            glm::vec3(0.f),
            glm::vec3(_size.x * 0.75f + 1.0f, 0.f, 0.f),
        };
        bool const firstReset = _systems[0].Positions.empty();
        float const young = firstReset ? 20000.f : _systems[0].YoungModulus;
        float const poisson = firstReset ? 0.2f : _systems[0].PoissonRatio;
        float const density = firstReset ? 400.f : _systems[0].Density;
        float const gravity = firstReset ? 0.05f : _systems[0].Gravity;
        float const damping = firstReset ? 1.2f : _systems[0].VelocityDamping;

        for (int i = 0; i < 3; ++i) {
            _systems[i].YoungModulus = young;
            _systems[i].PoissonRatio = poisson;
            _systems[i].Density = density;
            _systems[i].Gravity = gravity;
            _systems[i].VelocityDamping = damping;
            _systems[i].Model = static_cast<MaterialModel>(i);
            _systems[i].ResetBlock(_resolution, _size, offsets[i]);
        }
        _activeModel = 0;
        SetSelectedToFreeEnd();
        UpdateStaticBuffers();
    }

    void CaseSoftBody::UpdateStaticBuffers() {
        _renderTriangles.clear();
        _renderLines.clear();
        std::uint32_t base = 0;
        int const count = _compareMaterials ? 3 : 1;
        for (int s = 0; s < count; ++s) {
            for (auto idx : _systems[s].SurfaceTriangles) _renderTriangles.push_back(base + idx);
            for (auto idx : _systems[s].SurfaceLines) _renderLines.push_back(base + idx);
            base += static_cast<std::uint32_t>(_systems[s].Positions.size());
        }
        _solidItem.UpdateElementBuffer(_renderTriangles);
        _wireItem.UpdateElementBuffer(_renderLines);
    }

    void CaseSoftBody::UpdateDynamicBuffers(glm::vec3 const & frameControlForce) {
        _renderPositions.clear();
        int const count = _compareMaterials ? 3 : 1;
        for (int s = 0; s < count; ++s) {
            _renderPositions.insert(_renderPositions.end(), _systems[s].Positions.begin(), _systems[s].Positions.end());
        }
        if (!_renderPositions.empty()) {
            _solidItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_renderPositions));
            _wireItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_renderPositions));
            _particleItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_renderPositions));
        }

        _fixedPositions.clear();
        for (int s = 0; s < count; ++s) {
            for (std::size_t i = 0; i < _systems[s].Positions.size(); ++i) {
                if (_systems[s].Fixed[i]) {
                    _fixedPositions.push_back(_systems[s].Positions[i]);
                }
            }
        }
        if (!_fixedPositions.empty()) {
            _fixedItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_fixedPositions));
        }

        _selectedPosition.clear();
        _forceLine.clear();
        if (_selectedParticle < _systems[0].Positions.size()) {
            for (int s = 0; s < count; ++s) {
                _selectedPosition.push_back(_systems[s].Positions[_selectedParticle]);
            }
            _selectedItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_selectedPosition));

            glm::vec3 forceViz = frameControlForce;
            float const len = glm::length(forceViz);
            if (len > 1e-8f) {
                forceViz *= std::min(1.2f, len * 0.0012f) / len;
                for (int s = 0; s < count; ++s) {
                    glm::vec3 const p = _systems[s].Positions[_selectedParticle];
                    _forceLine.push_back(p);
                    _forceLine.push_back(p + forceViz);
                }
                _forceItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_forceLine));
            }
        }
    }

    void CaseSoftBody::SetSelectedToFreeEnd() {
        glm::vec3 const target {
            0.5f * _size.x,
            0.5f * _size.y,
            0.f,
        };
        _selectedParticle = _systems[0].FindNearestRestParticle(target);
    }

} // namespace VCX::Labs::FEM
