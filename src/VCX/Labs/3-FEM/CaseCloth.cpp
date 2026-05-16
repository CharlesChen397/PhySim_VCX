#include "Labs/3-FEM/CaseCloth.h"

#include <algorithm>
#include <cmath>

#include <glm/gtc/type_ptr.hpp>
#include <imgui.h>

#include "Engine/app.h"

namespace VCX::Labs::FEM {

    namespace {
        float length2(glm::vec3 const & v) {
            return glm::dot(v, v);
        }

        glm::vec3 clampForce(glm::vec3 const & f, float maxLen) {
            float const len = glm::length(f);
            if (len <= maxLen || len <= 1e-8f) return f;
            return f * (maxLen / len);
        }
    } // namespace

    CaseCloth::CaseCloth():
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

    void CaseCloth::OnSetupPropsUI() {
        if (ImGui::Button("Reset System")) ResetSystem();
        ImGui::SameLine();
        if (ImGui::Button(_stopped ? "Start Simulation" : "Stop Simulation")) _stopped = !_stopped;

        ImGui::Separator();
        ImGui::Text("FEM cloth: F is 3x2");
        ImGui::SliderFloat("Time Step", &_timeStep, 0.0002f, 0.003f, "%.4f");
        ImGui::SliderInt("Steps / Frame", &_stepsPerFrame, 1, 32);
        ImGui::SliderFloat("Young Modulus", &_system.YoungModulus, 5.f, 300.f, "%.1f");
        ImGui::SliderFloat("Poisson Ratio", &_system.PoissonRatio, 0.0f, 0.45f, "%.2f");
        if (ImGui::SliderFloat("Area Density", &_system.AreaDensity, 0.05f, 2.f, "%.2f")) _system.RecomputeMasses();
        ImGui::SliderFloat("Gravity", &_system.Gravity, 0.f, 15.f, "%.2f");
        ImGui::SliderFloat("Velocity Damping", &_system.VelocityDamping, 0.f, 2.f, "%.2f");
        ImGui::SliderFloat("Bending Springs", &_system.BendingStiffness, 0.f, 2.f, "%.3f");

        ImGui::Separator();
        bool meshEdited = false;
        int  nx = static_cast<int>(_resolution.X);
        int  ny = static_cast<int>(_resolution.Y);
        meshEdited |= ImGui::SliderInt("Cells U", &nx, 4, 48);
        meshEdited |= ImGui::SliderInt("Cells V", &ny, 4, 48);
        meshEdited |= ImGui::SliderFloat2("Cloth Size", glm::value_ptr(_size), 0.5f, 4.f, "%.2f");
        if (meshEdited) {
            _resolution.X = static_cast<std::size_t>(std::max(nx, 4));
            _resolution.Y = static_cast<std::size_t>(std::max(ny, 4));
            _size = glm::max(_size, glm::vec2(0.25f));
            ResetSystem();
        }

        ImGui::Separator();
        int maxParticle = std::max(0, static_cast<int>(_system.Positions.size()) - 1);
        int selected = static_cast<int>(std::min<std::size_t>(_selectedParticle, std::size_t(maxParticle)));
        if (ImGui::SliderInt("Control Particle", &selected, 0, maxParticle)) _selectedParticle = static_cast<std::size_t>(selected);
        if (ImGui::Button("Select Free Corner")) SetSelectedToFreeCorner();
        ImGui::SliderFloat("Control Force", &_controlForceMagnitude, 0.f, 200.f, "%.1f");
        ImGui::SliderFloat("Mouse Force Scale", &_mouseForceScale, 0.f, 200.f, "%.1f");

        auto forceButton = [&](char const * label, glm::vec3 dir) {
            if (ImGui::Button(label)) _buttonForce += dir * _controlForceMagnitude;
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
        ImGui::Text("Alt + Left Drag: pull selected vertex");

        ImGui::Separator();
        ImGui::Checkbox("Surface", &_showSurface);
        ImGui::Checkbox("Wireframe", &_showWireframe);
        ImGui::Checkbox("Particles", &_showParticles);
        ImGui::Checkbox("Fixed Points", &_showFixed);
    }

    Common::CaseRenderResult CaseCloth::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        glm::vec3 const frameControlForce = clampForce(_keyboardForce + _mouseForce + _buttonForce, 400.f);
        if (!_stopped) {
            for (int i = 0; i < std::max(1, _stepsPerFrame); ++i) {
                _system.Step(_timeStep, _selectedParticle, frameControlForce);
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
            _program.GetUniforms().SetByName("u_Color", glm::vec3(0.88f, 0.62f, 0.32f));
            _solidItem.Draw({ _program.Use() });
        }
        if (_showWireframe) {
            glLineWidth(1.f);
            _program.GetUniforms().SetByName("u_Color", glm::vec3(0.10f, 0.08f, 0.06f));
            _wireItem.Draw({ _program.Use() });
        }
        if (_showParticles) {
            glPointSize(3.f);
            _program.GetUniforms().SetByName("u_Color", glm::vec3(0.95f, 0.95f, 0.75f));
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
            glPointSize(10.f);
            _program.GetUniforms().SetByName("u_Color", glm::vec3(1.0f, 0.90f, 0.18f));
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

    void CaseCloth::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
        _keyboardForce = glm::vec3(0.f);
        _mouseForce = glm::vec3(0.f);

        ImGuiIO const & io = ImGui::GetIO();
        if (ImGui::IsItemFocused()) {
            if (ImGui::IsKeyDown(ImGuiKey_J)) _keyboardForce.x -= _controlForceMagnitude;
            if (ImGui::IsKeyDown(ImGuiKey_L)) _keyboardForce.x += _controlForceMagnitude;
            if (ImGui::IsKeyDown(ImGuiKey_U)) _keyboardForce.y += _controlForceMagnitude;
            if (ImGui::IsKeyDown(ImGuiKey_O)) _keyboardForce.y -= _controlForceMagnitude;
            if (ImGui::IsKeyDown(ImGuiKey_I)) _keyboardForce.z -= _controlForceMagnitude;
            if (ImGui::IsKeyDown(ImGuiKey_K)) _keyboardForce.z += _controlForceMagnitude;
        }

        bool const dragging = ImGui::IsItemHovered() && io.KeyAlt && ImGui::IsMouseDown(ImGuiMouseButton_Left) && (io.MouseDelta.x != 0.f || io.MouseDelta.y != 0.f);
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
            _mouseForce = (right * (io.MouseDelta.x * heightNorm * panScale) + up * (-io.MouseDelta.y * heightNorm * panScale)) * _mouseForceScale;
        }
        (void) pos;
    }

    void CaseCloth::ResetSystem() {
        _stopped = true;
        bool const firstReset = _system.Positions.empty();
        if (firstReset) {
            _system.YoungModulus = 50.f;
            _system.PoissonRatio = 0.3f;
            _system.AreaDensity = 0.5f;
            _system.Gravity = 9.8f;
            _system.VelocityDamping = 0.18f;
            _system.BendingStiffness = 0.04f;
        }
        _system.ResetSheet(_resolution, _size);
        SetSelectedToFreeCorner();
        UpdateStaticBuffers();
    }

    void CaseCloth::UpdateStaticBuffers() {
        _solidItem.UpdateElementBuffer(_system.SurfaceTriangles);
        _wireItem.UpdateElementBuffer(_system.SurfaceLines);
    }

    void CaseCloth::UpdateDynamicBuffers(glm::vec3 const & frameControlForce) {
        _solidItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_system.Positions));
        _wireItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_system.Positions));
        _particleItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_system.Positions));

        _fixedPositions.clear();
        for (std::size_t i = 0; i < _system.Positions.size(); ++i) {
            if (_system.Fixed[i]) _fixedPositions.push_back(_system.Positions[i]);
        }
        if (!_fixedPositions.empty()) _fixedItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_fixedPositions));

        _selectedPosition.clear();
        _forceLine.clear();
        if (_selectedParticle < _system.Positions.size()) {
            glm::vec3 const p = _system.Positions[_selectedParticle];
            _selectedPosition.push_back(p);
            _selectedItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_selectedPosition));
            float const len = glm::length(frameControlForce);
            if (len > 1e-8f) {
                glm::vec3 const forceViz = frameControlForce * (std::min(0.6f, len * 0.015f) / len);
                _forceLine.push_back(p);
                _forceLine.push_back(p + forceViz);
                _forceItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_forceLine));
            }
        }
    }

    void CaseCloth::SetSelectedToFreeCorner() {
        _selectedParticle = _system.GetID(_resolution.X, _resolution.Y);
    }

} // namespace VCX::Labs::FEM
