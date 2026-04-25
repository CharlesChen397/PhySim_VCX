#include <spdlog/spdlog.h>
#include "Engine/app.h"
#include "Labs/2-FluidSimulation/CaseFluid.h"
#include "Labs/Common/ImGuiHelper.h"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace VCX::Labs::Fluid {
    const std::vector<glm::vec3> vertex_pos = {
        glm::vec3(-0.5f, -0.5f, -0.5f),
        glm::vec3(0.5f, -0.5f, -0.5f),
        glm::vec3(0.5f, 0.5f, -0.5f),
        glm::vec3(-0.5f, 0.5f, -0.5f),
        glm::vec3(-0.5f, -0.5f, 0.5f),
        glm::vec3(0.5f, -0.5f, 0.5f),
        glm::vec3(0.5f, 0.5f, 0.5f),
        glm::vec3(-0.5f, 0.5f, 0.5f)
    };
    const std::vector<std::uint32_t> line_index = { 0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7 };

    CaseFluid::CaseFluid() :
        _program(
            Engine::GL::UniqueProgram({
                Engine::GL::SharedShader("assets/shaders/fluid.vert"),
                Engine::GL::SharedShader("assets/shaders/fluid.frag") })),
        _lineprogram(
            Engine::GL::UniqueProgram({
                Engine::GL::SharedShader("assets/shaders/flat.vert"),
                Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _sceneObject(1),
        _BoundaryItem(Engine::GL::VertexLayout()
            .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines) {
        _cameraManager.AutoRotate = false;
        _program.BindUniformBlock("PassConstants", 1);
        _lineprogram.GetUniforms().SetByName("u_Color", glm::vec3(1.0f));
        _BoundaryItem.UpdateElementBuffer(line_index);

        // 初始化场景
        _sceneObject.ReplaceScene(VCX::Labs::Rendering::Content::Scenes[0]);
        _cameraManager.Save(_sceneObject.Camera);

        ResetSystem();
        _sphere = Engine::Model { Engine::Sphere(6, _simulation.m_particleRadius), 0 };
        _obstacleSphere = Engine::Model { Engine::Sphere(10, _obstacleRadius), 0 };
    }

    void CaseFluid::OnSetupPropsUI() {
        if (ImGui::Button("Reset System"))
            ResetSystem();
        ImGui::SameLine();
        if (ImGui::Button(_stopped ? "Start Simulation" : "Stop Simulation"))
            _stopped = !_stopped;
        ImGui::Spacing();

        ImGui::Text("Simulation Parameters:");
        ImGui::SliderFloat("Time Step", &_timeStep, 0.001f, 0.05f, "%.4f");
        ImGui::SliderFloat("FLIP Ratio", &_flipRatio, 0.0f, 1.0f, "%.2f");
        ImGui::Text("FLIP Ratio: 0.0 = PIC, 1.0 = FLIP, 0.95 = FLIP95");
        ImGui::Text("Cmd + Left Drag: move obstacle");

        ImGui::Spacing();
        ImGui::Text("Particles: %d", _simulation.m_iNumSpheres);
        ImGui::Text("Grid: %dx%dx%d", _simulation.m_iCellX, _simulation.m_iCellY, _simulation.m_iCellZ);
    }

    Common::CaseRenderResult CaseFluid::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        SyncObstacle(Engine::GetDeltaTime());
        if (!_stopped) {
            _simulation.m_fRatio = _flipRatio;
            _simulation.SimulateTimestep(_timeStep);
        }

        _BoundaryItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(vertex_pos));
        _frame.Resize(desiredSize);

        _cameraManager.Update(_sceneObject.Camera);
        _sceneObject.PassConstantsBlock.Update(&VCX::Labs::Rendering::SceneObject::PassConstants::Projection, _sceneObject.Camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _sceneObject.PassConstantsBlock.Update(&VCX::Labs::Rendering::SceneObject::PassConstants::View, _sceneObject.Camera.GetViewMatrix());
        _sceneObject.PassConstantsBlock.Update(&VCX::Labs::Rendering::SceneObject::PassConstants::ViewPosition, _sceneObject.Camera.Eye);
        _lineprogram.GetUniforms().SetByName("u_Projection", _sceneObject.Camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _lineprogram.GetUniforms().SetByName("u_View", _sceneObject.Camera.GetViewMatrix());

        if (_uniformDirty) {
            _uniformDirty = false;
            _program.GetUniforms().SetByName("u_AmbientScale", _ambientScale);
            _program.GetUniforms().SetByName("u_UseBlinn", 0);
            _program.GetUniforms().SetByName("u_Shininess", _shininess);
            _program.GetUniforms().SetByName("u_UseGammaCorrection", int(_useGammaCorrection));
            _program.GetUniforms().SetByName("u_AttenuationOrder", _attenuationOrder);
        }

        gl_using(_frame);

        glEnable(GL_DEPTH_TEST);
        glLineWidth(_BndWidth);
        _BoundaryItem.Draw({ _lineprogram.Use() });
        glLineWidth(1.f);

        Rendering::ModelObject m = Rendering::ModelObject(_sphere, _simulation.m_particlePos, _simulation.m_particleColor);
        m.Mesh.Draw({ _program.Use() },
            _sphere.Mesh.Indices.size(), 0, _simulation.m_iNumSpheres);

        std::vector<glm::vec3> obstaclePos { _obstaclePos };
        std::vector<glm::vec3> obstacleColor { glm::vec3(0.95f, 0.6f, 0.2f) };
        Rendering::ModelObject obstacle = Rendering::ModelObject(_obstacleSphere, obstaclePos, obstacleColor);
        obstacle.Mesh.Draw({ _program.Use() }, _obstacleSphere.Mesh.Indices.size(), 0, 1);

        glDepthFunc(GL_LEQUAL);
        glDepthFunc(GL_LESS);
        glDisable(GL_DEPTH_TEST);

        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

    void CaseFluid::OnProcessInput(ImVec2 const & pos) {
        ImGuiIO const & io = ImGui::GetIO();
        bool const hovered = ImGui::IsItemHovered();
        bool const obstacleDrag = hovered && ImGui::IsMouseDown(ImGuiMouseButton_Left) && ImGui::IsKeyDown(ImGuiKey_ModSuper);

        if (obstacleDrag) {
            auto const * window = ImGui::GetCurrentWindow();
            glm::vec3    front  = glm::normalize(_sceneObject.Camera.Target - _sceneObject.Camera.Eye);
            glm::vec3    right  = glm::normalize(glm::cross(front, _sceneObject.Camera.Up));
            float        heightNorm = 1.0f / window->Rect().GetHeight();
            float        targetDistance = glm::length(_sceneObject.Camera.Target - _sceneObject.Camera.Eye);
            float        panScale = 2.0f * targetDistance * std::tan(glm::radians(_sceneObject.Camera.Fovy) * 0.5f);
            glm::vec3    deltaWorld =
                right * (-io.MouseDelta.x * heightNorm * panScale) +
                _sceneObject.Camera.Up * (io.MouseDelta.y * heightNorm * panScale);

            _obstaclePos += deltaWorld;
            float const bound = 0.5f - 2.0f * _simulation.m_h - _obstacleRadius;
            _obstaclePos.x = std::clamp(_obstaclePos.x, -bound, bound);
            _obstaclePos.y = std::clamp(_obstaclePos.y, -bound, bound);
            _obstaclePos.z = std::clamp(_obstaclePos.z, -bound, bound);
            _draggingObstacle = true;
            return;
        }

        _cameraManager.ProcessInput(_sceneObject.Camera, pos);
    }

    void CaseFluid::ResetSystem() {
        _simulation.setupScene(_res);
        _sphere = Engine::Model { Engine::Sphere(6, _simulation.m_particleRadius), 0 };
        _obstacleSphere = Engine::Model { Engine::Sphere(10, _obstacleRadius), 0 };
        _simulation.setObstacle(_obstaclePos, glm::vec3(0.0f), _obstacleRadius, true);
        _stopped = true; // 初始静止
    }

    void CaseFluid::SyncObstacle(float dt) {
        if (_draggingObstacle) {
            _obstacleVel = (_obstaclePos - _simulation.m_obstaclePos) / std::max(dt, 1e-6f);
        } else {
            _obstacleVel = glm::vec3(0.0f);
        }
        _simulation.setObstacle(_obstaclePos, _obstacleVel, _obstacleRadius, true);
        _draggingObstacle = false;
    }
}
