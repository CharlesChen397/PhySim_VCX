#include <spdlog/spdlog.h>
#include "Engine/app.h"
#include "Labs/2-FluidSimulation/CaseFluid.h"
#include "Labs/Common/ImGuiHelper.h"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace VCX::Labs::Fluid {
    namespace {
        constexpr int   kTetIndices[6][4] = {
            { 0, 5, 1, 6 },
            { 0, 5, 6, 4 },
            { 0, 1, 2, 6 },
            { 0, 2, 3, 6 },
            { 0, 3, 7, 6 },
            { 0, 7, 4, 6 },
        };
        constexpr glm::ivec3 kCubeOffsets[8] = {
            { 0, 0, 0 }, { 1, 0, 0 }, { 1, 1, 0 }, { 0, 1, 0 },
            { 0, 0, 1 }, { 1, 0, 1 }, { 1, 1, 1 }, { 0, 1, 1 },
        };

        Engine::Model makeBoxModel(float halfExtent) {
            Engine::SurfaceMesh mesh;

            auto cornerNormal = [](glm::vec3 const & p) {
                return glm::normalize(p);
            };

            auto addFace = [&](glm::vec3 const & a, glm::vec3 const & b, glm::vec3 const & c, glm::vec3 const & d) {
                std::uint32_t const base = static_cast<std::uint32_t>(mesh.Positions.size());
                mesh.Positions.insert(mesh.Positions.end(), { a, b, c, d });
                mesh.Normals.insert(mesh.Normals.end(), { cornerNormal(a), cornerNormal(b), cornerNormal(c), cornerNormal(d) });
                mesh.Indices.insert(mesh.Indices.end(), { base, base + 1, base + 2, base, base + 2, base + 3 });
            };

            addFace({ halfExtent, -halfExtent, -halfExtent }, { halfExtent, -halfExtent, halfExtent }, { halfExtent, halfExtent, halfExtent }, { halfExtent, halfExtent, -halfExtent });
            addFace({ -halfExtent, -halfExtent, halfExtent }, { -halfExtent, -halfExtent, -halfExtent }, { -halfExtent, halfExtent, -halfExtent }, { -halfExtent, halfExtent, halfExtent });
            addFace({ -halfExtent, halfExtent, -halfExtent }, { halfExtent, halfExtent, -halfExtent }, { halfExtent, halfExtent, halfExtent }, { -halfExtent, halfExtent, halfExtent });
            addFace({ -halfExtent, -halfExtent, halfExtent }, { halfExtent, -halfExtent, halfExtent }, { halfExtent, -halfExtent, -halfExtent }, { -halfExtent, -halfExtent, -halfExtent });
            addFace({ -halfExtent, -halfExtent, halfExtent }, { -halfExtent, halfExtent, halfExtent }, { halfExtent, halfExtent, halfExtent }, { halfExtent, -halfExtent, halfExtent });
            addFace({ halfExtent, -halfExtent, -halfExtent }, { halfExtent, halfExtent, -halfExtent }, { -halfExtent, halfExtent, -halfExtent }, { -halfExtent, -halfExtent, -halfExtent });

            return Engine::Model { mesh, 0 };
        }

        ImVec2 projectToCanvas(Engine::Camera const & camera, glm::vec3 const & worldPos, std::pair<std::uint32_t, std::uint32_t> canvasSize, bool & visible) {
            glm::mat4 const proj = camera.GetProjectionMatrix(float(canvasSize.first) / float(canvasSize.second));
            glm::mat4 const view = camera.GetViewMatrix();
            glm::vec4 const clip = proj * view * glm::vec4(worldPos, 1.0f);

            visible = clip.w > 0.0f;
            if (!visible) {
                return ImVec2(0.0f, 0.0f);
            }

            glm::vec3 const ndc = glm::vec3(clip) / clip.w;
            visible = ndc.x >= -1.2f && ndc.x <= 1.2f && ndc.y >= -1.2f && ndc.y <= 1.2f && ndc.z >= -1.2f && ndc.z <= 1.2f;

            return ImVec2(
                (ndc.x * 0.5f + 0.5f) * float(canvasSize.first),
                (1.0f - (ndc.y * 0.5f + 0.5f)) * float(canvasSize.second));
        }

    } // namespace

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
        _surfaceProgram(
            Engine::GL::UniqueProgram({
                Engine::GL::SharedShader("assets/shaders/flat.vert"),
                Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _sceneObject(1),
        _BoundaryItem(Engine::GL::VertexLayout()
            .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines),
        _surfaceItem(Engine::GL::VertexLayout()
            .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles) {
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
        _obstacleBox = makeBoxModel(_obstacleRadius);
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
        ImGui::Checkbox("Mesh", &_showMesh);
        if (_showMesh) {
            ImGui::SliderInt("Mesh Detail", &_meshDetail, 1, 4);
        }
        int obstacleType = static_cast<int>(_obstacleShape);
        if (ImGui::Combo("Obstacle Type", &obstacleType, "Sphere\0Cube\0")) {
            _obstacleShape = static_cast<ObstacleShape>(obstacleType);
            _simulation.setObstacle(_obstacleShape, _obstaclePos, glm::vec3(0.0f), _obstacleRadius, true);
        }
        ImGui::Text("FLIP Ratio: 0.0 = PIC, 1.0 = FLIP, 0.95 = FLIP95");
        ImGui::Text("Z + Left Drag On Obstacle: move obstacle");

        ImGui::Spacing();
        ImGui::Text("Particles: %d", _simulation.m_iNumSpheres);
        ImGui::Text("Grid: %dx%dx%d", _simulation.m_iCellX, _simulation.m_iCellY, _simulation.m_iCellZ);
    }

    Common::CaseRenderResult CaseFluid::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        _canvasSize = desiredSize;
        SyncObstacle(Engine::GetDeltaTime());
        if (!_stopped) {
            _simulation.m_fRatio = _flipRatio;
            _simulation.SimulateTimestep(_timeStep);
        }
        if (_showMesh) {
            UpdateSurfaceMesh();
        }

        _BoundaryItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(vertex_pos));
        _frame.Resize(desiredSize);

        _cameraManager.Update(_sceneObject.Camera);
        _sceneObject.PassConstantsBlock.Update(&VCX::Labs::Rendering::SceneObject::PassConstants::Projection, _sceneObject.Camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _sceneObject.PassConstantsBlock.Update(&VCX::Labs::Rendering::SceneObject::PassConstants::View, _sceneObject.Camera.GetViewMatrix());
        _sceneObject.PassConstantsBlock.Update(&VCX::Labs::Rendering::SceneObject::PassConstants::ViewPosition, _sceneObject.Camera.Eye);
        _lineprogram.GetUniforms().SetByName("u_Projection", _sceneObject.Camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _lineprogram.GetUniforms().SetByName("u_View", _sceneObject.Camera.GetViewMatrix());
        _surfaceProgram.GetUniforms().SetByName("u_Projection", _sceneObject.Camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _surfaceProgram.GetUniforms().SetByName("u_View", _sceneObject.Camera.GetViewMatrix());
        _surfaceProgram.GetUniforms().SetByName("u_Color", glm::vec3(0.18f, 0.52f, 0.94f));

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

        if (_showMesh) {
            _surfaceItem.Draw({ _surfaceProgram.Use() });
        } else {
            Rendering::ModelObject m = Rendering::ModelObject(_sphere, _simulation.m_particlePos, _simulation.m_particleColor);
            m.Mesh.Draw({ _program.Use() },
                _sphere.Mesh.Indices.size(), 0, _simulation.m_iNumSpheres);
        }

        std::vector<glm::vec3> obstaclePos { _obstaclePos };
        std::vector<glm::vec3> obstacleColor { glm::vec3(0.95f, 0.6f, 0.2f) };
        Engine::Model const & obstacleModel = _obstacleShape == ObstacleShape::Sphere ? _obstacleSphere : _obstacleBox;
        Rendering::ModelObject obstacle = Rendering::ModelObject(obstacleModel, obstaclePos, obstacleColor);
        obstacle.Mesh.Draw({ _program.Use() }, obstacleModel.Mesh.Indices.size(), 0, 1);

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
        bool const modifierHeld = ImGui::IsKeyDown(ImGuiKey_Z);
        bool const leftDown = ImGui::IsMouseDown(ImGuiMouseButton_Left);

        if (_draggingObstacle && (!leftDown || !modifierHeld)) {
            _draggingObstacle = false;
        }

        if (!_draggingObstacle && hovered && modifierHeld && ImGui::IsMouseClicked(ImGuiMouseButton_Left) && IsMouseOnObstacle(pos)) {
            _draggingObstacle = true;
        }

        if (_draggingObstacle && leftDown && modifierHeld) {
            auto const * window = ImGui::GetCurrentWindow();
            glm::vec3    front  = glm::normalize(_sceneObject.Camera.Target - _sceneObject.Camera.Eye);
            glm::vec3    right  = glm::normalize(glm::cross(front, _sceneObject.Camera.Up));
            float        heightNorm = 1.0f / window->Rect().GetHeight();
            float        targetDistance = glm::length(_sceneObject.Camera.Target - _sceneObject.Camera.Eye);
            float        panScale = 2.0f * targetDistance * std::tan(glm::radians(_sceneObject.Camera.Fovy) * 0.5f);
            glm::vec3    deltaWorld =
                right * (io.MouseDelta.x * heightNorm * panScale) +
                _sceneObject.Camera.Up * (-io.MouseDelta.y * heightNorm * panScale);

            _obstaclePos += deltaWorld;
            float const bound = 0.5f - 2.0f * _simulation.m_h - _obstacleRadius;
            _obstaclePos.x = std::clamp(_obstaclePos.x, -bound, bound);
            _obstaclePos.y = std::clamp(_obstaclePos.y, -bound, bound);
            _obstaclePos.z = std::clamp(_obstaclePos.z, -bound, bound);
            _obstacleDraggedThisFrame = true;
            return;
        }

        _cameraManager.ProcessInput(_sceneObject.Camera, pos);
    }

    void CaseFluid::ResetSystem() {
        _simulation.setupScene(_res);
        _sphere = Engine::Model { Engine::Sphere(6, _simulation.m_particleRadius), 0 };
        _obstacleSphere = Engine::Model { Engine::Sphere(10, _obstacleRadius), 0 };
        _obstacleBox = makeBoxModel(_obstacleRadius);
        _simulation.setObstacle(_obstacleShape, _obstaclePos, glm::vec3(0.0f), _obstacleRadius, true);
        _simulation.transferVelocities(true, _flipRatio);
        _simulation.updateParticleDensity();
        _simulation.updateParticleColors();
        UpdateSurfaceMesh();
        _stopped = true; // 初始静止
    }

    void CaseFluid::SyncObstacle(float dt) {
        if (_obstacleDraggedThisFrame) {
            _obstacleVel = (_obstaclePos - _simulation.m_obstaclePos) / std::max(dt, 1e-6f);
        } else {
            _obstacleVel = glm::vec3(0.0f);
        }
        _simulation.setObstacle(_obstacleShape, _obstaclePos, _obstacleVel, _obstacleRadius, true);
        _obstacleDraggedThisFrame = false;
    }

    bool CaseFluid::IsMouseOnObstacle(ImVec2 const & pos) const {
        bool  centerVisible = false;
        ImVec2 const center = projectToCanvas(_sceneObject.Camera, _obstaclePos, _canvasSize, centerVisible);
        if (!centerVisible) {
            return false;
        }

        float radius = 0.0f;
        if (_obstacleShape == ObstacleShape::Sphere) {
            glm::vec3 const front = glm::normalize(_sceneObject.Camera.Target - _sceneObject.Camera.Eye);
            glm::vec3 const right = glm::normalize(glm::cross(front, _sceneObject.Camera.Up));
            glm::vec3 const up    = glm::normalize(_sceneObject.Camera.Up);

            bool  rightVisible = false;
            bool  upVisible    = false;
            ImVec2 const rightEdge = projectToCanvas(_sceneObject.Camera, _obstaclePos + right * _obstacleRadius, _canvasSize, rightVisible);
            ImVec2 const upEdge    = projectToCanvas(_sceneObject.Camera, _obstaclePos + up * _obstacleRadius, _canvasSize, upVisible);
            if (!rightVisible && !upVisible) {
                return false;
            }

            if (rightVisible) {
                float const dx = rightEdge.x - center.x;
                float const dy = rightEdge.y - center.y;
                radius = std::max(radius, std::sqrt(dx * dx + dy * dy));
            }
            if (upVisible) {
                float const dx = upEdge.x - center.x;
                float const dy = upEdge.y - center.y;
                radius = std::max(radius, std::sqrt(dx * dx + dy * dy));
            }
        } else {
            for (int dx = -1; dx <= 1; dx += 2) {
                for (int dy = -1; dy <= 1; dy += 2) {
                    for (int dz = -1; dz <= 1; dz += 2) {
                        bool const cornerVisible = [&]() {
                            bool visible = false;
                            ImVec2 const corner = projectToCanvas(
                                _sceneObject.Camera,
                                _obstaclePos + glm::vec3(dx, dy, dz) * _obstacleRadius,
                                _canvasSize,
                                visible);
                            if (visible) {
                                float const ddx = corner.x - center.x;
                                float const ddy = corner.y - center.y;
                                radius = std::max(radius, std::sqrt(ddx * ddx + ddy * ddy));
                            }
                            return visible;
                        }();
                        (void) cornerVisible;
                    }
                }
            }
        }

        radius = std::max(radius, 8.0f);
        float const dx = pos.x - center.x;
        float const dy = pos.y - center.y;
        return dx * dx + dy * dy <= radius * radius;
    }

    void CaseFluid::UpdateSurfaceMesh() {
        int const nx = _simulation.cellCountX();
        int const ny = _simulation.cellCountY();
        int const nz = _simulation.cellCountZ();
        if (nx < 2 || ny < 2 || nz < 2) {
            return;
        }

        int const refine = std::max(1, _meshDetail);
        int const fx = (nx - 1) * refine + 1;
        int const fy = (ny - 1) * refine + 1;
        int const fz = (nz - 1) * refine + 1;
        float const fineH = _simulation.m_h / float(refine);
        float const restDensity = std::max(_simulation.m_particleRestDensity, 1e-4f);
        float const isoValue    = 0.55f;

        auto coarseDensityAt = [&](int i, int j, int k) {
            i = std::clamp(i, 0, nx - 1);
            j = std::clamp(j, 0, ny - 1);
            k = std::clamp(k, 0, nz - 1);
            return _simulation.m_particleDensity[_simulation.index2GridOffset(glm::ivec3(i, j, k))] / restDensity;
        };

        auto densityAt = [&](int i, int j, int k) {
            float gx = float(i) / float(refine);
            float gy = float(j) / float(refine);
            float gz = float(k) / float(refine);

            int const x0 = std::clamp(int(std::floor(gx)), 0, nx - 1);
            int const y0 = std::clamp(int(std::floor(gy)), 0, ny - 1);
            int const z0 = std::clamp(int(std::floor(gz)), 0, nz - 1);
            int const x1 = std::min(x0 + 1, nx - 1);
            int const y1 = std::min(y0 + 1, ny - 1);
            int const z1 = std::min(z0 + 1, nz - 1);

            float const tx = std::clamp(gx - float(x0), 0.0f, 1.0f);
            float const ty = std::clamp(gy - float(y0), 0.0f, 1.0f);
            float const tz = std::clamp(gz - float(z0), 0.0f, 1.0f);

            float const c000 = coarseDensityAt(x0, y0, z0);
            float const c100 = coarseDensityAt(x1, y0, z0);
            float const c010 = coarseDensityAt(x0, y1, z0);
            float const c110 = coarseDensityAt(x1, y1, z0);
            float const c001 = coarseDensityAt(x0, y0, z1);
            float const c101 = coarseDensityAt(x1, y0, z1);
            float const c011 = coarseDensityAt(x0, y1, z1);
            float const c111 = coarseDensityAt(x1, y1, z1);

            float const c00 = glm::mix(c000, c100, tx);
            float const c10 = glm::mix(c010, c110, tx);
            float const c01 = glm::mix(c001, c101, tx);
            float const c11 = glm::mix(c011, c111, tx);
            float const c0 = glm::mix(c00, c10, ty);
            float const c1 = glm::mix(c01, c11, ty);
            return glm::mix(c0, c1, tz);
        };

        auto emitTriangle = [&](std::vector<glm::vec3> & positions, std::vector<std::uint32_t> & indices, glm::vec3 const & a, glm::vec3 const & b, glm::vec3 const & c) {
            std::uint32_t const base = static_cast<std::uint32_t>(positions.size());
            positions.insert(positions.end(), { a, b, c });
            indices.insert(indices.end(), { base, base + 1, base + 2 });
        };

        auto interpolate = [&](glm::vec3 const & pa, float va, glm::vec3 const & pb, float vb) {
            float const denom = std::abs(vb - va) > 1e-6f ? (vb - va) : 1e-6f;
            float const t = std::clamp((isoValue - va) / denom, 0.0f, 1.0f);
            return glm::mix(pa, pb, t);
        };

        std::vector<glm::vec3>     positions;
        std::vector<std::uint32_t> indices;
        positions.reserve(30000);
        indices.reserve(30000);

        for (int i = 0; i < fx - 1; ++i) {
            for (int j = 0; j < fy - 1; ++j) {
                for (int k = 0; k < fz - 1; ++k) {
                    glm::vec3 nodePos[8];
                    float     nodeValue[8];
                    int       insideCube = 0;

                    for (int v = 0; v < 8; ++v) {
                        glm::ivec3 const coord = glm::ivec3(i, j, k) + kCubeOffsets[v];
                        nodePos[v]   = glm::vec3(0.5f * _simulation.m_h - 0.5f) + glm::vec3(coord) * fineH;
                        nodeValue[v] = densityAt(coord.x, coord.y, coord.z);
                        insideCube  += nodeValue[v] >= isoValue ? 1 : 0;
                    }

                    if (insideCube == 0 || insideCube == 8) {
                        continue;
                    }

                    for (auto const & tet : kTetIndices) {
                        int inside[4];
                        int outside[4];
                        int insideCount  = 0;
                        int outsideCount = 0;

                        for (int t = 0; t < 4; ++t) {
                            int const id = tet[t];
                            if (nodeValue[id] >= isoValue) {
                                inside[insideCount++] = id;
                            } else {
                                outside[outsideCount++] = id;
                            }
                        }

                        if (insideCount == 0 || insideCount == 4) {
                            continue;
                        }

                        if (insideCount == 1 || insideCount == 3) {
                            int const apex = insideCount == 1 ? inside[0] : outside[0];
                            int const * rim = insideCount == 1 ? outside : inside;
                            glm::vec3 const a = interpolate(nodePos[apex], nodeValue[apex], nodePos[rim[0]], nodeValue[rim[0]]);
                            glm::vec3 const b = interpolate(nodePos[apex], nodeValue[apex], nodePos[rim[1]], nodeValue[rim[1]]);
                            glm::vec3 const c = interpolate(nodePos[apex], nodeValue[apex], nodePos[rim[2]], nodeValue[rim[2]]);
                            if (insideCount == 1) {
                                emitTriangle(positions, indices, a, b, c);
                            } else {
                                emitTriangle(positions, indices, a, c, b);
                            }
                            continue;
                        }

                        glm::vec3 const ac = interpolate(nodePos[inside[0]], nodeValue[inside[0]], nodePos[outside[0]], nodeValue[outside[0]]);
                        glm::vec3 const ad = interpolate(nodePos[inside[0]], nodeValue[inside[0]], nodePos[outside[1]], nodeValue[outside[1]]);
                        glm::vec3 const bc = interpolate(nodePos[inside[1]], nodeValue[inside[1]], nodePos[outside[0]], nodeValue[outside[0]]);
                        glm::vec3 const bd = interpolate(nodePos[inside[1]], nodeValue[inside[1]], nodePos[outside[1]], nodeValue[outside[1]]);
                        emitTriangle(positions, indices, ac, ad, bc);
                        emitTriangle(positions, indices, ad, bd, bc);
                    }
                }
            }
        }

        _surfaceItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(positions));
        _surfaceItem.UpdateElementBuffer(indices);
    }
}
