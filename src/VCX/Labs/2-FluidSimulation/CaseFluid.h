#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/UniformBlock.hpp"
#include "Engine/Sphere.h"
#include "Labs/2-FluidSimulation/FluidSimulator.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/Scene/Content.h"
#include "Labs/Scene/SceneObject.h"

namespace VCX::Labs::Fluid {

    class CaseFluid : public Common::ICase {
    public:
        CaseFluid();

        virtual std::string_view const GetName() override { return "FLIP Fluid Simulation"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

    private:
        Engine::GL::UniqueProgram         _program;
        Engine::GL::UniqueProgram         _lineprogram;
        Engine::GL::UniqueRenderFrame     _frame;
        VCX::Labs::Rendering::SceneObject _sceneObject;
        bool                              _uniformDirty { true };
        float                             _shininess { 32 };
        float                             _ambientScale { 1 };
        bool                              _useGammaCorrection { true };
        int                               _attenuationOrder { 2 };

        Engine::GL::UniqueIndexedRenderItem _BoundaryItem;
        Common::OrbitCameraManager          _cameraManager;
        float                               _BndWidth { 2.0 };
        bool                                _stopped { false };
        Engine::Model                       _sphere;
        Engine::Model                       _obstacleSphere;
        int                                 _res { 24 };
        float                               _timeStep { 0.016f };
        float                               _flipRatio { 0.95f };
        glm::vec3                           _obstaclePos { 0.18f, -0.1f, 0.0f };
        glm::vec3                           _obstacleVel { 0.0f };
        float                               _obstacleRadius { 0.09f };
        bool                                _draggingObstacle { false };

        Simulator _simulation;

        void ResetSystem();
        void SyncObstacle(float dt);
    };
} // namespace VCX::Labs::Fluid
