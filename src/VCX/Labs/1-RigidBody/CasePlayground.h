#pragma once

#include <array>
#include <string_view>

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/1-RigidBody/RigidBodySystem.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/OrbitCameraManager.h"

namespace VCX::Labs::RigidBody {

    class CasePlayground : public Common::ICase {
    public:
        CasePlayground();

        std::string_view const GetName() override { return "Rigid Body Playground"; }

        void                     OnSetupPropsUI() override;
        Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        void                     OnProcessInput(ImVec2 const & pos) override;

    private:
        enum class Preset : int {
            SingleBody               = 0,
            DoubleEdge               = 1,
            DoubleVertexFace         = 2,
            DoubleFaceFace           = 3,
            ComplexScene             = 4,
            BonusNewtonCradle        = 5,
            BonusStacking            = 6,
            BonusMixedPrimitives     = 7,
            BonusConstrainedContacts = 8,
            BonusArticulatedBodies   = 9,
            BonusCCD                 = 10,
        };

        Engine::GL::UniqueProgram           _program;
        Engine::GL::UniqueRenderFrame       _frame;
        Engine::GL::UniqueIndexedRenderItem _solidItem;
        Engine::GL::UniqueIndexedRenderItem _wireItem;
        Engine::GL::UniqueIndexedRenderItem _impulseItem;

        Engine::Camera             _camera { .Eye = glm::vec3(-8.f, 6.f, 8.f), .Target = glm::vec3(0.f, 0.f, 0.f) };
        Common::OrbitCameraManager _cameraManager;

        RigidBodySystem _system;

        Preset _preset { Preset::SingleBody };
        bool   _stopped { true };
        bool   _presetDirty { true };
        bool   _showWireframe { true };
        bool   _showImpulseViz { true };
        float  _timeScale { 1.f };
        int    _substeps { 1 };

        int   _selectedBody { 0 };
        float _userImpulse { 4.f };
        float _impulseVizDuration { 1.2f };
        float _impulseVizScale { 0.2f };

        bool            _hasImpulseViz { false };
        float           _impulseVizTimer { 0.f };
        Eigen::Vector3f _lastImpulsePoint { 0.f, 0.f, 0.f };
        Eigen::Vector3f _lastImpulseVector { 0.f, 0.f, 0.f };

        std::array<char const *, 11> const _presetNames {
            "Base: Single Rigid Body",
            "Base: Double (Edge-Edge)",
            "Base: Double (Vertex-Face)",
            "Base: Double (Face-Face)",
            "Base: Complex Scene",
            "Bonus B1: Newton Cradle",
            "Bonus B2: Stable Stacking",
            "Bonus B3: Mixed Primitives",
            "Bonus B4: Schur Constraints",
            "Bonus B5: Articulated Chain",
            "Bonus B6: CCD Bullet",
        };

        void resetPreset();
        void stepSimulation(float dt);
        void applyUserInteraction(float dt);
        void applyImpulseWithViz(int bodyId, Eigen::Vector3f const & worldPoint, Eigen::Vector3f const & impulse);

        void initSingleBody();
        void initDoubleEdge();
        void initDoubleVertexFace();
        void initDoubleFaceFace();
        void initComplexScene();
        void initBonusNewtonCradle();
        void initBonusStacking();
        void initBonusMixedPrimitives();
        void initBonusConstrainedContacts();
        void initBonusArticulatedBodies();
        void initBonusCCD();
    };

} // namespace VCX::Labs::RigidBody
