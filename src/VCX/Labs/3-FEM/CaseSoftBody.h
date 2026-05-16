#pragma once

#include <cstddef>
#include <array>
#include <string_view>
#include <vector>

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/3-FEM/FEMSystem.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/OrbitCameraManager.h"

namespace VCX::Labs::FEM {

    class CaseSoftBody : public Common::ICase {
    public:
        CaseSoftBody();

        std::string_view const GetName() override { return "Explicit Linear FEM Soft Body"; }

        void                     OnSetupPropsUI() override;
        Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        void                     OnProcessInput(ImVec2 const & pos) override;

    private:
        Engine::GL::UniqueProgram           _program;
        Engine::GL::UniqueRenderFrame       _frame;
        Engine::GL::UniqueIndexedRenderItem _solidItem;
        Engine::GL::UniqueIndexedRenderItem _wireItem;
        Engine::GL::UniqueRenderItem        _particleItem;
        Engine::GL::UniqueRenderItem        _fixedItem;
        Engine::GL::UniqueRenderItem        _selectedItem;
        Engine::GL::UniqueRenderItem        _forceItem;

        Engine::Camera             _camera { .Eye = glm::vec3(-7.f, 3.5f, 6.f), .Target = glm::vec3(0.f, 0.f, 0.f) };
        Common::OrbitCameraManager _cameraManager;

        std::array<FEMSystem, 3>  _systems;
        FEMSystem::GridResolution _resolution { 8, 2, 2 };
        glm::vec3                 _size { 8.f, 2.f, 2.f };

        bool  _stopped { true };
        bool  _compareMaterials { true };
        bool  _showSurface { true };
        bool  _showWireframe { false };
        bool  _showParticles { false };
        bool  _showFixed { true };
        float _timeStep { 0.001f };
        int   _stepsPerFrame { 8 };
        int   _activeModel { 0 };

        std::size_t _selectedParticle { 0 };
        float       _controlForceMagnitude { 600.f };
        float       _mouseForceScale { 1200.f };
        glm::vec3   _keyboardForce { 0.f };
        glm::vec3   _mouseForce { 0.f };
        glm::vec3   _buttonForce { 0.f };

        std::vector<glm::vec3> _fixedPositions;
        std::vector<glm::vec3> _selectedPosition;
        std::vector<glm::vec3> _forceLine;

        std::vector<glm::vec3>     _renderPositions;
        std::vector<std::uint32_t> _renderTriangles;
        std::vector<std::uint32_t> _renderLines;

        void ResetSystem();
        void UpdateStaticBuffers();
        void UpdateDynamicBuffers(glm::vec3 const & frameControlForce);
        void SetSelectedToFreeEnd();
    };

} // namespace VCX::Labs::FEM
