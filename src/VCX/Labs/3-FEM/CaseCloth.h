#pragma once

#include <cstddef>
#include <string_view>
#include <vector>

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/3-FEM/ClothSystem.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/OrbitCameraManager.h"

namespace VCX::Labs::FEM {

    class CaseCloth : public Common::ICase {
    public:
        CaseCloth();

        std::string_view const GetName() override { return "Bonus2 FEM Cloth"; }

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

        Engine::Camera             _camera { .Eye = glm::vec3(-2.8f, 2.6f, 3.2f), .Target = glm::vec3(0.f, 0.4f, 0.f) };
        Common::OrbitCameraManager _cameraManager;

        ClothSystem                 _system;
        ClothSystem::GridResolution _resolution { 24, 24 };
        glm::vec2                   _size { 2.f, 2.f };

        bool  _stopped { true };
        bool  _showSurface { true };
        bool  _showWireframe { true };
        bool  _showParticles { false };
        bool  _showFixed { true };
        float _timeStep { 0.001f };
        int   _stepsPerFrame { 6 };

        std::size_t _selectedParticle { 0 };
        float       _controlForceMagnitude { 12.f };
        float       _mouseForceScale { 45.f };
        glm::vec3   _keyboardForce { 0.f };
        glm::vec3   _mouseForce { 0.f };
        glm::vec3   _buttonForce { 0.f };

        std::vector<glm::vec3> _fixedPositions;
        std::vector<glm::vec3> _selectedPosition;
        std::vector<glm::vec3> _forceLine;

        void ResetSystem();
        void UpdateStaticBuffers();
        void UpdateDynamicBuffers(glm::vec3 const & frameControlForce);
        void SetSelectedToFreeCorner();
    };

} // namespace VCX::Labs::FEM
