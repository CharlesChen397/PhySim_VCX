#pragma once

#include <vector>

#include "Engine/app.h"
#include "Labs/1-RigidBody/CasePlayground.h"
#include "Labs/Common/UI.h"

namespace VCX::Labs::RigidBody {

    class App : public Engine::IApp {
    public:
        App();

        void OnFrame() override;

    private:
        Common::UI                                         _ui;
        CasePlayground                                     _playground;
        std::size_t                                        _caseId { 0 };
        std::vector<std::reference_wrapper<Common::ICase>> _cases { _playground };
    };

} // namespace VCX::Labs::RigidBody
