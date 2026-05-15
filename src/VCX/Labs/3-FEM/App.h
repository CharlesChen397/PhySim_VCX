#pragma once

#include <functional>
#include <vector>

#include "Engine/app.h"
#include "Labs/3-FEM/CaseSoftBody.h"
#include "Labs/Common/UI.h"

namespace VCX::Labs::FEM {

    class App : public Engine::IApp {
    public:
        App();

        void OnFrame() override;

    private:
        Common::UI _ui;

        CaseSoftBody _caseSoftBody;

        std::size_t _caseId { 0 };
        std::vector<std::reference_wrapper<Common::ICase>> _cases { _caseSoftBody };
    };

} // namespace VCX::Labs::FEM
