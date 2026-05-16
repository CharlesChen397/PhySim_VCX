#pragma once

#include <functional>
#include <vector>

#include "Engine/app.h"
#include "Labs/3-FEM/CaseCloth.h"
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
        CaseCloth    _caseCloth;

        std::size_t _caseId { 0 };
        std::vector<std::reference_wrapper<Common::ICase>> _cases { _caseSoftBody, _caseCloth };
    };

} // namespace VCX::Labs::FEM
