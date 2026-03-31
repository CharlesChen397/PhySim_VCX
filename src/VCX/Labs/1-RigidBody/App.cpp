#include "Labs/1-RigidBody/App.h"

namespace VCX::Labs::RigidBody {

    App::App():
        _ui(Labs::Common::UIOptions {
            .SideWindowWidth = 420.f,
        }) {
    }

    void App::OnFrame() {
        _ui.Setup(_cases, _caseId);
    }

} // namespace VCX::Labs::RigidBody
