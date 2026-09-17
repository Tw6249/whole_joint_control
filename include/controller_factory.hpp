#pragma once

#include "controller_interface.hpp"
#include "eid_controller.hpp"
#ifdef H1IF_ENABLE_TORCH_POLICY
#include "online_policy_eid_controller.hpp"
#endif
#include "position_pd_controller.hpp"
#include "runtime_config.hpp"

#include <memory>
#include <stdexcept>

namespace h1if {

inline std::unique_ptr<IController> createController(const RuntimeConfig& cfg) {
    switch (cfg.controller.kind) {
        case ControllerKind::Eid:
            return std::make_unique<EidMultiJointController>(cfg);
        case ControllerKind::PositionPd:
            return std::make_unique<PositionPdMultiJointController>(cfg);
        case ControllerKind::OnlinePolicyEid:
#ifdef H1IF_ENABLE_TORCH_POLICY
            return std::make_unique<OnlinePolicyEidMultiJointController>(cfg);
#else
            throw std::runtime_error("controller.kind=online_policy_eid requires H1IF_BUILD_TORCH_POLICY=ON");
#endif
    }
    throw std::runtime_error("unsupported controller kind: " + controllerKindName(cfg.controller.kind));
}

}  // namespace h1if
