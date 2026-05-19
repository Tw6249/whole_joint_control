#pragma once

#include "controller_interface.hpp"
#include "eid_controller.hpp"
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
    }
    throw std::runtime_error("unsupported controller kind: " + controllerKindName(cfg.controller.kind));
}

}  // namespace h1if
