#include "kitezh/lock.hpp"

#include <stdexcept>

namespace kitezh {

Lock::Lock() {
    m_level = Level::Low;
    m_lower_gate = GateState::Closed;
    m_upper_gate = GateState::Closed;
    m_pump_cycles = 0;
    m_inside.clear();
}

Level Lock::level() const {
    return m_level;
}

GateState Lock::gate(Gate gate) const {
    if (gate == Gate::Lower) {
        return m_lower_gate;
    } else {
        return m_upper_gate;
    }
}

int Lock::pumpCycles() const {
    return m_pump_cycles;
}

std::vector<Vessel> Lock::inside() const {
    return m_inside;
}

void Lock::open(Gate gate) {
    if (gate == Gate::Lower) {
        if (m_level != Level::Low) {
            throw LockError("Cannot open lower gate at high level");
        }
        m_lower_gate = GateState::Open;
    } else {
        if (m_level != Level::High) {
            throw LockError("Cannot open upper gate at low level");
        }
        m_upper_gate = GateState::Open;
    }
}

void Lock::close(Gate gate) {
    if (gate == Gate::Lower) {
        m_lower_gate = GateState::Closed;
    } else {
        m_upper_gate = GateState::Closed;
    }
}

void Lock::pump() {
    if (m_lower_gate != GateState::Closed || m_upper_gate != GateState::Closed) {
        throw LockError("Cannot pump while gates are open");
    }
    
    Level next_level;
    if (m_level == Level::Low) {
        next_level = Level::High;
    } else {
        next_level = Level::Low;
    }
    
    int next_depth;
    if (next_level == Level::Low) {
        next_depth = kLowDepthDm;
    } else {
        next_depth = kHighDepthDm;
    }
    
    for (const auto& v : m_inside) {
        if (v.draft_dm > next_depth) {
            throw LockError("Vessel draft exceeds depth of the next level");
        }
    }
    
    m_level = next_level;
    m_pump_cycles++;
}


void Lock::enter(const Vessel& vessel) {
    if (validateVessel(vessel).has_value()) {
        throw LockError("Invalid vessel parameters");
    }
    
    for (const auto& v : m_inside) {
        if (v.call_sign == vessel.call_sign) {
            throw LockError("Vessel with this call sign is already inside");
        }
    }
    
    int current_depth;
    if (m_level == Level::Low) {
        current_depth = kLowDepthDm;
    } else {
        current_depth = kHighDepthDm;
    }
    
    if (vessel.draft_dm > current_depth) {
        throw LockError("Vessel draft exceeds current depth");
    }
    
    if (vessel.direction == Direction::Upstream) {
        if (m_lower_gate != GateState::Open) {
            throw LockError("Lower gate closed for entry up");
        }
    } else {
        if (m_upper_gate != GateState::Open) {
            throw LockError("Upper gate closed for entry down");
        }
    }
    
    if (vessel.kind == VesselKind::Oversized && !m_inside.empty()) {
        throw LockError("Oversized vessel cannot enter non-empty chamber");
    }
    for (const auto& v : m_inside) {
        if (v.kind == VesselKind::Oversized) {
            throw LockError("Chamber contains an oversized vessel");
        }
    }
    
    if (!fitsInChamber(m_inside, vessel)) {
        throw LockError("Vessel does not fit by length");
    }
    
    m_inside.push_back(vessel);
}

void Lock::exit(const std::string& call_sign) {
    if (m_inside.empty()) {
        throw LockError("Chamber is empty");
    }
    
    Vessel head = m_inside.front();
    
    if (head.call_sign != call_sign) {
        throw LockError("Only the head of the queue can exit");
    }
    
    if (head.direction == Direction::Upstream) {
        if (m_upper_gate != GateState::Open) {
            throw LockError("Upper gate closed for exit up");
        }
    } else {
        if (m_lower_gate != GateState::Open) {
            throw LockError("Lower gate closed for exit down");
        }
    }
    
    m_inside.erase(m_inside.begin());
}

}  // namespace kitezh
