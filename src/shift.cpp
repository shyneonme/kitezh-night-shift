#include "kitezh/shift.hpp"

#include "kitezh/lock.hpp"

#include <stdexcept>

#include <set>

namespace kitezh {

ShiftReport runNightShift(const std::vector<Vessel>& queue) {
    ShiftReport report;
    std::vector<Vessel> valid_queue;
    std::set<std::string> seen_call_signs;

    for (const auto& v : queue) {
        if (seen_call_signs.count(v.call_sign)) {
            report.refused.push_back({v.call_sign, "duplicate"});
            continue;
        }
        seen_call_signs.insert(v.call_sign);

        auto validation_err = validateVessel(v);
        if (validation_err.has_value()) {
            report.refused.push_back({v.call_sign, "invalid"});
            continue;
        }

        if (v.draft_dm > kLowDepthDm) {
            report.refused.push_back({v.call_sign, "draft"});
            continue;
        }

        valid_queue.push_back(v);
    }

    Lock lock; 

    while (!valid_queue.empty() || !lock.inside().empty()) {
        if (!lock.inside().empty()) {
            lock.close(Gate::Lower);
            lock.close(Gate::Upper);
            
            Direction dir = lock.inside().front().direction;
            
            Level target_level;
            if (dir == Direction::Upstream) {
                target_level = Level::High;
            } else {
                target_level = Level::Low;
            }
            
            if (lock.level() != target_level) {
                lock.pump();
            }
            
            if (dir == Direction::Upstream) {
                lock.open(Gate::Upper);
            } else {
                lock.open(Gate::Lower);
            }
            
            while (!lock.inside().empty()) {
                std::string call = lock.inside().front().call_sign;
                lock.exit(call);
                report.passed.push_back(call);
            }
            continue;
        }

        if (valid_queue.empty()) {
            break;
        }

        const Vessel leader = valid_queue[0];
        Direction current_dir = leader.direction;

        Level entry_level;
        if (current_dir == Direction::Upstream) {
            entry_level = Level::Low;
        } else {
            entry_level = Level::High;
        }
        
        lock.close(Gate::Lower);
        lock.close(Gate::Upper);
        
        if (lock.level() != entry_level) {
            lock.pump();
        }

        if (current_dir == Direction::Upstream) {
            lock.open(Gate::Lower);
        } else {
            lock.open(Gate::Upper);
        }

        std::vector<Vessel> batch;
        lock.enter(leader);
        batch.push_back(leader);
        valid_queue.erase(valid_queue.begin());

        size_t next_idx = 0;
        while (next_idx < valid_queue.size()) {
            const auto& next_vessel = valid_queue[next_idx];
            if (next_vessel.kind == VesselKind::Ambulance && next_vessel.direction != current_dir) {
                break;
            }
            next_idx++;
        }
        size_t allowed_scenery_limit = next_idx;

        next_idx = 0;
        while (next_idx < allowed_scenery_limit) {
            const auto& next_vessel = valid_queue[next_idx];
            if (next_vessel.direction == current_dir && next_vessel.kind == VesselKind::Ambulance) {
                if (batch.front().kind == VesselKind::Oversized) {
                    break;
                }
                if (fitsInChamber(batch, next_vessel)) {
                    lock.enter(next_vessel);
                    batch.push_back(next_vessel);
                    valid_queue.erase(valid_queue.begin() + next_idx);
                    allowed_scenery_limit--;
                    continue;
                }
            }
            next_idx++;
        }

        next_idx = 0;
        while (next_idx < allowed_scenery_limit) {
            const auto& next_vessel = valid_queue[next_idx];
            if (next_vessel.direction != current_dir) {
                break;
            }
            if (next_vessel.kind == VesselKind::Oversized || batch.front().kind == VesselKind::Oversized) {
                break;
            }
            if (fitsInChamber(batch, next_vessel)) {
                lock.enter(next_vessel);
                batch.push_back(next_vessel);
                valid_queue.erase(valid_queue.begin() + next_idx);
                allowed_scenery_limit--;
                continue;
            } else {
                break;
            }
        }
    }

    report.pump_cycles = lock.pumpCycles();
    return report;
}

}  // namespace kitezh
