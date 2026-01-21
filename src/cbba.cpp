/*
  cbba.cpp - Consensus-Based Bundle Algorithm implementation
  
  Ported from Python implementation (cbba.py)
  for onboard execution on ESP32.
  
  [MODIFIED] Added filtering of completed tasks in message sharing
*/

#include "cbba.h"
#include <set>

// ===================== CBBATask Implementation =====================

void CBBATask::copy_from(const void* task_ptr) {
    const CBBATask* src = static_cast<const CBBATask*>(task_ptr);
    if (src) {
        task_id = src->task_id;
        x = src->x;
        y = src->y;
        amount = src->amount;
        completed = src->completed;
    }
}

// ===================== CBBA Implementation =====================

CBBA::CBBA(int agent_id, const CBBAConfig& config)
    : _agent_id(agent_id),
      _pos_x(0), _pos_y(0),
      _config(config),
      _phase(CBBAPhase::BUILD_BUNDLE),
      _assigned_task_id(-1),
      _no_bundle_duration(0),
      _last_decide_time(0)
{
    _z.clear();
    _y.clear();
    _s.clear();
    _bundle.clear();
    _path.clear();
}

void CBBA::set_position(float x, float y) {
    _pos_x = x;
    _pos_y = y;
}

int CBBA::decide(const std::vector<CBBATask>& tasks) {
    unsigned long now = millis();
    float dt = (now - _last_decide_time) / 1000.0f;
    _last_decide_time = now;
    
    // Cache local tasks
    _local_tasks = tasks;
    
    // ===== [MODIFIED] Clean up completed tasks from _y and _z =====
    _cleanup_completed_tasks();
    
    // Check if the existing task is done
    if (_assigned_task_id >= 0) {
        CBBATask* assigned = _find_task(_assigned_task_id);
        if (assigned == nullptr || assigned->completed) {
            if (!_path.empty() && _path[0] == _assigned_task_id) {
                _path.erase(_path.begin());
                if (!_bundle.empty()) {
                    _bundle.erase(_bundle.begin());
                }
            }
            _assigned_task_id = -1;
            _phase = CBBAPhase::BUILD_BUNDLE;
        }
    }
    
    if (_bundle.empty()) {
        _phase = CBBAPhase::BUILD_BUNDLE;
    }
    
    if (tasks.empty() && _bundle.empty()) {
        return -1;
    }
    
    // Winning bid cancel on timeout
    if (_config.winning_bid_cancel) {
        if (_bundle.empty()) {
            _no_bundle_duration += dt;
        }
        
        if (_no_bundle_duration > _config.no_bundle_duration_limit) {
            _z.clear();
            _y.clear();
            _s.clear();
            _no_bundle_duration = 0;
            
            Serial.println("[CBBA] Bid timeout - reset all bids");
        }
    }
    
    // ===== Phase 1: Build Bundle =====
    if (_phase == CBBAPhase::BUILD_BUNDLE) {
        _build_bundle(tasks);
        _phase = CBBAPhase::ASSIGNMENT_CONSENSUS;
        _assigned_task_id = -1;
        return -1;
    }
    
    // ===== Phase 2: Assignment Consensus =====
    if (_phase == CBBAPhase::ASSIGNMENT_CONSENSUS) {
        _update_time_stamp();
        
        for (const CBBATask& task : tasks) {
            for (const CBBAMessage& msg : _messages_received) {
                if (msg.agent_id == _agent_id) continue;
                _apply_consensus_rules(task.task_id, msg);
            }
        }
        
        _messages_received.clear();
        
        std::vector<int> updated_bundle, updated_path;
        _update_bundle_and_path(updated_bundle, updated_path);
        
        if (_config.winning_bid_cancel) {
            if (!updated_bundle.empty()) {
                _no_bundle_duration = 0;
            }
        }
        
        if (updated_bundle == _bundle) {
            _assigned_task_id = _path.empty() ? -1 : _path[0];
            return _assigned_task_id;
        } else {
            _bundle = updated_bundle;
            _path = updated_path;
            _assigned_task_id = -1;
            _phase = CBBAPhase::BUILD_BUNDLE;
        }
    }
    
    if (_config.keep_moving_during_convergence) {
        _assigned_task_id = _path.empty() ? -1 : _path[0];
        return _assigned_task_id;
    }
    
    return -1;
}

// ===== [NEW] Clean up completed tasks from internal maps =====
void CBBA::_cleanup_completed_tasks() {
    // Build set of valid (non-completed) task IDs
    std::set<int> valid_task_ids;
    for (const auto& task : _local_tasks) {
        if (!task.completed) {
            valid_task_ids.insert(task.task_id);
        }
    }
    
    // Remove completed tasks from _y (winning bids)
    auto y_it = _y.begin();
    while (y_it != _y.end()) {
        if (valid_task_ids.find(y_it->first) == valid_task_ids.end()) {
            y_it = _y.erase(y_it);
        } else {
            ++y_it;
        }
    }
    
    // Remove completed tasks from _z (winning agents)
    auto z_it = _z.begin();
    while (z_it != _z.end()) {
        if (valid_task_ids.find(z_it->first) == valid_task_ids.end()) {
            z_it = _z.erase(z_it);
        } else {
            ++z_it;
        }
    }
}

void CBBA::_build_bundle(const std::vector<CBBATask>& tasks) {
    int max_bundle_size = std::min(_config.max_tasks_per_agent, (int)tasks.size());
    
    while ((int)_bundle.size() < max_bundle_size) {
        std::map<int, float> my_bid_list;
        std::map<int, int> best_insertion_idx_list;
        _get_my_bid_value_list(tasks, my_bid_list, best_insertion_idx_list);
        
        int best_task_id = _get_best_task(my_bid_list);
        
        if (best_task_id < 0) {
            break;
        }
        
        int best_idx = best_insertion_idx_list[best_task_id];
        
        if (best_idx >= (int)_bundle.size()) {
            _bundle.push_back(best_task_id);
        } else {
            _bundle.insert(_bundle.begin() + best_idx, best_task_id);
        }
        
        if (best_idx >= (int)_path.size()) {
            _path.push_back(best_task_id);
        } else {
            _path.insert(_path.begin() + best_idx, best_task_id);
        }
        
        _y[best_task_id] = my_bid_list[best_task_id];
        _z[best_task_id] = _agent_id;
    }
}

void CBBA::_get_my_bid_value_list(
    const std::vector<CBBATask>& tasks,
    std::map<int, float>& my_bid_list,
    std::map<int, int>& best_insertion_idx_list)
{
    float S_p = _calculate_score_along_path(_path);
    
    my_bid_list.clear();
    best_insertion_idx_list.clear();
    
    for (const CBBATask& task : tasks) {
        if (_is_task_in_path(task.task_id)) {
            continue;
        }
        
        if (task.completed) {
            continue;
        }
        
        std::vector<float> marginal_scores;
        
        for (size_t idx = 0; idx <= _path.size(); idx++) {
            std::vector<int> alt_path = _path;
            alt_path.insert(alt_path.begin() + idx, task.task_id);
            
            float S_alt = _calculate_score_along_path(alt_path);
            marginal_scores.push_back(S_alt - S_p);
        }
        
        int best_idx = 0;
        float best_score = marginal_scores[0];
        for (size_t i = 1; i < marginal_scores.size(); i++) {
            if (marginal_scores[i] > best_score) {
                best_score = marginal_scores[i];
                best_idx = i;
            }
        }
        
        my_bid_list[task.task_id] = best_score;
        best_insertion_idx_list[task.task_id] = best_idx;
    }
}

float CBBA::_calculate_score_along_path(const std::vector<int>& path) {
    float current_x = _pos_x;
    float current_y = _pos_y;
    float expected_reward = 0;
    float distance_from_start = 0;

    for (int task_id : path) {
        CBBATask* task = _find_task(task_id);
        if (task == nullptr) continue;

        float dx = task->x - current_x;
        float dy = task->y - current_y;
        float dist = sqrt(dx * dx + dy * dy);
        distance_from_start += dist;

        // Convert distance to time (matching Python)
        float time_to_task = distance_from_start / _config.max_speed;
        float work_time = task->amount / _config.work_rate;
        float total_time = time_to_task + work_time;

        // Apply discount factor
        float discount = pow(_config.lambda, total_time);
        expected_reward += task->amount * discount;

        current_x = task->x;
        current_y = task->y;
    }

    return expected_reward;
}

int CBBA::_get_best_task(const std::map<int, float>& my_bid_list) {
    int best_task = -1;
    float best_bid = -1e9;
    
    for (const auto& pair : my_bid_list) {
        int task_id = pair.first;
        float my_bid = pair.second;
        
        // Get current winning bid
        float current_winning = 0;
        auto y_it = _y.find(task_id);
        if (y_it != _y.end()) {
            current_winning = y_it->second;
        }
        
        // Only consider if my bid is higher than current winning bid
        if (my_bid > current_winning && my_bid > best_bid) {
            best_bid = my_bid;
            best_task = task_id;
        }
    }
    
    return best_task;
}

void CBBA::_update_time_stamp() {
    unsigned long now = millis();
    _s[_agent_id] = now;
    
    // Merge timestamps from nearby agents
    for (int neighbor_id : _nearby_agents) {
        if (_s.find(neighbor_id) == _s.end()) {
            _s[neighbor_id] = 0;
        }
    }
}

void CBBA::_apply_consensus_rules(int task_id, const CBBAMessage& msg) {
    int i = _agent_id;      // me
    int k = msg.agent_id;   // sender
    
    // Get sender's info about this task
    float y_k = 0;
    int z_k = -1;
    
    auto y_it = msg.winning_bids.find(task_id);
    if (y_it != msg.winning_bids.end()) {
        y_k = y_it->second;
    }
    
    auto z_it = msg.winning_agents.find(task_id);
    if (z_it != msg.winning_agents.end()) {
        z_k = z_it->second;
    }
    
    // Get my info about this task
    float y_i = 0;
    int z_i = -1;
    
    auto my_y_it = _y.find(task_id);
    if (my_y_it != _y.end()) {
        y_i = my_y_it->second;
    }
    
    auto my_z_it = _z.find(task_id);
    if (my_z_it != _z.end()) {
        z_i = my_z_it->second;
    }
    
    // Merge timestamps
    _merge_timestamps(msg.timestamps);
    
    // Helper lambda for timestamp comparison
    auto get_ts = [](const std::map<int, unsigned long>& s, int agent) -> unsigned long {
        auto it = s.find(agent);
        return (it != s.end()) ? it->second : 0;
    };
    
    const auto& s_k = msg.timestamps;
    const auto& s_i = _s;
    
    // Apply CBBA consensus rules (Rules 1-17 from the paper)
    if (z_k == k) {
        // Sender thinks they are the winner
        if (z_i == i) {
            // Rule 1: I also think I'm the winner
            if (y_k > y_i) {
                _update(task_id, y_k, z_k);
            }
        } else if (z_i == k) {
            // Rule 2: I already think sender is winner
            _update(task_id, y_k, z_k);
        } else if (z_i < 0) {
            // Rule 5: I have no winner
            _update(task_id, y_k, z_k);
        } else {
            // Rule 3-4: I think someone else is winner
            int m = z_i;
            if (get_ts(s_k, m) > get_ts(s_i, m)) {
                _update(task_id, y_k, z_k);
            } else if (y_k > y_i) {
                _update(task_id, y_k, z_k);
            }
        }
    } else if (z_k == i) {
        // Sender thinks I am the winner
        if (z_i == i) {
            // Rule 6: I also think I'm the winner - leave
            _leave();
        } else if (z_i == k) {
            // Rule 7: I think sender is winner
            _reset_task(task_id);
        } else if (z_i < 0) {
            // Rule 8: I have no winner
            // Leave (no action)
        } else {
            // Rule 9: I think someone else (m) is winner
            int m = z_i;
            if (get_ts(s_k, m) > get_ts(s_i, m)) {
                _reset_task(task_id);
            }
        }
    } else {
        // Sender thinks someone else (m) is winner
        int m = z_k;
        
        if (z_i == i) {
            // Rule 14: I think I'm winner
            if (get_ts(s_k, m) > get_ts(s_i, m) && y_k > y_i) {
                _update(task_id, y_k, z_k);
            }
        } else if (z_i == k) {
            // Rule 10: I think sender is winner
            if (get_ts(s_k, m) > get_ts(s_i, m)) {
                _update(task_id, y_k, z_k);
            } else {
                _reset_task(task_id);
            }
        } else if (z_i == m) {
            // Rule 11: I also think m is winner
            if (get_ts(s_k, m) > get_ts(s_i, m)) {
                _update(task_id, y_k, z_k);
            }
        } else if (z_i < 0) {
            // Rule 13: I have no winner
            if (get_ts(s_k, m) > get_ts(s_i, m)) {
                _update(task_id, y_k, z_k);
            }
        } else {
            // Rule 12: I think someone else (n) is winner
            int n = z_i;
            if (get_ts(s_k, m) > get_ts(s_i, m) && get_ts(s_k, n) > get_ts(s_i, n)) {
                _update(task_id, y_k, z_k);
            } else if (get_ts(s_k, m) > get_ts(s_i, m) && y_k > y_i) {
                _update(task_id, y_k, z_k);
            } else if (get_ts(s_k, n) > get_ts(s_i, n) && get_ts(s_i, m) > get_ts(s_k, m)) {
                _reset_task(task_id);
            }
        }
    }
}

void CBBA::_update(int task_id, float y_k, int z_k) {
    _y[task_id] = y_k;
    _z[task_id] = z_k;
}

void CBBA::_reset_task(int task_id) {
    _y[task_id] = 0;
    _z[task_id] = -1;
}

void CBBA::_leave() {
    // No action (matches Python implementation)
}

bool CBBA::_update_bundle_and_path(std::vector<int>& updated_bundle, std::vector<int>& updated_path) {
    int n_bar = _bundle.size();
    
    for (size_t idx = 0; idx < _bundle.size(); idx++) {
        int task_id = _bundle[idx];
        auto z_it = _z.find(task_id);
        if (z_it == _z.end() || z_it->second != _agent_id) {
            n_bar = idx;
            break;
        }
    }
    
    updated_bundle.clear();
    updated_path.clear();
    
    for (int i = 0; i < n_bar; i++) {
        updated_bundle.push_back(_bundle[i]);
        if (i < (int)_path.size()) {
            updated_path.push_back(_path[i]);
        }
    }
    
    return (updated_bundle != _bundle);
}

void CBBA::receive_message(const CBBAMessage& message) {
    if (message.agent_id == _agent_id) return;
    
    _messages_received.push_back(message);
    
    bool found = false;
    for (int id : _nearby_agents) {
        if (id == message.agent_id) {
            found = true;
            break;
        }
    }
    if (!found) {
        _nearby_agents.push_back(message.agent_id);
    }
}

void CBBA::receive_message_json(const JsonDocument& doc) {
    CBBAMessage msg;
    
    if (doc.containsKey("agent_id")) {
        msg.agent_id = doc["agent_id"];
    } else {
        return;
    }
    
    if (msg.agent_id == _agent_id) return;
    
    if (doc.containsKey("winning_bids")) {
        JsonObjectConst bids = doc["winning_bids"].as<JsonObjectConst>();
        for (JsonPairConst kv : bids) {
            int task_id = atoi(kv.key().c_str());
            float bid = kv.value().as<float>();
            msg.winning_bids[task_id] = bid;
        }
    }
    
    if (doc.containsKey("winning_agents")) {
        JsonObjectConst agents = doc["winning_agents"].as<JsonObjectConst>();
        for (JsonPairConst kv : agents) {
            int task_id = atoi(kv.key().c_str());
            int agent = kv.value().as<int>();
            msg.winning_agents[task_id] = agent;
        }
    }
    
    if (doc.containsKey("message_received_time_stamp")) {
        JsonObjectConst timestamps = doc["message_received_time_stamp"].as<JsonObjectConst>();
        for (JsonPairConst kv : timestamps) {
            int agent_id = atoi(kv.key().c_str());
            unsigned long ts = kv.value().as<unsigned long>();
            msg.timestamps[agent_id] = ts;
        }
    }
    
    receive_message(msg);
}

void CBBA::get_message_to_share(CBBAMessage& message) {
    message.agent_id = _agent_id;
    
    // ===== [MODIFIED] Only include valid (non-completed) tasks =====
    std::set<int> valid_task_ids;
    for (const auto& task : _local_tasks) {
        if (!task.completed) {
            valid_task_ids.insert(task.task_id);
        }
    }
    
    message.winning_bids.clear();
    for (const auto& pair : _y) {
        if (valid_task_ids.find(pair.first) != valid_task_ids.end()) {
            message.winning_bids[pair.first] = pair.second;
        }
    }
    
    message.winning_agents.clear();
    for (const auto& pair : _z) {
        if (valid_task_ids.find(pair.first) != valid_task_ids.end()) {
            message.winning_agents[pair.first] = pair.second;
        }
    }
    
    message.timestamps = _s;
}

void CBBA::get_message_to_share_json(JsonDocument& doc) {
    doc.clear();
    doc["agent_id"] = _agent_id;

    // ===== [MODIFIED] Only include valid (non-completed) tasks =====
    std::set<int> valid_task_ids;
    for (const auto& task : _local_tasks) {
        if (!task.completed) {
            valid_task_ids.insert(task.task_id);
        }
    }

    JsonObject bids = doc.createNestedObject("winning_bids");
    for (const auto& pair : _y) {
        // Only transmit bids for valid tasks
        if (valid_task_ids.find(pair.first) != valid_task_ids.end()) {
            bids[String(pair.first)] = pair.second;
        }
    }

    JsonObject agents = doc.createNestedObject("winning_agents");
    for (const auto& pair : _z) {
        // Only transmit winners for valid tasks
        if (valid_task_ids.find(pair.first) != valid_task_ids.end()) {
            agents[String(pair.first)] = pair.second;
        }
    }

    JsonObject timestamps = doc.createNestedObject("message_received_time_stamp");
    for (const auto& pair : _s) {
        timestamps[String(pair.first)] = pair.second;
    }
}

void CBBA::add_nearby_agent(int agent_id) {
    if (agent_id == _agent_id) return;
    
    for (int id : _nearby_agents) {
        if (id == agent_id) return;
    }
    _nearby_agents.push_back(agent_id);
}

void CBBA::clear_nearby_agents() {
    _nearby_agents.clear();
}

void CBBA::reset() {
    _z.clear();
    _y.clear();
    _s.clear();
    _bundle.clear();
    _path.clear();
    _phase = CBBAPhase::BUILD_BUNDLE;
    _assigned_task_id = -1;
    _no_bundle_duration = 0;
    _messages_received.clear();
    _nearby_agents.clear();
}

void CBBA::clear_task(int task_id) {
    auto it = std::find(_bundle.begin(), _bundle.end(), task_id);
    if (it != _bundle.end()) {
        _bundle.erase(it);
    }
    
    it = std::find(_path.begin(), _path.end(), task_id);
    if (it != _path.end()) {
        _path.erase(it);
    }
    
    _y.erase(task_id);
    _z.erase(task_id);
    
    if (_assigned_task_id == task_id) {
        _assigned_task_id = -1;
    }
}

CBBATask* CBBA::_find_task(int task_id) {
    for (auto& task : _local_tasks) {
        if (task.task_id == task_id) {
            return &task;
        }
    }
    return nullptr;
}

bool CBBA::_is_task_in_path(int task_id) const {
    for (int id : _path) {
        if (id == task_id) return true;
    }
    return false;
}

void CBBA::_merge_timestamps(const std::map<int, unsigned long>& other) {
    for (const auto& pair : other) {
        auto it = _s.find(pair.first);
        if (it == _s.end() || pair.second > it->second) {
            _s[pair.first] = pair.second;
        }
    }
}

void CBBA::print_state() const {
    Serial.printf("[CBBA] Agent %d | Phase: %s | Assigned: %d\n",
        _agent_id,
        (_phase == CBBAPhase::BUILD_BUNDLE) ? "BUILD" : "CONSENSUS",
        _assigned_task_id);
    
    Serial.print("  Bundle: [");
    for (size_t i = 0; i < _bundle.size(); i++) {
        if (i > 0) Serial.print(", ");
        Serial.print(_bundle[i]);
    }
    Serial.println("]");
    
    Serial.print("  Winners (z): {");
    bool first = true;
    for (const auto& pair : _z) {
        if (!first) Serial.print(", ");
        Serial.printf("%d:%d", pair.first, pair.second);
        first = false;
    }
    Serial.println("}");
    
    Serial.print("  Bids (y): {");
    first = true;
    for (const auto& pair : _y) {
        if (!first) Serial.print(", ");
        Serial.printf("%d:%.2f", pair.first, pair.second);
        first = false;
    }
    Serial.println("}");
}