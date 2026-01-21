/*
  cbba.h - Consensus-Based Bundle Algorithm for MONA ESP robot
  
  Ported from Python implementation (cbba.py)
  for onboard execution on ESP32.
  
  This implementation follows the CBBA paper algorithm:
  - Phase 1: Build Bundle (Algorithm 3)
  - Phase 2: Assignment Consensus (Rules 1-17)
  
  [MODIFIED] Added _cleanup_completed_tasks() for filtering completed tasks
*/

#ifndef CBBA_H
#define CBBA_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <vector>
#include <map>
#include <set>
#include <cmath>
#include <algorithm>

// Forward declaration
class Task;

// ===================== CBBA Configuration =====================
struct CBBAConfig {
    int max_tasks_per_agent;          // Maximum tasks in bundle
    float lambda;                      // Task reward discount factor
    bool winning_bid_cancel;           // Enable bid cancellation on timeout
    float no_bundle_duration_limit;    // Seconds before bid reset
    bool keep_moving_during_convergence;
    float work_rate;                   // Agent work rate for score calculation
    float max_speed;                   // Agent max speed (mm/s)
    
    CBBAConfig()
        : max_tasks_per_agent(4),
          lambda(0.999f),
          winning_bid_cancel(true),
          no_bundle_duration_limit(5.0f),
          keep_moving_during_convergence(false),
          work_rate(1.0f),
          max_speed(0.25f) {}
};

// ===================== CBBA Phase =====================
enum class CBBAPhase {
    BUILD_BUNDLE = 1,
    ASSIGNMENT_CONSENSUS = 2
};

// ===================== Task Class =====================
/**
 * Task representation for CBBA.
 * Must match the Task class used in bt_nodes.h
 */
class CBBATask {
public:
    int task_id;
    float x;
    float y;
    float amount;
    bool completed;

    CBBATask() : task_id(-1), x(0), y(0), amount(0), completed(false) {}

    CBBATask(int id, float x_pos, float y_pos, float amt)
        : task_id(id), x(x_pos), y(y_pos), amount(amt), completed(false) {}

    float distance_to(float px, float py) const {
        float dx = x - px;
        float dy = y - py;
        return sqrt(dx * dx + dy * dy);
    }
    
    // Copy from Task pointer (from bt_nodes.h)
    void copy_from(const void* task_ptr);
};

// ===================== CBBA Message =====================
/**
 * Message structure for ESP-NOW communication
 */
struct CBBAMessage {
    int agent_id;
    std::map<int, float> winning_bids;      // y: task_id -> bid value
    std::map<int, int> winning_agents;      // z: task_id -> agent_id
    std::map<int, unsigned long> timestamps; // s: agent_id -> timestamp
    
    void clear() {
        agent_id = -1;
        winning_bids.clear();
        winning_agents.clear();
        timestamps.clear();
    }
};

// ===================== CBBA Class =====================
/**
 * Consensus-Based Bundle Algorithm implementation.
 * Follows the algorithm from the CBBA paper with full consensus rules.
 */
class CBBA {
public:
    /**
     * Constructor
     * @param agent_id This agent's ID
     * @param config CBBA configuration parameters
     */
    CBBA(int agent_id, const CBBAConfig& config = CBBAConfig());
    
    /**
     * Set agent's current position
     */
    void set_position(float x, float y);
    
    /**
     * Main decision function - called each tick
     * @param tasks List of available tasks
     * @return Assigned task ID, or -1 if no assignment
     */
    int decide(const std::vector<CBBATask>& tasks);
    
    /**
     * Process received message from another agent
     * @param message Received CBBA message
     */
    void receive_message(const CBBAMessage& message);
    
    /**
     * Process received message from JSON (ESP-NOW)
     * @param doc JSON document containing message
     */
    void receive_message_json(const JsonDocument& doc);
    
    /**
     * Get message to share with other agents
     * [MODIFIED] Only includes valid (non-completed) tasks
     * @param message Output message structure
     */
    void get_message_to_share(CBBAMessage& message);
    
    /**
     * Get message to share as JSON (for ESP-NOW)
     * [MODIFIED] Only includes valid (non-completed) tasks
     * @param doc Output JSON document
     */
    void get_message_to_share_json(JsonDocument& doc);
    
    /**
     * Add a nearby agent ID (for timestamp updates)
     */
    void add_nearby_agent(int agent_id);
    
    /**
     * Clear nearby agents list
     */
    void clear_nearby_agents();
    
    /**
     * Reset the algorithm state
     */
    void reset();
    
    /**
     * Clear a specific task from bundle (when task is completed)
     */
    void clear_task(int task_id);
    
    // Getters
    bool has_task() const { return !_bundle.empty(); }
    int get_assigned_task_id() const { return _assigned_task_id; }
    CBBAPhase get_phase() const { return _phase; }
    const std::vector<int>& get_bundle() const { return _bundle; }
    const std::vector<int>& get_path() const { return _path; }
    
    // Debug
    void print_state() const;

private:
    // Agent info
    int _agent_id;
    float _pos_x, _pos_y;
    CBBAConfig _config;
    
    // CBBA state
    std::map<int, int> _z;           // Winning agent list (task_id -> agent_id)
    std::map<int, float> _y;         // Winning bid list (task_id -> bid value)
    std::map<int, unsigned long> _s; // Timestamp list (agent_id -> timestamp)
    std::vector<int> _bundle;        // Bundle (list of task IDs)
    std::vector<int> _path;          // Path (list of task IDs, same as bundle for now)
    
    CBBAPhase _phase;
    int _assigned_task_id;
    float _no_bundle_duration;
    unsigned long _last_decide_time;
    
    // Nearby agents for timestamp update
    std::vector<int> _nearby_agents;
    
    // Received messages buffer
    std::vector<CBBAMessage> _messages_received;
    
    // Local tasks cache
    std::vector<CBBATask> _local_tasks;
    
    // ===== Phase 1: Build Bundle (Algorithm 3) =====
    void _build_bundle(const std::vector<CBBATask>& tasks);
    
    /**
     * Get bid values for all tasks (Algorithm 3, Line 7)
     * @param tasks Available tasks
     * @param my_bid_list Output: task_id -> bid value
     * @param best_insertion_idx_list Output: task_id -> best insertion index
     */
    void _get_my_bid_value_list(
        const std::vector<CBBATask>& tasks,
        std::map<int, float>& my_bid_list,
        std::map<int, int>& best_insertion_idx_list
    );
    
    /**
     * Calculate score along path (Eqn 11 in CBBA paper)
     */
    float _calculate_score_along_path(const std::vector<int>& path);
    
    /**
     * Get best task to add (Algorithm 3, Lines 8-9)
     * @return Task ID or -1 if none
     */
    int _get_best_task(const std::map<int, float>& my_bid_list);
    
    // ===== Phase 2: Consensus =====
    
    /**
     * Update timestamp (Eqn 5)
     */
    void _update_time_stamp();
    
    /**
     * Apply consensus rules for a task
     */
    void _apply_consensus_rules(int task_id, const CBBAMessage& msg);
    
    /**
     * Update winning bid and agent (used in consensus rules)
     */
    void _update(int task_id, float y_k, int z_k);
    
    /**
     * Reset winning bid and agent
     */
    void _reset_task(int task_id);
    
    /**
     * Leave action (currently no-op, matches Python)
     */
    void _leave();
    
    /**
     * Update bundle and path after consensus
     * @return true if bundle changed
     */
    bool _update_bundle_and_path(std::vector<int>& updated_bundle, std::vector<int>& updated_path);
    
    /**
     * [NEW] Clean up completed tasks from internal maps (_y, _z)
     * Called at the beginning of decide() to remove stale entries
     */
    void _cleanup_completed_tasks();
    
    // Utility
    CBBATask* _find_task(int task_id);
    bool _is_task_in_path(int task_id) const;
    
    // Merge timestamps (max of each)
    void _merge_timestamps(const std::map<int, unsigned long>& other);
};

#endif // CBBA_H