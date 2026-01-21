/*
  bt_nodes.h - Behavior Tree nodes for MONA ESP robot
  
  Ported from Python implementation (base_bt_nodes.py, bt_nodes.py)
  for onboard execution on ESP32.
*/

#ifndef BT_NODES_H
#define BT_NODES_H

#include <Arduino.h>
#include <vector>
#include <map>
#include <cmath>

// Forward declaration for CBBA (defined in cbba.h)
class CBBA;

// ===================== BT Status =====================
enum BTStatus {
    BT_SUCCESS = 1,
    BT_FAILURE = 2,
    BT_RUNNING = 3
};

// ===================== Task Class =====================
/**
 * Simple task representation for onboard processing.
 * Defined here so BT nodes can access task properties.
 */
class Task {
public:
    int task_id;
    float x;
    float y;
    float amount;
    bool completed;

    Task() : task_id(-1), x(0), y(0), amount(0), completed(false) {}

    Task(int id, float x_pos, float y_pos, float amt)
        : task_id(id), x(x_pos), y(y_pos), amount(amt), completed(false) {}

    float distance_to(float px, float py) const {
        float dx = x - px;
        float dy = y - py;
        return sqrt(dx * dx + dy * dy);
    }
};

// ===================== BT Context (Blackboard) =====================
/**
 * Shared context/blackboard for all BT nodes.
 * Contains agent state, task information, and inter-node communication.
 */
struct BTContext {
    // Agent identification
    int agent_id;
    
    // Position and orientation (updated from PC via UDP)
    float position_x;
    float position_y;
    float yaw;
    
    // Task information
    std::vector<Task*> tasks;
    int assigned_task_id;
    
    // CBBA reference for decision making (void* to avoid circular include)
    void* cbba;
    
    // Thresholds and parameters
    float arrive_threshold;
    float work_rate;
    
    // Exploration parameters
    float explore_target_x;
    float explore_target_y;
    bool has_explore_target;
    unsigned long explore_start_ms;
    unsigned long explore_duration_ms;
    struct {
        float x_min, x_max, y_min, y_max;
    } explore_bounds;
    
    // Motion output (set by action nodes, read by motion controller)
    bool has_target_position;
    float target_position_x;
    float target_position_y;
    
    // Constructor with defaults
    BTContext(int id = 0);
    
    // Reset exploration state
    void reset_exploration();
};

// ===================== Base Node Class =====================
/**
 * Abstract base class for all behavior tree nodes.
 */
class BTNode {
public:
    BTNode(const char* name);
    virtual ~BTNode() = default;
    
    // Main execution method - must be implemented by derived classes
    virtual BTStatus tick(BTContext& ctx) = 0;
    
    // Called when node is halted (e.g., parent fails/succeeds)
    virtual void halt();
    
    // Reset node state
    virtual void reset();
    
    // Getters
    const char* get_name() const { return _name; }
    BTStatus get_status() const { return _status; }
    
protected:
    const char* _name;
    BTStatus _status;
};

// ===================== Control Nodes =====================

/**
 * Sequence: Executes children in order until one fails.
 * Remembers which child was running for next tick.
 */
class Sequence : public BTNode {
public:
    Sequence(const char* name, std::vector<BTNode*> children);
    BTStatus tick(BTContext& ctx) override;
    void halt() override;
    void reset() override;
    
private:
    std::vector<BTNode*> _children;
    size_t _current_child_index;
    
    void _halt_children();
};

/**
 * ReactiveSequence: Re-evaluates all children from the start each tick.
 * Does not remember running child position.
 */
class ReactiveSequence : public BTNode {
public:
    ReactiveSequence(const char* name, std::vector<BTNode*> children);
    BTStatus tick(BTContext& ctx) override;
    void halt() override;
    
private:
    std::vector<BTNode*> _children;
    
    void _halt_children();
};

/**
 * Fallback (Selector): Executes children until one succeeds.
 * Remembers which child was running for next tick.
 */
class Fallback : public BTNode {
public:
    Fallback(const char* name, std::vector<BTNode*> children);
    BTStatus tick(BTContext& ctx) override;
    void halt() override;
    void reset() override;
    
private:
    std::vector<BTNode*> _children;
    size_t _current_child_index;
    
    void _halt_children();
};

/**
 * ReactiveFallback: Re-evaluates all children from the start each tick.
 * Does not remember running child position.
 */
class ReactiveFallback : public BTNode {
public:
    ReactiveFallback(const char* name, std::vector<BTNode*> children);
    BTStatus tick(BTContext& ctx) override;
    void halt() override;
    
private:
    std::vector<BTNode*> _children;
    
    void _halt_children();
};

// ===================== Action Nodes =====================

/**
 * GatherLocalInfo: Updates context with local sensing information.
 * In onboard implementation, this mainly syncs CBBA position.
 */
class GatherLocalInfo : public BTNode {
public:
    GatherLocalInfo(const char* name = "GatherLocalInfo");
    BTStatus tick(BTContext& ctx) override;
};

/**
 * AssignTask: Uses CBBA to assign a task to this agent.
 * Returns SUCCESS if task assigned, FAILURE otherwise.
 */
class AssignTask : public BTNode {
public:
    AssignTask(const char* name = "AssignTask");
    BTStatus tick(BTContext& ctx) override;
};

/**
 * MoveToTarget: Sets motion target to assigned task position.
 * Returns RUNNING while moving.
 */
class MoveToTarget : public BTNode {
public:
    MoveToTarget(const char* name = "MoveToTarget");
    BTStatus tick(BTContext& ctx) override;
};

/**
 * ExecuteTask: Performs work at task location while following.
 * In PC-managed mode, just maintains position at task.
 * Returns RUNNING while executing.
 */
class ExecuteTask : public BTNode {
public:
    ExecuteTask(const char* name = "ExecuteTask");
    BTStatus tick(BTContext& ctx) override;
};

/**
 * Explore: Random exploration when no task is assigned.
 * Generates random waypoints within bounds.
 * Returns RUNNING while exploring.
 */
class Explore : public BTNode {
public:
    Explore(const char* name = "Explore");
    BTStatus tick(BTContext& ctx) override;
    void halt() override;
};

// ===================== Condition Nodes =====================

/**
 * IsTaskCompleted: Checks if assigned task is completed.
 * Task is completed if it's no longer in the task list (removed by PC).
 * Returns SUCCESS if completed, FAILURE otherwise.
 */
class IsTaskCompleted : public BTNode {
public:
    IsTaskCompleted(const char* name = "IsTaskCompleted");
    BTStatus tick(BTContext& ctx) override;
};

/**
 * IsArrivedAtTarget: Checks if agent is within threshold of task.
 * Returns SUCCESS if arrived, FAILURE otherwise.
 */
class IsArrivedAtTarget : public BTNode {
public:
    IsArrivedAtTarget(const char* name = "IsArrivedAtTarget");
    BTStatus tick(BTContext& ctx) override;
};

// ===================== Behavior Tree =====================
/**
 * Main behavior tree class.
 * Owns the root node and provides tick interface.
 */
class BehaviorTree {
public:
    BehaviorTree(BTNode* root);
    ~BehaviorTree();
    
    // Execute one tick of the behavior tree
    BTStatus tick(BTContext& ctx);
    
    // Reset tree state
    void reset();
    
private:
    BTNode* _root;
};

// ===================== Tree Builder Helper =====================
/**
 * Helper class for building behavior trees in a readable way.
 * 
 * Usage example (matches default_bt.xml):
 *   BTNode* tree = TreeBuilder::ReactiveSequence("Root", {
 *       new GatherLocalInfo(),
 *       TreeBuilder::ReactiveFallback("TaskAssign", {
 *           new AssignTask(),
 *           new Explore()
 *       })
 *   });
 */
class TreeBuilder {
public:
    static BTNode* Sequence(const char* name, std::vector<BTNode*> children);
    static BTNode* ReactiveSequence(const char* name, std::vector<BTNode*> children);
    static BTNode* Fallback(const char* name, std::vector<BTNode*> children);
    static BTNode* ReactiveFallback(const char* name, std::vector<BTNode*> children);
};

#endif // BT_NODES_H
