/*
  bt_nodes.cpp - Behavior Tree nodes implementation for MONA ESP robot
  
  Ported from Python implementation (base_bt_nodes.py, bt_nodes.py)
  for onboard execution on ESP32.
*/

#include "bt_nodes.h"

// ===================== BTContext Implementation =====================

BTContext::BTContext(int id)
    : agent_id(id),
      position_x(0), position_y(0), yaw(0),
      assigned_task_id(-1),
      cbba(nullptr),
      arrive_threshold(40.0),
      work_rate(1.0),
      explore_target_x(0), explore_target_y(0),
      has_explore_target(false),
      explore_start_ms(0),
      explore_duration_ms(2000),
      has_target_position(false),
      target_position_x(0), target_position_y(0)
{
    // Default exploration bounds
    explore_bounds.x_min = 40;
    explore_bounds.x_max = 1300;
    explore_bounds.y_min = 40;
    explore_bounds.y_max = 700;
}

void BTContext::reset_exploration() {
    has_explore_target = false;
    explore_start_ms = 0;
}

// ===================== BTNode Base Class =====================

BTNode::BTNode(const char* name)
    : _name(name), _status(BT_FAILURE)
{
}

void BTNode::halt() {
    // Default implementation does nothing
}

void BTNode::reset() {
    _status = BT_FAILURE;
}

// ===================== Sequence =====================

Sequence::Sequence(const char* name, std::vector<BTNode*> children)
    : BTNode(name), _children(children), _current_child_index(0)
{
}

BTStatus Sequence::tick(BTContext& ctx) {
    while (_current_child_index < _children.size()) {
        BTStatus status = _children[_current_child_index]->tick(ctx);
        _status = status;
        
        if (status == BT_RUNNING) {
            return BT_RUNNING;
        } else if (status == BT_FAILURE) {
            _halt_children();
            _current_child_index = 0;
            return BT_FAILURE;
        } else if (status == BT_SUCCESS) {
            _current_child_index++;
        }
    }
    
    _current_child_index = 0;
    _halt_children();
    return BT_SUCCESS;
}

void Sequence::halt() {
    _current_child_index = 0;
    _halt_children();
}

void Sequence::reset() {
    BTNode::reset();
    _current_child_index = 0;
    for (auto* child : _children) {
        child->reset();
    }
}

void Sequence::_halt_children() {
    for (auto* child : _children) {
        child->halt();
    }
}

// ===================== ReactiveSequence =====================

ReactiveSequence::ReactiveSequence(const char* name, std::vector<BTNode*> children)
    : BTNode(name), _children(children)
{
}

BTStatus ReactiveSequence::tick(BTContext& ctx) {
    for (auto* child : _children) {
        BTStatus status = child->tick(ctx);
        _status = status;
        
        if (status == BT_FAILURE) {
            _halt_children();
            return BT_FAILURE;
        }
        if (status == BT_RUNNING) {
            return BT_RUNNING;
        }
    }
    
    _halt_children();
    return BT_SUCCESS;
}

void ReactiveSequence::halt() {
    _halt_children();
}

void ReactiveSequence::_halt_children() {
    for (auto* child : _children) {
        child->halt();
    }
}

// ===================== Fallback =====================

Fallback::Fallback(const char* name, std::vector<BTNode*> children)
    : BTNode(name), _children(children), _current_child_index(0)
{
}

BTStatus Fallback::tick(BTContext& ctx) {
    while (_current_child_index < _children.size()) {
        BTStatus status = _children[_current_child_index]->tick(ctx);
        _status = status;
        
        if (status == BT_RUNNING) {
            return BT_RUNNING;
        } else if (status == BT_SUCCESS) {
            _halt_children();
            _current_child_index = 0;
            return BT_SUCCESS;
        } else if (status == BT_FAILURE) {
            _current_child_index++;
        }
    }
    
    _current_child_index = 0;
    _halt_children();
    return BT_FAILURE;
}

void Fallback::halt() {
    _current_child_index = 0;
    _halt_children();
}

void Fallback::reset() {
    BTNode::reset();
    _current_child_index = 0;
    for (auto* child : _children) {
        child->reset();
    }
}

void Fallback::_halt_children() {
    for (auto* child : _children) {
        child->halt();
    }
}

// ===================== ReactiveFallback =====================

ReactiveFallback::ReactiveFallback(const char* name, std::vector<BTNode*> children)
    : BTNode(name), _children(children)
{
}

BTStatus ReactiveFallback::tick(BTContext& ctx) {
    for (auto* child : _children) {
        BTStatus status = child->tick(ctx);
        _status = status;
        
        if (status == BT_SUCCESS) {
            _halt_children();
            return BT_SUCCESS;
        }
        if (status == BT_RUNNING) {
            return BT_RUNNING;
        }
    }
    
    _halt_children();
    return BT_FAILURE;
}

void ReactiveFallback::halt() {
    _halt_children();
}

void ReactiveFallback::_halt_children() {
    for (auto* child : _children) {
        child->halt();
    }
}

// ===================== GatherLocalInfo =====================

GatherLocalInfo::GatherLocalInfo(const char* name)
    : BTNode(name)
{
}

BTStatus GatherLocalInfo::tick(BTContext& ctx) {
    // This node mainly serves as a sync point in the tree
    // CBBA position is set externally when UDP message is received
    _status = BT_SUCCESS;
    return BT_SUCCESS;
}

// ===================== AssignTask =====================

AssignTask::AssignTask(const char* name)
    : BTNode(name)
{
}

BTStatus AssignTask::tick(BTContext& ctx) {
    // Check if a task is assigned (CBBA::decide() is called from main code)
    int task_id = ctx.assigned_task_id;
    
    if (task_id >= 0) {
        // Verify task is still valid
        bool task_valid = false;
        for (Task* task : ctx.tasks) {
            if (task != nullptr && task->task_id == task_id) {
                task_valid = true;
                break;
            }
        }
        
        if (task_valid) {
            _status = BT_SUCCESS;
            return BT_SUCCESS;
        }
    }
    
    // No valid task assigned
    ctx.assigned_task_id = -1;
    _status = BT_FAILURE;
    return BT_FAILURE;
}

// ===================== MoveToTarget =====================

MoveToTarget::MoveToTarget(const char* name)
    : BTNode(name)
{
}

BTStatus MoveToTarget::tick(BTContext& ctx) {
    if (ctx.assigned_task_id < 0) {
        _status = BT_FAILURE;
        return BT_FAILURE;
    }
    
    // Find assigned task
    Task* assigned_task = nullptr;
    for (Task* task : ctx.tasks) {
        if (task != nullptr && task->task_id == ctx.assigned_task_id) {
            assigned_task = task;
            break;
        }
    }
    
    if (assigned_task == nullptr) {
        _status = BT_FAILURE;
        return BT_FAILURE;
    }
    
    // Set motion target
    ctx.has_target_position = true;
    ctx.target_position_x = assigned_task->x;
    ctx.target_position_y = assigned_task->y;
    
    _status = BT_RUNNING;
    return BT_RUNNING;
}

// ===================== ExecuteTask =====================

ExecuteTask::ExecuteTask(const char* name)
    : BTNode(name)
{
}

BTStatus ExecuteTask::tick(BTContext& ctx) {
    if (ctx.assigned_task_id < 0) {
        _status = BT_FAILURE;
        return BT_FAILURE;
    }
    
    // Find assigned task
    Task* assigned_task = nullptr;
    for (Task* task : ctx.tasks) {
        if (task != nullptr && task->task_id == ctx.assigned_task_id) {
            assigned_task = task;
            break;
        }
    }
    
    if (assigned_task == nullptr) {
        _status = BT_FAILURE;
        return BT_FAILURE;
    }
    
    // In PC-managed mode, work processing is handled by PC
    // Robot just maintains position at task location
    ctx.has_target_position = true;
    ctx.target_position_x = assigned_task->x;
    ctx.target_position_y = assigned_task->y;
    
    _status = BT_RUNNING;
    return BT_RUNNING;
}

// ===================== Explore =====================

Explore::Explore(const char* name)
    : BTNode(name)
{
}

BTStatus Explore::tick(BTContext& ctx) {
    unsigned long now = millis();
    
    // Check if we need a new exploration target
    if (!ctx.has_explore_target || 
        (now - ctx.explore_start_ms) >= ctx.explore_duration_ms) {
        
        // Generate random waypoint within bounds
        ctx.explore_target_x = random(
            (long)ctx.explore_bounds.x_min, 
            (long)ctx.explore_bounds.x_max + 1
        );
        ctx.explore_target_y = random(
            (long)ctx.explore_bounds.y_min, 
            (long)ctx.explore_bounds.y_max + 1
        );
        ctx.has_explore_target = true;
        ctx.explore_start_ms = now;
    }
    
    // Set motion target to exploration waypoint
    ctx.has_target_position = true;
    ctx.target_position_x = ctx.explore_target_x;
    ctx.target_position_y = ctx.explore_target_y;
    
    _status = BT_RUNNING;
    return BT_RUNNING;
}

void Explore::halt() {
    BTNode::halt();
}

// ===================== IsTaskCompleted =====================

IsTaskCompleted::IsTaskCompleted(const char* name)
    : BTNode(name)
{
}

BTStatus IsTaskCompleted::tick(BTContext& ctx) {
    if (ctx.assigned_task_id < 0) {
        _status = BT_FAILURE;
        return BT_FAILURE;
    }
    
    // Check if task is still in the task list
    bool task_found = false;
    for (Task* task : ctx.tasks) {
        if (task != nullptr && task->task_id == ctx.assigned_task_id) {
            if (task->completed) {
                ctx.assigned_task_id = -1;
                _status = BT_SUCCESS;
                return BT_SUCCESS;
            }
            task_found = true;
            break;
        }
    }
    
    if (!task_found) {
        // Task no longer in list - completed
        ctx.assigned_task_id = -1;
        _status = BT_SUCCESS;
        return BT_SUCCESS;
    }
    
    _status = BT_FAILURE;
    return BT_FAILURE;
}

// ===================== IsArrivedAtTarget =====================

IsArrivedAtTarget::IsArrivedAtTarget(const char* name)
    : BTNode(name)
{
}

BTStatus IsArrivedAtTarget::tick(BTContext& ctx) {
    if (ctx.assigned_task_id < 0) {
        _status = BT_FAILURE;
        return BT_FAILURE;
    }
    
    // Find assigned task
    Task* assigned_task = nullptr;
    for (Task* task : ctx.tasks) {
        if (task != nullptr && task->task_id == ctx.assigned_task_id) {
            assigned_task = task;
            break;
        }
    }
    
    if (assigned_task == nullptr) {
        _status = BT_FAILURE;
        return BT_FAILURE;
    }
    
    // Calculate distance to task
    float dx = assigned_task->x - ctx.position_x;
    float dy = assigned_task->y - ctx.position_y;
    float distance = sqrt(dx * dx + dy * dy);
    
    if (distance < ctx.arrive_threshold) {
        _status = BT_SUCCESS;
        return BT_SUCCESS;
    }
    
    _status = BT_FAILURE;
    return BT_FAILURE;
}

// ===================== BehaviorTree =====================

BehaviorTree::BehaviorTree(BTNode* root)
    : _root(root)
{
}

BehaviorTree::~BehaviorTree() {
    // Note: In production, implement proper cleanup
}

BTStatus BehaviorTree::tick(BTContext& ctx) {
    ctx.has_target_position = false;
    
    if (_root == nullptr) {
        return BT_FAILURE;
    }
    
    return _root->tick(ctx);
}

void BehaviorTree::reset() {
    if (_root != nullptr) {
        _root->reset();
    }
}

// ===================== TreeBuilder =====================

BTNode* TreeBuilder::Sequence(const char* name, std::vector<BTNode*> children) {
    return new ::Sequence(name, children);
}

BTNode* TreeBuilder::ReactiveSequence(const char* name, std::vector<BTNode*> children) {
    return new ::ReactiveSequence(name, children);
}

BTNode* TreeBuilder::Fallback(const char* name, std::vector<BTNode*> children) {
    return new ::Fallback(name, children);
}

BTNode* TreeBuilder::ReactiveFallback(const char* name, std::vector<BTNode*> children) {
    return new ::ReactiveFallback(name, children);
}
