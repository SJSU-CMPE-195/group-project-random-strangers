//compile-time fault definitions with runtime fault state and callbacks
#pragma once

/**
 * @file FaultManager.h
 * @brief Defines a type-safe, compile-time fault manager.
 *
 * Fault definitions are supplied as template arguments. Their IDs and reason
 * strings are validated at compile time. The manager stores only runtime state
 * and callback pointers; fault reason strings are never copied at runtime.
 *
 * EXAMPLE USAGE
 *
 * using ControllerFaultManager = FaultManager<
 *     FaultCondition<"Overcurrent", 80>,
 *     FaultCondition<"Overvoltage", 90>
 * >;
 *
 * ControllerFaultManager fault_manager;
 *
 * fault_manager.attach_master_fault_set_callback(on_any_fault_set);
 * fault_manager.attach_fault_clear_callback(80, on_overcurrent_cleared);
 * fault_manager.dispatch_fault(80);
 *
 * if (fault_manager.get_master_fault_state()) {
 *     const char* reason = fault_manager.get_master_fault_reason();
 * }
 *
 * fault_manager.clear_fault_state(80);
 *
 * Fault IDs must be non-zero and unique. A fault ID of zero is reserved as
 * the return value of get_master_fault_code() when no fault is active.
 */

/**
 * @brief A fixed-size string that can be used as a non-type template argument.
 *
 * The character data is stored in the type at compile time. FaultManager only
 * keeps a pointer to that data, so it does not copy reason strings at runtime.
 */
template<unsigned N>
struct FixedString {
    char data[N];

    //constexpr constructor allows deduction from a string literal
    constexpr FixedString(const char (&str)[N]) {
        for (unsigned index = 0; index < N; ++index) {
            data[index] = str[index];
        }
    }
};

/**
 * @brief Describes one fault at compile time.
 *
 * @tparam fault_reason_value Null-terminated human-readable fault reason.
 * @tparam fault_id_value Non-zero fault ID. IDs must be unique in a manager.
 */
template <const FixedString fault_reason_value, const unsigned char fault_id_value>
struct FaultCondition {
    FaultCondition() = delete;

    static constexpr FixedString fault_reason = fault_reason_value;
    static constexpr unsigned char fault_id = fault_id_value;
};

template <const FixedString fault_reason_value>
struct FixedStringChecks {
    static constexpr unsigned size = sizeof(fault_reason_value.data);
    static constexpr bool non_empty = fault_reason_value.data[0] != '\0';
    static constexpr bool null_terminated = fault_reason_value.data[size - 1] == '\0';
};

/**
 * @brief Stores fault state and dispatches fault callbacks.
 *
 * The fault list is fixed when the type is compiled. No heap allocation,
 * string copying, or fault registration is performed at runtime.
 */
template<typename... Faults>
class FaultManager {
public:
    using FaultId = unsigned char;
    using Callback = void (*)(void);

    /**
     * @brief Creates a manager with all faults initially cleared.
     */
    FaultManager() = default;

    FaultManager(const FaultManager&) = delete;
    FaultManager& operator=(const FaultManager&) = delete;

    /**
     * @brief Returns true when at least one fault is active.
     */
    bool get_master_fault_state() const;

    /**
     * @brief Returns the first active fault ID, or zero when none is active.
     */
    FaultId get_master_fault_code() const;

    /**
     * @brief Returns the number of currently active faults.
     */
    unsigned get_active_fault_count() const;

    /**
     * @brief Returns the ID of the active fault at a zero-based position.
     *
     * Faults are returned in the same order as their FaultCondition template
     * arguments. Returns zero when the position is outside the active list.
     */
    FaultId get_active_fault_code(unsigned active_fault_index) const;

    /**
     * @brief Returns the reason for an active fault at a zero-based position.
     *
     * Returns nullptr when the position is outside the active list.
     */
    const char* get_active_fault_reason(unsigned active_fault_index) const;

    /**
     * @brief Returns the reason for an active fault ID, or nullptr otherwise.
     */
    const char* get_fault_reason(FaultId fault_id) const;

    /**
     * @brief Returns true when the specified fault ID is currently active.
     */
    bool is_fault_active(FaultId fault_id) const;

    /**
     * @brief Returns the reason for the first active fault, or nullptr when
     * no fault is active.
     *
     * The returned pointer refers to compile-time storage and remains valid
     * for the lifetime of the program. The caller must not modify it or free it.
     */
    const char* get_master_fault_reason() const;

    /**
     * @brief Sets the callback invoked when a new fault becomes active.
     *
     * Passing nullptr disables the callback.
     */
    void attach_master_fault_set_callback(Callback callback);

    /**
     * @brief Disables both master set and master clear callbacks.
     */
    void clear_master_fault_callbacks();

    /**
     * @brief Sets the callback invoked when an active fault is cleared.
     *
     * Passing nullptr disables the callback.
     */
    void attach_master_fault_clear_callback(Callback callback);

    /**
     * @brief Activates a fault and invokes its callbacks once.
     *
     * Invalid IDs are ignored. Dispatching an already-active fault does not
     * invoke callbacks again.
     */
    void dispatch_fault(FaultId fault_id);

    /**
     * @brief Sets the callback invoked when a specific fault becomes active.
     *
     * Invalid IDs are ignored. Passing nullptr disables the callback.
     */
    void attach_fault_set_callback(FaultId fault_id, Callback callback);

    /**
     * @brief Disables the set callback for a specific fault.
     */
    void clear_fault_set_callback(FaultId fault_id);

    /**
     * @brief Clears a fault and invokes its callbacks once.
     *
     * Invalid IDs are ignored. Clearing an already-cleared fault does not
     * invoke callbacks again.
     */
    void clear_fault_state(FaultId fault_id);

    /**
     * @brief Sets the callback invoked when a specific fault is cleared.
     *
     * Invalid IDs are ignored. Passing nullptr disables the callback.
     */
    void attach_fault_clear_callback(FaultId fault_id, Callback callback);

    /**
     * @brief Disables the clear callback for a specific fault.
     */
    void clear_fault_clear_callback(FaultId fault_id);

    /**
     * @brief Returns the set callback for a specific fault, or nullptr.
     */
    Callback get_fault_set_callback_fn(FaultId fault_id) const;

    /**
     * @brief Returns the clear callback for a specific fault, or nullptr.
     */
    Callback get_fault_clear_callback_fn(FaultId fault_id) const;

private:
    static constexpr unsigned fault_condition_count = sizeof...(Faults);

    //A one-element fallback keeps the private arrays well-formed long enough
    //for the clearer static_assert below to report an empty fault list.
    static constexpr unsigned storage_count =
        fault_condition_count == 0 ? 1 : fault_condition_count;

    inline static constexpr FaultId fault_ids[storage_count] = {
        Faults::fault_id...
    };

    inline static constexpr const char* fault_reasons[storage_count] = {
        Faults::fault_reason.data...
    };

    unsigned find_fault_index(FaultId fault_id) const;
    bool has_active_fault() const;
    void dispatch_fault_at(unsigned fault_index);
    void clear_fault_at(unsigned fault_index);

    bool fault_triggered[storage_count]{};
    Callback fault_set_callbacks[storage_count]{};
    Callback fault_clear_callbacks[storage_count]{};
    Callback master_set_callback = nullptr;
    Callback master_clear_callback = nullptr;

    //ensure that fault ids are non-zero and unique at compile time
    constexpr static bool fault_ids_valid = []() constexpr {
        for (unsigned index = 0; index < fault_condition_count; ++index) {
            if (fault_ids[index] == 0) {
                return false;
            }

            for (unsigned other_index = index + 1;
                 other_index < fault_condition_count;
                 ++other_index) {
                if (fault_ids[index] == fault_ids[other_index]) {
                    return false;
                }
            }
        }

        return true;
    }();

    static_assert(fault_condition_count > 0,
                  "At least one FaultCondition is required.");
    static_assert(fault_ids_valid,
                  "FaultCondition IDs must be non-zero and unique.");
    static_assert((FixedStringChecks<Faults::fault_reason>::non_empty && ...),
                  "FaultCondition reason strings must be non-empty.");
    static_assert((FixedStringChecks<Faults::fault_reason>::null_terminated && ...),
                  "FaultCondition reason strings must be null-terminated.");
};

#include "FaultManager.tpp"
