//class that groups fault conditions and dispatches master and per fault actions
#pragma once

/**
 * EXAMPLE USAGE
 * 
 * FaultManager<
 *      FaultCondition<"Overcurrent", 80>
 *      FaultCondition<"Overvoltage", 90>
 * > fault_manager;
 */

template<unsigned N>
struct FixedString {
    char data[N];

    // Constexpr constructor allows deduction from a literal
    constexpr FixedString(const char (&str)[N]) {
        for (unsigned index = 0; index < N; ++index) {
            data[index] = str[index];
        }
    }
};

template <const FixedString fault_reason_value, const unsigned short fault_id_value>
struct FaultCondition{
    FaultCondition() = delete;
    static constexpr FixedString fault_reason = fault_reason_value;
    static constexpr const unsigned short fault_id = fault_id_value;
};

template <const FixedString fault_reason_value>
struct FixedStringChecks {
    static constexpr unsigned size = sizeof(fault_reason_value.data);
    static constexpr bool non_empty = fault_reason_value.data[0] != '\0';
    static constexpr bool null_terminated = fault_reason_value.data[size - 1] == '\0';
};

template<typename... FaultCondition>
class FaultManager {
public:
    /**
     * @brief Constructs the manager and registers the singleton instance.
     */
    FaultManager();
    /**
     * @brief Gets the current fault state of the manager
     * 
     * @return 1 if any fault is set
     */
    bool get_master_fault_state();

    /**
     * @brief Gets the first active fault code
     * 
     * @return the id of the first active fault
     */
    unsigned short get_master_fault_code();

    /**
     * @brief gets the current fault reason of the manager
     * 
     * @return the fault reason of the first active fault
     */
    const char* get_master_fault_reason();

    /**
     * @brief Attaches a callback invoked when any fault is set.
     *
     * @param callback Callback to invoke when a fault is set.
     */
    void attach_master_fault_set_callback(void (*callback)(void));

    /**
     * @brief Clears all master fault callbacks.
     */
    void clear_master_fault_callbacks();

    /**
     * @brief Attaches a callback invoked when any fault is cleared.
     *
     * @param callback Callback to invoke when a fault is cleared.
     */
    void attach_master_fault_clear_callback(void (*callback)(void));

    /**
     * @brief Sets a specific fault condition and triggers callbacks.
     *
     * @param fault_code Fault ID to set.
     */
    void dispatch_fault(const unsigned short fault_code);

    /**
     * @brief Attaches a callback for a specific fault set event.
     *
     * @param fault_id Fault ID to attach.
     * @param callback Callback to invoke when the fault is set.
     */
    void attach_fault_set_callback(const unsigned short fault_id, void (*callback)(void));

    /**
     * @brief Clears the set callback for a specific fault.
     *
     * @param fault_id Fault ID to clear.
     */
    void clear_fault_set_callback(const unsigned short fault_id);

    /**
     * @brief Clears a specific fault condition and triggers callbacks.
     *
     * @param fault_code Fault ID to clear.
     */
    void clear_fault_state(const unsigned short fault_code);

    /**
     * @brief Attaches a callback for a specific fault clear event.
     *
     * @param fault_id Fault ID to attach.
     * @param callback Callback to invoke when the fault is cleared.
     */
    void attach_fault_clear_callback(const unsigned short fault_id, void (*callback)(void));

    /**
     * @brief Clears the clear callback for a specific fault.
     *
     * @param fault_id Fault ID to clear.
     */
    void clear_fault_clear_callback(const unsigned short fault_id);

    /**
     * @brief Gets the current set callback for a specific fault.
     *
     * @param fault_id Fault ID to query.
     * @return Callback pointer, or nullptr if not set.
     */
    void (*get_fault_set_callback_fn(const unsigned short fault_id))(void);

    /**
     * @brief Gets the current clear callback for a specific fault.
     *
     * @param fault_id Fault ID to query.
     * @return Callback pointer, or nullptr if not set.
     */
    void (*get_fault_clear_callback_fn(const unsigned short fault_id))(void);
private:
    constexpr static unsigned fault_condition_count = sizeof...(FaultCondition);
    inline static FaultManager* singleton_instance = nullptr;

    template <unsigned short fault_id>
    static void dispatch_fault_dummy_function();
    template <unsigned short fault_id>
    static void clear_fault_dummy_function();

    struct internal_fault_storage {
        const char* fault_reason;
        const unsigned short fault_id;
        bool fault_triggered;
        void (*set_callback)(void);
        void (*clear_callback)(void);
    };

    void dispatch_fault_internal(internal_fault_storage*);
    void clear_fault_internal(internal_fault_storage*);

    internal_fault_storage* find_fault_storage(const unsigned short fault_id);
    const internal_fault_storage* find_fault_storage(const unsigned short fault_id) const;

    static void (*master_set_callback)(void) = nullptr;
    static void (*master_clear_callback)(void) = nullptr;
    static internal_fault_storage faults[fault_condition_count] = {
        {FaultCondition::fault_reason.data, FaultCondition::fault_id, false, nullptr, nullptr}...
    };

    //ensure that fault ids are unique at compiletime
    constexpr static bool fault_ids_unique = []() constexpr {
        const unsigned short ids[] = {FaultCondition::fault_id...};
        for (unsigned i = 0; i < fault_condition_count; ++i) {
            for (unsigned j = i + 1; j < fault_condition_count; ++j) {
                if (ids[i] == ids[j]) {
                    return false;
                }
            }
        }
        return true;
    }();
    static_assert(fault_condition_count > 0, "At least one FaultCondition is required.");
    static_assert((FixedStringChecks<FaultCondition::fault_reason>::non_empty && ...),
                  "FaultCondition reason strings must be non-empty.");
    static_assert((FixedStringChecks<FaultCondition::fault_reason>::null_terminated && ...),
                  "FaultCondition reason strings must be null-terminated.");
    static_assert(fault_ids_unique, "FaultCondition IDs must be unique.");
};