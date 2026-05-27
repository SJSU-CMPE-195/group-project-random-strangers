#include "FaultManager.h"

template<typename... FaultCondition>
FaultManager<FaultCondition...>::FaultManager() {
    singleton_instance = this;
}

template<typename... FaultCondition>
bool FaultManager<FaultCondition...>::get_master_fault_state() {
    for (unsigned index = 0; index < fault_condition_count; ++index) {
        if (faults[index].fault_triggered) {
            return true;
        }
    }

    return false;
}

template<typename... FaultCondition>
unsigned short FaultManager<FaultCondition...>::get_master_fault_code() {
    for (unsigned index = 0; index < fault_condition_count; ++index) {
        if (faults[index].fault_triggered) {
            return faults[index].fault_id;
        }
    }

    return 0;
}

template<typename... FaultCondition>
const char* FaultManager<FaultCondition...>::get_master_fault_reason() {
    for (unsigned index = 0; index < fault_condition_count; ++index) {
        if (faults[index].fault_triggered) {
            return faults[index].fault_reason;
        }
    }

    return nullptr;
}

template<typename... FaultCondition>
void FaultManager<FaultCondition...>::attach_master_fault_set_callback(void (*callback)(void)) {
    master_set_callback = callback;
}

template<typename... FaultCondition>
void FaultManager<FaultCondition...>::clear_master_fault_callbacks() {
    master_set_callback = nullptr;
    master_clear_callback = nullptr;
}

template<typename... FaultCondition>
void FaultManager<FaultCondition...>::attach_master_fault_clear_callback(void (*callback)(void)) {
    master_clear_callback = callback;
}

template<typename... FaultCondition>
void FaultManager<FaultCondition...>::dispatch_fault(const unsigned short fault_code) {
    internal_fault_storage* fault = find_fault_storage(fault_code);
    dispatch_fault_internal(fault);
}

template<typename... FaultCondition>
void FaultManager<FaultCondition...>::attach_fault_set_callback(const unsigned short fault_id, void (*callback)(void)) {
    internal_fault_storage* fault = find_fault_storage(fault_id);
    if (fault == nullptr) {
        return;
    }

    fault->set_callback = callback;
}

template<typename... FaultCondition>
void FaultManager<FaultCondition...>::clear_fault_set_callback(const unsigned short fault_id) {
    internal_fault_storage* fault = find_fault_storage(fault_id);
    if (fault == nullptr) {
        return;
    }

    fault->set_callback = nullptr;
}

template<typename... FaultCondition>
void FaultManager<FaultCondition...>::clear_fault_state(const unsigned short fault_code) {
    internal_fault_storage* fault = find_fault_storage(fault_code);
    clear_fault_internal(fault);
}

template<typename... FaultCondition>
void FaultManager<FaultCondition...>::attach_fault_clear_callback(const unsigned short fault_id, void (*callback)(void)) {
    internal_fault_storage* fault = find_fault_storage(fault_id);
    if (fault == nullptr) {
        return;
    }

    fault->clear_callback = callback;
}

template<typename... FaultCondition>
void FaultManager<FaultCondition...>::clear_fault_clear_callback(const unsigned short fault_id) {
    internal_fault_storage* fault = find_fault_storage(fault_id);
    if (fault == nullptr) {
        return;
    }

    fault->clear_callback = nullptr;
}

template<typename... FaultCondition>
void (*FaultManager<FaultCondition...>::get_fault_set_callback_fn(const unsigned short fault_id))(void) {
    const internal_fault_storage* fault = find_fault_storage(fault_id);
    if (fault == nullptr) {
        return nullptr;
    }

    return fault->set_callback;
}

template<typename... FaultCondition>
void (*FaultManager<FaultCondition...>::get_fault_clear_callback_fn(const unsigned short fault_id))(void) {
    const internal_fault_storage* fault = find_fault_storage(fault_id);
    if (fault == nullptr) {
        return nullptr;
    }

    return fault->clear_callback;
}

template<typename... FaultCondition>
typename FaultManager<FaultCondition...>::internal_fault_storage*
FaultManager<FaultCondition...>::find_fault_storage(const unsigned short fault_id) {
    for (unsigned index = 0; index < fault_condition_count; ++index) {
        if (faults[index].fault_id == fault_id) {
            return &faults[index];
        }
    }

    return nullptr;
}

template<typename... FaultCondition>
const typename FaultManager<FaultCondition...>::internal_fault_storage*
FaultManager<FaultCondition...>::find_fault_storage(const unsigned short fault_id) const {
    for (unsigned index = 0; index < fault_condition_count; ++index) {
        if (faults[index].fault_id == fault_id) {
            return &faults[index];
        }
    }

    return nullptr;
}

template<typename... FaultCondition>
template <unsigned short fault_id>
void FaultManager<FaultCondition...>::dispatch_fault_dummy_function() {
    if (singleton_instance == nullptr) {
        return;
    }

    internal_fault_storage* fault = singleton_instance->find_fault_storage(fault_id);
    if (fault != nullptr && fault->fault_triggered) {
        return;
    }

    singleton_instance->dispatch_fault_internal(fault);
}

template<typename... FaultCondition>
template <unsigned short fault_id>
void FaultManager<FaultCondition...>::clear_fault_dummy_function() {
    if (singleton_instance == nullptr) {
        return;
    }

    internal_fault_storage* fault = singleton_instance->find_fault_storage(fault_id);
    if (fault != nullptr && !fault->fault_triggered) {
        return;
    }

    singleton_instance->clear_fault_internal(fault);
}

template<typename... FaultCondition>
void FaultManager<FaultCondition...>::dispatch_fault_internal(internal_fault_storage* fault) {
    if (fault == nullptr) {
        return;
    }

    fault->fault_triggered = true;

    if (fault->set_callback != nullptr) {
        fault->set_callback();
    }

    if (master_set_callback != nullptr) {
        master_set_callback();
    }
}

template<typename... FaultCondition>
void FaultManager<FaultCondition...>::clear_fault_internal(internal_fault_storage* fault) {
    if (fault == nullptr) {
        return;
    }

    fault->fault_triggered = false;

    if (fault->clear_callback != nullptr) {
        fault->clear_callback();
    }

    if (master_clear_callback != nullptr) {
        master_clear_callback();
    }
}