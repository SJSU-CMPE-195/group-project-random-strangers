template<typename... Faults>
bool FaultManager<Faults...>::get_master_fault_state() const {
    return has_active_fault();
}

template<typename... Faults>
typename FaultManager<Faults...>::FaultId
FaultManager<Faults...>::get_master_fault_code() const {
    for (unsigned index = 0; index < fault_condition_count; ++index) {
        if (fault_triggered[index]) {
            return fault_ids[index];
        }
    }

    return 0;
}

template<typename... Faults>
const char* FaultManager<Faults...>::get_master_fault_reason() const {
    for (unsigned index = 0; index < fault_condition_count; ++index) {
        if (fault_triggered[index]) {
            return fault_reasons[index];
        }
    }

    return nullptr;
}

template<typename... Faults>
void FaultManager<Faults...>::attach_master_fault_set_callback(Callback callback) {
    master_set_callback = callback;
}

template<typename... Faults>
void FaultManager<Faults...>::clear_master_fault_callbacks() {
    master_set_callback = nullptr;
    master_clear_callback = nullptr;
}

template<typename... Faults>
void FaultManager<Faults...>::attach_master_fault_clear_callback(Callback callback) {
    master_clear_callback = callback;
}

template<typename... Faults>
void FaultManager<Faults...>::dispatch_fault(FaultId fault_id) {
    const unsigned fault_index = find_fault_index(fault_id);

    if (fault_index < fault_condition_count) {
        dispatch_fault_at(fault_index);
    }
}

template<typename... Faults>
void FaultManager<Faults...>::attach_fault_set_callback(
    FaultId fault_id,
    Callback callback) {
    const unsigned fault_index = find_fault_index(fault_id);

    if (fault_index < fault_condition_count) {
        fault_set_callbacks[fault_index] = callback;
    }
}

template<typename... Faults>
void FaultManager<Faults...>::clear_fault_set_callback(FaultId fault_id) {
    attach_fault_set_callback(fault_id, nullptr);
}

template<typename... Faults>
void FaultManager<Faults...>::clear_fault_state(FaultId fault_id) {
    const unsigned fault_index = find_fault_index(fault_id);

    if (fault_index < fault_condition_count) {
        clear_fault_at(fault_index);
    }
}

template<typename... Faults>
void FaultManager<Faults...>::attach_fault_clear_callback(
    FaultId fault_id,
    Callback callback) {
    const unsigned fault_index = find_fault_index(fault_id);

    if (fault_index < fault_condition_count) {
        fault_clear_callbacks[fault_index] = callback;
    }
}

template<typename... Faults>
void FaultManager<Faults...>::clear_fault_clear_callback(FaultId fault_id) {
    attach_fault_clear_callback(fault_id, nullptr);
}

template<typename... Faults>
typename FaultManager<Faults...>::Callback
FaultManager<Faults...>::get_fault_set_callback_fn(FaultId fault_id) const {
    const unsigned fault_index = find_fault_index(fault_id);

    if (fault_index >= fault_condition_count) {
        return nullptr;
    }

    return fault_set_callbacks[fault_index];
}

template<typename... Faults>
typename FaultManager<Faults...>::Callback
FaultManager<Faults...>::get_fault_clear_callback_fn(FaultId fault_id) const {
    const unsigned fault_index = find_fault_index(fault_id);

    if (fault_index >= fault_condition_count) {
        return nullptr;
    }

    return fault_clear_callbacks[fault_index];
}

template<typename... Faults>
unsigned FaultManager<Faults...>::find_fault_index(FaultId fault_id) const {
    for (unsigned index = 0; index < fault_condition_count; ++index) {
        if (fault_ids[index] == fault_id) {
            return index;
        }
    }

    return fault_condition_count;
}

template<typename... Faults>
bool FaultManager<Faults...>::has_active_fault() const {
    for (unsigned index = 0; index < fault_condition_count; ++index) {
        if (fault_triggered[index]) {
            return true;
        }
    }

    return false;
}

template<typename... Faults>
void FaultManager<Faults...>::dispatch_fault_at(unsigned fault_index) {
    if (fault_triggered[fault_index]) {
        return;
    }

    fault_triggered[fault_index] = true;

    if (fault_set_callbacks[fault_index] != nullptr) {
        fault_set_callbacks[fault_index]();
    }

    if (master_set_callback != nullptr) {
        master_set_callback();
    }
}

template<typename... Faults>
void FaultManager<Faults...>::clear_fault_at(unsigned fault_index) {
    if (!fault_triggered[fault_index]) {
        return;
    }

    fault_triggered[fault_index] = false;

    if (fault_clear_callbacks[fault_index] != nullptr) {
        fault_clear_callbacks[fault_index]();
    }

    if (master_clear_callback != nullptr) {
        master_clear_callback();
    }
}
