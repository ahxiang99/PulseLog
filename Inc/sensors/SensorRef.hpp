#pragma once
#include "SensorConcepts.hpp"

struct SensorRef {
    void (*data_received_fn_)(void *ctx);
    void (*read_fn_)(void *ctx);
    void *ctx_;

    template <typename T> explicit SensorRef(T &obj)
    {
	data_received_fn_ = [](void *ctx) { static_cast<T *>(ctx)->onDataReceived(); };
	read_fn_ = [](void *ctx) { static_cast<T *>(ctx)->read(); };
	ctx_ = &obj;
    }

    SensorRef() : data_received_fn_(nullptr), read_fn_(nullptr), ctx_(nullptr)
    {
    }

    void notify() const
    {
	if (data_received_fn_) {
	    data_received_fn_(ctx_);
	}
    }

    void read() const
    {
	if (read_fn_) {
	    read_fn_(ctx_);
	}
    }
};
