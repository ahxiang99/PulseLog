#pragma once
#include <concepts>

template <typename T>
concept Sensor = requires(T t) {
    t.read();
    t.onDataReceived();
};