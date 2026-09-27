#pragma once

#include <atomic>

// Set by TIM3_IRQHandler every 5th tick (100ms @ 20ms/tick) to request that
// the main loop enqueue the next round of sensor reads. Shared between ISR
// (Src/isr.cpp) and the main loop (Src/main.cpp), hence atomic rather than a
// plain volatile bool: volatile only stops the compiler caching the value,
// it does not make the read-modify-write in the main loop race-free with
// the ISR's write.
extern std::atomic_bool g_cmd_execute;
