# FreeRTOS Real-Time Scheduler

## Overview

ESP32 FreeRTOS scheduler with 6 periodic + 1 sporadic task. SYNC-anchored release (T0-relative phase alignment), rate-monotonic priorities, GPIO task acknowledgement, and monitor statistics reporting. All periodic tasks achieve ≥99% deadline compliance; sporadic task responds within 30ms. Configured: 240 MHz CPU, 1 ms FreeRTOS tick.