# FreeRTOS Real-Time Scheduler

## Overview

ESP32 FreeRTOS scheduler with 6 periodic + 1 sporadic task. SYNC-anchored release (T0-relative phase alignment), rate-monotonic priorities, GPIO task acknowledgement, and monitor statistics reporting. All periodic tasks achieve ≥99% deadline compliance; sporadic task responds within 30ms. Configured: 240 MHz CPU, 1 ms FreeRTOS tick.

## main.c Breakdown

- Setup: Initializes monitor, GPIO, and PCNT edge counters for inputs A/B.
- Sync start: Waits for SYNC rising edge to define T0, resets scheduler state, and captures `g_sync_tick` for phase-aligned periodic release.
- RTOS primitives: Creates a binary semaphore (SYNC), counting semaphore (sporadic releases), and mutex (shared token protection).
- Tasks:
	- Periodic: A (10 ms), B (20 ms), AGG (20 ms), C (50 ms), D (50 ms)
	- Sporadic: S (ISR-triggered)
	- Monitor: low-priority reporting task
- Data flow: A/B count edges and generate tokens; AGG combines A/B tokens; C/D are mode-gated by `PIN_IN_MODE`; S runs on interrupt-triggered semaphore release.
- Scheduling: Fixed-priority, rate-monotonic ordering; all tasks pinned to core 0.
- Observability: ACK GPIOs go high/low around task execution; monitor hooks/reporting provide runtime timing and deadline stats.]

## FreeRTOS API and RTOS usage

![flowchart](docs/mermaid-diagram-2026-04-08-181007.png) 
