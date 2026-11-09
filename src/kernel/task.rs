// ZerOS — Preemptive Multitasking & Task Lifecycle Manager (M6.1 + M7.2)
//
// Implements a preemptive round-robin kernel task scheduler for AArch64 with:
//   - Hardware context switching (__switch_context in assembly: x19-x30, sp)
//   - Time-slice preemption (`TIME_SLICE_TICKS = 5` = 50ms) directly from Timer IRQ
//   - Cooperative yielding (`yield_now`) and timer-driven sleep (`sleep_ms`)
//   - Full task lifecycle: `spawn`, `kill(pid)`, and automatic stack page reaping
//     (`reap_zombies_locked`) that returns 16KB stacks to `page_alloc` upon exit
//   - CPU time & preemption accounting (`ticks_run`, `switches`, `preemptions`)

#![allow(dead_code)]

use core::arch::global_asm;
use crate::arch::timer;
use crate::kernel::interrupts;
use crate::mm::page_alloc::{self, PAGE_SIZE};

/// Maximum number of concurrent kernel tasks
pub const MAX_TASKS: usize = 16;

/// Stack size per task in pages (4 pages = 16 KB)
const STACK_PAGES: usize = 4;

/// Time slice per task in 100Hz timer ticks (5 ticks = 50ms)
pub const TIME_SLICE_TICKS: u64 = 5;

/// Saved CPU callee-saved registers for context switching (AAPCS64)
#[repr(C)]
#[derive(Default, Clone, Copy)]
pub struct TaskContext {
    pub x19: u64,
    pub x20: u64,
    pub x21: u64,
    pub x22: u64,
    pub x23: u64,
    pub x24: u64,
    pub x25: u64,
    pub x26: u64,
    pub x27: u64,
    pub x28: u64,
    pub x29: u64, // Frame Pointer (FP)
    pub x30: u64, // Link Register (LR / return address)
    pub sp:  u64, // Stack Pointer (SP_EL1)
}

/// Task execution state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TaskState {
    Unused,
    Ready,
    Running,
    Sleeping(u64), // Wake-up tick number
    Finished,      // Waiting to be reaped once CPU switches off its stack
}

/// Task Control Block (TCB)
#[derive(Clone, Copy)]
pub struct Task {
    pub id: usize,
    pub name: &'static str,
    pub state: TaskState,
    pub context: TaskContext,
    pub entry: Option<fn()>,
    pub stack_base: usize,
    pub stack_size: usize,
    pub ticks_run: u64,
    pub slice_ticks: u64,
    pub switches: u64,
    pub preemptions: u64,
}

impl Task {
    pub const fn empty() -> Self {
        Task {
            id: 0,
            name: "",
            state: TaskState::Unused,
            context: TaskContext {
                x19: 0, x20: 0, x21: 0, x22: 0,
                x23: 0, x24: 0, x25: 0, x26: 0,
                x27: 0, x28: 0, x29: 0, x30: 0,
                sp: 0,
            },
            entry: None,
            stack_base: 0,
            stack_size: 0,
            ticks_run: 0,
            slice_ticks: 0,
            switches: 0,
            preemptions: 0,
        }
    }
}

// ─── Assembly context switch ───
global_asm!(r#"
.global __switch_context
__switch_context:
    // x0 = *mut TaskContext (old)
    // x1 = *const TaskContext (new)
    stp     x19, x20, [x0, #0]
    stp     x21, x22, [x0, #16]
    stp     x23, x24, [x0, #32]
    stp     x25, x26, [x0, #48]
    stp     x27, x28, [x0, #64]
    stp     x29, x30, [x0, #80]
    mov     x9, sp
    str     x9, [x0, #96]

    ldp     x19, x20, [x1, #0]
    ldp     x21, x22, [x1, #16]
    ldp     x23, x24, [x1, #32]
    ldp     x25, x26, [x1, #48]
    ldp     x27, x28, [x1, #64]
    ldp     x29, x30, [x1, #80]
    ldr     x9, [x1, #96]
    mov     sp, x9

    ret
"#);

extern "C" {
    fn __switch_context(old: *mut TaskContext, new: *const TaskContext);
}

/// Global task table
static mut TASKS: [Task; MAX_TASKS] = [Task::empty(); MAX_TASKS];

/// Index of the currently running task
static mut CURRENT_TASK: usize = 0;

/// Whether the scheduler has been initialized
static mut INITIALIZED: bool = false;

/// Total number of tasks reaped (stacks freed back to `page_alloc`)
static mut TOTAL_REAPED: u64 = 0;

/// Total number of involuntary time-slice preemptions across the system
static mut TOTAL_PREEMPTIONS: u64 = 0;

/// Initialize the task scheduler.
/// Registers the current execution flow as Task 0 ("idle").
pub fn init() {
    interrupts::without_interrupts(|| unsafe {
        TASKS[0] = Task {
            id: 0,
            name: "idle",
            state: TaskState::Running,
            context: TaskContext::default(),
            entry: None,
            stack_base: 0, // Uses boot stack (0x00000000 - 0x00080000)
            stack_size: 0x80000,
            ticks_run: 0,
            slice_ticks: 0,
            switches: 1,
            preemptions: 0,
        };
        CURRENT_TASK = 0;
        INITIALIZED = true;
    });

    crate::println!(
        "[OK] Scheduler: preemptive RR (max {} tasks, {}KB stack, {}ms slice)",
        MAX_TASKS,
        STACK_PAGES * PAGE_SIZE / 1024,
        TIME_SLICE_TICKS * 10
    );
}

/// Allocate a contiguous multi-page stack from the page allocator.
fn alloc_stack(pages: usize) -> Option<(usize, usize)> {
    let mut highest = 0usize;
    let mut lowest = 0usize;

    for i in 0..pages {
        let page = page_alloc::alloc_page()?;
        if i == 0 {
            highest = page;
            lowest = page;
        } else {
            if page < lowest {
                lowest = page;
            }
            if page > highest {
                highest = page;
            }
        }
    }

    let size = pages * PAGE_SIZE;
    if highest - lowest + PAGE_SIZE == size {
        Some((lowest, lowest + size))
    } else {
        Some((highest, highest + PAGE_SIZE))
    }
}

/// Free all physical pages belonging to a task's stack back to `page_alloc`.
unsafe fn free_task_stack(stack_base: usize, stack_size: usize) {
    if stack_base == 0 || stack_size == 0 {
        return;
    }
    let mut addr = stack_base;
    let end = stack_base + stack_size;
    while addr < end {
        page_alloc::free_page(addr);
        addr += PAGE_SIZE;
    }
}

/// Reap any `Finished` tasks that are no longer the currently executing task.
/// Frees their physical stack pages and marks their TCB slot `Unused`.
/// MUST be called with interrupts disabled.
unsafe fn reap_zombies_locked() {
    let cur = CURRENT_TASK;
    for i in 1..MAX_TASKS {
        if i != cur && TASKS[i].state == TaskState::Finished {
            let base = TASKS[i].stack_base;
            let size = TASKS[i].stack_size;
            let name = TASKS[i].name;
            free_task_stack(base, size);
            TASKS[i] = Task::empty();
            TOTAL_REAPED += 1;
            crate::println!(
                "[TASK] Reaped task #{} '{}' -> freed {}KB stack at 0x{:08x}",
                i,
                name,
                size / 1024,
                base
            );
        }
    }
}

/// Trampoline function where all newly spawned tasks begin execution.
extern "C" fn task_entry_trampoline() -> ! {
    // Re-enable IRQs because __switch_context was called inside without_interrupts / IRQ
    interrupts::enable();

    // Retrieve entry function for the current task
    let entry_fn = unsafe {
        let cur = CURRENT_TASK;
        TASKS[cur].entry
    };

    if let Some(func) = entry_fn {
        func();
    }

    // If the task function returns, mark task as Finished and yield so another task reaps us
    interrupts::without_interrupts(|| unsafe {
        let cur = CURRENT_TASK;
        TASKS[cur].state = TaskState::Finished;
        crate::println!("[TASK] Task #{} '{}' finished", cur, TASKS[cur].name);
    });

    loop {
        yield_now();
    }
}

/// Spawn a new kernel task.
/// Returns `Some(task_id)` on success, or `None` if the task table is full or out of memory.
pub fn spawn(name: &'static str, entry: fn()) -> Option<usize> {
    interrupts::without_interrupts(|| unsafe {
        reap_zombies_locked();

        let mut slot = None;
        for i in 1..MAX_TASKS {
            if TASKS[i].state == TaskState::Unused {
                slot = Some(i);
                break;
            }
        }
        let id = slot?;

        let (stack_base, stack_top) = alloc_stack(STACK_PAGES)?;
        let aligned_sp = stack_top & !0xF;

        let mut ctx = TaskContext::default();
        ctx.sp = aligned_sp as u64;
        ctx.x30 = task_entry_trampoline as usize as u64;

        TASKS[id] = Task {
            id,
            name,
            state: TaskState::Ready,
            context: ctx,
            entry: Some(entry),
            stack_base,
            stack_size: stack_top - stack_base,
            ticks_run: 0,
            slice_ticks: 0,
            switches: 0,
            preemptions: 0,
        };

        crate::println!(
            "[OK] Task spawned: #{} '{}' (stack 0x{:08x}-0x{:08x})",
            id,
            name,
            stack_base,
            stack_top
        );

        Some(id)
    })
}

/// Terminate a task by PID (`pid >= 1`).
/// Frees its stack pages immediately if it is not the currently running task,
/// or marks it `Finished` and schedules away if it is the current task.
pub fn kill(pid: usize) -> bool {
    if pid == 0 || pid >= MAX_TASKS {
        return false; // Cannot kill Task 0 (idle)
    }

    interrupts::without_interrupts(|| unsafe {
        if TASKS[pid].state == TaskState::Unused || TASKS[pid].state == TaskState::Finished {
            return false;
        }
        crate::println!("[TASK] Killing task #{} '{}'...", pid, TASKS[pid].name);
        TASKS[pid].state = TaskState::Finished;
        if pid == CURRENT_TASK {
            schedule_locked();
        } else {
            reap_zombies_locked();
        }
        true
    })
}

/// Cooperatively yield the CPU to the next Ready task.
pub fn yield_now() {
    interrupts::without_interrupts(|| unsafe {
        if !INITIALIZED {
            return;
        }
        schedule_locked();
    });
}

/// Put the current task to sleep for `ms` milliseconds.
pub fn sleep_ms(ms: u64) {
    let ticks = ((ms + 9) / 10).max(1);
    sleep_ticks(ticks);
}

/// Put the current task to sleep for `ticks` timer interrupts.
pub fn sleep_ticks(ticks: u64) {
    let wake_tick = timer::ticks() + ticks;

    interrupts::without_interrupts(|| unsafe {
        if !INITIALIZED {
            return;
        }
        let cur = CURRENT_TASK;
        if cur != 0 {
            TASKS[cur].state = TaskState::Sleeping(wake_tick);
        }
        schedule_locked();
    });
}

/// Called from the timer IRQ handler every tick (100Hz).
/// Wakes up sleeping tasks whose deadline has passed and accounts CPU time & slice ticks.
pub fn on_tick(current_tick: u64) {
    unsafe {
        if !INITIALIZED {
            return;
        }

        let cur = CURRENT_TASK;
        TASKS[cur].ticks_run += 1;
        TASKS[cur].slice_ticks += 1;

        for i in 1..MAX_TASKS {
            if let TaskState::Sleeping(wake_tick) = TASKS[i].state {
                if current_tick >= wake_tick {
                    TASKS[i].state = TaskState::Ready;
                }
            }
        }
    }
}

/// Called at the end of `irq_dispatch` (while IRQs are still masked on the current core).
/// Preempts the running task if its time slice (`TIME_SLICE_TICKS`) has expired
/// (or if it is the idle task) and another task is `Ready`.
pub fn preempt_from_irq() {
    unsafe {
        if !INITIALIZED {
            return;
        }
        let cur = CURRENT_TASK;
        let slice_expired = cur == 0 || TASKS[cur].slice_ticks >= TIME_SLICE_TICKS;
        if !slice_expired {
            return;
        }

        // Check if any non-idle task is Ready to run
        let mut has_ready = false;
        for i in 1..MAX_TASKS {
            if i != cur && TASKS[i].state == TaskState::Ready {
                has_ready = true;
                break;
            }
        }

        if has_ready {
            if cur != 0 {
                TASKS[cur].preemptions += 1;
                TOTAL_PREEMPTIONS += 1;
            }
            schedule_locked();
        }
    }
}

/// Internal scheduler — MUST be called with interrupts disabled.
/// Selects the next Ready task in round-robin order and switches context.
unsafe fn schedule_locked() {
    reap_zombies_locked();

    let cur = CURRENT_TASK;

    if TASKS[cur].state == TaskState::Running {
        TASKS[cur].state = TaskState::Ready;
    }
    TASKS[cur].slice_ticks = 0;

    let mut next = None;
    for offset in 1..=MAX_TASKS {
        let idx = (cur + offset) % MAX_TASKS;
        if idx == 0 {
            continue;
        }
        if TASKS[idx].state == TaskState::Ready {
            next = Some(idx);
            break;
        }
    }

    let next_idx = next.unwrap_or(0);
    TASKS[next_idx].state = TaskState::Running;
    TASKS[next_idx].slice_ticks = 0;

    if next_idx == cur {
        return;
    }

    TASKS[next_idx].switches += 1;
    CURRENT_TASK = next_idx;

    let old_ctx = &raw mut TASKS[cur].context;
    let new_ctx = &raw const TASKS[next_idx].context;
    __switch_context(old_ctx, new_ctx);

    // After switching back to this task, reap any task that just finished
    reap_zombies_locked();
}

/// Get the ID of the currently running task
pub fn current_id() -> usize {
    unsafe { CURRENT_TASK }
}

/// Get the name of the currently running task
pub fn current_name() -> &'static str {
    unsafe { TASKS[CURRENT_TASK].name }
}

/// Get total number of reaped tasks since boot
pub fn reaped_count() -> u64 {
    unsafe { TOTAL_REAPED }
}

/// Get total number of preemptions since boot
pub fn preemptions_count() -> u64 {
    unsafe { TOTAL_PREEMPTIONS }
}

/// Get the number of active (Ready, Running, or Sleeping) tasks
pub fn active_count() -> usize {
    interrupts::without_interrupts(|| unsafe {
        let mut count = 0;
        for i in 0..MAX_TASKS {
            match TASKS[i].state {
                TaskState::Ready | TaskState::Running | TaskState::Sleeping(_) => {
                    count += 1;
                }
                _ => {}
            }
        }
        count
    })
}

/// Print a summary table of all tasks
pub fn print_tasks() {
    interrupts::without_interrupts(|| unsafe {
        crate::println!("PID  NAME          STATE       TICKS  PREEMPT  STACK");
        crate::println!("---  ------------  ----------  -----  -------  -------------------");
        for i in 0..MAX_TASKS {
            let t = &TASKS[i];
            if t.state == TaskState::Unused {
                continue;
            }
            let state_str = match t.state {
                TaskState::Unused => "UNUSED",
                TaskState::Ready => "READY",
                TaskState::Running => "RUNNING",
                TaskState::Sleeping(_) => "SLEEPING",
                TaskState::Finished => "FINISHED",
            };
            crate::println!(
                " #{:<2} {:<12}  {:<10}  {:>5}  {:>7}  0x{:08x} ({}KB)",
                t.id,
                t.name,
                state_str,
                t.ticks_run,
                t.preemptions,
                t.stack_base,
                t.stack_size / 1024
            );
        }
    });
}

/// Iterate over all non-unused tasks safely with interrupts masked (used by GUI System Monitor).
pub fn with_tasks<F: FnMut(&Task)>(mut f: F) {
    interrupts::without_interrupts(|| unsafe {
        for i in 0..MAX_TASKS {
            let t = &TASKS[i];
            if t.state != TaskState::Unused {
                f(t);
            }
        }
    });
}
