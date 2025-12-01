/**
 * @file interrupts.cpp
 * @author Sasisekhar Govind
 * @brief template main.cpp file for Assignment 3 Part 1 of SYSC4001
 * 
 */

#include"interrupts_101308579_101299663.hpp"
#include<map>
#include<algorithm>

void FCFS(std::vector<PCB> &ready_queue) {
    std::sort( 
                ready_queue.begin(),
                ready_queue.end(),
                []( const PCB &first, const PCB &second ){
                    return (first.arrival_time > second.arrival_time); 
                } 
            );
}

static constexpr unsigned int QUANTUM_MS = 100;

std::tuple<std::string /* add std::string for bonus mark */ > run_simulation(std::vector<PCB> list_processes) {

    std::vector<PCB> ready_queue;   //The ready queue of processes
    std::vector<PCB> wait_queue;    //The wait queue of processes

    unsigned int current_time = 0;
    PCB running;

    //Initialize an empty running process
    idle_CPU(running);

    std::string execution_status;

    //make the output table (the header row)
    execution_status = print_exec_header();

    // CPU + I/O accounting
    std::map<int, unsigned int> total_cpu_time;   // total CPU ms per PID
    std::map<int, unsigned int> io_remaining;     // remaining I/O ms for WAITING PIDs

    unsigned int quantum_used = 0;

    // Helper: find the canonical PCB in list_processes by PID
    auto get_process = [&](int pid) -> PCB* {
        for (auto &p : list_processes) {
            if (p.PID == pid) {
                return &p;
            }
        }
        return nullptr;
    };

    // Helper: pick the highest-priority READY process 
    auto pick_next_ready = [&]() -> PCB {
        std::sort(ready_queue.begin(), ready_queue.end(),
                  [](const PCB &a, const PCB &b) {
                      if (a.priority == b.priority) {
                          return a.arrival_time < b.arrival_time;
                      }
                      return a.priority < b.priority;
                  });
        PCB next = ready_queue.front();
        ready_queue.erase(ready_queue.begin());
        return next;
    };

    while (!all_process_terminated(list_processes)) {

        // 1) Advance I/O for processes in WAITING
        for (std::size_t i = 0; i < wait_queue.size(); ) {
            int pid = wait_queue[i].PID;
            auto it = io_remaining.find(pid);

            if (it != io_remaining.end()) {
                if (it->second > 0) {
                    it->second -= 1;   
                }

                if (it->second == 0) {
                    // I/O complete: WAITING -> READY
                    wait_queue[i].state = READY;
                    execution_status += print_exec_status(current_time,
                                                          pid,
                                                          WAITING,
                                                          READY);

                    if (PCB* p = get_process(pid)) {
                        p->state          = READY;
                        p->remaining_time = wait_queue[i].remaining_time;
                    }

                    ready_queue.push_back(wait_queue[i]);
                    wait_queue.erase(wait_queue.begin() + i);
                    io_remaining.erase(it);
                    continue;   
                }
            }
            ++i;
        }

        // 2) Admit new arrivals and try to allocate memory
        for (auto &proc : list_processes) {
            if (proc.arrival_time > current_time) {
                continue;
            }

            // First appearance: NOT_ASSIGNED -> NEW
            if (proc.state == NOT_ASSIGNED) {
                proc.state = NEW;
                execution_status += print_exec_status(current_time,
                                                      proc.PID,
                                                      NOT_ASSIGNED,
                                                      NEW);
            }

            // NEW process waiting for memory
            if (proc.state == NEW && proc.partition_number == -1) {
                if (assign_memory(proc)) {
                    proc.state = READY;
                    execution_status += print_exec_status(current_time,
                                                          proc.PID,
                                                          NEW,
                                                          READY);
                    ready_queue.push_back(proc);
                }
            }
        }

        // 2b) Priority-based preemption: if a higher-priority process is READY, preempt current RUNNING.
        if (running.state == RUNNING && !ready_queue.empty()) {
            bool higher_exists = false;
            for (const auto &p : ready_queue) {
                if (p.priority < running.priority) {  
                    higher_exists = true;
                    break;
                }
            }

            if (higher_exists) {
                int pid = running.PID;

                running.state = READY;
                execution_status += print_exec_status(current_time,
                                                      pid,
                                                      RUNNING,
                                                      READY);

                if (PCB* p = get_process(pid)) {
                    p->state          = READY;
                    p->remaining_time = running.remaining_time;
                }

                ready_queue.push_back(running);
                idle_CPU(running);
                quantum_used = 0;
            }
        }

        // 3) If CPU is idle, dispatch next READY process (EP and RR)
        if (running.state != RUNNING && !ready_queue.empty()) {
            PCB next = pick_next_ready();
            int pid  = next.PID;

            if (PCB* p = get_process(pid)) {
                if (p->start_time < 0) {
                    p->start_time = static_cast<int>(current_time);
                }
                p->state          = RUNNING;
                p->remaining_time = next.remaining_time;
                running           = *p;
            } else {
                running = next;
                if (running.start_time < 0) {
                    running.start_time = static_cast<int>(current_time);
                }
                running.state = RUNNING;
            }

            quantum_used = 0;

            execution_status += print_exec_status(current_time,
                                                  running.PID,
                                                  READY,
                                                  RUNNING);
        }

        // 4) Execute 1 ms on the CPU (if someone is RUNNING)
        if (running.state == RUNNING) {
            if (running.remaining_time > 0) {
                running.remaining_time -= 1;
            }

            total_cpu_time[running.PID] += 1;
            quantum_used += 1;

            bool finished = (running.remaining_time == 0);
            bool needs_io = false;

            // I/O request based on total CPU time
            if (!finished && running.io_freq > 0) {
                unsigned int used = total_cpu_time[running.PID];
                if (used % running.io_freq == 0) {
                    needs_io = true;
                }
            }

            if (finished) {
                // RUNNING -> TERMINATED
                int pid = running.PID;
                running.state = TERMINATED;
                execution_status += print_exec_status(current_time,
                                                      pid,
                                                      RUNNING,
                                                      TERMINATED);

                free_memory(running);

                if (PCB* p = get_process(pid)) {
                    p->state          = TERMINATED;
                    p->remaining_time = 0;
                }

                idle_CPU(running);
                quantum_used = 0;
            }
            else if (needs_io) {
                // RUNNING -> WAITING
                int pid = running.PID;
                running.state = WAITING;
                execution_status += print_exec_status(current_time,
                                                      pid,
                                                      RUNNING,
                                                      WAITING);

                io_remaining[pid] = running.io_duration;

                if (PCB* p = get_process(pid)) {
                    p->state          = WAITING;
                    p->remaining_time = running.remaining_time;
                }

                wait_queue.push_back(running);
                idle_CPU(running);
                quantum_used = 0;
            }
            else {
                // Quantum-based preemption (RR) 
                bool quantum_expired = (quantum_used >= QUANTUM_MS);

                if (quantum_expired && !ready_queue.empty()) {
                    bool competitor_exists = false;
                    for (const auto &p : ready_queue) {
                        if (p.priority <= running.priority) {
                            competitor_exists = true;
                            break;
                        }
                    }

                    if (competitor_exists) {
                        int pid = running.PID;
                        running.state = READY;
                        execution_status += print_exec_status(current_time,
                                                              pid,
                                                              RUNNING,
                                                              READY);

                        if (PCB* p = get_process(pid)) {
                            p->state          = READY;
                            p->remaining_time = running.remaining_time;
                        }

                        ready_queue.push_back(running);
                        idle_CPU(running);
                        quantum_used = 0;
                    } else {
                        // No competitor at same/higher priority
                        if (PCB* p = get_process(running.PID)) {
                            p->remaining_time = running.remaining_time;
                            p->state          = RUNNING;
                        }
                    }
                } else {
                    // No quantum expiry
                    if (PCB* p = get_process(running.PID)) {
                        p->remaining_time = running.remaining_time;
                        p->state          = RUNNING;
                    }
                }
            }
        }

        current_time += 1;
    }
    
    //Close the output table
    execution_status += print_exec_footer();

    return std::make_tuple(execution_status);
}


int main(int argc, char** argv) {

    //Get the input file from the user
    if(argc != 2) {
        std::cout << "ERROR!\nExpected 1 argument, received " << argc - 1 << std::endl;
        std::cout << "To run the program, do: ./interrutps <your_input_file.txt>" << std::endl;
        return -1;
    }

    //Open the input file
    auto file_name = argv[1];
    std::ifstream input_file;
    input_file.open(file_name);

    //Ensure that the file actually opens
    if (!input_file.is_open()) {
        std::cerr << "Error: Unable to open file: " << file_name << std::endl;
        return -1;
    }

    //Parse the entire input file and populate a vector of PCBs.
    //To do so, the add_process() helper function is used (see include file).
    std::string line;
    std::vector<PCB> list_process;
    while(std::getline(input_file, line)) {
        auto input_tokens = split_delim(line, ", ");
        auto new_process = add_process(input_tokens);
        list_process.push_back(new_process);
    }
    input_file.close();

    //With the list of processes, run the simulation
    auto [exec] = run_simulation(list_process);

    write_output(exec, "execution.txt");

    return 0;
}