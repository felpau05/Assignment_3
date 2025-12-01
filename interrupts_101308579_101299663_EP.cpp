/**
 * @file interrupts_101308579_101299663_EP.cpp
 * @author Ajay Uppal 101308579
 * @author Paul Felfli 101299663
 * @brief Assignment 3 Part 1 of SYSC4001
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

std::tuple<std::string> run_simulation(std::vector<PCB> list_processes) {

    std::vector<PCB> ready_queue;   //The ready queue of processes
    std::vector<PCB> wait_queue;    //The wait queue of processes

    //ms
    unsigned int current_time = 0;
    PCB running;

    //Initialize an empty running process
    idle_CPU(running);

    std::string execution_status;

    //make the output table (the header row)
    execution_status = print_exec_header();

    // Helper: get pointer to the PCB in the master list by PID
    auto get_process = [&](int pid) -> PCB* {
        for (auto &p : list_processes) {
            if (p.PID == pid) {
                return &p;
            }
        }
        return nullptr;
    };

    //Pre-process
    std::map<int, unsigned int> total_cpu_time;
    std::map<int, unsigned int> io_remaining;

    while (!all_process_terminated(list_processes)) {

        // 1) Advance I/O for processes in WAITING
        for (std::size_t i = 0; i < wait_queue.size(); ) {
            int pid = wait_queue[i].PID;
            auto it = io_remaining.find(pid);

            if (it != io_remaining.end()) {
                if (it->second > 0) {
                    it->second -= 1;   // 1 ms of I/O passes
                }
                if (it->second == 0) {
                    // I/O finished: WAITING -> READY
                    wait_queue[i].state = READY;
                    execution_status += print_exec_status(current_time,
                                                          pid,
                                                          WAITING,
                                                          READY);

                    if (PCB* p = get_process(pid)) {
                        p->state = READY;
                    }

                    ready_queue.push_back(wait_queue[i]);
                    wait_queue.erase(wait_queue.begin() + i);
                    io_remaining.erase(it);
                    continue; 
                }
            }
            ++i;
        }

        // 2) Admit new arrivals 
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

            // NEW processes with no partition 
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

        // 3) If CPU is idle, dispatch next process by priority. External priorities are NON-PREEMPTIVE.
        if (running.state != RUNNING && !ready_queue.empty()) {
            std::sort(ready_queue.begin(), ready_queue.end(),
                      [](const PCB &a, const PCB &b) {
                          if (a.priority == b.priority) {
                              return a.arrival_time < b.arrival_time;
                          }
                          return a.priority < b.priority;
                      });

            PCB next = ready_queue.front();
            ready_queue.erase(ready_queue.begin());

            // Align with master list, set RUNNING
            PCB *p = get_process(next.PID);
            if (p) {
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

            execution_status += print_exec_status(current_time,
                                                  running.PID,
                                                  READY,
                                                  RUNNING);
        }

        // 4) Execute one millisecond on the CPU (if RUNNING)
        if (running.state == RUNNING) {
            if (running.remaining_time > 0) {
                running.remaining_time -= 1;
            }

            total_cpu_time[running.PID]++;

            bool finished = (running.remaining_time == 0);
            bool needs_io = false;

            // I/O request based on total CPU time used (ms)
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
            }
            else if (needs_io) {
                // RUNNING -> WAITING (start I/O)
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
            }
            else {
                // Still RUNNING
                if (PCB* p = get_process(running.PID)) {
                    p->remaining_time = running.remaining_time;
                    p->state          = RUNNING;
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