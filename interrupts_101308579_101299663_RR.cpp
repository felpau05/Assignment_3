/**
 * @file interrupts_101308579_101299663_RR.cpp
 * @author Ajay Uppal 101308579
 * @author Paul Felfli 101299663
 * @brief Assignment 3 Part 1 of SYSC4001
 * 
 */

#include<interrupts_101308579_101299663.hpp>
#include<map>

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
    std::vector<PCB> job_list;      //A list to keep track of all the processes. This is similar
                                    //to the "Process, Arrival time, Burst time" table that you
                                    //see in questions. You don't need to use it, I put it here
                                    //to make the code easier :).

    unsigned int current_time = 0;
    PCB running;

    //Initialize an empty running process
    idle_CPU(running);

    // Per-process 
    std::map<int, unsigned int> total_cpu_time;   // total CPU time each process has received
    std::map<int, unsigned int> io_remaining;     // remaining I/O time for processes in WAITING

    unsigned int quantum_used = 0;

    std::string execution_status;

    //make the output table (the header row)
    execution_status = print_exec_header();

    // Run until every process reaches TERMINATED
    while(!all_process_terminated(job_list) || job_list.empty()) {

        // 1) Advance I/O for processes in the WAITING queue
        for (std::size_t i = 0; i < wait_queue.size(); /* increment inside */) {
            int pid = wait_queue[i].PID;

            auto it = io_remaining.find(pid);
            if (it == io_remaining.end()) {
                wait_queue[i].state = READY;
                execution_status += print_exec_status(current_time, pid, WAITING, READY);
                ready_queue.push_back(wait_queue[i]);
                sync_queue(job_list, wait_queue[i]);
                wait_queue.erase(wait_queue.begin() + i);
                continue;
            }

            if (it->second > 0) {
                it->second -= 1;
            }

            if (it->second == 0) {
                // I/O finished: move process back to READY
                wait_queue[i].state = READY;
                execution_status += print_exec_status(current_time, pid, WAITING, READY);
                ready_queue.push_back(wait_queue[i]);
                sync_queue(job_list, wait_queue[i]);
                wait_queue.erase(wait_queue.begin() + i);
                io_remaining.erase(it);
            } else {
                ++i;
            }
        }

        // 2) Admit newly-arrived processes and those waiting for memory
        for (auto &proc : job_list) {
            // Only consider processes whose arrival time has passed
            if (proc.arrival_time > current_time) {
                continue;
            }

            // First time we see the process: NOT_ASSIGNED -> NEW
            if (proc.state == NOT_ASSIGNED) {
                proc.state = NEW;
                execution_status += print_exec_status(current_time,
                                                      proc.PID,
                                                      NOT_ASSIGNED,
                                                      NEW);
            }

            // For NEW processes with no memory yet, try to allocate a partition
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

        // 3) If CPU is idle, dispatch next process using Round Robin
        if (running.state != RUNNING) {
            if (!ready_queue.empty()) {
                // Round Robin: take from front (FIFO queue)
                PCB next = ready_queue.front();
                ready_queue.erase(ready_queue.begin());

                // Sync with canonical job_list entry and set RUNNING
                for (auto &p : job_list) {
                    if (p.PID == next.PID) {
                        if (p.start_time < 0) {
                            // First time this process ever gets CPU
                            p.start_time = static_cast<int>(current_time);
                        }
                        p.state = RUNNING;
                        next = p;   
                        break;
                    }
                }

                running = next;
                quantum_used = 0;

                execution_status += print_exec_status(current_time,
                                                      running.PID,
                                                      READY,
                                                      RUNNING);
            }
        }

        // 4) Execute one millisecond on the CPU 
        if (running.state == RUNNING) {
            // Give 1 ms of CPU
            if (running.remaining_time > 0) {
                running.remaining_time -= 1;
            }
            quantum_used += 1;
            total_cpu_time[running.PID] += 1;

            bool finished = (running.remaining_time == 0);
            bool needs_io = (!finished &&
                             running.io_freq > 0 &&
                             (total_cpu_time[running.PID] % running.io_freq == 0));
            bool quantum_expired = (!finished &&
                                    !needs_io &&
                                    quantum_used >= QUANTUM_MS);

            if (finished) {
                // RUNNING -> TERMINATED
                running.state = TERMINATED;
                execution_status += print_exec_status(current_time,
                                                      running.PID,
                                                      RUNNING,
                                                      TERMINATED);

                // Release memory partition
                free_memory(running);
                sync_queue(job_list, running);

                idle_CPU(running);
            } else if (needs_io) {
                // RUNNING -> WAITING
                running.state = WAITING;
                execution_status += print_exec_status(current_time,
                                                      running.PID,
                                                      RUNNING,
                                                      WAITING);

                io_remaining[running.PID] = running.io_duration;
                sync_queue(job_list, running);
                wait_queue.push_back(running);

                idle_CPU(running);
            } else if (quantum_expired) {
                // Quantum expired: RUNNING -> READY and requeue at tail
                running.state = READY;
                execution_status += print_exec_status(current_time,
                                                      running.PID,
                                                      RUNNING,
                                                      READY);

                sync_queue(job_list, running);
                ready_queue.push_back(running);

                idle_CPU(running);
            } else {
                // Still RUNNING for next tick; just sync back to job_list
                sync_queue(job_list, running);
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