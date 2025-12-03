#include <iostream>
#include <vector>
#include <limits>
#include <utility>
#include <cmath>
#include <iomanip>
#include <chrono>
#include <string>
#include <algorithm>
#include <random>
#include <omp.h>

using namespace std;

// ========================= ESTRUCTURAS Y FUNCIONES BASE ======================

struct Coord {
    double lat;
    double lon;
};

struct BranchBoundStats {
    long long nodes_explored = 0;
    long long nodes_pruned = 0;
    int max_depth = 0;
    int branching_factor = 0;
    double alpha = 0.0;
};

BranchBoundStats bb_stats_seq;
BranchBoundStats bb_stats_par;

double calculate_distances_btw_two_city(const Coord& coord1, const Coord& coord2) {
    const double R = 6371.0;
    const double PI = std::acos(-1.0);
    double dlat = (coord2.lat - coord1.lat) * PI / 180.0;
    double dlon = (coord2.lon - coord1.lon) * PI / 180.0;
    double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
               std::cos(coord1.lat * PI / 180.0) * std::cos(coord2.lat * PI / 180.0) *
               std::sin(dlon / 2) * std::sin(dlon / 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    return R * c;
}

// ========================== COTA INFERIOR (BOUND) ============================

double calculate_lower_bound(const std::vector<std::vector<double>>& distances,
                             const std::vector<bool>& visited,
                             int current_city, double current_cost) {
    int n = distances.size();
    double bound = current_cost;

    double min_edge_from_current = std::numeric_limits<double>::infinity();
    for (int i = 0; i < n; ++i) {
        if (!visited[i] && i != current_city) {
            min_edge_from_current = std::min(min_edge_from_current, distances[current_city][i]);
        }
    }
    if (min_edge_from_current != std::numeric_limits<double>::infinity()) {
        bound += min_edge_from_current * 0.5;
    }

    for (int i = 0; i < n; ++i) {
        if (!visited[i] && i != current_city) {
            double first_min = std::numeric_limits<double>::infinity();
            double second_min = std::numeric_limits<double>::infinity();

            for (int j = 0; j < n; ++j) {
                if (i != j) {
                    if (distances[i][j] < first_min) {
                        second_min = first_min;
                        first_min = distances[i][j];
                    } else if (distances[i][j] < second_min) {
                        second_min = distances[i][j];
                    }
                }
            }
            bound += (first_min + second_min) * 0.5;
        }
    }

    if (current_city != 0) {
        double min_return = std::numeric_limits<double>::infinity();
        for (int i = 0; i < n; ++i) {
            if (!visited[i] && i != current_city) {
                min_return = std::min(min_return, distances[i][0]);
            }
        }
        if (min_return != std::numeric_limits<double>::infinity()) {
            bound += min_return * 0.5;
        }
    }

    return bound;
}

// ==================== VERSIÓN SECUENCIAL ====================================

void tsp_bb_recursive_seq(const std::vector<std::vector<double>>& distances,
                          std::vector<int>& current_path,
                          std::vector<bool>& visited,
                          double current_cost,
                          double& best_cost,
                          std::vector<int>& best_path,
                          int depth) {
    int n = distances.size();
    bb_stats_seq.nodes_explored++;
    bb_stats_seq.max_depth = std::max(bb_stats_seq.max_depth, depth);

    if ((int)current_path.size() == n) {
        double total_cost = current_cost + distances[current_path.back()][0];
        if (total_cost < best_cost) {
            best_cost = total_cost;
            best_path = current_path;
        }
        return;
    }

    int current_city = current_path.back();
    double bound = calculate_lower_bound(distances, visited, current_city, current_cost);

    if (bound >= best_cost) {
        bb_stats_seq.nodes_pruned++;
        return;
    }

    int branches = 0;
    for (int i = 0; i < n; ++i) {
        if (!visited[i]) {
            branches++;
            current_path.push_back(i);
            visited[i] = true;
            double new_cost = current_cost + distances[current_city][i];

            if (new_cost < best_cost) {
                tsp_bb_recursive_seq(distances, current_path, visited,
                                    new_cost, best_cost, best_path, depth + 1);
            } else {
                bb_stats_seq.nodes_pruned++;
            }

            current_path.pop_back();
            visited[i] = false;
        }
    }
    bb_stats_seq.branching_factor = std::max(bb_stats_seq.branching_factor, branches);
}

std::pair<std::vector<int>, double> tsp_branch_bound_sequential(
        const std::vector<std::vector<double>>& distances) {
    int n = distances.size();
    bb_stats_seq = BranchBoundStats{};

    std::vector<int> best_path;
    double best_cost = std::numeric_limits<double>::infinity();

    std::vector<int> current_path = {0};
    std::vector<bool> visited(n, false);
    visited[0] = true;

    tsp_bb_recursive_seq(distances, current_path, visited, 0.0,
                        best_cost, best_path, 1);

    return {best_path, best_cost};
}

// ==================== VERSIÓN PARALELA OPTIMIZADA ===========================

// Worker secuencial puro para los subárboles (sin overhead de tareas ni atomics por nodo)
void tsp_seq_worker(const std::vector<std::vector<double>>& distances,
                    std::vector<int>& current_path,
                    std::vector<bool>& visited,
                    double current_cost,
                    double& best_cost,
                    std::vector<int>& best_path,
                    long long& local_nodes) {
    int n = distances.size();
    local_nodes++;

    if ((int)current_path.size() == n) {
        double total_cost = current_cost + distances[current_path.back()][0];
        
        // Lectura relajada (sin atomic) para velocidad. 
        // En x86_64 los doubles alineados suelen ser atómicos, y si leemos basura
        // solo fallará la poda (seguro) o entraremos al critical innecesariamente.
        if (total_cost < best_cost) {
            #pragma omp critical(update_best)
            {
                if (total_cost < best_cost) {
                    best_cost = total_cost;
                    best_path = current_path;
                }
            }
        }
        return;
    }

    int current_city = current_path.back();
    double bound = calculate_lower_bound(distances, visited, current_city, current_cost);

    // Lectura relajada para poda rápida
    if (bound >= best_cost) {
        return;
    }

    for (int i = 0; i < n; ++i) {
        if (!visited[i]) {
            current_path.push_back(i);
            visited[i] = true;
            double new_cost = current_cost + distances[current_city][i];

            if (new_cost < best_cost) {
                tsp_seq_worker(distances, current_path, visited, new_cost, 
                             best_cost, best_path, local_nodes);
            }

            visited[i] = false;
            current_path.pop_back();
        }
    }
}

// Gestor de tareas: Genera tareas hasta cierta profundidad y luego llama al worker secuencial
void tsp_par_manager(const std::vector<std::vector<double>>& distances,
                     std::vector<int> current_path, // Copia por valor para la tarea
                     std::vector<bool> visited,     // Copia por valor para la tarea
                     double current_cost,
                     double& best_cost,
                     std::vector<int>& best_path,
                     int depth) {
    
    const int TASK_CUTOFF = 3; // Profundidad hasta la cual generar tareas

    // Si alcanzamos el límite de profundidad, cambiamos a ejecución secuencial
    if (depth > TASK_CUTOFF) {
        long long local_nodes = 0;
        
        tsp_seq_worker(distances, current_path, visited, current_cost, 
                      best_cost, best_path, local_nodes);
        
        // Actualización atómica de estadísticas al terminar el subárbol (poco frecuente)
        #pragma omp atomic
        bb_stats_par.nodes_explored += local_nodes;
        return;
    }

    int n = distances.size();
    
    #pragma omp atomic
    bb_stats_par.nodes_explored++;

    if ((int)current_path.size() == n) {
        double total_cost = current_cost + distances[current_path.back()][0];
        #pragma omp critical(update_best)
        {
            if (total_cost < best_cost) {
                best_cost = total_cost;
                best_path = current_path;
            }
        }
        return;
    }

    int current_city = current_path.back();
    double bound = calculate_lower_bound(distances, visited, current_city, current_cost);

    if (bound >= best_cost) {
        return;
    }

    for (int i = 0; i < n; ++i) {
        if (!visited[i]) {
            std::vector<int> next_path = current_path;
            next_path.push_back(i);
            std::vector<bool> next_visited = visited;
            next_visited[i] = true;
            double new_cost = current_cost + distances[current_city][i];

            if (new_cost < best_cost) {
                #pragma omp task shared(distances, best_cost, best_path) firstprivate(next_path, next_visited, new_cost, depth)
                tsp_par_manager(distances, next_path, next_visited, new_cost, 
                              best_cost, best_path, depth + 1);
            }
        }
    }
}

std::pair<std::vector<int>, double> tsp_branch_bound_parallel(
        const std::vector<std::vector<double>>& distances,
        int num_threads) {

    int n = distances.size();
    bb_stats_par = BranchBoundStats{}; // Reset stats
    
    double best_cost = std::numeric_limits<double>::infinity();
    std::vector<int> best_path;

    omp_set_num_threads(num_threads);

    std::vector<int> initial_path = {0};
    std::vector<bool> visited(n, false);
    visited[0] = true;

    #pragma omp parallel
    {
        #pragma omp single
        {
            tsp_par_manager(distances, initial_path, visited, 0.0, 
                          best_cost, best_path, 1);
        }
    }

    return {best_path, best_cost};
}

// ================================= MAIN =====================================

int main() {
    std::vector<Coord> all_cities = {
        {52.5200, 13.4050}, {53.5511, 9.9937}, {48.1351, 11.5820}, {50.9375, 6.9603},
        {50.1109, 8.6821}, {48.8566, 2.3522}, {51.5074, -0.1278}, {41.9028, 12.4964},
        {40.4168, -3.7038}, {52.3676, 4.9041}, {55.6761, 12.5683}, {59.9139, 10.7522},
        {59.3293, 18.0686}, {60.1699, 24.9384}, {50.0755, 14.4378}, {47.4979, 19.0402},
        {44.4268, 26.1025}, {52.2297, 21.0122}, {38.7223, -9.1393}, {46.9480, 7.4474}
    };

    // Configuración del experimento
    std::vector<int> problem_sizes = {10, 11, 12, 13, 14};
    std::vector<int> thread_counts = {1, 2, 4, 8, 16};
    int repetitions = 3;

    std::cout << "N,Threads,Time_Seq_ms,Time_Par_ms,Speedup,Efficiency,Nodes_Seq,Nodes_Par,Cost_Seq,Cost_Par,FLOPs_Seq,FLOPs_Par,MFLOPs_Seq,MFLOPs_Par\n";

    for (int n : problem_sizes) {
        // Estimación de operaciones de punto flotante y comparaciones por nodo
        // Loop 1 (min_edge): N comparaciones
        // Loop 2 (2 min edges): N * N * 2 comparaciones + 3*N ops aritméticas
        // Loop 3 (return): N comparaciones + 2 ops aritméticas
        // Total aprox: 2*N^2 + 5*N
        long long ops_per_node = 2LL * n * n + 5LL * n;

        // Preparar datos para tamaño N
        std::vector<Coord> cities;
        for (int i = 0; i < n; ++i) cities.push_back(all_cities[i]);

        std::vector<std::vector<double>> distances(n, std::vector<double>(n));
        for (size_t i = 0; i < n; ++i)
            for (size_t j = 0; j < n; ++j)
                distances[i][j] = calculate_distances_btw_two_city(cities[i], cities[j]);

        // Ejecutar Secuencial (Promedio)
        double total_time_seq = 0;
        double cost_seq = 0;
        long long nodes_seq = 0;

        for (int r = 0; r < repetitions; ++r) {
            double start = omp_get_wtime();
            auto res = tsp_branch_bound_sequential(distances);
            double end = omp_get_wtime();
            total_time_seq += (end - start) * 1000.0;
            cost_seq = res.second;
            nodes_seq = bb_stats_seq.nodes_explored;
        }
        double avg_time_seq = total_time_seq / repetitions;
        long long total_flops_seq = nodes_seq * ops_per_node;
        double mflops_seq = (total_flops_seq / 1e6) / (avg_time_seq / 1000.0);

        // Ejecutar Paralelo para cada configuración de hilos
        for (int p : thread_counts) {
            double total_time_par = 0;
            double cost_par = 0;
            long long nodes_par = 0;

            for (int r = 0; r < repetitions; ++r) {
                double start = omp_get_wtime();
                auto res = tsp_branch_bound_parallel(distances, p);
                double end = omp_get_wtime();
                total_time_par += (end - start) * 1000.0;
                cost_par = res.second;
                nodes_par = bb_stats_par.nodes_explored;
            }
            double avg_time_par = total_time_par / repetitions;
            long long total_flops_par = nodes_par * ops_per_node;
            double mflops_par = (total_flops_par / 1e6) / (avg_time_par / 1000.0);

            double speedup = avg_time_seq / avg_time_par;
            double efficiency = speedup / p;

            std::cout << n << "," << p << "," 
                      << std::fixed << std::setprecision(3) << avg_time_seq << "," 
                      << avg_time_par << "," 
                      << std::setprecision(4) << speedup << "," 
                      << efficiency << "," 
                      << nodes_seq << "," << nodes_par << ","
                      << std::setprecision(2) << cost_seq << "," << cost_par << ","
                      << total_flops_seq << "," << total_flops_par << ","
                      << std::setprecision(2) << mflops_seq << "," << mflops_par << std::endl;
        }
    }

    return 0;
}
