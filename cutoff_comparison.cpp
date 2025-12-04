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
#include <fstream>

using namespace std;

// ========================= ESTRUCTURAS Y FUNCIONES BASE ======================

struct Coord {
    double lat;
    double lon;
};

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
                          long long& nodes) {
    int n = distances.size();
    nodes++;

    if ((int)current_path.size() == n) {
        double total_cost = current_cost + distances[current_path.back()][0];
        if (total_cost < best_cost) {
            best_cost = total_cost;
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
            current_path.push_back(i);
            visited[i] = true;
            double new_cost = current_cost + distances[current_city][i];

            if (new_cost < best_cost) {
                tsp_bb_recursive_seq(distances, current_path, visited,
                                    new_cost, best_cost, nodes);
            }

            current_path.pop_back();
            visited[i] = false;
        }
    }
}

std::pair<double, long long> tsp_branch_bound_sequential(
        const std::vector<std::vector<double>>& distances) {
    int n = distances.size();

    double best_cost = std::numeric_limits<double>::infinity();
    long long nodes = 0;

    std::vector<int> current_path = {0};
    std::vector<bool> visited(n, false);
    visited[0] = true;

    tsp_bb_recursive_seq(distances, current_path, visited, 0.0,
                        best_cost, nodes);

    return {best_cost, nodes};
}

// ==================== VERSIÓN PARALELA ======================================

// Worker secuencial puro para los subárboles
void tsp_seq_worker(const std::vector<std::vector<double>>& distances,
                    std::vector<int>& current_path,
                    std::vector<bool>& visited,
                    double current_cost,
                    double& best_cost,
                    long long& local_nodes) {
    int n = distances.size();
    local_nodes++;

    if ((int)current_path.size() == n) {
        double total_cost = current_cost + distances[current_path.back()][0];
        
        if (total_cost < best_cost) {
            #pragma omp critical(update_best)
            {
                if (total_cost < best_cost) {
                    best_cost = total_cost;
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
                tsp_seq_worker(distances, current_path, visited, new_cost, best_cost, local_nodes);
            }

            visited[i] = false;
            current_path.pop_back();
        }
    }
}

// Gestor de tareas
void tsp_par_manager(const std::vector<std::vector<double>>& distances,
                     std::vector<int> current_path, 
                     std::vector<bool> visited,     
                     double current_cost,
                     double& best_cost,
                     int depth,
                     long long& total_nodes_explored,
                     int task_cutoff) {
    
    if (depth > task_cutoff) {
        long long local_nodes = 0;
        tsp_seq_worker(distances, current_path, visited, current_cost, best_cost, local_nodes);
        
        #pragma omp atomic
        total_nodes_explored += local_nodes;
        return;
    }

    int n = distances.size();
    
    #pragma omp atomic
    total_nodes_explored++;

    if ((int)current_path.size() == n) {
        double total_cost = current_cost + distances[current_path.back()][0];
        #pragma omp critical(update_best)
        {
            if (total_cost < best_cost) {
                best_cost = total_cost;
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
                #pragma omp task shared(distances, best_cost, total_nodes_explored) firstprivate(next_path, next_visited, new_cost, depth, task_cutoff)
                tsp_par_manager(distances, next_path, next_visited, new_cost, best_cost, depth + 1, total_nodes_explored, task_cutoff);
            }
        }
    }
}

std::pair<double, long long> tsp_branch_bound_parallel(
        const std::vector<std::vector<double>>& distances,
        int num_threads,
        int task_cutoff) {

    int n = distances.size();
    double best_cost = std::numeric_limits<double>::infinity();
    long long total_nodes = 0;

    omp_set_num_threads(num_threads);

    std::vector<int> initial_path = {0};
    std::vector<bool> visited(n, false);
    visited[0] = true;

    #pragma omp parallel
    {
        #pragma omp single
        {
            tsp_par_manager(distances, initial_path, visited, 0.0, best_cost, 1, total_nodes, task_cutoff);
        }
    }

    return {best_cost, total_nodes};
}

// ================================= MAIN =====================================

int main() {
    // Base de datos de 20 ciudades europeas
    std::vector<Coord> all_cities = {
        {52.5200, 13.4050},  // Berlin
        {53.5511, 9.9937},   // Hamburg
        {48.1351, 11.5820},  // Munich
        {50.9375, 6.9603},   // Cologne
        {50.1109, 8.6821},   // Frankfurt
        {48.8566, 2.3522},   // Paris
        {51.5074, -0.1278},  // London
        {41.9028, 12.4964},  // Rome
        {40.4168, -3.7038},  // Madrid
        {52.3676, 4.9041},   // Amsterdam
        {55.6761, 12.5683},  // Copenhagen
        {59.9139, 10.7522},  // Oslo
        {59.3293, 18.0686},  // Stockholm
        {60.1699, 24.9384},  // Helsinki
        {50.0755, 14.4378},  // Prague
        {47.4979, 19.0402},  // Budapest
        {44.4268, 26.1025},  // Bucharest
        {52.2297, 21.0122},  // Warsaw
        {38.7223, -9.1393},  // Lisbon
        {46.9480, 7.4474}    // Bern
    };

    // Selección de ciudades (usamos todas para tener el pool completo)
    std::vector<Coord> cities = all_cities;
    
    // Semilla fija para reproducibilidad
    int seed = 42;
    std::mt19937 gen(seed);
    
    // Configuración del experimento
    int N = 12;
    std::vector<int> thread_counts = {1, 2, 4, 8, 16, 32};
    std::vector<int> cutoffs = {1, 2, 3, 4, 5, N};
    
    std::cout << "=== CUTOFF & THREADS COMPARISON (N=" << N << ") ===\n";
    std::cout << "Threads\tCutoff\tTime(ms)\tNodes\tSpeedup\tEfficiency\n";

    std::ofstream csv_file("cutoff_results.csv");
    csv_file << "Threads,Cutoff,Time_ms,Nodes,Speedup,Efficiency\n";

    // Preparar instancia (una sola instancia)
    std::shuffle(cities.begin(), cities.end(), gen);
    std::vector<Coord> cities_s(cities.begin(), cities.begin() + N);
    std::vector<std::vector<double>> dist_s(N, std::vector<double>(N));

    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            dist_s[i][j] = calculate_distances_btw_two_city(cities_s[i], cities_s[j]);

    // 1. Medir Secuencial (Baseline)
    double start_seq = omp_get_wtime();
    auto res_seq = tsp_branch_bound_sequential(dist_s);
    double end_seq = omp_get_wtime();
    double time_seq = (end_seq - start_seq) * 1000.0;
    
    std::cout << "Sequential Baseline: " << time_seq << " ms\n";

    // 2. Ejecutar experimentos
    for (int p : thread_counts) {
        for (int cutoff : cutoffs) {
            double start = omp_get_wtime();
            auto res = tsp_branch_bound_parallel(dist_s, p, cutoff);
            double end = omp_get_wtime();
            double time_ms = (end - start) * 1000.0;
            
            double speedup = time_seq / time_ms;
            double efficiency = speedup / p;
            
            std::string cutoff_label = (cutoff == N) ? "No Limit" : std::to_string(cutoff);
            
            std::cout << p << "\t" << cutoff_label << "\t" << time_ms << "\t" << res.second 
                      << "\t" << speedup << "\t" << efficiency << "\n";
            
            csv_file << p << "," << cutoff << "," << time_ms << "," << res.second 
                     << "," << speedup << "," << efficiency << "\n";
        }
    }
    csv_file.close();
    std::cout << "Resultados guardados en cutoff_results.csv\n";

return 0;
}
