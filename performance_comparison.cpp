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
                          long long& nodes) {
    int n = distances.size();
    nodes++;

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
        return;
    }

    for (int i = 0; i < n; ++i) {
        if (!visited[i]) {
            current_path.push_back(i);
            visited[i] = true;
            double new_cost = current_cost + distances[current_city][i];

            if (new_cost < best_cost) {
                tsp_bb_recursive_seq(distances, current_path, visited,
                                    new_cost, best_cost, best_path, nodes);
            }

            current_path.pop_back();
            visited[i] = false;
        }
    }
}

struct TspResult {
    std::vector<int> path;
    double cost;
    long long nodes;
};

TspResult tsp_branch_bound_sequential(
        const std::vector<std::vector<double>>& distances) {
    int n = distances.size();

    std::vector<int> best_path;
    double best_cost = std::numeric_limits<double>::infinity();
    long long nodes = 0;

    std::vector<int> current_path = {0};
    std::vector<bool> visited(n, false);
    visited[0] = true;

    tsp_bb_recursive_seq(distances, current_path, visited, 0.0,
                        best_cost, best_path, nodes);

    return {best_path, best_cost, nodes};
}

// ==================== VERSIÓN PARALELA OPTIMIZADA ===========================

// Worker secuencial puro para los subárboles (sin overhead de tareas)
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
        
        // Lectura relajada para velocidad.
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
                     int depth,
                     long long& total_nodes) {
    
    const int TASK_CUTOFF = 3; // Profundidad hasta la cual generar tareas

    // Si alcanzamos el límite de profundidad, cambiamos a ejecución secuencial
    if (depth > TASK_CUTOFF) {
        long long local_nodes = 0;
        tsp_seq_worker(distances, current_path, visited, current_cost, 
                      best_cost, best_path, local_nodes);
        #pragma omp atomic
        total_nodes += local_nodes;
        return;
    }

    int n = distances.size();
    
    #pragma omp atomic
    total_nodes++;

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
                #pragma omp task shared(distances, best_cost, best_path, total_nodes) firstprivate(next_path, next_visited, new_cost, depth)
                tsp_par_manager(distances, next_path, next_visited, new_cost, 
                              best_cost, best_path, depth + 1, total_nodes);
            }
        }
    }
}

TspResult tsp_branch_bound_parallel(
        const std::vector<std::vector<double>>& distances,
        int num_threads) {

    int n = distances.size();
    
    double best_cost = std::numeric_limits<double>::infinity();
    std::vector<int> best_path;
    long long total_nodes = 0;

    omp_set_num_threads(num_threads);

    std::vector<int> initial_path = {0};
    std::vector<bool> visited(n, false);
    visited[0] = true;

    #pragma omp parallel
    {
        #pragma omp single
        {
            tsp_par_manager(distances, initial_path, visited, 0.0, 
                          best_cost, best_path, 1, total_nodes);
        }
    }

    return {best_path, best_cost, total_nodes};
}

// ==================== VERSIÓN PARALELA (PARALLEL FOR) =======================

// Versión alternativa usando parallel for en el primer nivel
TspResult tsp_branch_bound_parallel_for(
        const std::vector<std::vector<double>>& distances,
        int num_threads) {

    int n = distances.size();
    
    double best_cost = std::numeric_limits<double>::infinity();
    std::vector<int> best_path;
    long long total_nodes = 0;

    omp_set_num_threads(num_threads);

    #pragma omp parallel for schedule(dynamic) reduction(+:total_nodes)
    for (int next_city = 1; next_city < n; ++next_city) {
        std::vector<int> current_path;
        std::vector<bool> visited(n, false);

        current_path.push_back(0);
        current_path.push_back(next_city);
        visited[0] = true;
        visited[next_city] = true;

        double current_cost = distances[0][next_city];

        long long local_nodes = 0;
        // Llamamos directamente al worker secuencial
        tsp_seq_worker(distances, current_path, visited, current_cost, 
                       best_cost, best_path, local_nodes);
        total_nodes += local_nodes;
    }

    return {best_path, best_cost, total_nodes};
}

// ================================= MAIN =====================================
#include <fstream>

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

    std::cout << "╔════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║   TEST DE RENDIMIENTO: TSP Branch & Bound (Comparativa)       ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════════╝\n\n";

    std::ofstream csv_file("performance_comparison.csv");
    csv_file << "Threads,Version,Time_ms,Speedup,Efficiency\n";

    int N = 12;
    std::vector<int> thread_counts = {1, 2, 4, 8, 16, 32};
    int seed = 42;
    std::mt19937 gen(seed);

    std::cout << "--- Testing Fixed N=" << N << " ---\n";
    
    // Preparar instancia (una sola vez)
    std::vector<Coord> cities;
    std::vector<int> indices(all_cities.size());
    for (int i = 0; i < (int)all_cities.size(); ++i) indices[i] = i;
    std::shuffle(indices.begin(), indices.end(), gen);

    for (int i = 0; i < N; ++i) {
        cities.push_back(all_cities[indices[i]]);
    }

    // Calcular matriz de distancias
    std::vector<std::vector<double>> distances(N, std::vector<double>(N, 0.0));
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j)
            distances[i][j] = calculate_distances_btw_two_city(cities[i], cities[j]);

    // 1. Secuencial (Base para Speedup)
    std::cout << "Running Sequential...\n";
    double start_seq = omp_get_wtime();
    auto result_seq = tsp_branch_bound_sequential(distances);
    double end_seq = omp_get_wtime();
    double time_seq = (end_seq - start_seq) * 1000.0;
    
    std::cout << "  Sequential: " << time_seq << " ms, " << result_seq.nodes << " nodes\n";

    // Iterar sobre cantidad de hilos
    for (int num_threads : thread_counts) {
        std::cout << "\n--- Threads: " << num_threads << " ---\n";

        // 2. Paralelo (Tasks)
        double start_par = omp_get_wtime();
        auto result_par = tsp_branch_bound_parallel(distances, num_threads);
        double end_par = omp_get_wtime();
        double time_par = (end_par - start_par) * 1000.0;
        double speedup_par = time_seq / time_par;
        double eff_par = speedup_par / num_threads;

        csv_file << num_threads << ",Parallel_Tasks," << time_par << "," << speedup_par << "," << eff_par << "\n";
        std::cout << "  Parallel Tasks: " << time_par << " ms, Speedup=" << speedup_par << "x\n";

        // 3. Paralelo (For)
        double start_for = omp_get_wtime();
        auto result_for = tsp_branch_bound_parallel_for(distances, num_threads);
        double end_for = omp_get_wtime();
        double time_for = (end_for - start_for) * 1000.0;
        double speedup_for = time_seq / time_for;
        double eff_for = speedup_for / num_threads;

        csv_file << num_threads << ",Parallel_For," << time_for << "," << speedup_for << "," << eff_for << "\n";
        std::cout << "  Parallel For:   " << time_for << " ms, Speedup=" << speedup_for << "x\n";
    }

    csv_file.close();
    std::cout << "\nResultados guardados en performance_comparison.csv\n";

    return 0;
}
