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

// ====================== ESTADÍSTICAS BRANCH & BOUND ==========================

struct BranchBoundStats {
    long long nodes_explored = 0;
    long long nodes_pruned = 0;
    int max_depth = 0;
    int branching_factor = 0;
    double alpha = 0.0;
};

BranchBoundStats bb_stats_seq;
BranchBoundStats bb_stats_par;

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
        bound += min_edge_from_current;
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
            bound += (first_min + second_min) / 2.0;
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
            bound += min_return;
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

    int max_theoretical_nodes = 1;
    for (int i = 1; i < n; ++i) {
        max_theoretical_nodes *= (n - i);
    }

    tsp_bb_recursive_seq(distances, current_path, visited, 0.0,
                        best_cost, best_path, 1);

    bb_stats_seq.alpha = (double)bb_stats_seq.nodes_explored / max_theoretical_nodes;

    return {best_path, best_cost};
}

// ==================== VERSIÓN PARALELA ======================================

// Función auxiliar secuencial (backtracking) para las hojas del árbol de tareas
void tsp_bb_par_serial_worker(const std::vector<std::vector<double>>& distances,
                              std::vector<int>& current_path,
                              std::vector<bool>& visited,
                              double current_cost,
                              double& global_best_cost,
                              std::vector<int>& global_best_path,
                              int depth) {
    int n = distances.size();

    #pragma omp atomic
    bb_stats_par.nodes_explored++;

    #pragma omp critical(stats)
    {
        bb_stats_par.max_depth = std::max(bb_stats_par.max_depth, depth);
    }

    if ((int)current_path.size() == n) {
        double total_cost = current_cost + distances[current_path.back()][0];
        
        // Lectura optimista antes de sección crítica
        if (total_cost < global_best_cost) {
            #pragma omp critical(best_update)
            {
                if (total_cost < global_best_cost) {
                    global_best_cost = total_cost;
                    global_best_path = current_path;
                }
            }
        }
        return;
    }

    int current_city = current_path.back();
    double bound = calculate_lower_bound(distances, visited, current_city, current_cost);

    if (bound >= global_best_cost) {
        #pragma omp atomic
        bb_stats_par.nodes_pruned++;
        return;
    }

    int branches = 0;
    for (int i = 0; i < n; ++i) {
        if (!visited[i]) {
            branches++;
            current_path.push_back(i);
            visited[i] = true;
            double new_cost = current_cost + distances[current_city][i];

            if (new_cost < global_best_cost) {
                tsp_bb_par_serial_worker(distances, current_path, visited,
                                       new_cost, global_best_cost, global_best_path, depth + 1);
            } else {
                #pragma omp atomic
                bb_stats_par.nodes_pruned++;
            }

            current_path.pop_back();
            visited[i] = false;
        }
    }
    
    #pragma omp critical(stats)
    {
        bb_stats_par.branching_factor = std::max(bb_stats_par.branching_factor, branches);
    }
}

// Función recursiva con Tareas (OpenMP Tasks)
void tsp_bb_recursive_par(const std::vector<std::vector<double>>& distances,
                         std::vector<int> current_path,    // Por valor (copia para la tarea)
                         std::vector<bool> visited,        // Por valor (copia para la tarea)
                         double current_cost,
                         double& global_best_cost,
                         std::vector<int>& global_best_path,
                         int depth) {
    
    // Umbral para cambiar a secuencial (evitar overhead excesivo de tareas)
    const int TASK_DEPTH_CUTOFF = 4; 

    if (depth > TASK_DEPTH_CUTOFF) {
        tsp_bb_par_serial_worker(distances, current_path, visited, current_cost, 
                               global_best_cost, global_best_path, depth);
        return;
    }

    int n = distances.size();

    #pragma omp atomic
    bb_stats_par.nodes_explored++;

    #pragma omp critical(stats)
    {
        bb_stats_par.max_depth = std::max(bb_stats_par.max_depth, depth);
    }

    int current_city = current_path.back();
    double bound = calculate_lower_bound(distances, visited, current_city, current_cost);

    if (bound >= global_best_cost) {
        #pragma omp atomic
        bb_stats_par.nodes_pruned++;
        return;
    }

    int branches = 0;
    for (int i = 0; i < n; ++i) {
        if (!visited[i]) {
            branches++;
            // Preparar datos para la tarea (copias)
            std::vector<int> next_path = current_path;
            next_path.push_back(i);
            std::vector<bool> next_visited = visited;
            next_visited[i] = true;
            double next_cost = current_cost + distances[current_city][i];

            if (next_cost < global_best_cost) {
                #pragma omp task shared(distances, global_best_cost, global_best_path) firstprivate(next_path, next_visited, next_cost, depth)
                tsp_bb_recursive_par(distances, next_path, next_visited, 
                                   next_cost, global_best_cost, global_best_path, depth + 1);
            } else {
                #pragma omp atomic
                bb_stats_par.nodes_pruned++;
            }
        }
    }

    #pragma omp critical(stats)
    {
        bb_stats_par.branching_factor = std::max(bb_stats_par.branching_factor, branches);
    }
}

std::pair<std::vector<int>, double> tsp_branch_bound_parallel(
        const std::vector<std::vector<double>>& distances,
        int num_threads) {
    int n = distances.size();
    bb_stats_par = BranchBoundStats{};

    double global_best_cost = std::numeric_limits<double>::infinity();
    std::vector<int> global_best_path;

    int max_theoretical_nodes = 1;
    for (int i = 1; i < n; ++i) {
        max_theoretical_nodes *= (n - i);
    }

    std::vector<int> initial_path = {0};
    std::vector<bool> initial_visited(n, false);
    initial_visited[0] = true;

    #pragma omp parallel num_threads(num_threads)
    {
        #pragma omp single
        {
            tsp_bb_recursive_par(distances, initial_path, initial_visited, 
                               0.0, global_best_cost, global_best_path, 1);
        }
    }

    bb_stats_par.alpha = (double)bb_stats_par.nodes_explored / max_theoretical_nodes;

    return {global_best_path, global_best_cost};
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

    std::vector<std::string> all_city_names = {
        "Berlin", "Hamburg", "Munich", "Cologne", "Frankfurt",
        "Paris", "London", "Rome", "Madrid", "Amsterdam",
        "Copenhagen", "Oslo", "Stockholm", "Helsinki", "Prague",
        "Budapest", "Bucharest", "Warsaw", "Lisbon", "Bern"
    };

    std::cout << "╔════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║   COMPARACIÓN: TSP Branch & Bound Secuencial vs Paralelo      ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════════╝\n\n";

    // Configuración del experimento
    int sample_size;
    int num_threads;
    int seed;

    std::cout << "Número de ciudades a usar (max 20): ";
    std::cin >> sample_size;
    if (sample_size > 20) sample_size = 20;
    if (sample_size < 3) sample_size = 3;

    std::cout << "Número de hilos para versión paralela: ";
    std::cin >> num_threads;

    std::cout << "Semilla aleatoria (0 = aleatoria): ";
    std::cin >> seed;

    // Selección de ciudades
    std::vector<Coord> cities;
    std::vector<std::string> city_names;
    std::vector<int> indices(all_cities.size());

    for (int i = 0; i < (int)all_cities.size(); ++i) indices[i] = i;

    if (seed == 0) {
        std::random_device rd;
        seed = rd();
    }
    std::mt19937 gen(seed);
    std::shuffle(indices.begin(), indices.end(), gen);

    for (int i = 0; i < sample_size; ++i) {
        cities.push_back(all_cities[indices[i]]);
        city_names.push_back(all_city_names[indices[i]]);
    }

    std::cout << "\n┌────────────────────────────────────────────────────────────────┐\n";
    std::cout << "│ CONFIGURACIÓN DEL EXPERIMENTO                                  │\n";
    std::cout << "└────────────────────────────────────────────────────────────────┘\n";
    std::cout << "Número de ciudades: " << sample_size << std::endl;
    std::cout << "Número de hilos (paralelo): " << num_threads << std::endl;
    std::cout << "Semilla: " << seed << std::endl;

    std::cout << "\nCiudades seleccionadas:\n";
    for (int i = 0; i < sample_size; ++i) {
        std::cout << "  " << i << ": " << city_names[i] << std::endl;
    }

    // Calcular matriz de distancias
    size_t c = cities.size();
    std::vector<std::vector<double>> distances(c, std::vector<double>(c, 0.0));

    for (size_t i = 0; i < c; ++i)
        for (size_t j = 0; j < c; ++j)
            distances[i][j] = calculate_distances_btw_two_city(cities[i], cities[j]);

    // ==================== VERSIÓN SECUENCIAL ====================
    std::cout << "\n┌────────────────────────────────────────────────────────────────┐\n";
    std::cout << "│ EJECUTANDO: Branch & Bound SECUENCIAL                         │\n";
    std::cout << "└────────────────────────────────────────────────────────────────┘\n";

    auto start_seq = std::chrono::high_resolution_clock::now();
    auto result_seq = tsp_branch_bound_sequential(distances);
    auto end_seq = std::chrono::high_resolution_clock::now();
    auto duration_seq = std::chrono::duration_cast<std::chrono::milliseconds>(end_seq - start_seq);

    std::cout << "Mejor ruta: ";
    for (size_t i = 0; i < result_seq.first.size(); ++i) {
        std::cout << result_seq.first[i];
        if (i < result_seq.first.size() - 1) std::cout << " -> ";
    }
    std::cout << " -> 0" << std::endl;

    std::cout << "Costo total: " << std::fixed << std::setprecision(2)
              << result_seq.second << " km" << std::endl;
    std::cout << "Tiempo de ejecución: " << duration_seq.count() << " ms" << std::endl;

    std::cout << "\nEstadísticas secuencial:\n";
    std::cout << "  Nodos explorados: " << bb_stats_seq.nodes_explored << std::endl;
    std::cout << "  Nodos podados: " << bb_stats_seq.nodes_pruned << std::endl;
    std::cout << "  Profundidad máxima: " << bb_stats_seq.max_depth << std::endl;
    std::cout << "  Factor ramificación: " << bb_stats_seq.branching_factor << std::endl;
    std::cout << "  Alpha (α): " << std::scientific << std::setprecision(6) 
              << bb_stats_seq.alpha << std::endl;

    // ==================== VERSIÓN PARALELA ====================
    std::cout << "\n┌────────────────────────────────────────────────────────────────┐\n";
    std::cout << "│ EJECUTANDO: Branch & Bound PARALELO (" << num_threads << " hilos)";
    for (int i = 0; i < 21 - std::to_string(num_threads).length(); i++) std::cout << " ";
    std::cout << "│\n";
    std::cout << "└────────────────────────────────────────────────────────────────┘\n";

    auto start_par = std::chrono::high_resolution_clock::now();
    auto result_par = tsp_branch_bound_parallel(distances, num_threads);
    auto end_par = std::chrono::high_resolution_clock::now();
    auto duration_par = std::chrono::duration_cast<std::chrono::milliseconds>(end_par - start_par);

    std::cout << "Mejor ruta: ";
    for (size_t i = 0; i < result_par.first.size(); ++i) {
        std::cout << result_par.first[i];
        if (i < result_par.first.size() - 1) std::cout << " -> ";
    }
    std::cout << " -> 0" << std::endl;

    std::cout << "Costo total: " << std::fixed << std::setprecision(2)
              << result_par.second << " km" << std::endl;
    std::cout << "Tiempo de ejecución: " << duration_par.count() << " ms" << std::endl;

    std::cout << "\nEstadísticas paralelo:\n";
    std::cout << "  Nodos explorados: " << bb_stats_par.nodes_explored << std::endl;
    std::cout << "  Nodos podados: " << bb_stats_par.nodes_pruned << std::endl;
    std::cout << "  Profundidad máxima: " << bb_stats_par.max_depth << std::endl;
    std::cout << "  Factor ramificación: " << bb_stats_par.branching_factor << std::endl;
    std::cout << "  Alpha (α): " << std::scientific << std::setprecision(6) 
              << bb_stats_par.alpha << std::endl;

    // ==================== COMPARACIÓN Y ANÁLISIS ====================
    std::cout << "\n╔════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║                    ANÁLISIS COMPARATIVO                        ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════════╝\n";

    // Validación de resultados
    double cost_diff = std::abs(result_seq.second - result_par.second);
    bool same_solution = cost_diff < 1e-6;

    std::cout << "\n▶ VALIDACIÓN DE RESULTADOS:\n";
    std::cout << "  Costo secuencial: " << std::fixed << std::setprecision(6) 
              << result_seq.second << " km" << std::endl;
    std::cout << "  Costo paralelo:   " << std::fixed << std::setprecision(6) 
              << result_par.second << " km" << std::endl;
    std::cout << "  Diferencia:       " << std::fixed << std::setprecision(6) 
              << cost_diff << " km" << std::endl;

    if (same_solution) {
        std::cout << "  ✓ CORRECTO: Ambas versiones encontraron la misma solución\n";
    } else {
        double error_pct = (cost_diff / result_seq.second) * 100.0;
        std::cout << "  ⚠ Las soluciones difieren en " << std::fixed << std::setprecision(4) 
                  << error_pct << "%\n";
    }

    // Análisis de rendimiento
    std::cout << "\n▶ ANÁLISIS DE RENDIMIENTO:\n";
    std::cout << "  Tiempo secuencial: " << std::setw(10) << duration_seq.count() << " ms\n";
    std::cout << "  Tiempo paralelo:   " << std::setw(10) << duration_par.count() << " ms\n";

    double speedup = (double)duration_seq.count() / duration_par.count();
    double efficiency = speedup / num_threads;

    std::cout << "\n  Speed-up (Sp):     " << std::fixed << std::setprecision(4) 
              << speedup << "x\n";
    std::cout << "  Eficiencia (Ep):   " << std::fixed << std::setprecision(4) 
              << efficiency * 100 << "%\n";
    std::cout << "  Ganancia temporal: " << duration_seq.count() - duration_par.count() 
              << " ms\n";

    if (speedup > num_threads * 0.7) {
        std::cout << "  ✓ EXCELENTE paralelización (eficiencia > 70%)\n";
    } else if (speedup > num_threads * 0.5) {
        std::cout << "  ✓ BUENA paralelización (eficiencia > 50%)\n";
    } else if (speedup > 1.0) {
        std::cout << "  ⚠ MODERADA paralelización\n";
    } else {
        std::cout << "  ✗ Paralelización ineficiente\n";
    }

    // Análisis de exploración
    std::cout << "\n▶ ANÁLISIS DE EXPLORACIÓN:\n";
    std::cout << "  Nodos explorados (seq): " << bb_stats_seq.nodes_explored << std::endl;
    std::cout << "  Nodos explorados (par): " << bb_stats_par.nodes_explored << std::endl;

    long long node_diff = bb_stats_par.nodes_explored - bb_stats_seq.nodes_explored;
    double overhead_pct = (double)node_diff / bb_stats_seq.nodes_explored * 100.0;

    std::cout << "  Diferencia:             " << node_diff;
    if (node_diff > 0) {
        std::cout << " (+" << std::fixed << std::setprecision(2) << overhead_pct << "% overhead)\n";
    } else {
        std::cout << " (" << std::fixed << std::setprecision(2) << overhead_pct << "%)\n";
    }

    std::cout << "\n▶ RUTA CON NOMBRES DE CIUDADES:\n";
    std::cout << "  ";
    for (size_t i = 0; i < result_seq.first.size(); ++i) {
        std::cout << city_names[result_seq.first[i]];
        if (i < result_seq.first.size() - 1) std::cout << " → ";
    }
    std::cout << " → " << city_names[0] << std::endl;

    std::cout << "\n" << std::string(64, '=') << std::endl;

    return 0;
}
