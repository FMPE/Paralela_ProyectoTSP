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
    const double R = 6371.0;  // Radio de la Tierra en km
    const double PI = std::acos(-1.0);
    double dlat = (coord2.lat - coord1.lat) * PI / 180.0;
    double dlon = (coord2.lon - coord1.lon) * PI / 180.0;
    double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
               std::cos(coord1.lat * PI / 180.0) * std::cos(coord2.lat * PI / 180.0) *
               std::sin(dlon / 2) * std::sin(dlon / 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    return R * c;
}

// ====================== FUERZA BRUTA (opcional, validación) ==================

std::pair<std::vector<int>, double> tsp_brute_force(
        const std::vector<std::vector<double>>& distances) {

    int n = distances.size();
    std::vector<int> cities;
    for (int i = 1; i < n; ++i) cities.push_back(i);

    std::vector<int> best_path = {0};
    double best_cost = std::numeric_limits<double>::infinity();

    do {
        std::vector<int> current_path = {0};
        current_path.insert(current_path.end(), cities.begin(), cities.end());

        double current_cost = 0.0;
        for (int i = 0; i < n - 1; ++i) {
            current_cost += distances[current_path[i]][current_path[i + 1]];
        }
        current_cost += distances[current_path[n - 1]][0];

        if (current_cost < best_cost) {
            best_cost = current_cost;
            best_path = current_path;
        }

    } while (std::next_permutation(cities.begin(), cities.end()));

    return {best_path, best_cost};
}

// ====================== ESTADÍSTICAS BRANCH & BOUND ==========================

struct BranchBoundStats {
    long long nodes_explored = 0;
    long long nodes_pruned   = 0;
    int       max_depth      = 0;
    int       branching_factor = 0;
    double    alpha          = 0.0; // Factor de efectividad de la poda
};

BranchBoundStats bb_stats;

// ========================== COTA INFERIOR (BOUND) ============================

double calculate_lower_bound(const std::vector<std::vector<double>>& distances,
                             const std::vector<bool>& visited,
                             int current_city, double current_cost) {
    int n = distances.size();
    double bound = current_cost;

    // Arista mínima saliente del nodo actual
    double min_edge_from_current = std::numeric_limits<double>::infinity();
    for (int i = 0; i < n; ++i) {
        if (!visited[i] && i != current_city) {
            min_edge_from_current = std::min(min_edge_from_current, distances[current_city][i]);
        }
    }
    if (min_edge_from_current != std::numeric_limits<double>::infinity()) {
        bound += min_edge_from_current;
    }

    // Para cada nodo no visitado, sumar la mitad de la suma de sus dos aristas mínimas
    for (int i = 0; i < n; ++i) {
        if (!visited[i] && i != current_city) {
            double first_min  = std::numeric_limits<double>::infinity();
            double second_min = std::numeric_limits<double>::infinity();

            for (int j = 0; j < n; ++j) {
                if (i != j) {
                    if (distances[i][j] < first_min) {
                        second_min = first_min;
                        first_min  = distances[i][j];
                    } else if (distances[i][j] < second_min) {
                        second_min = distances[i][j];
                    }
                }
            }
            bound += (first_min + second_min) / 2.0;
        }
    }

    // Estimación de retorno al nodo 0
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

// ================== BRANCH & BOUND RECURSIVO (thread-safe) ===================

void tsp_branch_bound_recursive(const std::vector<std::vector<double>>& distances,
                                std::vector<int>& current_path,
                                std::vector<bool>& visited,
                                double current_cost,
                                double& best_cost,
                                std::vector<int>& best_path,
                                int depth) {
    int n = distances.size();

    // estadísticas globales
    #pragma omp atomic
    bb_stats.nodes_explored++;

    #pragma omp critical
    {
        bb_stats.max_depth = std::max(bb_stats.max_depth, depth);
    }

    // Caso base: todas las ciudades visitadas
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
        #pragma omp atomic
        bb_stats.nodes_pruned++;
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
                tsp_branch_bound_recursive(distances, current_path, visited,
                                           new_cost, best_cost, best_path, depth + 1);
            } else {
                #pragma omp atomic
                bb_stats.nodes_pruned++;
            }

            current_path.pop_back();
            visited[i] = false;
        }
    }

    #pragma omp critical
    {
        bb_stats.branching_factor = std::max(bb_stats.branching_factor, branches);
    }
}

// ========================= VERSIÓN PARALELA (OpenMP) ========================

std::pair<std::vector<int>, double> tsp_branch_bound_parallel(
        const std::vector<std::vector<double>>& distances,
        int num_threads) {

    int n = distances.size();
    bb_stats = BranchBoundStats{};   // reset stats

    double global_best_cost = std::numeric_limits<double>::infinity();
    std::vector<int> global_best_path;

    // para alpha (máx. teórico de nodos)
    int max_theoretical_nodes = 1;
    for (int i = 1; i < n; ++i) {
        max_theoretical_nodes *= (n - i);
    }

    #pragma omp parallel num_threads(num_threads)
    {
        std::vector<int>  local_best_path;
        double            local_best_cost = std::numeric_limits<double>::infinity();
        std::vector<int>  current_path;
        std::vector<bool> visited(n, false);

        // cada hilo explora los subárboles que empiezan en 0 -> next_city
        #pragma omp for schedule(dynamic)
        for (int next_city = 1; next_city < n; ++next_city) {
            std::fill(visited.begin(), visited.end(), false);
            current_path.clear();

            current_path.push_back(0);
            current_path.push_back(next_city);
            visited[0]         = true;
            visited[next_city] = true;

            double current_cost = distances[0][next_city];

            tsp_branch_bound_recursive(distances,
                                       current_path,
                                       visited,
                                       current_cost,
                                       local_best_cost,
                                       local_best_path,
                                       /*depth=*/2);
        }

        // reducción al mejor global
        #pragma omp critical
        {
            if (local_best_cost < global_best_cost) {
                global_best_cost = local_best_cost;
                global_best_path = local_best_path;
            }
        }
    }

    bb_stats.alpha = (double)bb_stats.nodes_explored / max_theoretical_nodes;

    return {global_best_path, global_best_cost};
}

// ====================== ACCESO A LAS ESTADÍSTICAS ===========================

BranchBoundStats get_branch_bound_stats() {
    return bb_stats;
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

    int sample_size = 12;

    std::cout << "=== CONFIGURACION DEL EXPERIMENTO (PARALELO) ===\n";
    std::cout << "Ciudades disponibles: " << all_cities.size() << std::endl;
    std::cout << "Numero de ciudades usado: " << sample_size << std::endl;

    // Seleccion aleatoria de ciudades
    std::vector<Coord> cities;
    std::vector<std::string> city_names;
    std::vector<int> indices(all_cities.size());

    for (int i = 0; i < (int)all_cities.size(); ++i) indices[i] = i;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(indices.begin(), indices.end(), gen);

    for (int i = 0; i < sample_size; ++i) {
        cities.push_back(all_cities[indices[i]]);
        city_names.push_back(all_city_names[indices[i]]);
    }

    std::cout << "\nCiudades seleccionadas para el experimento:\n";
    for (int i = 0; i < sample_size; ++i) {
        std::cout << i << ": " << city_names[i]
                  << " (" << cities[i].lat << ", " << cities[i].lon << ")\n";
    }
    std::cout << std::endl;

    size_t c = cities.size();
    std::vector<std::vector<double>> distances(c, std::vector<double>(c, 0.0));

    for (size_t i = 0; i < c; ++i)
        for (size_t j = 0; j < c; ++j)
            distances[i][j] = calculate_distances_btw_two_city(cities[i], cities[j]);

    // Matriz de distancias (opcional)
    if (c <= 15) {
        std::cout << "Matriz de distancias entre ciudades:\n";
        for (const auto& row : distances) {
            for (double dist : row) {
                std::cout << std::fixed << std::setprecision(2) << dist << "\t";
            }
            std::cout << "\n";
        }
        std::cout << "\n";
    }

    // Fuerza bruta solo para validacion si es pequeño
    std::pair<std::vector<int>, double> brute_result;
    std::chrono::milliseconds brute_duration{0};
    bool run_brute_force = (c <= 12);

    if (run_brute_force) {
        std::cout << "=== SOLUCION EXACTA (Fuerza Bruta, para validar) ===\n";
        auto start_brute = std::chrono::high_resolution_clock::now();

        brute_result = tsp_brute_force(distances);

        auto end_brute = std::chrono::high_resolution_clock::now();
        brute_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_brute - start_brute);

        std::cout << "Mejor ruta (bruta): ";
        for (size_t i = 0; i < brute_result.first.size(); ++i) {
            std::cout << brute_result.first[i];
            if (i < brute_result.first.size() - 1) std::cout << " -> ";
        }
        std::cout << " -> 0\n";
        std::cout << "Costo total: " << std::fixed << std::setprecision(2)
                  << brute_result.second << " km\n";
        std::cout << "Tiempo de ejecucion: " << brute_duration.count() << " ms\n\n";
    }

    // ====== BRANCH & BOUND PARALELO ======
    std::cout << "=== BRANCH AND BOUND PARALELO (OpenMP) ===\n";
    int num_threads;
    std::cout << "Ingrese numero de hilos (p): ";
    std::cin >> num_threads;

    auto start_par = std::chrono::high_resolution_clock::now();
    auto result_par = tsp_branch_bound_parallel(distances, num_threads);
    auto end_par = std::chrono::high_resolution_clock::now();
    auto duration_par = std::chrono::duration_cast<std::chrono::milliseconds>(end_par - start_par);
    auto stats_par = get_branch_bound_stats();

    std::cout << "\nMejor ruta (paralela): ";
    for (size_t i = 0; i < result_par.first.size(); ++i) {
        std::cout << result_par.first[i];
        if (i < result_par.first.size() - 1) std::cout << " -> ";
    }
    std::cout << " -> 0\n";
    std::cout << "Costo total (par): " << std::fixed << std::setprecision(2)
              << result_par.second << " km\n";
    std::cout << "Tiempo de ejecucion (par): " << duration_par.count() << " ms\n";

    std::cout << "\n=== ESTADISTICAS PARALELO ===\n";
    std::cout << "Nodos explorados: " << stats_par.nodes_explored << "\n";
    std::cout << "Nodos podados: " << stats_par.nodes_pruned << "\n";
    std::cout << "Profundidad maxima: " << stats_par.max_depth << "\n";
    std::cout << "Factor ramificacion maximo: " << stats_par.branching_factor << "\n";
    std::cout << "Alpha (par): " << std::fixed << std::setprecision(8) << stats_par.alpha << "\n";

    std::cout << "\nRuta con nombres de ciudades (paralela):\n";
    for (size_t i = 0; i < result_par.first.size(); ++i) {
        std::cout << city_names[result_par.first[i]];
        if (i < result_par.first.size() - 1) std::cout << " -> ";
    }
    std::cout << " -> " << city_names[0] << "\n";

    // Validación contra fuerza bruta
    if (run_brute_force) {
        double cost_diff_par = std::abs(brute_result.second - result_par.second);
        std::cout << "\n=== VALIDACION ===\n";
        std::cout << "Costo optimo (bruta): " << brute_result.second << " km\n";
        std::cout << "Costo paralelo:      " << result_par.second << " km\n";
        std::cout << "Diferencia absoluta: " << cost_diff_par << " km\n";
    }

    return 0;
}
