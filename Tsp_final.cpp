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

// ============ BRANCH & BOUND PARALELO CON OpenMP TASKS =======================
// Misma lógica recursiva que el secuencial, pero cada subárbol nace en una tarea.

void tsp_branch_bound_recursive_par(const std::vector<std::vector<double>>& distances,
                                    std::vector<int>& current_path,
                                    std::vector<bool>& visited,
                                    double current_cost,
                                    double& best_cost,
                                    std::vector<int>& best_path) {
    int n = distances.size();

    // Caso base: todas las ciudades visitadas
    if ((int)current_path.size() == n) {
        double total_cost = current_cost + distances[current_path.back()][0];

        // Único punto crítico: actualizar solución global
        #pragma omp critical
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

    // Si la cota ya es peor que la mejor solución, podar
    if (bound >= best_cost) {
        return;
    }

    for (int i = 0; i < n; ++i) {
        if (!visited[i]) {
            current_path.push_back(i);
            visited[i] = true;

            double new_cost = current_cost + distances[current_city][i];

            if (new_cost < best_cost) {
                tsp_branch_bound_recursive_par(distances,
                                               current_path,
                                               visited,
                                               new_cost,
                                               best_cost,
                                               best_path);
            }

            visited[i] = false;
            current_path.pop_back();
        }
    }
}

std::pair<std::vector<int>, double> tsp_branch_bound_parallel(
        const std::vector<std::vector<double>>& distances,
        int num_threads) {

    int n = distances.size();
    double best_cost = std::numeric_limits<double>::infinity();
    std::vector<int> best_path;

    omp_set_num_threads(num_threads);

    #pragma omp parallel
    {
        #pragma omp single
        {
            // Creamos una tarea por cada ciudad siguiente a 0
            for (int next_city = 1; next_city < n; ++next_city) {
                #pragma omp task firstprivate(next_city)
                {
                    std::vector<int>  current_path;
                    std::vector<bool> visited(n, false);

                    current_path.push_back(0);
                    current_path.push_back(next_city);
                    visited[0]         = true;
                    visited[next_city] = true;

                    double current_cost = distances[0][next_city];

                    tsp_branch_bound_recursive_par(distances,
                                                   current_path,
                                                   visited,
                                                   current_cost,
                                                   best_cost,
                                                   best_path);
                }
            }
            #pragma omp taskwait
        }
    }

    return {best_path, best_cost};
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

    int sample_size = 12; // puedes cambiar este valor

    std::cout << "=== TSP Branch & Bound Paralelo (OpenMP tasks) ===\n";
    std::cout << "Ciudades disponibles: " << all_cities.size() << std::endl;
    std::cout << "Numero de ciudades usado: " << sample_size << std::endl;

    // Selección aleatoria de ciudades
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

    // ====== BRANCH & BOUND PARALELO ======
    std::cout << "=== BRANCH AND BOUND PARALELO (OpenMP tasks) ===\n";
    int num_threads;
    std::cout << "Ingrese numero de hilos (p): ";
    std::cin >> num_threads;

    auto start_par = std::chrono::high_resolution_clock::now();
    auto result_par = tsp_branch_bound_parallel(distances, num_threads);
    auto end_par = std::chrono::high_resolution_clock::now();
    auto duration_par = std::chrono::duration_cast<std::chrono::milliseconds>(end_par - start_par);

    std::cout << "\nMejor ruta (paralela): ";
    for (size_t i = 0; i < result_par.first.size(); ++i) {
        std::cout << result_par.first[i];
        if (i < result_par.first.size() - 1) std::cout << " -> ";
    }
    std::cout << " -> 0\n";
    std::cout << "Costo total (par): " << std::fixed << std::setprecision(2)
              << result_par.second << " km\n";
    std::cout << "Tiempo de ejecucion (par): " << duration_par.count() << " ms\n";

    std::cout << "\nRuta con nombres de ciudades:\n";
    for (size_t i = 0; i < result_par.first.size(); ++i) {
        std::cout << city_names[result_par.first[i]];
        if (i < result_par.first.size() - 1) std::cout << " -> ";
    }
    std::cout << " -> " << city_names[0] << "\n";

    return 0;
}
