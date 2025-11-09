#include <iostream>
#include <vector>
#include <set>
#include <stack>
#include <limits>
#include <utility>
#include <cmath>
#include <array>
#include <iomanip>
#include <chrono>
#include <string>
#include <algorithm>

using namespace std;

struct Coord {
    double lat;
    double lon;
};

double calculate_distances_btw_two_city(const Coord& coord1, const Coord& coord2) {
    const double R = 6371.0;  // Earth radius in kilometers
    const double PI = std::acos(-1.0);
    double dlat = (coord2.lat - coord1.lat) * PI / 180.0;
    double dlon = (coord2.lon - coord1.lon) * PI / 180.0;
    double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
               std::cos(coord1.lat * PI / 180.0) * std::cos(coord2.lat * PI / 180.0) *
               std::sin(dlon / 2) * std::sin(dlon / 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    return R * c;
}

// Función para calcular la solución exacta usando fuerza bruta (para validación)
std::pair<std::vector<int>, double> tsp_brute_force(const std::vector<std::vector<double>>& distances) {
    int n = distances.size();
    std::vector<int> cities;
    
    // Crear vector de ciudades (excluyendo la ciudad 0 que es el punto de partida)
    for (int i = 1; i < n; ++i) {
        cities.push_back(i);
    }
    
    std::vector<int> best_path = {0};
    double best_cost = std::numeric_limits<double>::infinity();
    
    // Generar todas las permutaciones posibles
    do {
        std::vector<int> current_path = {0};
        current_path.insert(current_path.end(), cities.begin(), cities.end());
        
        // Calcular el costo de esta ruta
        double current_cost = 0.0;
        for (int i = 0; i < n - 1; ++i) {
            current_cost += distances[current_path[i]][current_path[i + 1]];
        }
        // Agregar el costo de volver al inicio
        current_cost += distances[current_path[n - 1]][0];
        
        // Actualizar la mejor ruta si es necesario
        if (current_cost < best_cost) {
            best_cost = current_cost;
            best_path = current_path;
        }
        
    } while (std::next_permutation(cities.begin(), cities.end()));
    
    return {best_path, best_cost};
}

std::pair<std::vector<int>, double> tsp_branch_bound(const std::vector<std::vector<double>>& distances) {
    int n = distances.size();
    std::vector<int> best_path;
    double best_cost = std::numeric_limits<double>::infinity();

    // stack elements: (current_node, path, visited_set, current_cost)
    std::stack<std::tuple<int, std::vector<int>, std::set<int>, double>> stack;
    stack.push({0, {0}, {0}, 0.0});

    while (!stack.empty()) {
        auto node = stack.top();
        stack.pop();

        int current_node = std::get<0>(node);
        std::vector<int> path = std::get<1>(node);
        std::set<int> visited = std::get<2>(node);
        double current_cost = std::get<3>(node);

        if ((int)path.size() == n) {
            double cost = current_cost + distances[path.back()][0];
            if (cost < best_cost) {
                best_path = path;
                best_cost = cost;
            }
        } else {
            for (int i = 0; i < n; ++i) {
                if (visited.find(i) == visited.end()) {
                    double child_cost = current_cost + distances[path.back()][i];
                    if (child_cost < best_cost) {
                        std::vector<int> child_path = path;
                        child_path.push_back(i);
                        std::set<int> child_visited = visited;
                        child_visited.insert(i);
                        stack.push({i, child_path, child_visited, child_cost});
                    }
                }
            }
        }
    }

    return {best_path, best_cost};
}

int main() {
    std::vector<Coord> cities = {
        {52.5200, 13.4050},  // Berlin
        {53.5511, 9.9937},   // Hamburg
        {48.1351, 11.5820},  // Munich
        {50.9375, 6.9603},   // Cologne
        {50.1109, 8.6821}    // Frankfurt
    };

    size_t n = cities.size();
    std::vector<std::vector<double>> distances(n, std::vector<double>(n, 0.0));

    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            distances[i][j] = calculate_distances_btw_two_city(cities[i], cities[j]);
        }
    }

    // Mostrar matriz de distancias
    std::cout << "Matriz de distancias entre ciudades:\n";
    for (const auto& row : distances) {
        for (double dist : row) {
            std::cout << std::fixed << std::setprecision(2) << dist << "\t";
        }
        std::cout << "\n";
    }
    std::cout << "\n";

    // Resolver TSP usando fuerza bruta (solución exacta para validación)
    std::cout << "=== SOLUCIÓN EXACTA (Fuerza Bruta) ===\n";
    auto start_brute = std::chrono::high_resolution_clock::now();
    
    auto brute_result = tsp_brute_force(distances);
    
    auto end_brute = std::chrono::high_resolution_clock::now();
    auto brute_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_brute - start_brute);

    std::cout << "Mejor ruta (fuerza bruta): ";
    for (size_t i = 0; i < brute_result.first.size(); ++i) {
        std::cout << brute_result.first[i];
        if (i < brute_result.first.size() - 1) std::cout << " -> ";
    }
    std::cout << " -> 0" << std::endl;
    
    std::cout << "Costo total: " << std::fixed << std::setprecision(2) << brute_result.second << " km" << std::endl;
    std::cout << "Tiempo de ejecución: " << brute_duration.count() << " ms" << std::endl;

    // Resolver TSP usando branch and bound
    std::cout << "\n=== BRANCH AND BOUND ===\n";
    auto start_time = std::chrono::high_resolution_clock::now();
    
    auto result = tsp_branch_bound(distances);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    // Mostrar resultados
    std::cout << "Mejor ruta encontrada: ";
    for (size_t i = 0; i < result.first.size(); ++i) {
        std::cout << result.first[i];
        if (i < result.first.size() - 1) std::cout << " -> ";
    }
    std::cout << " -> 0" << std::endl;  // Volver al inicio
    
    std::cout << "Costo total: " << std::fixed << std::setprecision(2) << result.second << " km" << std::endl;
    std::cout << "Tiempo de ejecución: " << duration.count() << " ms" << std::endl;
    
    // Mostrar nombres de ciudades en la ruta
    std::vector<std::string> city_names = {"Berlin", "Hamburg", "Munich", "Cologne", "Frankfurt"};
    std::cout << "\nRuta con nombres de ciudades:\n";
    for (size_t i = 0; i < result.first.size(); ++i) {
        std::cout << city_names[result.first[i]];
        if (i < result.first.size() - 1) std::cout << " -> ";
    }
    std::cout << " -> " << city_names[0] << std::endl;

    // Comparación y validación
    std::cout << "\n=== COMPARACIÓN Y VALIDACIÓN ===\n";
    double cost_difference = std::abs(brute_result.second - result.second);
    bool algorithms_match = cost_difference < 1e-6; // Tolerancia para errores de punto flotante
    
    std::cout << "Costo fuerza bruta: " << std::fixed << std::setprecision(6) << brute_result.second << " km" << std::endl;
    std::cout << "Costo branch & bound: " << std::fixed << std::setprecision(6) << result.second << " km" << std::endl;
    std::cout << "Diferencia: " << std::fixed << std::setprecision(6) << cost_difference << " km" << std::endl;
    
    if (algorithms_match) {
        std::cout << "✓ VALIDACIÓN EXITOSA: Ambos algoritmos encontraron la misma solución óptima!" << std::endl;
    } else {
        std::cout << "✗ ERROR: Los algoritmos encontraron soluciones diferentes!" << std::endl;
    }
    
    // Comparación de tiempos
    std::cout << "\nComparación de tiempos:\n";
    std::cout << "Fuerza bruta: " << brute_duration.count() << " ms" << std::endl;
    std::cout << "Branch & bound: " << duration.count() << " ms" << std::endl;
    
    if (duration.count() < brute_duration.count()) {
        double speedup = (double)brute_duration.count() / duration.count();
        std::cout << "Branch & bound es " << std::fixed << std::setprecision(2) 
                  << speedup << "x más rápido que fuerza bruta" << std::endl;
    } else if (duration.count() > brute_duration.count()) {
        double slowdown = (double)duration.count() / brute_duration.count();
        std::cout << "Fuerza bruta es " << std::fixed << std::setprecision(2) 
                  << slowdown << "x más rápido que branch & bound" << std::endl;
    } else {
        std::cout << "Ambos algoritmos tuvieron el mismo tiempo de ejecución" << std::endl;
    }

    return 0;
}

