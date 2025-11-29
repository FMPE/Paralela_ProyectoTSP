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
#include <random>

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

// Funcion para calcular la solucion exacta usando fuerza bruta (para validacion)
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
// Estructura para almacenar estadisticas de Branch and Bound
struct BranchBoundStats {
    int nodes_explored = 0;
    int nodes_pruned = 0;
    int max_depth = 0;
    int branching_factor = 0;
    double alpha = 0.0; // Factor de efectividad de la poda
};

// Variable global para estadisticas
BranchBoundStats bb_stats;

// Funcion para calcular la cota inferior (bound) usando suma de aristas minimas
double calculate_lower_bound(const std::vector<std::vector<double>>& distances, 
                            const std::vector<bool>& visited, 
                            int current_city, double current_cost) {
    int n = distances.size();
    double bound = current_cost;
    
    // Agregar la arista minima saliente del nodo actual
    double min_edge_from_current = std::numeric_limits<double>::infinity();
    for (int i = 0; i < n; ++i) {
        if (!visited[i] && i != current_city) {
            min_edge_from_current = std::min(min_edge_from_current, distances[current_city][i]);
        }
    }
    if (min_edge_from_current != std::numeric_limits<double>::infinity()) {
        bound += min_edge_from_current;
    }
    
    // Para cada nodo no visitado, agregar la mitad de la suma de sus dos aristas minimas
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
    
    // Agregar estimacion de arista de retorno
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

// Funcion recursiva de Branch and Bound con búsqueda en profundidad
void tsp_branch_bound_recursive(const std::vector<std::vector<double>>& distances,
                                std::vector<int>& current_path,
                                std::vector<bool>& visited,
                                double current_cost,
                                double& best_cost,
                                std::vector<int>& best_path,
                                int depth) {
    int n = distances.size();
    bb_stats.nodes_explored++;
    bb_stats.max_depth = std::max(bb_stats.max_depth, depth);
    
    // Caso base: hemos visitado todas las ciudades
    if (current_path.size() == n) {
        // Agregar el costo de retorno al nodo inicial
        double total_cost = current_cost + distances[current_path.back()][0];
        if (total_cost < best_cost) {
            best_cost = total_cost;
            best_path = current_path;
        }
        return;
    }
    
    // Calcular bound para poda
    int current_city = current_path.back();
    double bound = calculate_lower_bound(distances, visited, current_city, current_cost);
    
    // Poda: si el bound es mayor o igual al mejor costo, no explorar
    if (bound >= best_cost) {
        bb_stats.nodes_pruned++;
        return;
    }
    
    // Explorar todos los nodos no visitados (búsqueda en profundidad)
    int branches = 0;
    for (int i = 0; i < n; ++i) {
        if (!visited[i]) {
            branches++;
            
            // Agregar nodo al camino
            current_path.push_back(i);
            visited[i] = true;
            double new_cost = current_cost + distances[current_city][i];
            
            // Poda temprana: si el costo parcial ya supera el mejor, no explorar
            if (new_cost < best_cost) {
                // Llamada recursiva (DFS)
                tsp_branch_bound_recursive(distances, current_path, visited, 
                                         new_cost, best_cost, best_path, depth + 1);
            } else {
                bb_stats.nodes_pruned++;
            }
            
            // Backtrack
            current_path.pop_back();
            visited[i] = false;
        }
    }
    
    bb_stats.branching_factor = std::max(bb_stats.branching_factor, branches);
}

std::pair<std::vector<int>, double> tsp_branch_bound(const std::vector<std::vector<double>>& distances) {
    int n = distances.size();
    
    // Reiniciar estadisticas
    bb_stats = BranchBoundStats{};
    
    std::vector<int> best_path;
    double best_cost = std::numeric_limits<double>::infinity();
    
    // Inicializacion para búsqueda desde nodo 0
    std::vector<int> current_path = {0};
    std::vector<bool> visited(n, false);
    visited[0] = true;
    
    // Calcular número maximo teorico de nodos (b^d)
    int max_theoretical_nodes = 1;
    for (int i = 1; i < n; ++i) {
        max_theoretical_nodes *= (n - i);
    }
    
    // Iniciar la búsqueda recursiva
    tsp_branch_bound_recursive(distances, current_path, visited, 0.0, 
                              best_cost, best_path, 1);
    
    // Calcular alpha (factor de efectividad de poda)
    bb_stats.alpha = (double)bb_stats.nodes_explored / max_theoretical_nodes;
    
    return {best_path, best_cost};
}

// Funcion para obtener estadisticas del algoritmo
BranchBoundStats get_branch_bound_stats() {
    return bb_stats;
}

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
    
    // Configuración del experimento
    int sample_size = 10;

    std::cout << "=== CONFIGURACION DEL EXPERIMENTO ===\n";
    std::cout << "Ciudades disponibles: " << all_cities.size() << std::endl;
    //std::cout << "Ingrese el numero de ciudades para el experimento (3-20): ";
    //std::cin >> sample_size;
    /*
    if (sample_size < 3 || sample_size > (int)all_cities.size()) {
        std::cout << "Numero invalido. Usando 5 ciudades por defecto." << std::endl;
        sample_size = 5;
    }
    */
    
    // Selección aleatoria de ciudades
    std::vector<Coord> cities;
    std::vector<std::string> city_names;
    std::vector<int> indices(all_cities.size());
    
    // Crear vector de indices
    for (int i = 0; i < (int)all_cities.size(); ++i) {
        indices[i] = i;
    }
    
    // Mezclar aleatoriamente y tomar los primeros sample_size
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
                  << " (" << cities[i].lat << ", " << cities[i].lon << ")" << std::endl;
    }
    std::cout << std::endl;
    
    size_t c = cities.size();
    std::vector<std::vector<double>> distances(c, std::vector<double>(c, 0.0));

    for (size_t i = 0; i < c; ++i) {
        for (size_t j = 0; j < c; ++j) {
            distances[i][j] = calculate_distances_btw_two_city(cities[i], cities[j]);
        }
    }

    // Mostrar matriz de distancias solo para casos pequeños
    if (c <= 15) {
        std::cout << "Matriz de distancias entre ciudades:\n";
        for (const auto& row : distances) {
            for (double dist : row) {
                std::cout << std::fixed << std::setprecision(2) << dist << "\t";
            }
            std::cout << "\n";
        }
    } else {
        std::cout << "Matriz de distancias (" << c << "x" << c << ") - demasiado grande para mostrar\n";
    }
    std::cout << "\n";

    // Variables para fuerza bruta
    std::pair<std::vector<int>, double> brute_result;
    std::chrono::milliseconds brute_duration{0};
    bool run_brute_force = (c <= 15); // Solo para casos muy pequeños
    
    if (run_brute_force) {
        // Resolver TSP usando fuerza bruta (solucion exacta para validacion)
        std::cout << "=== SOLUCION EXACTA (Fuerza Bruta) ===\n";
        auto start_brute = std::chrono::high_resolution_clock::now();
        
        brute_result = tsp_brute_force(distances);
        
        auto end_brute = std::chrono::high_resolution_clock::now();
        brute_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_brute - start_brute);

        std::cout << "Mejor ruta (fuerza bruta): ";
        for (size_t i = 0; i < brute_result.first.size(); ++i) {
            std::cout << brute_result.first[i];
            if (i < brute_result.first.size() - 1) std::cout << " -> ";
        }
        std::cout << " -> 0" << std::endl;
        
        std::cout << "Costo total: " << std::fixed << std::setprecision(2) << brute_result.second << " km" << std::endl;
        std::cout << "Tiempo de ejecucion: " << brute_duration.count() << " ms" << std::endl;
    } else {
        std::cout << "=== FUERZA BRUTA OMITIDA ===\n";
        std::cout << "Problema demasiado grande (n=" << c << ") para fuerza bruta.\n";
        std::cout << "Complejidad O(n!) = O(" << c << "!) seria impracticable.\n";
    }

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
    std::cout << "Tiempo de ejecucion: " << duration.count() << " ms" << std::endl;
    
    // Mostrar estadisticas de Branch and Bound
    auto stats = get_branch_bound_stats();
    std::cout << "\n=== ESTADiSTICAS DE BRANCH AND BOUND ===\n";
    std::cout << "Nodos explorados (Nexp): " << std::fixed << std::setw(12) << stats.nodes_explored << std::endl;
    std::cout << "Nodos podados: " << std::fixed << std::setw(18) << stats.nodes_pruned << std::endl;
    std::cout << "Nodos totales visitados: " << std::fixed << std::setw(8) << (stats.nodes_explored + stats.nodes_pruned) << std::endl;
    std::cout << "Profundidad maxima (d): " << std::fixed << std::setw(11) << stats.max_depth << std::endl;
    std::cout << "Factor de ramificacion maximo (b): " << std::fixed << std::setw(3) << stats.branching_factor << std::endl;
    std::cout << "Factor de efectividad α: " << std::fixed << std::setprecision(8) << std::setw(15) << stats.alpha << std::endl;
    
    // Calcular complejidad teorica
    int n = distances.size();
    double theoretical_bd = std::pow(stats.branching_factor, stats.max_depth);
    double theoretical_factorial = 1.0;
    for (int i = 2; i <= n; i++) theoretical_factorial *= i;
    
    std::cout << "\n=== ANaLISIS DE COMPLEJIDAD ===\n";
    std::cout << "Problema: n = " << n << " ciudades" << std::endl;
    std::cout << "Complejidad teorica maxima: O(b^d) = O(" << stats.branching_factor << "^" << stats.max_depth << ") = " << std::scientific << std::setprecision(3) << theoretical_bd << std::endl;
    std::cout << "Complejidad fuerza bruta: O(n!) = O(" << n << "!) = " << std::scientific << std::setprecision(3) << theoretical_factorial << std::endl;
    std::cout << "Nodos realmente explorados: " << std::fixed << std::setprecision(0) << std::setw(12) << stats.nodes_explored << std::endl;
    std::cout << "Ratio exploracion vs teorico: " << std::fixed << std::setprecision(6) << std::setw(12) << (stats.nodes_explored / theoretical_bd) * 100 << "%" << std::endl;
    std::cout << "Efectividad de la poda: " << std::fixed << std::setprecision(4) << std::setw(12) << (1.0 - stats.alpha) * 100 << "%" << std::endl;
    std::cout << "Tiempo secuencial Ts = Θ(Nexp) = " << std::fixed << std::setprecision(0) << stats.nodes_explored << " operaciones" << std::endl;
    
    if (stats.alpha < 1.0) {
        std::cout << "Poda efectiva: Ts = Θ(α × b^d) donde α = " << std::fixed << std::setprecision(8) << stats.alpha << std::endl;
        std::cout << "Reduccion de complejidad: " << std::fixed << std::setprecision(2) << (1.0 / stats.alpha) << "x menos nodos que el peor caso" << std::endl;
    }
    
    // Mostrar nombres de ciudades en la ruta
    std::cout << "\nRuta con nombres de ciudades:\n";
    for (size_t i = 0; i < result.first.size(); ++i) {
        std::cout << city_names[result.first[i]];
        if (i < result.first.size() - 1) std::cout << " -> ";
    }
    std::cout << " -> " << city_names[0] << std::endl;

    // Comparacion y validacion
    std::cout << "\n=== ANALISIS DE RESULTADOS ===\n";
    
    if (run_brute_force) {
        double cost_difference = std::abs(brute_result.second - result.second);
        bool algorithms_match = cost_difference < 1e-6; // Tolerancia para errores de punto flotante
        
        std::cout << "=== COMPARACION Y VALIDACION ===\n";
        std::cout << "Costo fuerza bruta (optimo): " << std::fixed << std::setprecision(6) << brute_result.second << " km" << std::endl;
        std::cout << "Costo branch & bound: " << std::fixed << std::setprecision(6) << result.second << " km" << std::endl;
        std::cout << "Diferencia absoluta: " << std::fixed << std::setprecision(6) << cost_difference << " km" << std::endl;
        
        // Calcular precision del Branch and Bound
        double precision_percentage = 100.0;
        double error_percentage = 0.0;
        if (brute_result.second > 0) {
            error_percentage = (cost_difference / brute_result.second) * 100.0;
            precision_percentage = 100.0 - error_percentage;
        }
        
        std::cout << "\n=== PRECISION DEL BRANCH AND BOUND ===\n";
        std::cout << "Error relativo: " << std::fixed << std::setprecision(8) << error_percentage << "%" << std::endl;
        std::cout << "Precision: " << std::fixed << std::setprecision(8) << precision_percentage << "%" << std::endl;
        
        if (algorithms_match) {
            std::cout << "✓ SOLUCION OPTIMA: Branch & Bound encontro la solucion exacta (precision = 100%)" << std::endl;
        } else {
            if (error_percentage < 0.001) {
                std::cout << "✓ EXCELENTE: Error despreciable (< 0.001%)" << std::endl;
            } else if (error_percentage < 0.01) {
                std::cout << "✓ MUY BUENO: Error muy pequeño (< 0.01%)" << std::endl;
            } else if (error_percentage < 0.1) {
                std::cout << "⚠ BUENO: Error pequeño (< 0.1%)" << std::endl;
            } else if (error_percentage < 1.0) {
                std::cout << "⚠ ACEPTABLE: Error moderado (< 1%)" << std::endl;
            } else {
                std::cout << "✗ PROBLEMA: Error significativo (>= 1%)" << std::endl;
            }
        }
        
        // Comparacion de tiempos
        std::cout << "\n=== COMPARACION DE RENDIMIENTO ===\n";
        std::cout << "Tiempo fuerza bruta: " << std::fixed << std::setprecision(3) << std::setw(10) << brute_duration.count() << " ms" << std::endl;
        std::cout << "Tiempo branch & bound: " << std::fixed << std::setprecision(3) << std::setw(8) << duration.count() << " ms" << std::endl;
        std::cout << "Diferencia temporal: " << std::fixed << std::setprecision(3) << std::setw(10) << std::abs((double)brute_duration.count() - duration.count()) << " ms" << std::endl;
        
        if (duration.count() < brute_duration.count()) {
            double speedup = (double)brute_duration.count() / duration.count();
            std::cout << "Branch & bound es " << std::fixed << std::setprecision(2) 
                      << speedup << "x mas rapido que fuerza bruta" << std::endl;
        } else if (duration.count() > brute_duration.count()) {
            double slowdown = (double)duration.count() / brute_duration.count();
            std::cout << "Fuerza bruta es " << std::fixed << std::setprecision(2) 
                      << slowdown << "x mas rapido que branch & bound" << std::endl;
        } else {
            std::cout << "Ambos algoritmos tuvieron el mismo tiempo de ejecucion" << std::endl;
        }
    } else {
        std::cout << "=== RESULTADO BRANCH & BOUND ===\n";
        std::cout << "Solucion encontrada: " << std::fixed << std::setprecision(6) << result.second << " km" << std::endl;
        std::cout << "Tiempo de ejecucion: " << std::fixed << std::setprecision(3) << duration.count() << " ms" << std::endl;
        
        // Estimacion de calidad basada en la efectividad de la poda
        std::cout << "\n=== ESTIMACION DE CALIDAD ===\n";
        auto stats = get_branch_bound_stats();
        
        if (stats.alpha < 0.01) {
            std::cout << "✓ ALTA CONFIANZA: Poda muy efectiva (α = " << std::fixed << std::setprecision(6) << stats.alpha << ")" << std::endl;
            std::cout << "  La solucion probablemente es optima o muy cercana al optimo" << std::endl;
        } else if (stats.alpha < 0.1) {
            std::cout << "✓ BUENA CONFIANZA: Poda efectiva (α = " << std::fixed << std::setprecision(6) << stats.alpha << ")" << std::endl;
            std::cout << "  La solucion deberia ser de buena calidad" << std::endl;
        } else if (stats.alpha < 0.5) {
            std::cout << "⚠ CONFIANZA MODERADA: Poda parcial (α = " << std::fixed << std::setprecision(6) << stats.alpha << ")" << std::endl;
            std::cout << "  La solucion puede no ser optima" << std::endl;
        } else {
            std::cout << "⚠ BAJA CONFIANZA: Poca poda (α = " << std::fixed << std::setprecision(6) << stats.alpha << ")" << std::endl;
            std::cout << "  Se recomienda verificar la solucion o usar otras heuristicas" << std::endl;
        }
        
        // Calcular estimacion de bound inferior
        double avg_edge_length = 0.0;
        int edge_count = 0;
        for (int i = 0; i < c; i++) {
            for (int j = i + 1; j < c; j++) {
                avg_edge_length += distances[i][j];
                edge_count++;
            }
        }
        avg_edge_length /= edge_count;
        double lower_bound_estimate = avg_edge_length * c; // Estimacion muy basica
        
        double quality_ratio = result.second / lower_bound_estimate;
        std::cout << "\nEstimacion de bound inferior: " << std::fixed << std::setprecision(2) << lower_bound_estimate << " km" << std::endl;
        std::cout << "Ratio solucion/bound: " << std::fixed << std::setprecision(4) << quality_ratio << std::endl;
        
        if (quality_ratio < 1.5) {
            std::cout << "✓ Solucion probablemente muy buena (ratio < 1.5)" << std::endl;
        } else if (quality_ratio < 2.0) {
            std::cout << "✓ Solucion probablemente buena (ratio < 2.0)" << std::endl;
        } else {
            std::cout << "⚠ Solucion puede mejorarse (ratio >= 2.0)" << std::endl;
        }
        
        std::cout << "\nNota: Para n=" << c << ", fuerza bruta tendria complejidad O(" << c << "!) que es impracticable." << std::endl;
        std::cout << "Branch & Bound proporciona una solucion eficiente para este tamaño de problema." << std::endl;
    }

    return 0;
}

