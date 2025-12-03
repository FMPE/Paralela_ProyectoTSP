import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Intentar cargar el archivo CSV
try:
    df = pd.read_csv('tsp_results.csv')
except FileNotFoundError:
    print("Error: No se encontró el archivo 'tsp_results.csv'. Ejecuta primero el programa C++.")
    exit()

# Calcular GFLOPs/s
# FLOPs_par es el total de operaciones de punto flotante
# T_par está en milisegundos (ms) -> segundos = T_par / 1000
# GFLOPs = (FLOPs / 1e9) / (T_par / 1000) = FLOPs / (T_par * 1e6)
df['GFLOPS'] = df['FLOPs_par'] / (df['T_par'] * 1e6)

# Identificar series de Strong Scaling vs Weak Scaling
# Heurística: 
# - Strong Scaling: Mismo N, múltiples P. (N aparece > 1 vez)
# - Weak Scaling: N cambia con P. (Resto de datos, o N aparece 1 vez)

n_counts = df['n'].value_counts()
strong_sizes = sorted(n_counts[n_counts > 1].index.tolist())

# Separar DataFrames
df_strong = df[df['n'].isin(strong_sizes)].copy()
df_weak = df[~df['n'].isin(strong_sizes)].sort_values('p').copy()

# Si df_weak está vacío pero hay datos que parecen weak scaling (ej. n=4*p),
# podría ser que coincidan con strong sizes. 
# Para este script, asumiremos que los "restantes" son weak.
# Si df_weak tiene muy pocos puntos, intentamos ver si hay una relación lineal n ~ p en todo el df
if df_weak.empty and not df.empty:
    # Intento de rescate: buscar n = 4*p explícitamente
    mask_weak = (df['n'] == 4 * df['p'])
    if mask_weak.any():
        df_weak = df[mask_weak].sort_values('p').copy()
        # Remover estos de strong para no duplicar en gráficos combinados (opcional)
        # df_strong = df_strong[~mask_weak] 

# Configuración de estilo
plt.style.use('ggplot') 
colors = plt.cm.viridis(np.linspace(0, 0.9, len(strong_sizes)))

# ==========================================
# 1. GRÁFICO STRONG SCALING (SPEEDUP)
# ==========================================
plt.figure(figsize=(10, 6))

for i, n in enumerate(strong_sizes):
    data_n = df_strong[df_strong['n'] == n].sort_values('p')
    plt.plot(data_n['p'], data_n['Speedup'], marker='o', label=f'N={n}', color=colors[i])

# Línea ideal
p_values = df['p'].unique()
max_p = max(p_values) if len(p_values) > 0 else 1
plt.plot([1, max_p], [1, max_p], 'k--', label='Ideal (Linear)', alpha=0.7)

plt.title('Strong Scaling: Speedup vs Threads')
plt.xlabel('Number of Threads (p)')
plt.ylabel('Speedup')
plt.legend()
plt.grid(True)
plt.savefig('strong_scaling_speedup.png', dpi=300)
plt.close()

# ==========================================
# 2. GRÁFICO WEAK SCALING (SPEEDUP) - DESACTIVADO
# ==========================================
# plt.figure(figsize=(10, 6))
# if not df_weak.empty:
#     plt.plot(df_weak['p'], df_weak['Speedup'], marker='s', linewidth=2, color='red', label='Weak Scaling')
#     for _, row in df_weak.iterrows():
#         plt.text(row['p'], row['Speedup'], f" N={int(row['n'])}", fontsize=9)
# plt.plot([1, max_p], [1, max_p], 'k--', label='Ideal (Linear)', alpha=0.7)
# plt.title('Weak Scaling: Speedup vs Threads')
# plt.xlabel('Number of Threads (p)')
# plt.ylabel('Speedup')
# plt.legend()
# plt.grid(True)
# plt.savefig('weak_scaling_speedup.png', dpi=300)
# plt.close()

# ==========================================
# 3. GRÁFICO FLOPs (SOLO STRONG)
# ==========================================
plt.figure(figsize=(10, 6))

# Strong
for i, n in enumerate(strong_sizes):
    data_n = df_strong[df_strong['n'] == n].sort_values('p')
    plt.plot(data_n['p'], data_n['GFLOPS'], marker='o', label=f'Strong N={n}', color=colors[i], alpha=0.7)

# Weak (Desactivado)
# if not df_weak.empty:
#     plt.plot(df_weak['p'], df_weak['GFLOPS'], marker='s', linewidth=2, color='red', label='Weak Scaling')

plt.title('Performance: GFLOPs/s vs Threads')
plt.xlabel('Number of Threads (p)')
plt.ylabel('GFLOPs/s')
plt.legend()
plt.grid(True)
plt.savefig('flops_performance.png', dpi=300)
plt.close()

# ==========================================
# 4. GRÁFICO EFICIENCIA (SOLO STRONG)
# ==========================================
plt.figure(figsize=(10, 6))

# Strong
for i, n in enumerate(strong_sizes):
    data_n = df_strong[df_strong['n'] == n].sort_values('p')
    
    # Asegurar que p=1, eff=1.0 esté presente
    p_vals = data_n['p'].tolist()
    eff_vals = data_n['Eficiencia'].tolist()
    if 1 not in p_vals:
        p_vals = [1] + p_vals
        eff_vals = [1.0] + eff_vals
        
    plt.plot(p_vals, eff_vals, marker='o', label=f'Strong N={n}', color=colors[i], alpha=0.7)

# Weak (Desactivado)
# if not df_weak.empty:
#     plt.plot(df_weak['p'], df_weak['Eficiencia'], marker='s', linewidth=2, color='red', label='Weak Scaling')

plt.axhline(y=1.0, color='k', linestyle=':', alpha=0.5, label='Ideal')
plt.title('Efficiency vs Threads')
plt.xlabel('Number of Threads (p)')
plt.ylabel('Efficiency (Speedup/p)')

# Eliminado el ajuste forzado de escala vertical
# plt.ylim(0, max(current_ylim[1] * 1.2, 1.5)) 

plt.legend()
plt.grid(True)
plt.savefig('efficiency.png', dpi=300)
plt.close()

# ==========================================
# 5. GRÁFICO TIEMPO vs HILOS (Strong Scaling)
# ==========================================
plt.figure(figsize=(10, 6))

for i, n in enumerate(strong_sizes):
    data_n = df_strong[df_strong['n'] == n].sort_values('p')
    # Usamos T_par que está en ms
    plt.plot(data_n['p'], data_n['T_par'], marker='o', label=f'N={n}', color=colors[i])

plt.title('Execution Time vs Threads (Log Scale)')
plt.xlabel('Number of Threads (p)')
plt.ylabel('Time (ms)')
plt.yscale('log') # Escala logarítmica para visualizar diferentes órdenes de magnitud
plt.legend()
plt.grid(True, which="both", ls="-", alpha=0.5)
plt.savefig('time_vs_threads.png', dpi=300)
plt.close()

# ==========================================
# 6. GRÁFICO TIEMPO vs TAMAÑO (N)
# ==========================================
plt.figure(figsize=(10, 6))

# Obtener lista de N únicos
unique_ns = sorted(df_strong['n'].unique())
seq_times = []
par_best_times = []

# Determinar el máximo número de hilos usado en general
max_threads_global = df_strong['p'].max()

for n in unique_ns:
    data_n = df_strong[df_strong['n'] == n]
    
    # Tiempo secuencial (promedio)
    t_seq = data_n['T_seq'].mean()
    seq_times.append(t_seq)
    
    # Tiempo paralelo con el máximo número de hilos disponible para este N
    # (Idealmente es max_threads_global, pero filtramos por si acaso)
    max_p_for_n = data_n['p'].max()
    t_par_best = data_n[data_n['p'] == max_p_for_n]['T_par'].mean()
    
    par_best_times.append(t_par_best)

plt.plot(unique_ns, seq_times, marker='o', linestyle='--', color='black', label='Sequential')
plt.plot(unique_ns, par_best_times, marker='s', linestyle='-', color='blue', label=f'Parallel (Best p)')

plt.title('Execution Time vs Problem Size (N)')
plt.xlabel('Problem Size (N)')
plt.ylabel('Time (ms)')
plt.yscale('log') # Escala logarítmica esencial para complejidad factorial
plt.legend()
plt.grid(True, which="both", ls="-", alpha=0.5)
plt.savefig('time_vs_size.png', dpi=300)
plt.close()

print("Gráficos generados exitosamente (Weak Scaling omitido):")
print(" - strong_scaling_speedup.png")
print(" - flops_performance.png")
print(" - efficiency.png")
print(" - time_vs_threads.png")
print(" - time_vs_size.png")
