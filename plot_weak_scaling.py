import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d

def estimate_weak_scaling():
    # 1. Cargar datos
    try:
        df = pd.read_csv('tsp_results.csv')
    except FileNotFoundError:
        print("Error: tsp_results.csv no encontrado.")
        return

    # Filtrar solo datos útiles
    # Asumimos que 'FLOPs_seq' es una buena medida de trabajo (Work)
    # Si no, usamos 'Nodes_seq'
    if 'FLOPs_seq' not in df.columns:
        df['FLOPs_seq'] = df['Nodes_seq'] # Fallback

    # 2. Definir funciones de interpolación
    # Work(n) -> FLOPs. Como crece factorialmente, interpolamos log(Work)
    # Tomamos el promedio de FLOPs_seq para cada n (debería ser constante para un n fijo, pero promediamos por si acaso)
    df_work = df.groupby('n')['FLOPs_seq'].mean().reset_index()
    # Interpolación: log(Work) vs n
    work_interp_func = interp1d(df_work['n'], np.log(df_work['FLOPs_seq']), kind='linear', fill_value="extrapolate")

    def get_work(n):
        return np.exp(work_interp_func(n))

    # 3. Seleccionar un Tiempo Objetivo (Baseline)
    # Para Weak Scaling, queremos mantener el tiempo constante.
    # Elegimos un N base que tenga datos para p=1.
    # N=12 es un buen candidato intermedio.
    baseline_n = 12
    
    # Verificar si tenemos datos para baseline_n
    if baseline_n not in df['n'].values:
        baseline_n = df['n'].min() + 2 # Fallback
    
    # Obtener tiempo secuencial promedio para baseline_n
    # Usamos T_seq directamente de cualquier fila con ese N (asumiendo que T_seq es el mismo para todo p)
    baseline_time = df[df['n'] == baseline_n]['T_seq'].mean()
    
    # Si T_seq es NaN o 0, intentar usar T_par de p=1 si existe
    if pd.isna(baseline_time) or baseline_time == 0:
        if 1 in df['p'].values:
             baseline_time = df[(df['n'] == baseline_n) & (df['p'] == 1)]['T_par'].mean()
    
    if pd.isna(baseline_time):
        print(f"Error: No se encontró tiempo secuencial válido para N={baseline_n}")
        return

    baseline_work = get_work(baseline_n)
    
    print(f"Baseline: N={baseline_n}, Time={baseline_time:.2f} ms, Work={baseline_work:.2e} FLOPs")

    # 4. Estimar Weak Scaling para cada p
    weak_scaling_data = []
    
    threads = sorted(df['p'].unique())
    
    for p in threads:
        # Obtener datos T_par vs n para este p
        df_p = df[df['p'] == p].sort_values('n')
        
        if df_p.empty:
            continue
            
        # Interpolación: log(Time) vs n para este p
        # Usamos log porque el tiempo crece exponencialmente con n
        # Necesitamos al menos 2 puntos para interpolar
        if len(df_p) < 2:
            continue
            
        time_interp_func = interp1d(np.log(df_p['T_par']), df_p['n'], kind='linear', fill_value="extrapolate")
        
        # Encontrar n* tal que T_par(n*, p) = baseline_time
        # n* = time_interp_func(log(baseline_time))
        try:
            # Verificar si baseline_time está dentro del rango razonable de extrapolación
            # Si el tiempo objetivo es mucho menor que el mínimo tiempo registrado para este p, 
            # la extrapolación puede dar n muy bajos o inválidos.
            
            n_star = float(time_interp_func(np.log(baseline_time)))
            
            # Calcular trabajo para este n*
            work_star = get_work(n_star)
            
            # Weak Speedup = Work(n*) / Work(baseline)
            # (Cuánto más trabajo pudimos hacer en el mismo tiempo)
            speedup_weak = work_star / baseline_work
            
            # Efficiency = Speedup / p
            efficiency_weak = speedup_weak / p
            
            weak_scaling_data.append({
                'p': p,
                'n_estimated': n_star,
                'speedup': speedup_weak,
                'efficiency': efficiency_weak
            })
            
        except Exception as e:
            print(f"No se pudo estimar para p={p}: {e}")

    # 5. Crear DataFrame y Graficar
    df_weak = pd.DataFrame(weak_scaling_data)
    print("\nEstimación de Weak Scaling:")
    print(df_weak)

    plt.figure(figsize=(10, 6))
    plt.plot(df_weak['p'], df_weak['efficiency'], marker='o', linestyle='-', color='purple', label='Estimated Weak Scaling')
    
    # Línea ideal
    plt.axhline(y=1.0, color='k', linestyle=':', alpha=0.5, label='Ideal')
    
    plt.title(f'Estimated Weak Scaling Efficiency\n(Constant Time $\\approx$ {baseline_time:.0f} ms, Baseline N={baseline_n})')
    plt.xlabel('Number of Threads (p)')
    plt.ylabel('Efficiency (Scaled Speedup / p)')
    plt.ylim(0, 1.2)
    plt.legend()
    plt.grid(True)
    
    output_file = 'weak_scaling_estimation.png'
    plt.savefig(output_file, dpi=300)
    print(f"\nGráfico guardado en: {output_file}")

if __name__ == "__main__":
    estimate_weak_scaling()
