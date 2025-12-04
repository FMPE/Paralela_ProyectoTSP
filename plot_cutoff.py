import pandas as pd
import matplotlib.pyplot as plt
import os

def plot_cutoff_results():
    csv_file = 'cutoff_results.csv'
    if not os.path.exists(csv_file):
        print(f"Error: {csv_file} no encontrado.")
        return

    try:
        df = pd.read_csv(csv_file)
    except Exception as e:
        print(f"Error al leer el CSV: {e}")
        return

    # Asegurar que los datos estén ordenados
    df = df.sort_values(by=['Threads', 'Cutoff'])

    cutoffs = sorted(df['Cutoff'].unique())
    threads = sorted(df['Threads'].unique())

    # Configuración de estilo básico
    plt.style.use('ggplot')

    # ---------------------------------------------------------
    # 1. Speedup vs Threads (Agrupado por Cutoff)
    # ---------------------------------------------------------
    plt.figure(figsize=(10, 6))
    for cutoff in cutoffs:
        subset = df[df['Cutoff'] == cutoff]
        label = f'Cutoff={cutoff}' if cutoff != 12 else 'Cutoff=No Limit (12)'
        plt.plot(subset['Threads'], subset['Speedup'], marker='o', label=label)
    
    # Línea ideal
    plt.plot(threads, threads, 'k--', alpha=0.3, label='Ideal Linear')
    
    plt.title('Speedup vs Threads (Impacto del Cutoff)')
    plt.xlabel('Number of Threads')
    plt.ylabel('Speedup')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('cutoff_speedup.png', dpi=300)
    print("Gráfico guardado: cutoff_speedup.png")

    # ---------------------------------------------------------
    # 2. Efficiency vs Threads (Agrupado por Cutoff)
    # ---------------------------------------------------------
    plt.figure(figsize=(10, 6))
    for cutoff in cutoffs:
        subset = df[df['Cutoff'] == cutoff]
        label = f'Cutoff={cutoff}' if cutoff != 12 else 'Cutoff=No Limit (12)'
        plt.plot(subset['Threads'], subset['Efficiency'], marker='o', label=label)
    
    plt.axhline(y=1.0, color='k', linestyle='--', alpha=0.3, label='Ideal')
    
    plt.title('Efficiency vs Threads (Impacto del Cutoff)')
    plt.xlabel('Number of Threads')
    plt.ylabel('Efficiency')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('cutoff_efficiency.png', dpi=300)
    print("Gráfico guardado: cutoff_efficiency.png")

    # ---------------------------------------------------------
    # 3. Time vs Cutoff (Agrupado por Threads)
    # ---------------------------------------------------------
    plt.figure(figsize=(10, 6))
    for p in threads:
        subset = df[df['Threads'] == p].sort_values('Cutoff')
        plt.plot(subset['Cutoff'], subset['Time_ms'], marker='o', label=f'Threads={p}')
        
    plt.title('Execution Time vs Task Cutoff')
    plt.xlabel('Cutoff Depth')
    plt.ylabel('Time (ms)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('cutoff_time.png', dpi=300)
    print("Gráfico guardado: cutoff_time.png")

if __name__ == "__main__":
    plot_cutoff_results()
