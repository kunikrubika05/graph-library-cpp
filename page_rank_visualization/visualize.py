import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

os.makedirs('page_rank_visualization/images', exist_ok=True)

perf_data = pd.read_csv('page_rank_visualization/data/performance.csv')
ranks_data = pd.read_csv('page_rank_visualization/data/pagerank_top1000.csv')

plt.figure(figsize=(12, 5))

plt.subplot(1, 2, 1)
plt.plot(perf_data['threads'], perf_data['time'], 'o-', color='blue', linewidth=2)
plt.xlabel('Количество потоков')
plt.ylabel('Время выполнения (сек)')
plt.title('Время выполнения PageRank')
plt.grid(True)

plt.subplot(1, 2, 2)
plt.plot(perf_data['threads'], perf_data['speedup'], 'o-', color='green', linewidth=2)
plt.plot(perf_data['threads'], perf_data['threads'], '--', color='gray', linewidth=1, label='Линейное ускорение')
plt.xlabel('Количество потоков')
plt.ylabel('Ускорение (раз)')
plt.title('Ускорение параллельного PageRank')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig('page_rank_visualization/images/performance.png', dpi=300)
print("График производительности сохранен")

plt.figure(figsize=(12, 5))

plt.subplot(1, 2, 1)
top100 = ranks_data.head(100)
plt.bar(top100['rank'], top100['pagerank'], color='skyblue')
plt.xlabel('Ранг')
plt.ylabel('Значение PageRank')
plt.title('Топ-100 вершин по PageRank')
plt.grid(True, axis='y', alpha=0.3)

plt.subplot(1, 2, 2)
plt.loglog(ranks_data['rank'], ranks_data['pagerank'], 'o-', markersize=3)
plt.xlabel('Ранг (log scale)')
plt.ylabel('PageRank (log scale)')
plt.title('Степенной закон распределения PageRank')
plt.grid(True, which='both', alpha=0.3)

plt.tight_layout()
plt.savefig('page_rank_visualization/images/pagerank_distribution.png', dpi=300)
print("График распределения PageRank сохранен")

top_vertices = ranks_data.head(100)

plt.figure(figsize=(12, 6))
plt.scatter(top_vertices['vertex_id'], top_vertices['pagerank'], alpha=0.7, s=50)

clusters = [
    (3150, 3180, 'red', 'Кластер 3150-3180'),
    (18966, 18969, 'green', 'Кластер 18966-18969'),
    (0, 200, 'blue', 'Кластер 0-200')
]

for start, end, color, label in clusters:
    cluster = top_vertices[(top_vertices['vertex_id'] >= start) &
                           (top_vertices['vertex_id'] <= end)]
    if not cluster.empty:
        plt.scatter(cluster['vertex_id'], cluster['pagerank'],
                    color=color, s=100, label=label)

plt.xlabel('ID вершины')
plt.ylabel('Значение PageRank')
plt.title('Кластеры вершин с высоким PageRank')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig('page_rank_visualization/images/pagerank_clusters.png', dpi=300)
print("График кластеров сохранен")

plt.figure(figsize=(10, 6))
plt.hist(ranks_data['pagerank'], bins=50, alpha=0.7, color='purple')
plt.xlabel('Значение PageRank')
plt.ylabel('Количество вершин')
plt.title('Распределение значений PageRank (топ-1000 вершин)')
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig('page_rank_visualization/images/pagerank_histogram.png', dpi=300)
print("Гистограмма распределения сохранена")

plt.figure(figsize=(10, 6))
plt.semilogy(ranks_data['rank'], ranks_data['pagerank'])
plt.xlabel('Позиция в рейтинге')
plt.ylabel('PageRank (log scale)')
plt.title('Падение значения PageRank ("длинный хвост")')
plt.grid(True)
plt.tight_layout()
plt.savefig('page_rank_visualization/images/long_tail.png', dpi=300)
print("График 'длинного хвоста' PageRank сохранен")

top_avg = ranks_data.head(10)['pagerank'].mean()
bottom_avg = ranks_data.tail(10)['pagerank'].mean()
print(f"Среднее значение PageRank для топ-10: {top_avg:.8f}")
print(f"Среднее значение PageRank для последних 10: {bottom_avg:.8f}")
print(f"Соотношение топ/хвост: {top_avg/bottom_avg:.2f}x")

plt.figure(figsize=(10, 6))
bins = [0, 200, 1000, 5000, 10000, 50000, 100000]
labels = ['0-200', '201-1K', '1K-5K', '5K-10K', '10K-50K', '50K+']
top100_ids = top_vertices['vertex_id']
hist, _ = np.histogram(top100_ids, bins=bins)
plt.bar(labels, hist, color='teal')
plt.xlabel('Диапазон ID вершин')
plt.ylabel('Количество в топ-100')
plt.title('Распределение ID вершин в топ-100 PageRank')
plt.xticks(rotation=45)
plt.tight_layout()
plt.savefig('page_rank_visualization/images/vertex_id_distribution.png', dpi=300)
print("График распределения ID вершин сохранен")

print("Визуализация завершена. Результаты сохранены в директории ../page_rank_visualization/images/")
