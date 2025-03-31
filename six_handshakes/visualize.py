import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

os.makedirs('six_handshakes/images', exist_ok=True)

distance_data = pd.read_csv('six_handshakes/data/distance_distribution.csv')
cumulative_data = pd.read_csv('six_handshakes/data/cumulative_distribution.csv')

plt.figure(figsize=(12, 6))
plt.bar(distance_data['distance'], distance_data['percent'], color='skyblue')
plt.xlabel('Расстояние (количество рукопожатий)')
plt.ylabel('Процент пар пользователей')
plt.title('Распределение расстояний между пользователями Facebook')
plt.grid(True, axis='y', alpha=0.3)
plt.savefig('six_handshakes/images/distance_distribution.png', dpi=300)
print("График распределения расстояний сохранен")

plt.figure(figsize=(12, 6))
plt.plot(cumulative_data['distance'], cumulative_data['cumulative_percent'], 'o-', linewidth=2, color='green')
plt.axvline(x=6, color='red', linestyle='--', label='Граница "шести рукопожатий"')
plt.axhline(y=95.7, color='red', linestyle=':', label='95.7% пользователей')
plt.xlabel('Расстояние (количество рукопожатий)')
plt.ylabel('Кумулятивный процент')
plt.title('Кумулятивное распределение расстояний (теория шести рукопожатий)')
plt.grid(True)
plt.legend()
plt.savefig('six_handshakes/images/cumulative_distribution.png', dpi=300)
print("График кумулятивного распределения сохранен")

plt.figure(figsize=(12, 6))
plt.bar(distance_data['distance'], distance_data['count'], color='skyblue', alpha=0.7, label='Реальное распределение')

plt.text(0.7, 0.8,
         'Теория шести рукопожатий:\n'
         f'Пути длиной ≤ 6: 95.7%\n'
         f'Средняя длина пути: 5.13\n'
         f'Максимальная длина: 11',
         transform=plt.gca().transAxes,
         bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

plt.xlabel('Расстояние (количество рукопожатий)')
plt.ylabel('Количество пар пользователей')
plt.title('Распределение расстояний в социальном графе Facebook')
plt.grid(True, axis='y')
plt.legend()
plt.savefig('six_handshakes/images/six_degrees_analysis.png', dpi=300)
print("График анализа шести рукопожатий сохранен")

plt.figure(figsize=(10, 8))

significant_data = distance_data[distance_data['percent'] >= 1.0]

colors = ['#ff9999', '#66b3ff', '#99ff99', '#ffcc99']

labels = [f'{d} шагов: {p:.1f}%' for d, p in zip(significant_data['distance'], significant_data['percent'])]

plt.pie(significant_data['percent'],
        labels=None,
        colors=colors,
        autopct='%1.1f%%',
        shadow=False,
        startangle=90,
        wedgeprops={'linewidth': 1, 'edgecolor': 'white'})

plt.legend(labels, loc='upper left', bbox_to_anchor=(1, 1))
plt.axis('equal')
plt.title('Распределение длин путей в социальном графе Facebook')
plt.tight_layout()
plt.savefig('six_handshakes/images/distance_pie_chart_simple.png', dpi=300)
print("Создана упрощенная круговая диаграмма")

plt.figure(figsize=(10, 6))

significant = distance_data[distance_data['percent'] >= 1.0]
significant = significant.sort_values('percent', ascending=True)

plt.barh(significant['distance'].astype(str) + ' шагов',
         significant['percent'],
         color=['#ff9999', '#66b3ff', '#99ff99', '#ffcc99'])

for i, v in enumerate(significant['percent']):
    plt.text(v + 0.5, i, f'{v:.1f}%', va='center')

plt.xlabel('Процент пар пользователей')
plt.ylabel('Длина пути')
plt.title('Распределение длин путей в социальном графе Facebook')
plt.grid(axis='x', linestyle='--', alpha=0.7)
plt.tight_layout()
plt.savefig('six_handshakes/images/distance_bar_horizontal.png', dpi=300)
print("Создана горизонтальная гистограмма")

# plt.figure(figsize=(10, 8))
# labels = [f'{d} шагов' for d in distance_data['distance']]
# sizes = distance_data['percent']
# explode = [0.1 if d == 5 else 0 for d in distance_data['distance']]
#
# plt.pie(sizes, explode=explode, labels=labels, autopct='%1.1f%%',
#         shadow=True, startangle=90, colors=plt.cm.viridis(np.linspace(0, 1, len(sizes))))
# plt.axis('equal')
# plt.title('Процентное распределение расстояний между пользователями')
# plt.savefig('six_handshakes/images/distance_pie_chart.png', dpi=300)
# print("Круговая диаграмма распределения сохранена")

print("Визуализация завершена. Результаты сохранены в директории six_handshakes/images/")
