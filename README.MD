# Graph Library C++

Масштабируемая и высокопроизводительная библиотека для анализа и обработки графов на C++ с поддержкой многопоточных вычислений.

## Содержание
- [Возможности](#возможности)
- [Установка](#установка)
- [Пример использования](#пример-использования)
- [Структура директорий](#структура-директорий)
- [Начало работы](#начало-работы)
- [Вклад в проект](#вклад-в-проект)
- [Полная документация](#полная-документация)

## Возможности
- Множество форматов представления графов (список смежности, матрица смежности, список рёбер, CSR)
- Поддержка как ориентированных, так и неориентированных графов
- Обширная коллекция алгоритмов для работы с графами:
    - Алгоритмы обхода (DFS, BFS)
    - Алгоритмы поиска кратчайшего пути (Дейкстра, A*, Беллман-Форд, Флойд-Уоршелл)
    - Алгоритмы минимального остовного дерева (Крускал, Прим)
    - Алгоритмы потоков в сетях (Форд-Фалкерсон, Эдмондс-Карп, Динец)
    - Алгоритмы паросочетаний (Кун, Хопкрофт-Карп, Эдмондс)
    - Анализ компонент, определение циклов, точек сочленения, мостов и т.д.
- Параллельные реализации ключевых алгоритмов (BFS, Дейкстра, PageRank)
- Визуализация графов через формат DOT и интеграцию с Graphviz

## Установка

### Требования
- Компилятор с поддержкой C++20
- CMake (версия 3.10 или выше)
- GoogleTest (для запуска тестов)
- Graphviz (для функций визуализации)

### Сборка библиотеки
```bash
mkdir build && cd build
cmake ..
make
```

## Пример использования

Ниже приведён пример создания простого графа и выполнения обхода в ширину (BFS):

```cpp
#include "graph/structure/directed_adjacency_list.hpp"
#include "graph/algorithms/bfs/breadth_first_search.hpp"
#include <iostream>

int main() {
    using Graph = graph::DirectedAdjacencyList<int, int>;
    Graph myGraph;
    
    // Добавляем вершины и ребра
    myGraph.addVertex(1).addVertex(2).addVertex(3);
    myGraph.addEdge(1, 2, 1).addEdge(2, 3, 1);
    
    // Создаём экземпляр BFS и выполняем обход графа
    graph::algorithms::BreadthFirstSearch bfs(myGraph);
    bfs.traverse(1, [](int v) {
        std::cout << "Посещена вершина: " << v << std::endl;
    });
    
    return 0;
}
```

## Структура директорий

- `include/graph/`: Основной код библиотеки
    - `structure/`: Структуры данных для графов
    - `algorithms/`: Алгоритмы для работы с графами
        - `bfs/`: Алгоритмы поиска в ширину
        - `dfs/`: Алгоритмы поиска в глубину
        - `dijkstra/`: Алгоритмы поиска кратчайшего пути
        - `mst/`: Алгоритмы минимального остовного дерева
        - `flow/`: Алгоритмы сетевых потоков
        - `matching/`: Алгоритмы паросочетаний в графах
        - `floyd/`: Алгоритм Флойда-Уоршелла и связанный код
        - `bellman_ford/`: Алгоритм Беллмана-Форда
    - `parallel_algorithms/`: Параллельные реализации
        - `bfs/`: Параллельный BFS
        - `dijkstra/`: Параллельный алгоритм Дейкстры
        - `page_rank/`: Параллельный PageRank
    - `visualization/`: Инструменты визуализации графов
- `tests/`: Тестовые файлы
    - `tests_structure/`: Тесты для структур графов
    - `tests_dfs/`: Тесты для алгоритмов DFS
    - и т.д.

## Начало работы

1. Включите необходимые заголовочные файлы для нужных вам структур графов и алгоритмов.
2. Создайте экземпляр графа и добавьте вершины и рёбра.
3. Создайте объект алгоритма с вашим графом.
4. Вычислите результат и получите данные.

## Вклад в проект

Если у вас есть предложения или улучшения, пожалуйста, создавайте issues или pull requests.

## Полная документация

Здесь представлена подробная документация по всем компонентам библиотеки.

### Содержание
1. [Структуры графов](#1-структуры-графов)
2. [Основные операции с графами](#2-основные-операции-с-графами)
3. [Алгоритмы поиска](#3-алгоритмы-поиска)
4. [Алгоритмы поиска путей](#4-алгоритмы-поиска-путей)
5. [Алгоритмы минимального остовного дерева](#5-алгоритмы-минимального-остовного-дерева)
6. [Алгоритмы потоков](#6-алгоритмы-потоков)
7. [Алгоритмы паросочетаний](#7-алгоритмы-паросочетаний)
8. [Анализ компонент](#8-анализ-компонент)
9. [Параллельные алгоритмы](#9-параллельные-алгоритмы)
10. [Визуализация](#10-визуализация)

### 1. Структуры графов

Библиотека предоставляет несколько представлений графов, каждое со своими характеристиками производительности:

#### 1.1 Список смежности
```cpp
// Ориентированный граф с вершинами типа int и весами типа double
graph::DirectedAdjacencyList<int, double> dirGraph;

// Неориентированный граф с вершинами типа string и весами типа int
graph::UndirectedAdjacencyList<std::string, int> undirGraph;
```

- **Преимущества**: Эффективен для разреженных графов, быстрый обход окрестностей вершин
- **Сложность**: O(1) для добавления вершины, O(1) для добавления ребра, O(degree(v)) для поиска соседей
- **Заголовок**: `include/graph/structure/directed_adjacency_list.hpp` или `include/graph/structure/undirected_adjacency_list.hpp`

#### 1.2 Матрица смежности
```cpp
// Создание ориентированной матрицы смежности с вершинами типа int и весами типа double
graph::DirectedAdjacencyMatrix<int, double> dirMatrix(100); // Максимум 100 вершин

// Неориентированная матрица смежности
graph::UndirectedAdjacencyMatrix<int, int> undirMatrix(50);
```

- **Преимущества**: O(1) для поиска веса ребра, эффективна для плотных графов
- **Недостатки**: O(V²) сложность по памяти
- **Заголовок**: `include/graph/structure/directed_adjacency_matrix.hpp` или `include/graph/structure/undirected_adjacency_matrix.hpp`

#### 1.3 Список рёбер
```cpp
// Ориентированный список рёбер
graph::DirectedEdgeList<int, double> dirEdgeList;

// Неориентированный список рёбер
graph::UndirectedEdgeList<int, double> undirEdgeList;
```

- **Преимущества**: Простое представление, эффективно для алгоритмов, обрабатывающих все рёбра
- **Недостатки**: Медленный поиск конкретных рёбер или вершин
- **Заголовок**: `include/graph/structure/directed_edge_list.hpp` или `include/graph/structure/undirected_edge_list.hpp`

#### 1.4 CSR граф (Compressed Sparse Row)
```cpp
// Создание CSR графа
graph::structure::CSRGraph<int, double> csrGraph;

// Или создание из другого графа
graph::structure::CSRGraph<int, double> csrGraph(existingGraph);

// Использование строителя для большего контроля
graph::structure::CSRGraphBuilder<int, double> builder;
builder.addVertex(1).addVertex(2).addVertex(3);
builder.addEdge(1, 2, 5.0).addEdge(2, 3, 3.0);
auto csrGraph = builder.build();
```

- **Преимущества**: Эффективен по памяти, учитывает кэш, отлично подходит для параллельных алгоритмов
- **Недостатки**: Статическая структура, сложно модифицировать после создания
- **Заголовок**: `include/graph/structure/csr_graph.hpp` и `include/graph/structure/csr_graph_builder.hpp`

#### 1.5 Мультиграф
```cpp
graph::MultiGraph<int, double> multigraph;
multigraph.addVertex(1).addVertex(2);
multigraph.addEdge(1, 2, 5.0); // Первое ребро между 1 и 2
multigraph.addEdge(1, 2, 3.0); // Второе ребро между 1 и 2
```

- Поддерживает множественные рёбра между одной парой вершин
- **Заголовок**: `include/graph/structure/multigraph.hpp`

### 2. Основные операции с графами

Все структуры графов поддерживают общий набор операций:

#### 2.1 Добавление и удаление вершин
```cpp
graph.addVertex(1);            // Добавить вершину 1
graph.addVertex("Node2");      // Добавить вершину "Node2" (если используем строковый тип)
graph.removeVertex(1);         // Удалить вершину 1 и все рёбра, связанные с ней
```

#### 2.2 Добавление и удаление рёбер
```cpp
graph.addEdge(1, 2, 5.0);      // Добавить ребро от 1 к 2 с весом 5.0
graph.removeEdge(1, 2);        // Удалить ребро от 1 к 2
```

#### 2.3 Запросы к графу
```cpp
bool hasVertex = graph.hasVertex(1);             // Проверка существования вершины
bool hasEdge = graph.hasEdge(1, 2);              // Проверка существования ребра
std::size_t numVertices = graph.vertexCount();   // Получение числа вершин
std::size_t numEdges = graph.edgeCount();        // Получение числа рёбер
double weight = graph.getEdgeWeight(1, 2);       // Получение веса ребра
```

#### 2.4 Обход графа
```cpp
std::vector<VertexType> vertices = graph.getVertices();  // Получение всех вершин
std::vector<VertexType> neighbors = graph.getNeighbors(1);  // Получение соседей вершины 1
```

#### 2.5 Преобразование между типами графов
```cpp
// Преобразование из списка смежности в список рёбер
auto dirEdgeList = graph::structure::GraphConverter::toDirectedEdgeList(dirAdjList);
auto undirEdgeList = graph::structure::GraphConverter::toUndirectedEdgeList(undirAdjList);

// Преобразование в матрицу смежности
auto dirAdjMatrix = graph::structure::GraphConverter::toDirectedAdjacencyMatrix(dirAdjList, 100);
auto undirAdjMatrix = graph::structure::GraphConverter::toUndirectedAdjacencyMatrix(undirAdjList, 100);
```

### 3. Алгоритмы поиска

#### 3.1 Поиск в глубину (DFS)
```cpp
graph::algorithms::DepthFirstSearch<GraphType> dfs(graph);
dfs.run(startVertex);  // Запуск DFS из стартовой вершины

// Доступ к результатам
auto visitOrder = dfs.getVisitOrder();  // Получить вершины в порядке посещения
auto visited = dfs.getVisited();        // Получить карту посещённых вершин

// Запуск DFS на всех вершинах (полезно для несвязных графов)
dfs.runOnAll();

// Установка пользовательского обработчика вершин
dfs.setVertexHandler([](auto vertex) { 
    std::cout << "Посещена: " << vertex << std::endl; 
});
```

#### 3.2 Поиск в ширину (BFS)
```cpp
graph::algorithms::BreadthFirstSearch<GraphType> bfs(graph);
bfs.traverse(
    startVertex,
    [](const auto& v) { std::cout << "Старт: " << v << std::endl; },  // on_start
    [](const auto& v) { std::cout << "Обнаружена: " << v << std::endl; },  // on_discover
    [](const auto& u, const auto& v) { std::cout << "Ребро: " << u << "->" << v << std::endl; },  // on_edge
    []() { std::cout << "Завершено" << std::endl; }  // on_finish
);

// Доступ к результатам
auto visited = bfs.getVisited();    // Карта посещённых вершин
auto distance = bfs.getDistance();  // Расстояния от стартовой вершины
auto parent = bfs.getParent();      // Родительские вершины в BFS-дереве

// Запуск на всём графе
bfs.traverseAll();
```

#### 3.3 Обход по уровням
```cpp
graph::algorithms::LevelOrderTraversal<GraphType> levelOrder(graph);
levelOrder.traverse(startVertex);

// Доступ к результатам по уровням
auto order = levelOrder.getOrder();     // Вершины в порядке обхода по уровням
auto distance = levelOrder.getDistance(); // Расстояние (уровень) каждой вершины
auto parent = levelOrder.getParent();     // Родитель в дереве обхода

// Получение пути к целевой вершине
auto path = levelOrder.retrievePath(targetVertex);
```

### 4. Алгоритмы поиска путей

#### 4.1 Алгоритм Дейкстры
```cpp
graph::algorithms::Dijkstra<GraphType> dijkstra(graph);
dijkstra.compute(sourceVertex);

// Получение кратчайшего пути
auto path = dijkstra.retrievePath(targetVertex);
// Получение расстояния до целевой вершины
auto distance = dijkstra.getDistance(targetVertex);
// Получение всех расстояний
auto allDistances = dijkstra.getAllDistances();
// Получение родительских указателей
auto parents = dijkstra.getParents();
```

#### 4.2 Алгоритм A*
```cpp
// Определение эвристической функции
auto heuristic = [](const Vertex& a, const Vertex& b) -> double {
    // Пример: Евклидово расстояние между точками
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
};

graph::algorithms::AStar<GraphType> astar(graph, heuristic);
astar.compute(sourceVertex, targetVertex);

// Получение кратчайшего пути
auto path = astar.retrievePath(targetVertex);
// Получение расстояния до целевой вершины
auto distance = astar.getDistance(targetVertex);
```

#### 4.3 Алгоритм Беллмана-Форда
```cpp
graph::algorithms::BellmanFord<GraphType> bellmanFord(graph);
bellmanFord.compute(sourceVertex);

// Проверка наличия отрицательных циклов
bool hasNegativeCycle = bellmanFord.hasNegativeCycle();

// Если нет отрицательных циклов:
if (!hasNegativeCycle) {
    auto path = bellmanFord.retrievePath(targetVertex);
    auto distance = bellmanFord.getDistance(targetVertex);
}
```

#### 4.4 Алгоритм Флойда-Уоршелла
```cpp
graph::algorithms::FloydWarshall<GraphType> floydWarshall(graph);
floydWarshall.compute();

// Проверка наличия отрицательных циклов
bool hasNegativeCycle = floydWarshall.hasNegativeCycle();

// Получение расстояния между любой парой вершин
auto distance = floydWarshall.getDistance(from, to);

// Получение пути между вершинами
auto path = floydWarshall.retrievePath(from, to);
```

#### 4.5 Метрики графа (с использованием Флойда-Уоршелла)
```cpp
graph::algorithms::GraphMetrics<GraphType> metrics(graph);

// Получение свойств графа
auto eccentricity = metrics.eccentricity(vertex);  // Максимальное расстояние от вершины
auto diameter = metrics.diameter();                // Максимальное расстояние в графе
auto radius = metrics.radius();                    // Минимальный эксцентриситет
auto centers = metrics.center();                   // Вершины с минимальным эксцентриситетом
```

### 5. Алгоритмы минимального остовного дерева

#### 5.1 Алгоритм Крускала
```cpp
graph::algorithms::Kruskal<GraphType> kruskal(graph);
kruskal.compute();

// Проверка связности графа
bool isConnected = kruskal.isConnected();

if (isConnected) {
    // Получение общего веса MST
    auto totalWeight = kruskal.getTotalWeight();
    
    // Получение рёбер MST
    auto mstEdges = kruskal.getMSTEdges();
    for (const auto& edge : mstEdges) {
        std::cout << edge.from << " -- " << edge.to << " : " << edge.weight << std::endl;
    }
}
```

#### 5.2 Алгоритм Прима
```cpp
graph::algorithms::Prim<GraphType> prim(graph);
prim.compute(startVertex);  // Опционально: указать стартовую вершину

// Проверка связности графа
bool isConnected = prim.isConnected();

if (isConnected) {
    // Получение общего веса MST
    auto totalWeight = prim.getTotalWeight();
    
    // Получение рёбер MST
    auto mstEdges = prim.getMSTEdges();
    for (const auto& edge : mstEdges) {
        std::cout << edge.from << " -- " << edge.to << " : " << edge.weight << std::endl;
    }
}
```

### 6. Алгоритмы потоков

#### 6.1 Алгоритм Форда-Фалкерсона
```cpp
graph::algorithms::flow::FordFulkerson<GraphType, VertexType, WeightType> ff(graph, source, sink);
ff.compute();

// Получение максимального потока
auto maxFlow = ff.getMaxFlow();

// Получение потока по конкретному ребру
auto edgeFlow = ff.getFlow(u, v);

// Получение всех рёбер с положительным потоком
auto flowEdges = ff.getFlowEdges();
for (const auto& [from, to, flow] : flowEdges) {
    std::cout << from << " -> " << to << " : " << flow << std::endl;
}
```

#### 6.2 Алгоритм Эдмондса-Карпа
```cpp
graph::algorithms::flow::EdmondsKarp<GraphType, VertexType, WeightType> ek(graph, source, sink);
ek.compute();

auto maxFlow = ek.getMaxFlow();
```

#### 6.3 Алгоритм Диница
```cpp
graph::algorithms::flow::Dinic<GraphType, VertexType, WeightType> dinic(graph, source, sink);
dinic.compute();

auto maxFlow = dinic.getMaxFlow();
```

### 7. Алгоритмы паросочетаний

#### 7.1 Алгоритм Куна (двудольное паросочетание)
```cpp
// Автоматическое определение двудольности
graph::algorithms::matching::KuhnMatching<GraphType, VertexType> kuhn(graph);

// Или явное указание левой доли
graph::algorithms::matching::KuhnMatching<GraphType, VertexType> kuhn(graph, leftPart);

kuhn.compute();

// Получение размера паросочетания
auto matchingSize = kuhn.getMatchingSize();

// Получение рёбер паросочетания
auto matching = kuhn.getMatching();
for (const auto& [left, right] : matching) {
    std::cout << left << " сопоставлено с " << right << std::endl;
}

// Проверка, входит ли вершина в паросочетание
bool isMatched = kuhn.isVertexMatched(vertex);

// Получение сопоставленной вершины для данной вершины
auto matchedVertex = kuhn.getMatchedVertex(vertex);
```

#### 7.2 Алгоритм Хопкрофта-Карпа (двудольное паросочетание)
```cpp
graph::algorithms::matching::HopcroftKarpMatching<GraphType, VertexType> hk(graph);
hk.compute();

auto matchingSize = hk.getMatchingSize();
auto matching = hk.getMatching();
```

#### 7.3 Алгоритм Эдмондса (общее паросочетание)
```cpp
graph::algorithms::matching::EdmondsMatching<GraphType, VertexType> edmonds(graph);
edmonds.compute();

auto matchingSize = edmonds.getMatchingSize();
auto matching = edmonds.getMatching();
```

### 8. Анализ компонент

#### 8.1 Компоненты связности (DFS)
```cpp
graph::algorithms::DFSConnectedComponents<GraphType> cc(graph);

// Получение всех компонент
auto components = cc.findAll();
std::cout << "Количество компонент: " << components.size() << std::endl;

// Поиск компоненты, содержащей определённую вершину
auto component = cc.findOne(startVertex);
```

#### 8.2 Компоненты связности (BFS)
```cpp
graph::algorithms::BFSConnectedComponents<GraphType> cc(graph);
auto components = cc.computeComponents();
```

#### 8.3 Определение циклов
```cpp
graph::algorithms::CycleDetection<GraphType> cycleDetector(graph);

bool hasCycle = cycleDetector.hasCycle();
if (hasCycle) {
    auto cycle = cycleDetector.getCycle();
    // cycle[0] == cycle[cycle.size()-1], если цикл найден
}
```

#### 8.4 Проверка двудольности
```cpp
graph::algorithms::BipartiteChecker<GraphType> bipartiteChecker(graph);
bool isBipartite = bipartiteChecker.isBipartite();

if (isBipartite) {
    auto [partA, partB] = bipartiteChecker.getPartition();
}
```

#### 8.5 Поиск мостов
```cpp
graph::algorithms::Bridges<GraphType> bridgeFinder(graph);
bridgeFinder.findBridges();

auto bridges = bridgeFinder.getBridges();
for (const auto& [u, v] : bridges) {
    std::cout << "Мост: " << u << " -- " << v << std::endl;
}
```

#### 8.6 Точки сочленения
```cpp
graph::algorithms::ArticulationPoints<GraphType> apFinder(graph);
apFinder.findArticulationPoints();

auto articulationPoints = apFinder.getArticulationPoints();
for (const auto& vertex : articulationPoints) {
    std::cout << "Точка сочленения: " << vertex << std::endl;
}
```

#### 8.7 Компоненты сильной связности (конденсация графа)
```cpp
graph::algorithms::GraphCondensation<GraphType> condensation(graph);
condensation.buildCondensation();

// Получение компонент сильной связности
auto components = condensation.getCondensedVertices();

// Получение ID компоненты для вершины
int componentId = condensation.getComponentId(vertex);

// Получение рёбер между компонентами
auto condensedEdges = condensation.getCondensedEdges();
```

#### 8.8 Топологическая сортировка
```cpp
graph::algorithms::TopologicalSort<GraphType> topSort(graph);
bool isDAG = topSort.sort();  // Возвращает false, если граф имеет циклы

if (isDAG) {
    auto sortedVertices = topSort.getSortedVertices();
    for (const auto& v : sortedVertices) {
        std::cout << v << " ";
    }
    std::cout << std::endl;
}
```

#### 8.9 Транзитивное замыкание
```cpp
graph::algorithms::TransitiveClosure<GraphType> tc(graph);
tc.compute();

// Проверка достижимости одной вершины из другой
bool isReachable = tc.isReachable(from, to);

// Получение полной матрицы достижимости
auto reachabilityMatrix = tc.getReachabilityMatrix();
```

#### 8.10 Определение отрицательных циклов
```cpp
graph::algorithms::NegativeCycleDetector<GraphType> ncDetector(graph);

bool hasNegativeCycle = ncDetector.hasNegativeCycle();
if (hasNegativeCycle) {
    auto cycle = ncDetector.findNegativeCycle();
}
```

### 9. Параллельные алгоритмы

#### 9.1 Параллельный поиск в ширину
```cpp
graph::parallel_algorithms::ParallelBreadthFirstSearch<CSRGraph> pbfs(csrGraph, numThreads);
pbfs.traverse(
    startVertex,
    [](const auto& v) { /* on_start */ },
    [](const auto& v) { /* on_discover */ },
    [](const auto& u, const auto& v) { /* on_edge */ },
    []() { /* on_finish */ }
);

auto visited = pbfs.getVisited();
auto distance = pbfs.getDistance();
auto parent = pbfs.getParent();
```

#### 9.2 Параллельные компоненты связности
```cpp
graph::parallel_algorithms::ParallelConnectedComponents<CSRGraph> pcc(csrGraph, numThreads);
auto components = pcc.computeComponents();
```

#### 9.3 Параллельный алгоритм Дейкстры
```cpp
graph::parallel_algorithms::ParallelDijkstra<CSRGraph, double> pdijkstra(csrGraph, numThreads);
pdijkstra.findShortestPaths(sourceVertex);

auto distances = pdijkstra.getDistances();
auto predecessors = pdijkstra.getPredecessors();
```

#### 9.4 Параллельный PageRank
```cpp
// Базовый параллельный PageRank
graph::parallel_algorithms::ParallelPageRank<GraphType> ppr(graph, numThreads);
auto ranks = ppr.compute(0.85, 1e-6, 100);

// Оптимизированный параллельный PageRank для CSR графов
graph::parallel_algorithms::ParallelPageRankOptimized<CSRGraph> pprOpt(csrGraph, numThreads);
auto optimizedRanks = pprOpt.compute(0.85, 1e-6, 100);
```

### 10. Визуализация

#### 10.1 Базовая визуализация графов
```cpp
// Экспорт графа в формат DOT
std::string dot = graph::visualization::GraphExporter<GraphType>::exportToDOT(graph);

// Сохранение в DOT-файл
graph::visualization::GraphExporter<GraphType>::saveToDOTFile(dot, "graph.dot");

// Рендеринг DOT в изображение (требуется Graphviz)
bool success = graph::visualization::GraphExporter<GraphType>::renderDOTToImage(
    "graph.dot", "graph.png", "png");
```

#### 10.2 Визуализация мультиграфа
```cpp
// Экспорт мультиграфа в формат DOT
std::string dot = graph::visualization::MultiGraphExporter<VertexType, WeightType>::exportToDOT(multigraph);

// Сохранение и рендеринг как выше
```

#### 10.3 Визуализация MST
```cpp
// Подсветка рёбер MST на исходном графе
std::string dot = exportWithHighlightedEdges(graph, mstEdges);

// Сравнение двух MST
std::string dot = exportWithTwoMSTs(graph, mstEdges1, mstEdges2);
```
