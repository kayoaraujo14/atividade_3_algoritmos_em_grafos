import heapq

def dijkstra(num_vertices, edges, start, end):
    # Inicializar o grafo como uma lista de adjacências
    graph = [[] for _ in range(num_vertices)]
    for u, v, w in edges:
        graph[u].append((v, w))
        graph[v].append((u, w))  # Se o grafo for direcionado, remova esta linha

    # Inicializar as distâncias como infinito e o caminho anterior como None
    distances = [float('inf')] * num_vertices
    previous = [None] * num_vertices
    distances[start] = 0

    # Fila de prioridade (min-heap) com distâncias iniciais
    min_heap = [(0, start)]  # (distância, vértice)

    while min_heap:
        current_distance, u = heapq.heappop(min_heap)

        # Se chegamos ao vértice final, podemos parar
        if u == end:
            break

        # Ignorar nós que já possuem uma distância menor registrada
        if current_distance > distances[u]:
            continue

        # Relaxar as arestas do vértice atual
        for v, weight in graph[u]:
            distance = current_distance + weight

            # Se a nova distância é menor, atualize e adicione ao heap
            if distance < distances[v]:
                distances[v] = distance
                previous[v] = u
                heapq.heappush(min_heap, (distance, v))

    # Construir o caminho do vértice final ao inicial
    path = []
    u = end
    while u is not None:
        path.append(u)
        u = previous[u]
    path.reverse()

    # Retornar o caminho mínimo e o custo
    return path, distances[end] if distances[end] != float('inf') else None

# Entrada de exemplo
num_vertices = int(input("Número de vértices: "))
num_edges = int(input("Número de arestas: "))
edges = []
for _ in range(num_edges):
    u, v, w = map(int, input("Digite a aresta (formato: u v w): ").split())
    edges.append((u, v, w))

start = int(input("Vértice inicial: "))
end = int(input("Vértice final: "))

# Executa o algoritmo de Dijkstra e exibe o resultado
path, cost = dijkstra(num_vertices, edges, start, end)
if cost is not None:
    print(f"Caminho mínimo: {' -> '.join(map(str, path))}")
    print(f"Custo do caminho: {cost}")
else:
    print("Não existe caminho entre os vértices fornecidos.")
