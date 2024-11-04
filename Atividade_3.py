import heapq

def dijkstra(num_vertices, edges, start, end):
    graph = [[] for _ in range(num_vertices)]
    for u, v, w in edges:
        graph[u].append((v, w))
        graph[v].append((u, w))

    distances = [float('inf')] * num_vertices
    previous = [None] * num_vertices
    distances[start] = 0

    min_heap = [(0, start)]

    while min_heap:
        current_distance, u = heapq.heappop(min_heap)
        if u == end:
            break
        if current_distance > distances[u]:
            continue

        for v, weight in graph[u]:
            distance = current_distance + weight
            if distance < distances[v]:
                distances[v] = distance
                previous[v] = u
                heapq.heappush(min_heap, (distance, v))

    path = []
    u = end
    while u is not None:
        path.append(u)
        u = previous[u]
    path.reverse()

    return path, distances[end] if distances[end] != float('inf') else None

num_vertices = int(input("Número de vértices: "))
num_edges = int(input("Número de arestas: "))
edges = []
for _ in range(num_edges):
    u, v, w = map(int, input("Digite a aresta (formato: u v w): ").split())
    edges.append((u, v, w))

start = int(input("Vértice inicial: "))
end = int(input("Vértice final: "))

path, cost = dijkstra(num_vertices, edges, start, end)
if cost is not None:
    print(f"Caminho mínimo: {' -> '.join(map(str, path))}")
    print(f"Custo do caminho: {cost}")
else:
    print("Não existe caminho entre os vértices fornecidos.")
