import heapq
from collections import deque
#Classe Grafo com as funções de verificação de propriedades de grafos
class Grafo:   
    def __init__(self, vertices, arestas, direcionado):
        self.vertices = vertices
        self.direcionado = direcionado
        self.adj_list = {i: [] for i in range(vertices)}
        self.weight = {} 
        self.id = {}
        self.arestas = arestas

        for id, u, v, p in arestas: # u -> v com peso p
            self.adj_list[u].append(v)
            self.weight[(u, v)] = p
            self.id[(u, v)] = id

            if not direcionado: # Se o grafo não for direcionado, adicione a aresta v -> u
                self.adj_list[v].append(u)
                self.weight[(u, v)] = p
                self.id[(u, v)] = id

import networkx as nx
import matplotlib.pyplot as plt

def desenhar_grafo(grafo):
    """
    Função para desenhar o grafo utilizando a biblioteca NetworkX.
    Se o grafo for direcionado, desenha setas; se houver pesos nas arestas, eles são exibidos.
    """
    G = nx.DiGraph() if grafo.direcionado else nx.Graph()

    for u in grafo.adj_list:
        for v in grafo.adj_list[u]:
            G.add_edge(u, v, weight=grafo.weight.get((u, v), 1))

    pos = nx.spring_layout(G)
    labels = nx.get_edge_attributes(G, 'weight')
    nx.draw(G, pos, with_labels=True, node_color='skyblue', node_size=700, font_size=12, font_weight='bold', arrows=grafo.direcionado)
    nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)
    plt.show()


def is_conexo(self):
    # Verifica se o grafo é conexo em grafos não direcionados utilizando o algoritmo de busca em profundidade
    if self.direcionado:
        return -1
    visited = [False] * self.vertices

    def dfs(v): # Função de busca em profundidade
        visited[v] = True
        for neighbor in self.adj_list[v]:
            if not visited[neighbor]:
                dfs(neighbor)

    dfs(0)
    return 1 if all(visited) else 0 # Retorna 1 se todos os vértices foram visitados, caso contrário, retorna 0

def is_conectividade_fraca(self): 
    # Verifica a conectividade fraca para grafos direcionados utilizando o algoritmo de busca em profundidade
    visited = [False] * self.vertices

    def dfs(v): # Função de busca em profundidade
        visited[v] = True
        for neighbor in self.adj_list[v]:
            if not visited[neighbor]:
                dfs(neighbor)

    dfs(0)
    return 1 if all(visited) else 0 # Retorna 1 se todos os vértices foram visitados, caso contrário, retorna 0
    
def is_bipartido(self): 
    # Verifica se o grafo é bipartido para grafos não direcionados utilizando o algoritmo de busca em profundidade
    if self.direcionado:
        return 0  # Grafos direcionados não são considerados bipartidos aqui

    color = [-1] * self.vertices

    def dfs(v, c): # Função de busca em profundidade
        color[v] = c
        for neighbor in self.adj_list[v]:
            if color[neighbor] == -1:
                if not dfs(neighbor, 1 - c):
                    return False
            elif color[neighbor] == c:
                return False
        return True

    for v in range(self.vertices): # Verifica se o grafo é bipartido
        if color[v] == -1:
            if not dfs(v, 0):
                return 0
    return 1 # Retorna 1 se o grafo for bipartido, caso contrário, retorna 0

def is_euleriano(self):
    # Verifica se o grafo é euleriano em grafos não direcionados utilizando o grau dos vértices (teorema de Euler)
    if self.direcionado:
        return 0  # Grafos direcionados não são considerados Eulerianos aqui

    if not is_conexo(self): # Verifica se o grafo é conexo
        return 0

    for v in range(self.vertices): # Verifica se todos os vértices têm grau par
        if len(self.adj_list[v]) % 2 != 0:
            return 0

    return 1

def has_ciclo(self):
    # Verifica se o grafo tem um ciclo em grafos não direcionados utilizando o algoritmo de busca em profundidade
    visited = [False] * self.vertices

    def dfs(v, parent): # Função de busca em profundidade
        visited[v] = True

        for neighbor in self.adj_list[v]: # Verifica se o vértice tem um ciclo
            if not visited[neighbor]: # Se o vizinho não foi visitado, chame a função recursivamente
                if dfs(neighbor, v):
                    return True
            elif neighbor!=parent: # Se o vizinho foi visitado e não é o pai do vértice atual, há um ciclo
                return True

        return False # Retorna False se não houver ciclo

    for v in range(self.vertices): # Verifica se o grafo tem um ciclo
        if not visited[v]:
            if dfs(v, -1):
                return 1 # Retorna 1 se houver um ciclo

    return 0 # Retorna 0 se não houver ciclo

def count_componentes_conexas(self):
    # Conta o número de componentes conexas em grafos não direcionados utilizando o algoritmo de busca em profundidade
    if self.direcionado: # Se o grafo for direcionado, retorne -1
        return -1
    visited = [False] * self.vertices
    count = 0

    def dfs(v): # Função de busca em profundidade
        visited[v] = True
        for neighbor in self.adj_list[v]: 
            if not visited[neighbor]:
                dfs(neighbor)

    for v in range(self.vertices): # Conta o número de componentes conexas
        if not visited[v]:
            dfs(v)
            count += 1

    return count # Retorna o número de componentes conexas

def count_componentes_fortemente_conexas(self):
    # Conta o número de componentes fortemente conectadas em grafos direcionados utilizando o algoritmo de Kosaraju
    if not self.direcionado: # Se o grafo não for direcionado, retorne -1
        return -1

    def dfs(v, visited, adj_list, stack=None): # Função de busca em profundidade
        visited[v] = True
        for neighbor in adj_list[v]:
            if not visited[neighbor]:
                dfs(neighbor, visited, adj_list, stack)
        if stack is not None:
            stack.append(v)

    def transpose_graph(): # Transpõe o grafo para encontrar o grafo transposto
        transposed = [[] for _ in range(self.vertices)]
        for v in range(self.vertices): 
            for neighbor in self.adj_list[v]:
                transposed[neighbor].append(v)
        return transposed

    visited = [False] * self.vertices
    stack = []
    for v in range(self.vertices): # Encontra a ordem de finalização dos vértices
        if not visited[v]:
            dfs(v, visited, self.adj_list, stack)

    transposed_graph = transpose_graph()

    visited = [False] * self.vertices
    count = 0
    while stack: # Conta o número de componentes fortemente conectadas
        v = stack.pop()
        if not visited[v]:
            dfs(v, visited, transposed_graph)
            count += 1

    return count # Retorna o número de componentes fortemente conectadas

def articulacao(self):
    # Retorna os pontos de articulação em grafos não direcionados utilizando Tarjan 
    if self.direcionado: # Se o grafo for direcionado, retorne -1
        return -1

    visited = [False] * self.vertices 
    articulation_points = []
    articulation_points.append(0)

    def dfs(v, parent, visited, low, disc, articulation_points): # Função de busca em profundidade
        children = 0
        visited[v] = True
        disc[v] = self.time
        low[v] = self.time
        self.time += 1

        for neighbor in self.adj_list[v]: # Verifica se o vértice é um ponto de articulação
            if not visited[neighbor]: # Se o vizinho não foi visitado, chame a função recursivamente
                children += 1
                parent[neighbor] = v
                dfs(neighbor, parent, visited, low, disc, articulation_points)
                low[v] = min(low[v], low[neighbor])
                if parent[v] == -1 and children > 1: # Se o vértice é a raiz da árvore DFS e tem mais de um filho, é um ponto de articulação
                    articulation_points.append(v)
                elif parent[v] != -1 and low[neighbor] >= disc[v]: # Se o vértice não é a raiz da árvore DFS e o valor de low do vizinho é maior ou igual ao valor de descoberta do vértice atual, é um ponto de articulação
                    articulation_points.append(v)
            elif neighbor != parent[v]: # Se o vizinho foi visitado e não é o pai do vértice atual, atualize o valor de low
                low[v] = min(low[v], disc[neighbor])

    parent = [-1] * self.vertices
    visited = [False] * self.vertices
    low = [float('inf')] * self.vertices
    disc = [float('inf')] * self.vertices
    self.time = 0

    for v in range(self.vertices): # Encontra os pontos de articulação
        if not visited[v]:
            dfs(v, parent, visited, low, disc, articulation_points)

    return articulation_points # Retorna os pontos de articulação

def count_arestas_ponte(self):
    # Conta o número de arestas ponte em grafos não direcionados
    if self.direcionado:
        return -1
    count = 0

    def dfs(v, parent, visited, low, disc): # Função de busca em profundidade
        nonlocal count
        visited[v] = True
        disc[v] = self.time
        low[v] = self.time
        self.time += 1

        for neighbor in self.adj_list[v]: # Verifica se a aresta é uma aresta ponte
            if not visited[neighbor]: # Se o vizinho não foi visitado, chame a função recursivamente
                parent[neighbor] = v
                dfs(neighbor, parent, visited, low, disc)
                low[v] = min(low[v], low[neighbor])
                if low[neighbor] > disc[v]: # Se o valor de low do vizinho é maior que o valor de descoberta do vértice atual, a aresta é uma aresta ponte
                    count += 1
            elif neighbor != parent[v]: # Se o vizinho foi visitado e não é o pai do vértice atual, atualize o valor de low
                low[v] = min(low[v], disc[neighbor])

    parent = [-1] * self.vertices
    visited = [False] * self.vertices
    low = [float('inf')] * self.vertices
    disc = [float('inf')] * self.vertices
    self.time = 0

    for v in range(self.vertices): # Conta o número de arestas ponte
        if not visited[v]:
            dfs(v, parent, visited, low, disc)

    return count # Retorna o número de arestas ponte

def dfs_tree(self):
    # Retorna a árvore DFS do grafo com base nos identificadores das arestas
    visited = [False] * self.vertices
    dfs_tree = []

    def dfs(v): # Função de busca em profundidade
        visited[v] = True

        for neighbor in self.adj_list[v]: # Explora os vizinhos na ordem lexicográfica
            if not visited[neighbor]:
                # Encontrar o identificador da aresta entre v e neighbor
                for aresta in self.arestas: # Verifica se a aresta foi visitada
                    id_aresta = aresta[0]
                    ligacao_v1 = aresta[1] 
                    ligacao_v2 = aresta[2]
                    if (ligacao_v1 == v and ligacao_v2 == neighbor) or (ligacao_v1 == neighbor and ligacao_v2 == v): # Se a aresta for encontrada, adicione-a à árvore DFS
                        dfs_tree.append(id_aresta)
                        break  # Aresta correspondente encontrada, parar a busca
                dfs(neighbor) # Chama a função recursivamente

    for v in range(self.vertices): # Encontra a árvore DFS
        if not visited[v]:
            dfs(v)

    return dfs_tree # Retorna a árvore DFS
 

def bfs_tree(grafo):
    # Retorna a árvore BFS do grafo com base nos identificadores das arestas
    visited = [False] * grafo.vertices  # Verificação de vértices visitados
    bfs_tree = []
    queue = [0]  # Fila para a BFS, começando pelo vértice 0
    visited[0] = True  # Marcar o vértice 0 como visitado
    

    while queue:
        vertex = queue.pop(0)

        # Explorar os vizinhos na ordem lexicográfica
        for neighbor in grafo.adj_list[vertex]:
            if not visited[neighbor]:
                visited[neighbor] = True
                queue.append(neighbor)
                # Encontrar o identificador da aresta entre vertex e neighbor
                for aresta in grafo.arestas:
                        id_aresta = aresta[0]
                        ligacao_v1 = aresta[1] 
                        ligacao_v2 = aresta[2]
                        
                        if (ligacao_v1 == vertex and ligacao_v2 == neighbor) or (ligacao_v1 == neighbor and ligacao_v2 == vertex): # Se a aresta for encontrada, adicione-a à árvore BFS
                            bfs_tree.append(id_aresta)
                            break
    return bfs_tree # Retorna a árvore BFS



def mst_value(self):
    # Calcula o valor da Arvore Geradora Mínima para grafos não direcionados
    if self.direcionado:
        return -1

    mst_value = 0
    visited = [False] * self.vertices
    key = [float('inf')] * self.vertices
    key[0] = 0

    for _ in range(self.vertices):
        # Encontra o vértice não visitado com a menor chave
        min_key = float('inf')
        min_vertex = -1
        for v in range(self.vertices): 
            if not visited[v] and key[v] < min_key:
                min_key = key[v]
                min_vertex = v

        # Se min_vertex é -1, o grafo não é totalmente conectado
        if min_vertex == -1:
            return -1  # Grafo desconectado

        visited[min_vertex] = True
        mst_value += key[min_vertex]

        # Atualiza a chave dos vizinhos
        for neighbor in self.adj_list[min_vertex]:
            # Verifique se a aresta (min_vertex, neighbor) existe
            if (min_vertex, neighbor) in self.weight:
                weight = self.weight[(min_vertex, neighbor)]
                if not visited[neighbor] and weight < key[neighbor]:
                    key[neighbor] = weight

    # Retorna o valor da Arvore Geradora Minima
    return mst_value if all(visited) else -1


def topological_sort(self):
    # Retorna a ordenação topológica para grafos direcionados
    if not self.direcionado:
        return -1
    
    if has_ciclo(self) == 1: # Se o grafo tiver um ciclo, retorne -1
        return -1 

    visited = [False] * self.vertices
    stack = []

    def dfs(v): # Função de busca em profundidade
        visited[v] = True
        for neighbor in self.adj_list[v]:
            if not visited[neighbor]:
                dfs(neighbor)
        stack.append(v)

    for v in range(self.vertices): # Encontra a ordenação topológica
        if not visited[v]:
            dfs(v)
    # Retorna a ordenação topológica
    return stack[::-1]

def shortest_path_value(self, source, destination):
    # Inicializa o array de distâncias com infinito para todos os vértices, exceto o source
    distance = [float('inf')] * self.vertices
    distance[source] = 0

    # Usamos uma fila de prioridade (min-heap) para obter o vértice com a menor distância atual
    priority_queue = [(0, source)]  # (distância, vértice)

    while priority_queue:
        current_distance, current_vertex = heapq.heappop(priority_queue)

        # Se a distância atual do heap for maior que a armazenada, ignore
        if current_distance > distance[current_vertex]:
            continue

        # Atualiza as distâncias para os vizinhos
        for neighbor in self.adj_list[current_vertex]:
            # Acesse o peso da aresta correta usando a chave (current_vertex, neighbor)
            if (current_vertex, neighbor) in self.weight:
                weight = self.weight[(current_vertex, neighbor)]
                if distance[current_vertex] + weight < distance[neighbor]:
                    distance[neighbor] = distance[current_vertex] + weight
                    heapq.heappush(priority_queue, (distance[neighbor], neighbor))

    # Retorna a distância do source ao destino, ou -1 se o destino não for alcançável
    return distance[destination] if distance[destination] != float('inf') else -1



def max_flow_value(self, source, sink):
    # Calcula o valor do fluxo máximo em grafos direcionados
    if not self.direcionado:
        return -1

    residual_graph = [[0] * self.vertices for _ in range(self.vertices)] # Inicializa o grafo residual
    for u in range(self.vertices): # Preenche o grafo residual
        for v in self.adj_list[u]:
            residual_graph[u][v] = self.weight[(u, v)]

    parent = [-1] * self.vertices
    max_flow = 0

    while bfs(self, source, sink, residual_graph, parent): # Enquanto houver um caminho de source a sink no grafo residual
        path_flow = float('inf')
        v = sink
        while v != source: # Encontra o fluxo máximo no caminho encontrado
            u = parent[v]
            path_flow = min(path_flow, residual_graph[u][v])
            v = u

        v = sink
        while v != source: # Atualiza o grafo residual
            u = parent[v]
            residual_graph[u][v] -= path_flow
            residual_graph[v][u] += path_flow
            v = u

        max_flow += path_flow # Adiciona o fluxo máximo do caminho ao fluxo total

    return max_flow # Retorna o valor do fluxo máximo

def bfs(self, source, sink, residual_graph, parent): # Função de busca em largura
    visited = [False] * self.vertices
    queue = []
    queue.append(source)
    visited[source] = True

    while queue: # Encontra o caminho de source a sink no grafo residual
        u = queue.pop(0)
        for v in range(self.vertices):
            if not visited[v] and residual_graph[u][v] > 0:
                queue.append(v)
                visited[v] = True
                parent[v] = u

    return visited[sink] # Retorna True se houver um caminho de source a sink no grafo residual, caso contrário, retorna False

def transitive_closure(self):
    # Retorna o fecho transitivo para grafos direcionados
    if not self.direcionado: # Se o grafo não for direcionado, retorne -1
        return -1

    closure = [[0] * self.vertices for _ in range(self.vertices)] # Inicializa o fecho transitivo
    for i in range(self.vertices): # Preenche o fecho transitivo
        dfs(self, i, i, closure)
    # Gera a lista dos vértices acessíveis a partir do vértice 0
    reachable_from_zero = [i for i in range(self.vertices) if closure[0][i] == 1 and i != 0] # Retorna o fecho transitivo
    return reachable_from_zero

def dfs(self, start, current, closure): # Função de busca em profundidade
    closure[start][current] = 1
    for neighbor in self.adj_list[current]:
        if closure[start][neighbor] == 0:
            dfs(self, start, neighbor, closure)

def ler_grafo_do_exemplo(): 
    while True: # Loop para ler os dados de entrada
        try:
            dados_entrada = [] # Lista para armazenar os dados de entrada
            linha = input().strip() # Lê a primeira linha
            dados_entrada.append(linha) # Adiciona a primeira linha à lista

            id_maximo_vertice, num_arestas = map(int, linha.split()) # Lê o número máximo de vértices e o número de arestas
            linha = input().strip() # Lê a segunda linha
            dados_entrada.append(linha) # Adiciona a segunda linha à lista

            for _ in range(num_arestas): # Lê as arestas
                linha = input().strip() # Lê as arestas
                dados_entrada.append(linha) # Adiciona as arestas à lista

            linhas = dados_entrada # Atribui a lista de dados de entrada à variável linhas
            direcionado = linhas[1].strip().lower() # Verifica se o grafo é direcionado ou não

            if direcionado not in ['direcionado', 'nao_direcionado']:
                raise ValueError("Formato inválido! A segunda linha deve conter 'direcionado' ou 'nao_direcionado'.")
            
            if direcionado == 'direcionado': # Se o grafo for direcionado, direcionado = True, caso contrário, direcionado = False
                direcionado = True
            else:
                direcionado = False
            arestas = []

            for linha in linhas[2:]: # Lê as arestas
                partes = list(filter(None, linha.split())) # Separa a linha em partes
                id = int(partes[0]) # Atribui o identificador da aresta
                u, v, p = int(partes[1]), int(partes[2]), int(partes[3]) # Atribui os vértices e o peso da aresta
                arestas.append((id, u, v, p)) # Adiciona a aresta à lista de arestas

            num_vertices = id_maximo_vertice 
            return Grafo(num_vertices, arestas, direcionado) # Retorna o grafo

        except ValueError as ve:
            print(f"\nErro: {ve}")
            print("Por favor, insira os dados novamente no formato correto.\n")

def perguntar_funcoes_ao_usuario(): # Função para perguntar ao usuário as funções a serem executadas
    entrada_funcoes = input("").split()
    numeros_funcoes = list(map(int, entrada_funcoes)) # Converte os números de entrada em inteiros
    return numeros_funcoes

def executar_funcoes(grafo, funcoes): # Função para executar as funções escolhidas pelo usuário

    for funcao in funcoes:
        if funcao == 0:
            print(1 if is_conexo(grafo) else 0)
        elif funcao == 1:
            print(1 if is_bipartido(grafo) else 0)
        elif funcao == 2:
            print(1 if is_euleriano(grafo) else 0)
        elif funcao == 3:
            print(1 if has_ciclo(grafo) else 0)
        elif funcao == 4:
            componentes = count_componentes_conexas(grafo)
            print(componentes if componentes != -1 else "-1")
        elif funcao == 5:
            print(count_componentes_fortemente_conexas(grafo))
        elif funcao == 6:
            pontos_articulacao = articulacao(grafo)
            print(" ".join(map(str, pontos_articulacao)) if isinstance(pontos_articulacao, list) else "-1")
        elif funcao == 7:
            arestas_pont = count_arestas_ponte(grafo)
            print(arestas_pont if arestas_pont != -1 else "-1")
        elif funcao == 8:
            resultado_arvore = " ".join(map(str, dfs_tree(grafo)))
            print(resultado_arvore)
        elif funcao == 9:
            resultado_arvore_largura = " ".join(map(str, bfs_tree(grafo)))
            print(resultado_arvore_largura)
        elif funcao == 10:
            print(mst_value(grafo))
        elif funcao == 11:
            ordenacao = topological_sort(grafo) # Ordenação topológica
            print(" ".join(map(str, ordenacao)) if isinstance(ordenacao, list) else "-1")
        elif funcao == 12:
            caminho_minimo_valor = shortest_path_value(grafo, 0, grafo.vertices - 1)
            print(caminho_minimo_valor if caminho_minimo_valor != float('inf') else "-1")
        elif funcao == 13:
            fluxo = max_flow_value(grafo, 0, grafo.vertices - 1)
            print(fluxo)
        elif funcao == 14:
            fecho_transitivo_resultado = ((" ".join(map(str,transitive_closure(grafo)))) if isinstance(transitive_closure(grafo), list) else "-1")
            print(fecho_transitivo_resultado)

def main(): # Função principal
    funcoes = perguntar_funcoes_ao_usuario() # Pergunta ao usuário as funções a serem executadas
    grafo = ler_grafo_do_exemplo() # Lê o grafo do exemplo
    executar_funcoes(grafo, funcoes) # Executa as funções escolhidas pelo usuário
    # Desenhar o grafo
    desenhar_grafo(grafo)

if __name__ == "__main__": # Chama a função principal
    main()
