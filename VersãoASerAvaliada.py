import heapq  # Importa a biblioteca heapq para manipulação de filas de prioridade (min-heaps)
from collections import deque  # Importa deque para manipulação de filas de forma eficiente

# Classe Grafo que implementa um grafo com suas funcionalidades básicas
class Grafo:   
    def __init__(self, vertices, arestas, direcionado):
        """
        Inicializa um grafo.
        :param vertices: Número de vértices no grafo.
        :param arestas: Lista de arestas, onde cada aresta é representada por uma tupla (id, u, v, peso).
        :param direcionado: Booleano indicando se o grafo é direcionado ou não.
        """
        self.vertices = vertices  # Número de vértices no grafo
        self.direcionado = direcionado  # Indica se o grafo é direcionado
        self.adj_list = {i: [] for i in range(vertices)}  # Lista de adjacências, representando o grafo
        self.weight = {}  # Dicionário para armazenar os pesos das arestas
        self.id = {}  # Dicionário para armazenar os identificadores das arestas
        self.arestas = arestas  # Lista de arestas

        # Preenche a lista de adjacências e os dicionários de peso e ID para cada aresta
        for id, u, v, p in arestas:  # Para cada aresta u -> v com peso p
            self.adj_list[u].append(v)  # Adiciona v à lista de adjacências de u
            self.weight[(u, v)] = p  # Armazena o peso da aresta
            self.id[(u, v)] = id  # Armazena o identificador da aresta

            # Se o grafo não for direcionado, adicione também a aresta inversa v -> u
            if not direcionado:
                self.adj_list[v].append(u)
                self.weight[(u, v)] = p
                self.id[(u, v)] = id

import networkx as nx  # Importa a biblioteca NetworkX para manipulação e visualização de grafos
import matplotlib.pyplot as plt  # Importa Matplotlib para visualização de grafos

def desenhar_grafo(grafo):
    """
    Função para desenhar o grafo utilizando a biblioteca NetworkX.
    Se o grafo for direcionado, desenha setas; se houver pesos nas arestas, eles são exibidos.
    """
    G = nx.DiGraph() if grafo.direcionado else nx.Graph()  # Cria um grafo direcionado ou não direcionado

    # Adiciona arestas ao grafo NetworkX, incluindo pesos
    for u in grafo.adj_list:
        for v in grafo.adj_list[u]:
            G.add_edge(u, v, weight=grafo.weight.get((u, v), 1))

    pos = nx.spring_layout(G)  # Define a posição dos nós para visualização
    labels = nx.get_edge_attributes(G, 'weight')  # Obtém os pesos das arestas
    nx.draw(G, pos, with_labels=True, node_color='skyblue', node_size=700, font_size=12, font_weight='bold', arrows=grafo.direcionado)  # Desenha o grafo
    nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)  # Adiciona os pesos das arestas como rótulos
    plt.show()  # Exibe o grafo

def is_conexo(self):
    """
    Verifica se o grafo não direcionado é conexo.
    Um grafo é conexo se existe um caminho entre qualquer par de vértices.
    Implementa uma busca em profundidade (DFS) para verificar a conectividade.
    :return: 1 se o grafo é conexo, 0 caso contrário, -1 se o grafo for direcionado.
    """
    if self.direcionado:
        return -1  # Retorna -1 se o grafo for direcionado, pois o conceito de conectividade aqui é aplicado apenas a grafos não direcionados

    visited = [False] * self.vertices  # Lista de verificação de vértices visitados

    def dfs(v):
        visited[v] = True  # Marca o vértice como visitado
        for neighbor in self.adj_list[v]:  # Explora todos os vizinhos
            if not visited[neighbor]:
                dfs(neighbor)  # Chama recursivamente o DFS

    dfs(0)  # Inicia a DFS a partir do primeiro vértice
    return 1 if all(visited) else 0  # Se todos os vértices foram visitados, o grafo é conexo

def is_conectividade_fraca(self): 
    """
    Verifica a conectividade fraca em grafos direcionados.
    Um grafo tem conectividade fraca se, ao ignorar as direções das arestas, o grafo resultante é conexo.
    Implementa uma busca em profundidade (DFS).
    :return: 1 se o grafo é fracamente conectado, 0 caso contrário.
    """
    visited = [False] * self.vertices  # Lista de verificação de vértices visitados

    def dfs(v):
        visited[v] = True  # Marca o vértice como visitado
        for neighbor in self.adj_list[v]:  # Explora todos os vizinhos
            if not visited[neighbor]:
                dfs(neighbor)  # Chama recursivamente o DFS

    dfs(0)  # Inicia a DFS a partir do primeiro vértice
    return 1 if all(visited) else 0  # Se todos os vértices foram visitados, o grafo é fracamente conectado

def is_bipartido(self): 
    """
    Verifica se o grafo não direcionado é bipartido.
    Um grafo é bipartido se seus vértices podem ser divididos em dois conjuntos, de forma que nenhuma aresta
    conecte vértices do mesmo conjunto. Utiliza DFS para atribuir cores aos vértices e verificar a bipartição.
    :return: 1 se o grafo é bipartido, 0 caso contrário, e 0 para grafos direcionados.
    """
    if self.direcionado:
        return 0  # Grafos direcionados não são considerados bipartidos aqui

    color = [-1] * self.vertices  # Inicializa todas as cores com -1 (não colorido)

    def dfs(v, c):
        color[v] = c  # Atribui a cor c ao vértice v
        for neighbor in self.adj_list[v]:  # Explora todos os vizinhos
            if color[neighbor] == -1:
                if not dfs(neighbor, 1 - c):  # Chama recursivamente o DFS com a cor oposta
                    return False
            elif color[neighbor] == c:
                return False  # Se um vizinho tem a mesma cor, o grafo não é bipartido
        return True

    for v in range(self.vertices):  # Verifica todos os vértices
        if color[v] == -1:
            if not dfs(v, 0):  # Se algum componente não for bipartido, retorna 0
                return 0
    return 1  # Retorna 1 se o grafo for bipartido

def is_euleriano(self):
    """
    Verifica se o grafo não direcionado é Euleriano.
    Um grafo é Euleriano se é conexo e todos os seus vértices têm grau par.
    :return: 1 se o grafo é Euleriano, 0 caso contrário, e 0 para grafos direcionados.
    """
    if self.direcionado:
        return 0  # Grafos direcionados não são considerados Eulerianos aqui

    if not is_conexo(self):  # Um grafo Euleriano deve ser conexo
        return 0

    for v in range(self.vertices):  # Verifica se todos os vértices têm grau par
        if len(self.adj_list[v]) % 2 != 0:
            return 0

    return 1  # Retorna 1 se o grafo for Euleriano

def has_ciclo(self):
    """
    Verifica se o grafo tem um ciclo.
    Um ciclo em um grafo é uma sequência de arestas que formam um caminho fechado.
    Implementa DFS para detectar ciclos em grafos não direcionados.
    :return: 1 se o grafo tem ciclo, 0 caso contrário.
    """
    visited = [False] * self.vertices  # Lista de verificação de vértices visitados

    def dfs(v, parent):
        visited[v] = True  # Marca o vértice como visitado

        for neighbor in self.adj_list[v]:  # Explora todos os vizinhos
            if not visited[neighbor]:  # Se o vizinho não foi visitado
                if dfs(neighbor, v):  # Chama recursivamente o DFS
                    return True
            elif neighbor != parent:  # Se o vizinho foi visitado e não é o pai, há um ciclo
                return True

        return False

    for v in range(self.vertices):  # Verifica se o grafo tem um ciclo
        if not visited[v]:
            if dfs(v, -1):
                return 1  # Retorna 1 se houver um ciclo

    return 0  # Retorna 0 se não houver ciclo

def count_componentes_conexas(self):
    """
    Conta o número de componentes conexas em grafos não direcionados.
    Uma componente conexa é um subgrafo onde há um caminho entre qualquer par de vértices.
    Implementa DFS para contar as componentes conexas.
    :return: Número de componentes conexas, ou -1 se o grafo for direcionado.
    """
    if self.direcionado:
        return -1  # Retorna -1 para grafos direcionados

    visited = [False] * self.vertices  # Lista de verificação de vértices visitados
    count = 0  # Contador de componentes conexas

    def dfs(v):
        visited[v] = True  # Marca o vértice como visitado
        for neighbor in self.adj_list[v]:  # Explora todos os vizinhos
            if not visited[neighbor]:
                dfs(neighbor)  # Chama recursivamente o DFS

    for v in range(self.vertices):  # Conta o número de componentes conexas
        if not visited[v]: 
            dfs(v)
            count += 1

    return count  # Retorna o número de componentes conexas

def count_componentes_fortemente_conexas(self):
    """
    Conta o número de componentes fortemente conectadas em grafos direcionados.
    Uma componente fortemente conectada é um subgrafo onde há um caminho entre qualquer par de vértices no sentido das arestas.
    Implementa o algoritmo de Kosaraju para encontrar as componentes fortemente conectadas.
    :return: Número de componentes fortemente conectadas, ou -1 se o grafo não for direcionado.
    """
    if not self.direcionado:
        return -1  # Retorna -1 para grafos não direcionados

    def dfs(v, visited, adj_list, stack=None):
        visited[v] = True  # Marca o vértice como visitado
        for neighbor in adj_list[v]:  # Explora todos os vizinhos
            if not visited[neighbor]:
                dfs(neighbor, visited, adj_list, stack)
        if stack is not None:
            stack.append(v)  # Adiciona o vértice à pilha de finalização

    def transpose_graph():
        # Transpõe o grafo, invertendo as direções das arestas
        transposed = [[] for _ in range(self.vertices)]
        for v in range(self.vertices): 
            for neighbor in self.adj_list[v]:
                transposed[neighbor].append(v)
        return transposed

    visited = [False] * self.vertices
    stack = []
    for v in range(self.vertices):
        if not visited[v]:
            dfs(v, visited, self.adj_list, stack)  # Primeira passagem de DFS

    transposed_graph = transpose_graph()  # Grafo transposto

    visited = [False] * self.vertices
    count = 0
    while stack: # Processa os vértices na ordem inversa da pilha de finalização
        v = stack.pop()
        if not visited[v]:
            dfs(v, visited, transposed_graph)  # Segunda passagem de DFS
            count += 1  # Incrementa o contador para cada componente fortemente conectada

    return count  # Retorna o número de componentes fortemente conectadas

def articulacao(self):
    """
    Encontra e retorna os pontos de articulação em grafos não direcionados.
    Um ponto de articulação é um vértice que, se removido, aumenta o número de componentes conexas.
    Implementa o algoritmo de Tarjan para encontrar os pontos de articulação.
    :return: Lista de pontos de articulação, ou -1 se o grafo for direcionado.
    """
    if self.direcionado:
        return -1  # Retorna -1 para grafos direcionados

    visited = [False] * self.vertices 
    articulation_points = []  # Lista de pontos de articulação

    def dfs(v, parent, visited, low, disc, articulation_points): 
        children = 0
        visited[v] = True
        disc[v] = self.time  # Registra o tempo de descoberta do vértice
        low[v] = self.time  # Inicializa low[v] com o tempo de descoberta
        self.time += 1

        for neighbor in self.adj_list[v]: # Explora os vizinhos
            if not visited[neighbor]:  # Se o vizinho não foi visitado
                children += 1
                parent[neighbor] = v
                dfs(neighbor, parent, visited, low, disc, articulation_points)
                low[v] = min(low[v], low[neighbor])  # Atualiza low[v]

                # Condições para identificar um ponto de articulação
                if parent[v] == -1 and children > 1:  # Raiz com mais de um filho
                    articulation_points.append(v)
                elif parent[v] != -1 and low[neighbor] >= disc[v]:  # Vértice com um low alto
                    articulation_points.append(v)
            elif neighbor != parent[v]:
                low[v] = min(low[v], disc[neighbor])  # Atualiza low[v] para vértices visitados

    parent = [-1] * self.vertices
    low = [float('inf')] * self.vertices # Inicializa low com infinito
    disc = [float('inf')] * self.vertices # Inicializa disc com infinito
    self.time = 0

    for v in range(self.vertices):
        if not visited[v]:
            dfs(v, parent, visited, low, disc, articulation_points)

    return articulation_points  # Retorna a lista de pontos de articulação

def count_arestas_ponte(self):
    """
    Conta o número de arestas ponte em grafos não direcionados.
    Uma aresta ponte é aquela que, se removida, aumenta o número de componentes conexas.
    Implementa o algoritmo de Tarjan para encontrar as arestas ponte.
    :return: Número de arestas ponte, ou -1 se o grafo for direcionado.
    """
    if self.direcionado:
        return -1  # Retorna -1 para grafos direcionados

    count = 0  # Contador de arestas ponte

    def dfs(v, parent, visited, low, disc):
        nonlocal count
        visited[v] = True
        disc[v] = self.time  # Registra o tempo de descoberta do vértice
        low[v] = self.time  # Inicializa low[v] com o tempo de descoberta
        self.time += 1

        for neighbor in self.adj_list[v]:
            if not visited[neighbor]:  # Se o vizinho não foi visitado
                parent[neighbor] = v
                dfs(neighbor, parent, visited, low, disc)
                low[v] = min(low[v], low[neighbor])  # Atualiza low[v]

                if low[neighbor] > disc[v]:  # Se o low do vizinho é maior que o discovery time do vértice atual
                    count += 1  # A aresta é uma ponte
            elif neighbor != parent[v]:
                low[v] = min(low[v], disc[neighbor])  # Atualiza low[v] para vértices visitados

    parent = [-1] * self.vertices
    low = [float('inf')] * self.vertices
    disc = [float('inf')] * self.vertices
    visited = [False] * self.vertices
    self.time = 0

    for v in range(self.vertices):
        if not visited[v]:
            dfs(v, parent, visited, low, disc)

    return count  # Retorna o número de arestas ponte

def dfs_tree(self):
    """
    Gera e retorna a árvore DFS do grafo com base nos identificadores das arestas.
    :return: Lista de identificadores de arestas que compõem a árvore DFS.
    """
    visited = [False] * self.vertices  # Lista de verificação de vértices visitados
    dfs_tree = []  # Lista para armazenar as arestas da árvore DFS

    def dfs(v):
        visited[v] = True  # Marca o vértice como visitado

        for neighbor in self.adj_list[v]:  # Explora os vizinhos
            if not visited[neighbor]:
                for aresta in self.arestas:  # Encontra o identificador da aresta entre v e neighbor
                    id_aresta = aresta[0]
                    ligacao_v1 = aresta[1]
                    ligacao_v2 = aresta[2]
                    if (ligacao_v1 == v and ligacao_v2 == neighbor) or (ligacao_v1 == neighbor and ligacao_v2 == v):
                        dfs_tree.append(id_aresta)  # Adiciona a aresta à árvore DFS
                        break  # Para a busca, pois a aresta correspondente foi encontrada
                dfs(neighbor)  # Chama recursivamente o DFS

    for v in range(self.vertices):
        if not visited[v]:
            dfs(v)

    return dfs_tree  # Retorna a árvore DFS

def bfs_tree(grafo):
    """
    Gera e retorna a árvore BFS do grafo com base nos identificadores das arestas.
    :return: Lista de identificadores de arestas que compõem a árvore BFS.
    """
    visited = [False] * grafo.vertices  # Lista de verificação de vértices visitados
    bfs_tree = []  # Lista para armazenar as arestas da árvore BFS
    queue = [0]  # Fila para a BFS, começando pelo vértice 0
    visited[0] = True  # Marca o vértice 0 como visitado

    while queue:
        vertex = queue.pop(0)

        for neighbor in grafo.adj_list[vertex]:  # Explora os vizinhos
            if not visited[neighbor]:
                visited[neighbor] = True
                queue.append(neighbor)
                for aresta in grafo.arestas:  # Encontra o identificador da aresta entre vertex e neighbor
                    id_aresta = aresta[0]
                    ligacao_v1 = aresta[1]
                    ligacao_v2 = aresta[2]
                    if (ligacao_v1 == vertex and ligacao_v2 == neighbor) or (ligacao_v1 == neighbor and ligacao_v2 == vertex):
                        bfs_tree.append(id_aresta)  # Adiciona a aresta à árvore BFS
                        break

    return bfs_tree  # Retorna a árvore BFS

def mst_value(self):
    """
    Calcula o valor da Árvore Geradora Mínima (MST) para grafos não direcionados.
    Utiliza uma versão do algoritmo de Prim para encontrar a MST.
    :return: Valor da MST, ou -1 se o grafo for direcionado ou desconectado.
    """
    if self.direcionado:
        return -1  # Retorna -1 para grafos direcionados

    mst_value = 0  # Inicializa o valor da MST
    visited = [False] * self.vertices  # Lista de verificação de vértices visitados
    key = [float('inf')] * self.vertices  # Inicializa as chaves com infinito
    key[0] = 0  # Começa pelo vértice 0

    for _ in range(self.vertices):
        min_key = float('inf') # Inicializa a chave mínima 
        min_vertex = -1 # Inicializa o vértice mínimo
        for v in range(self.vertices):
            if not visited[v] and key[v] < min_key:
                min_key = key[v]
                min_vertex = v

        if min_vertex == -1:
            return -1  # Se não há vértice mínimo, o grafo não é totalmente conectado

        visited[min_vertex] = True
        mst_value += key[min_vertex]  # Adiciona o valor da aresta à MST

        for neighbor in self.adj_list[min_vertex]:
            if (min_vertex, neighbor) in self.weight:  # Verifica se a aresta existe
                weight = self.weight[(min_vertex, neighbor)]
                if not visited[neighbor] and weight < key[neighbor]:
                    key[neighbor] = weight  # Atualiza a chave para o vizinho

    return mst_value if all(visited) else -1  # Retorna o valor da MST

def topological_sort(self):
    """
    Retorna a ordenação topológica para grafos direcionados.
    A ordenação topológica é possível apenas em DAGs (grafos direcionados acíclicos).
    :return: Lista com a ordenação topológica ou -1 se o grafo tiver ciclos ou não for direcionado.
    """
    if not self.direcionado:
        return -1  # Retorna -1 para grafos não direcionados
    
    if has_ciclo(self) == 1:
        return -1  # Se o grafo tiver ciclos, a ordenação topológica não é possível

    visited = [False] * self.vertices
    stack = []

    def dfs(v):
        visited[v] = True
        for neighbor in self.adj_list[v]:
            if not visited[neighbor]:
                dfs(neighbor)
        stack.append(v)  # Adiciona o vértice à pilha

    for v in range(self.vertices):
        if not visited[v]:
            dfs(v)

    return stack[::-1]  # Retorna a lista na ordem inversa para a ordenação topológica

def shortest_path_value(self, source, destination):
    """
    Calcula o valor do caminho mais curto entre dois vértices.
    Utiliza o algoritmo de Dijkstra para encontrar o caminho mais curto.
    :param source: Vértice de origem.
    :param destination: Vértice de destino.
    :return: Valor do caminho mais curto ou -1 se o destino não for alcançável.
    """
    distance = [float('inf')] * self.vertices  # Inicializa as distâncias com infinito
    distance[source] = 0  # A distância do vértice de origem para ele mesmo é 0

    priority_queue = [(0, source)]  # Fila de prioridade (min-heap)

    while priority_queue: # Enquanto a fila não estiver vazia
        current_distance, current_vertex = heapq.heappop(priority_queue)

        if current_distance > distance[current_vertex]: # Se a distância atual for maior que a armazenada
            continue

        for neighbor in self.adj_list[current_vertex]:
            if (current_vertex, neighbor) in self.weight:  # Verifica se a aresta existe
                weight = self.weight[(current_vertex, neighbor)]
                if distance[current_vertex] + weight < distance[neighbor]:
                    distance[neighbor] = distance[current_vertex] + weight
                    heapq.heappush(priority_queue, (distance[neighbor], neighbor)) # Insere na fila de prioridade

    return distance[destination] if distance[destination] != float('inf') else -1  # Retorna a distância ao destino

def max_flow_value(self, source, sink):
    """
    Calcula o valor do fluxo máximo em grafos direcionados.
    Utiliza o algoritmo de Edmonds-Karp, uma implementação do algoritmo de Ford-Fulkerson.
    :param source: Vértice de origem.
    :param sink: Vértice de destino.
    :return: Valor do fluxo máximo ou -1 se o grafo não for direcionado.
    """
    if not self.direcionado:
        return -1  # Retorna -1 para grafos não direcionados

    residual_graph = [[0] * self.vertices for _ in range(self.vertices)] # Inicializa o grafo residual
    for u in range(self.vertices):
        for v in self.adj_list[u]:
            residual_graph[u][v] = self.weight[(u, v)]  # Cria o grafo residual

    parent = [-1] * self.vertices # Inicializa o array de pais
    max_flow = 0

    while bfs(self, source, sink, residual_graph, parent):
        path_flow = float('inf')
        v = sink
        while v != source:
            u = parent[v]
            path_flow = min(path_flow, residual_graph[u][v]) # Encontra o fluxo máximo no caminho
            v = u

        v = sink
        
        while v != source: # Atualiza o grafo residual e encontra o caminho de volta  
            u = parent[v]
            residual_graph[u][v] -= path_flow 
            residual_graph[v][u] += path_flow
            v = u

        max_flow += path_flow

    return max_flow  # Retorna o valor do fluxo máximo

def bfs(self, source, sink, residual_graph, parent):
    """
    Implementa uma busca em largura (BFS) para encontrar um caminho aumentante no grafo residual.
    :param source: Vértice de origem.
    :param sink: Vértice de destino.
    :param residual_graph: Grafo residual.
    :param parent: Array para armazenar o caminho encontrado.
    :return: True se houver um caminho, False caso contrário.
    """
    visited = [False] * self.vertices
    queue = []
    queue.append(source)
    visited[source] = True

    while queue:
        u = queue.pop(0)
        for v in range(self.vertices):
            if not visited[v] and residual_graph[u][v] > 0:
                queue.append(v)
                visited[v] = True
                parent[v] = u

    return visited[sink]  # Retorna True se houver um caminho aumentante

def transitive_closure(self):
    """
    Calcula o fecho transitivo do grafo direcionado.
    O fecho transitivo de um grafo é o menor grafo em que há uma aresta direta de u para v sempre que há um caminho de u para v.
    :return: Lista de vértices acessíveis a partir do vértice 0, ou -1 se o grafo não for direcionado.
    """
    if not self.direcionado:
        return -1  # Retorna -1 para grafos não direcionados

    closure = [[0] * self.vertices for _ in range(self.vertices)] 
    for i in range(self.vertices):
        dfs(self, i, i, closure)  # Chama DFS para preencher o fecho transitivo

    reachable_from_zero = [i for i in range(self.vertices) if closure[0][i] == 1 and i != 0] # Vértices acessíveis a partir de 0 
    return reachable_from_zero  # Retorna a lista de vértices acessíveis a partir de 0

def dfs(self, start, current, closure):
    """
    Função auxiliar para a DFS utilizada no cálculo do fecho transitivo.
    :param start: Vértice de início.
    :param current: Vértice atual.
    :param closure: Matriz do fecho transitivo.
    """
    closure[start][current] = 1  # Marca que current é acessível a partir de start
    for neighbor in self.adj_list[current]:
        if closure[start][neighbor] == 0:
            dfs(self, start, neighbor, closure)

def ler_grafo_do_exemplo(): 
    """
    Lê o grafo de um exemplo fornecido pelo usuário.
    :return: Objeto Grafo construído com os dados fornecidos.
    """
    while True:
        try:
            dados_entrada = []  # Lista para armazenar os dados de entrada
            linha = input().strip()
            dados_entrada.append(linha)

            id_maximo_vertice, num_arestas = map(int, linha.split())  # Lê o número máximo de vértices e arestas
            linha = input().strip()
            dados_entrada.append(linha)

            for _ in range(num_arestas):
                linha = input().strip()
                dados_entrada.append(linha)

            linhas = dados_entrada
            direcionado = linhas[1].strip().lower()  # Lê se o grafo é direcionado ou não

            if direcionado not in ['direcionado', 'nao_direcionado']:
                raise ValueError("Formato inválido! A segunda linha deve conter 'direcionado' ou 'nao_direcionado'.")
            
            direcionado = True if direcionado == 'direcionado' else False
            arestas = []

            for linha in linhas[2:]:
                partes = list(filter(None, linha.split()))  # Separa a linha em partes
                id = int(partes[0])
                u, v, p = int(partes[1]), int(partes[2]), int(partes[3])
                arestas.append((id, u, v, p))

            num_vertices = id_maximo_vertice 
            return Grafo(num_vertices, arestas, direcionado)  # Retorna o grafo

        except ValueError as ve:
            print(f"\nErro: {ve}")
            print("Por favor, insira os dados novamente no formato correto.\n")

def perguntar_funcoes_ao_usuario():
    """
    Pergunta ao usuário quais funções ele deseja executar no grafo.
    :return: Lista de números que representam as funções escolhidas.
    """
    entrada_funcoes = input("").split()
    numeros_funcoes = list(map(int, entrada_funcoes))  # Converte a entrada em uma lista de inteiros
    return numeros_funcoes

def executar_funcoes(grafo, funcoes):
    """
    Executa as funções escolhidas pelo usuário no grafo.
    :param grafo: O objeto Grafo sobre o qual as funções serão executadas.
    :param funcoes: Lista de funções a serem executadas.
    """
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
            ordenacao = topological_sort(grafo)  # Ordenação topológica
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

def main():
    """
    Função principal que gerencia a execução do programa.
    Pergunta ao usuário quais funções executar, lê o grafo de exemplo e executa as funções.
    """
    funcoes = perguntar_funcoes_ao_usuario()  # Pergunta ao usuário as funções a serem executadas
    grafo = ler_grafo_do_exemplo()  # Lê o grafo do exemplo
    executar_funcoes(grafo, funcoes)  # Executa as funções escolhidas pelo usuário
    desenhar_grafo(grafo)  # Desenha o grafo

if __name__ == "__main__":
    main()
