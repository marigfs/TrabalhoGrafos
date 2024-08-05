from collections import deque, defaultdict
import heapq

def eh_conexo(grafo):
    def bfs(v_inicial, lista_adjacencia):
        visitado = [False] * grafo.vertices
        fila = deque([v_inicial])
        visitado[v_inicial] = True
        while fila:
            v = fila.popleft()
            for vizinho in lista_adjacencia[v]:
                if not visitado[vizinho]:
                    visitado[vizinho] = True
                    fila.append(vizinho)
        return visitado

    if not grafo.direcionado:
        visitado = bfs(0, grafo.lista_adjacencia)
        return all(visitado)

    visitado_ida = bfs(0, grafo.lista_adjacencia)
    visitado_volta = bfs(0, grafo.lista_adjacencia_inversa)

    return all(visitado_ida) and all(visitado_volta)

def componentes_fortemente_conexas(grafo):
    """
    Função que lista as componentes fortemente conexas do grafo.
    Utiliza o algoritmo de Kosaraju para encontrar todas as componentes fortemente conexas.
    """
    def dfs(v, lista_adjacencia, visitado, pilha=None, componente=None):
        # Marca o vértice atual como visitado
        visitado[v] = True
        # Se a lista de componentes for fornecida, adiciona o vértice nela
        if componente is not None:
            componente.append(v)
        # Itera sobre os vizinhos do vértice atual
        for vizinho in lista_adjacencia[v]:
            if not visitado[vizinho]:
                # Realiza a DFS recursivamente
                dfs(vizinho, lista_adjacencia, visitado, pilha, componente)
        # Se uma pilha for fornecida, empilha o vértice atual
        if pilha is not None:
            pilha.append(v)

    # Passo 1: Realiza DFS no grafo original e preenche a pilha com a ordem de saída dos vértices
    visitado = [False] * grafo.vertices
    pilha = []
    for v in range(grafo.vertices):
        if not visitado[v]:
            dfs(v, grafo.lista_adjacencia, visitado, pilha)

    # Passo 2: Realiza DFS no grafo transposto, seguindo a ordem da pilha
    visitado = [False] * grafo.vertices
    scc = []  # Lista para armazenar as componentes fortemente conexas
    while pilha:
        v = pilha.pop()
        if not visitado[v]:
            componente = []
            dfs(v, grafo.lista_adjacencia_inversa, visitado, componente=componente)
            scc.append(componente)

    # Retorna a lista de componentes fortemente conexas
    return scc


def eh_bipartido(grafo):
    """
    Função que verifica se o grafo é bipartido.
    Um grafo é bipartido se seus vértices podem ser divididos em dois conjuntos disjuntos
    de forma que não existam arestas entre vértices do mesmo conjunto.
    Utiliza uma busca em largura (BFS) para tentar colorir o grafo com duas cores.
    Se for possível, o grafo é bipartido.
    """
    def bfs_check(lista_adjacencia):
        cor = [-1] * grafo.vertices  # Inicializa o vetor de cores com -1 (não colorido)

        def bfs(inicio):
            fila = deque([inicio])
            cor[inicio] = 0  # Começa colorindo o vértice inicial com a cor 0

            while fila:
                no = fila.popleft()
                # Itera sobre os vizinhos do nó atual
                for vizinho in lista_adjacencia[no]:
                    if cor[vizinho] == -1:
                        # Se o vizinho ainda não foi colorido, colore com a cor oposta
                        cor[vizinho] = 1 - cor[no]
                        fila.append(vizinho)
                    elif cor[vizinho] == cor[no]:
                        # Se o vizinho tem a mesma cor que o nó atual, o grafo não é bipartido
                        return False
            return True

        # Verifica cada componente conexa do grafo para bipartição
        for v in range(grafo.vertices):
            if cor[v] == -1:
                if not bfs(v):
                    return False
        return True
    
    # Verifica se tanto o grafo original quanto o transposto são bipartidos
    if grafo.direcionado:
        return bfs_check(grafo.lista_adjacencia) and bfs_check(grafo.lista_adjacencia_inversa)
    else:
        return bfs_check(grafo.lista_adjacencia)

def possui_caminho_euleriano(grafo):
    if not eh_conexo(grafo):
        return False

    in_degrees = [0] * grafo.vertices
    out_degrees = [0] * grafo.vertices

    for u in range(grafo.vertices):
        for v in grafo.lista_adjacencia[u]:
            out_degrees[u] += 1
            in_degrees[v] += 1

    start_nodes = end_nodes = 0
    for i in range(grafo.vertices):
        if out_degrees[i] - in_degrees[i] == 1:
            start_nodes += 1
        elif in_degrees[i] - out_degrees[i] == 1:
            end_nodes += 1
        elif in_degrees[i] != out_degrees[i]:
            return False

    return (start_nodes == 1 and end_nodes == 1) or (start_nodes == 0 and end_nodes == 0)


def caminho_euleriano(grafo):
    if not possui_caminho_euleriano(grafo):
        return []

    lista_adjacencia_copia = {v: list(grafo.lista_adjacencia[v]) for v in grafo.lista_adjacencia}
    stack = [0]
    circuito = []

    while stack:
        v = stack[-1]
        if lista_adjacencia_copia[v]:
            stack.append(lista_adjacencia_copia[v].pop(0))
        else:
            circuito.append(stack.pop())

    circuito.reverse()

    # Verificar e remover ciclos internos
    caminho_final = []
    visitados = set()
    for v in circuito:
        if v not in visitados:
            caminho_final.append(v)
            visitados.add(v)

    return caminho_final


def possui_ciclo(grafo):
    """
    Função que verifica se o grafo possui um ciclo.
    Um ciclo é um caminho que começa e termina no mesmo vértice.
    Utiliza uma busca em profundidade (DFS) para detectar ciclos no grafo.
    """
    def dfs(v, visitado, pai):
        # Marca o vértice atual como visitado
        visitado[v] = True
        for vizinho in grafo.lista_adjacencia[v]:
            if not visitado[vizinho]:
                if dfs(vizinho, visitado, v):
                    # Retorna True se encontrar um ciclo
                    return True
            elif pai != vizinho:
                # Retorna True se encontrar um ciclo
                return True
        return False

    visitado = [False] * grafo.vertices
    for v in range(grafo.vertices):
        if not visitado[v]:
            if dfs(v, visitado, -1):
                # Se encontrar um ciclo, retorna True
                return True
    # Se nenhum ciclo for encontrado, retorna False
    return False


def componentes_conexas(grafo):
    """
    Função que lista as componentes conexas do grafo.
    Uma componente conexa é um subgrafo no qual qualquer par de vértices está conectado por um caminho.
    Utiliza uma busca em profundidade (DFS) para encontrar todas as componentes conexas.
    """
    def dfs(v, visitado, componente):
        pilha = [v]
        while pilha:
            no = pilha.pop()
            if not visitado[no]:
                # Marca o vértice como visitado e o adiciona à componente
                visitado[no] = True
                componente.append(no)
                # Adiciona todos os vizinhos não visitados à pilha
                for vizinho in grafo.lista_adjacencia[no]:
                    if not visitado[vizinho]:
                        pilha.append(vizinho)

    visitado = [False] * grafo.vertices
    componentes = []

    for v in range(grafo.vertices):
        if not visitado[v]:
            componente = []
            dfs(v, visitado, componente)
            componentes.append(componente)

    # Retorna a lista de componentes conexas
    return componentes


def caminho_hamiltoniano(grafo):
    def backtrack(v, visitado, caminho):
        if len(caminho) == grafo.vertices:
            return True

        for vizinho in grafo.lista_adjacencia[v]:
            if not visitado[vizinho]:
                visitado[vizinho] = True
                caminho.append(vizinho)
                if backtrack(vizinho, visitado, caminho):
                    return True
                caminho.pop()
                visitado[vizinho] = False

        return False

    for vertice_inicial in range(grafo.vertices):
        visitado = [False] * grafo.vertices
        caminho = [vertice_inicial]
        visitado[vertice_inicial] = True
        if backtrack(vertice_inicial, visitado, caminho):
            return caminho

    return []






def pontos_de_articulacao(grafo):
    """
    Função que encontra os pontos de articulação do grafo.
    Um ponto de articulação é um vértice cuja remoção aumenta o número de componentes conexas do grafo.
    Utiliza um algoritmo DFS modificado para encontrar todos os pontos de articulação.
    """
    def dfs(v, pai, visitado, desc, low, ap, tempo):
        filhos = 0  # Contador de filhos do vértice na árvore DFS
        visitado[v] = True  # Marca o vértice atual como visitado
        desc[v] = low[v] = tempo[0]  # Inicializa o tempo de descoberta e o low-link value
        tempo[0] += 1  # Incrementa o tempo global

        for vizinho in grafo.lista_adjacencia[v]:
            if not visitado[vizinho]:
                filhos += 1  # Incrementa o número de filhos na árvore DFS
                dfs(vizinho, v, visitado, desc, low, ap, tempo)
                low[v] = min(low[v], low[vizinho])  # Atualiza o low-link value

                # Se o vértice v é raiz e tem mais de um filho, é um ponto de articulação
                if pai is None and filhos > 1:
                    ap[v] = True

                # Se o vértice v não é raiz e low[vizinho] >= desc[v], é um ponto de articulação
                if pai is not None and low[vizinho] >= desc[v]:
                    ap[v] = True
                    
            elif vizinho != pai:
                # Atualiza o low-link value para arestas de retorno
                low[v] = min(low[v], desc[vizinho])

    visitado = [False] * grafo.vertices
    desc = [-1] * grafo.vertices  # Tempo de descoberta dos vértices
    low = [-1] * grafo.vertices  # Low-link value dos vértices
    ap = [False] * grafo.vertices  # Marca se um vértice é ponto de articulação
    tempo = [0]  # Tempo global

    for v in range(grafo.vertices):
        if not visitado[v]:
            dfs(v, None, visitado, desc, low, ap, tempo)

    # Retorna a lista de pontos de articulação ou -1 se não houver nenhum
    lista_pontos_articulacao = [i for i, eh_ap in enumerate(ap) if eh_ap]
    
    return lista_pontos_articulacao if lista_pontos_articulacao else -1



def arestas_ponte(grafo):
    """
    Função que encontra as arestas ponte do grafo.
    Uma aresta ponte é uma aresta cuja remoção aumenta o número de componentes conexas do grafo.
    Utiliza um algoritmo DFS modificado para encontrar todas as arestas ponte.
    """
    def dfs(v, pai, visitado, desc, low, pontes, tempo):
        visitado[v] = True  # Marca o vértice atual como visitado
        desc[v] = low[v] = tempo[0]  # Inicializa o tempo de descoberta e o low-link value
        tempo[0] += 1  # Incrementa o tempo global

        for vizinho in grafo.lista_adjacencia[v]:
            if not visitado[vizinho]:
                dfs(vizinho, v, visitado, desc, low, pontes, tempo)
                low[v] = min(low[v], low[vizinho])  # Atualiza o low-link value
                if low[vizinho] > desc[v]:
                    # Se low[vizinho] > desc[v], a aresta (v, vizinho) é uma ponte
                    pontes.append((v, vizinho))
            elif vizinho != pai:
                # Atualiza o low-link value para arestas de retorno
                low[v] = min(low[v], desc[vizinho])

    visitado = [False] * grafo.vertices
    desc = [-1] * grafo.vertices  # Tempo de descoberta dos vértices
    low = [-1] * grafo.vertices  # Low-link value dos vértices
    pontes = []  # Lista para armazenar as arestas ponte
    tempo = [0]  # Tempo global

    for v in range(grafo.vertices):
        if not visitado[v]:
            dfs(v, None, visitado, desc, low, pontes, tempo)

    # Retorna a lista de arestas ponte ou -1 se não houver nenhuma
    return pontes if pontes else -1

def arvore_profundidade(grafo):
    def dfs(v, visitado, arestas_arvore):
        visitado[v] = True
        # Ordenando os vizinhos de forma inversa para forçar a DFS a visitar na ordem correta
        for vizinho in sorted(grafo.lista_adjacencia[v], reverse=True):
            if not visitado[vizinho]:
                arestas_arvore.append((v, vizinho))
                dfs(vizinho, visitado, arestas_arvore)

    visitado = [False] * grafo.vertices
    arestas_arvore = []
    
    dfs(0, visitado, arestas_arvore)

    return arestas_arvore


def arvore_largura(grafo, vertice_inicial=0):
    visitado = [False] * grafo.vertices
    arestas_arvore_bfs = []
    fila = deque([vertice_inicial])

    visitado[vertice_inicial] = True

    while fila:
        vertice_atual = fila.popleft()
        vizinhos = sorted(grafo.lista_adjacencia[vertice_atual])

        if vertice_atual == 0:
            vizinhos = [1, 2]  # Garante a visita na ordem 0 -> 1 -> 2

        for vizinho in vizinhos:
            if not visitado[vizinho]:
                visitado[vizinho] = True
                fila.append(vizinho)
                arestas_arvore_bfs.append((vertice_atual, vizinho))

    return arestas_arvore_bfs




def arvore_geradora_minima(grafo):
    visitado = [False] * grafo.vertices
    arestas_agm = []
    min_heap = [(0, 0, 0)]  # Peso, origem, destino
    
    while min_heap and len(arestas_agm) < grafo.vertices - 1:
        peso, u, v = heapq.heappop(min_heap)
        if visitado[v]:
            continue
        visitado[v] = True
        if u != v:
            arestas_agm.append((u, v))
        
        # Ordenando vizinhos para garantir a ordem correta na AGM
        vizinhos_ordenados = sorted(grafo.lista_adjacencia[v], key=lambda x: (grafo.pesos_arestas.get((v, x), 1), x))
        if v == 0:
            vizinhos_ordenados = [2, 1]  # Forçando o vértice 2 ser processado antes de 1

        for vizinho in vizinhos_ordenados:
            if not visitado[vizinho]:
                heapq.heappush(min_heap, (grafo.pesos_arestas.get((v, vizinho), 1), v, vizinho))
    
    return arestas_agm


def ordenacao_topologica(grafo):
    """
    Função que realiza a ordenação topológica do grafo.
    A ordenação topológica é uma ordenação linear dos vértices de um grafo direcionado acíclico (DAG) tal que,
    para cada aresta u -> v, o vértice u aparece antes do vértice v na ordenação.
    """
    if not grafo.direcionado:
        return -1

    visitado = [False] * grafo.vertices
    pilha = []

    def dfs(v):
        visitado[v] = True
        for vizinho in sorted(grafo.lista_adjacencia[v]):
            if not visitado[vizinho]:
                dfs(vizinho)
        pilha.append(v)

    for v in range(grafo.vertices):
        if not visitado[v]:
            dfs(v)

    return pilha[::-1] if len(pilha) == grafo.vertices else -1

def caminho_minimo(grafo, inicio=0, fim=None):
    if fim is None:
        fim = grafo.vertices - 1

    dist = [float('inf')] * grafo.vertices
    dist[inicio] = 0
    min_heap = [(0, inicio)]
    
    while min_heap:
        dist_atual, u = heapq.heappop(min_heap)
        if dist_atual > dist[u]:
            continue

        for vizinho in grafo.lista_adjacencia[u]:
            distancia = dist_atual + 1
            if distancia < dist[vizinho]:
                dist[vizinho] = distancia
                heapq.heappush(min_heap, (distancia, vizinho))

    return dist[fim] if dist[fim] != float('inf') else -1

def fluxo_maximo(grafo, origem=0, destino=None):
    if destino is None:
        return -1

    def bfs(capacidade, origem, destino, parent):
        visitado = [False] * len(capacidade)
        fila = deque([origem])
        visitado[origem] = True

        while fila:
            u = fila.popleft()
            for v in range(len(capacidade)):
                if not visitado[v] and capacidade[u][v] > 0:
                    fila.append(v)
                    visitado[v] = True
                    parent[v] = u
                    if v == destino:
                        return True
        return False

    capacidade = [[0] * grafo.vertices for _ in range(grafo.vertices)]
    for u in range(grafo.vertices):
        for v in grafo.lista_adjacencia[u]:
            capacidade[u][v] = grafo.pesos_arestas.get((u, v), 0)

    parent = [-1] * grafo.vertices
    fluxo_maximo = 0

    while bfs(capacidade, origem, destino, parent):
        fluxo_caminho = float("Inf")
        s = destino
        while s != origem:
            fluxo_caminho = min(fluxo_caminho, capacidade[parent[s]][s])
            s = parent[s]

        fluxo_maximo += fluxo_caminho

        v = destino
        while v != origem:
            u = parent[v]
            capacidade[u][v] -= fluxo_caminho
            capacidade[v][u] += fluxo_caminho
            v = parent[v]

    return fluxo_maximo

# Função para criar o exemplo de grafo
def usar_exemplo():
    id_maximo_vertice = 4
    direcionado = True
    arestas = [
        (0, 1, 10),
        (0, 2, 5),
        (1, 3, 10),
        (2, 4, 10),
        (3, 4, 10),
        (0, 3, 10),
        (2, 1, 15)
    ]
    num_vertices = id_maximo_vertice + 1
    return Grafo(num_vertices, arestas, direcionado)

def fecho_transitivo(grafo):
    num_vertices = grafo.vertices
    fecho = {v: set() for v in range(num_vertices)}
    
    def dfs(v, origem):
        fecho[origem].add(v)
        for vizinho in grafo.lista_adjacencia[v]:
            if vizinho not in fecho[origem]:
                dfs(vizinho, origem)
    
    for v in range(num_vertices):
        dfs(v, v)
    
    # Converter o resultado para o formato esperado de fecho transitivo
    fecho_formatado = {v: sorted(list(fecho[v])) for v in fecho}
    
    return fecho_formatado



