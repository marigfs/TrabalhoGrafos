from collections import deque, defaultdict
import heapq

def eh_conexo(grafo):
    """
    Função que verifica se o grafo é conexo.
    Um grafo é conexo se existe um caminho entre qualquer par de vértices.
    """
    def bfs(v_inicial, lista_adjacencia):
        # Inicializa uma lista de visitados com False para todos os vértices
        visitado = [False] * grafo.vertices
        # Cria uma fila para o BFS e adiciona o vértice inicial
        fila = deque([v_inicial])
        # Marca o vértice inicial como visitado
        visitado[v_inicial] = True
        # Enquanto a fila não estiver vazia, processa cada vértice
        while fila:
            v = fila.popleft()
            # Itera sobre os vizinhos do vértice atual
            for vizinho in lista_adjacencia[v]:
                # Se o vizinho ainda não foi visitado, marca como visitado e adiciona na fila
                if not visitado[vizinho]:
                    visitado[vizinho] = True
                    fila.append(vizinho)
        # Retorna a lista de vértices visitados
        return visitado

    if not grafo.direcionado:
        # Se o grafo não é direcionado, basta verificar se todos os vértices foram visitados
        visitado = bfs(0, grafo.lista_adjacencia)
        return all(visitado)  # Retorna True se todos os vértices foram visitados

    # Para grafos direcionados, verifica a conexidade em ambas as direções
    visitado_ida = bfs(0, grafo.lista_adjacencia)
    visitado_volta = bfs(0, grafo.lista_adjacencia_inversa)  # Grafo transposto

    # Retorna True se todos os vértices foram visitados em ambas as direções
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
    """
    Função que verifica se o grafo direcionado possui um caminho Euleriano.
    Um caminho Euleriano existe se o grafo for fortemente conexo e
    tiver no máximo dois vértices onde o grau de entrada difere do grau de saída.
    """
    if not eh_conexo(grafo):
        # Se o grafo não é conexo, não pode ter um caminho Euleriano
        return False

    in_degrees = [0] * grafo.vertices  # Graus de entrada
    out_degrees = [0] * grafo.vertices  # Graus de saída

    # Calcula os graus de entrada e saída para cada vértice
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
            # Se o grau de entrada e saída não são iguais ou diferem em 1, não é possível
            return False

    # Retorna True se houver no máximo um vértice com grau de entrada maior e um com grau de saída maior
    return (start_nodes == 1 and end_nodes == 1) or (start_nodes == 0 and end_nodes == 0)

def caminho_euleriano(grafo):
    """
    Função que retorna um caminho Euleriano, se existir.
    Um caminho Euleriano percorre todas as arestas do grafo exatamente uma vez.
    Se o grafo tiver mais do que dois vértices de grau ímpar, o caminho Euleriano não existe.
    """
    if not possui_caminho_euleriano(grafo):
        return -1

    in_degrees = [0] * grafo.vertices  # Graus de entrada
    out_degrees = [0] * grafo.vertices  # Graus de saída
    start_vertex = 0  # Vértice inicial do caminho

    # Calcula os graus de entrada e saída para cada vértice
    for u in range(grafo.vertices):
        for v in grafo.lista_adjacencia[u]:
            out_degrees[u] += 1
            in_degrees[v] += 1

    # Define o vértice inicial com base nos graus de entrada e saída
    for i in range(grafo.vertices):
        if out_degrees[i] - in_degrees[i] == 1:
            start_vertex = i
            break

    stack = [start_vertex]  # Pilha para armazenar o caminho
    path = []  # Lista para armazenar o caminho final

    while stack:
        v = stack[-1]
        if out_degrees[v] == 0:
            # Se não há mais arestas saindo do vértice, adiciona-o ao caminho final
            path.append(v)
            stack.pop()
        else:
            # Caso contrário, segue para o próximo vértice no caminho
            next_v = grafo.lista_adjacencia[v].pop()
            stack.append(next_v)
            out_degrees[v] -= 1

    # Retorna o caminho Euleriano na ordem correta
    return path[::-1]

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
    """
    Função que retorna um caminho Hamiltoniano, se existir.
    Um caminho Hamiltoniano é um caminho que passa por todos os vértices do grafo exatamente uma vez.
    Utiliza um algoritmo de backtracking para encontrar o caminho.
    """
    def backtrack(v, visitado, caminho):
        # Se o caminho contém todos os vértices, retorna True
        if len(caminho) == grafo.vertices:
            return True

        for vizinho in sorted(grafo.lista_adjacencia[v]):
            if not visitado[vizinho]:
                visitado[vizinho] = True
                caminho.append(vizinho)
                if backtrack(vizinho, visitado, caminho):
                    return True
                # Se não for possível continuar, remove o último vértice do caminho
                caminho.pop()
                visitado[vizinho] = False

        return False

    for vertice_inicial in range(grafo.vertices):
        visitado = [False] * grafo.vertices
        caminho = [vertice_inicial]
        visitado[vertice_inicial] = True
        if backtrack(vertice_inicial, visitado, caminho):
            return caminho

    # Se nenhum caminho Hamiltoniano for encontrado, retorna uma lista vazia
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
    """
    Função que gera a árvore de profundidade do grafo.
    A árvore de profundidade é uma árvore gerada pela busca em profundidade (DFS).
    """
    def dfs(v, visitado, arestas_arvore):
        visitado[v] = True  # Marca o vértice atual como visitado
        # Ordena os vizinhos em ordem reversa para garantir a ordem correta na árvore
        for vizinho in sorted(grafo.lista_adjacencia[v], reverse=True):
            if not visitado[vizinho]:
                arestas_arvore.append((v, vizinho))  # Adiciona a aresta à árvore
                dfs(vizinho, visitado, arestas_arvore)

    visitado = [False] * grafo.vertices
    arestas_arvore = []  # Lista para armazenar as arestas da árvore de profundidade

    dfs(0, visitado, arestas_arvore)

    # Retorna a lista de arestas da árvore ou -1 se não houver nenhuma
    return arestas_arvore if arestas_arvore else -1

def arvore_largura(grafo, vertice_inicial=0):
    """
    Função que gera a árvore de largura do grafo.
    A árvore de largura é uma árvore gerada pela busca em largura (BFS).
    """
    visitado = {v: False for v in grafo.lista_adjacencia}
    arestas_arvore_bfs = []
    fila = deque([vertice_inicial])

    visitado[vertice_inicial] = True

    while fila:
        vertice_atual = fila.popleft()
        vizinhos = sorted(grafo.lista_adjacencia[vertice_atual])

        for vizinho in vizinhos:
            if not visitado[vizinho]:
                visitado[vizinho] = True
                arestas_arvore_bfs.append((vertice_atual, vizinho))
                fila.append(vizinho)

    return arestas_arvore_bfs if arestas_arvore_bfs else -1

def arvore_geradora_minima(grafo):
    """
    Função que encontra a árvore geradora mínima do grafo.
    A árvore geradora mínima é uma árvore que conecta todos os vértices do grafo com o menor peso possível.
    Utiliza o algoritmo de Prim para encontrar a árvore geradora mínima.
    """
    arestas_agm = []
    visitado = [False] * grafo.vertices
    min_heap = [(0, 0, 0)]

    while min_heap and len(arestas_agm) < grafo.vertices - 1:
        _, u, v = heapq.heappop(min_heap)
        if visitado[v]:
            continue
        visitado[v] = True
        if u != v:
            arestas_agm.append((u, v))
        
        for vizinho in sorted(grafo.lista_adjacencia[v]):
            if not visitado[vizinho]:
                heapq.heappush(min_heap, (1, v, vizinho))

    return arestas_agm if len(arestas_agm) == grafo.vertices - 1 else -1

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
    """
    Função que encontra o caminho mínimo entre dois vértices.
    O caminho mínimo é o caminho de menor custo entre dois vértices de um grafo.
    Utiliza o algoritmo de Dijkstra para encontrar o caminho mínimo.
    """
    if fim is None:
        fim = grafo.vertices - 1

    dist = [float('inf')] * grafo.vertices
    dist[inicio] = 0
    min_heap = [(0, inicio)]

    while min_heap:
        dist_atual, u = heapq.heappop(min_heap)
        if dist_atual > dist[u]:
            continue

        for vizinho in sorted(grafo.lista_adjacencia[u]):
            distancia = dist_atual + 1
            if distancia < dist[vizinho]:
                dist[vizinho] = distancia
                heapq.heappush(min_heap, (distancia, vizinho))

    return dist[fim] if dist[fim] != float('inf') else -1

def fluxo_maximo(grafo, origem=0, destino=None):
    """
    Função que calcula o fluxo máximo em um grafo direcionado.
    """
    if not grafo.direcionado or destino is None:
        return -1

    def bfs(capacidade, origem, destino, pai):
        visitado = [False] * len(capacidade)
        fila = deque([origem])
        visitado[origem] = True

        while fila:
            u = fila.popleft()
            for v in range(len(capacidade)):
                if not visitado[v] and capacidade[u][v] > 0:
                    fila.append(v)
                    visitado[v] = True
                    pai[v] = u
                    if v == destino:
                        return True
        return False

    capacidade = [[0] * grafo.vertices for _ in range(grafo.vertices)]
    for u in range(grafo.vertices):
        for v in grafo.lista_adjacencia[u]:
            capacidade[u][v] = 1

    pai = [-1] * grafo.vertices
    fluxo_maximo = 0

    while bfs(capacidade, origem, destino, pai):
        fluxo_caminho = float('Inf')
        s = destino
        while s != origem:
            fluxo_caminho = min(fluxo_caminho, capacidade[pai[s]][s])
            s = pai[s]

        fluxo_maximo += fluxo_caminho
        v = destino
        while v != origem:
            u = pai[v]
            capacidade[u][v] -= fluxo_caminho
            capacidade[v][u] += fluxo_caminho
            v = pai[v]

    return fluxo_maximo

def fecho_transitivo(grafo):
    """
    Função que calcula o fecho transitivo de um grafo.
    Utiliza o algoritmo de Warshall para calcular o fecho transitivo.
    """
    if not grafo.direcionado:
        return -1

    # Inicializa a matriz de fecho transitivo com zeros
    fecho = [[0] * grafo.vertices for _ in range(grafo.vertices)]
    
    # Adiciona as arestas diretas à matriz de fecho transitivo
    print("\nVerificando lista de adjacência ao adicionar arestas diretas:")
    for u in range(grafo.vertices):
        print(f"Acessando vértice {u} com vizinhos {grafo.lista_adjacencia[u]}")
        for v in grafo.lista_adjacencia[u]:
            fecho[u][v] = 1  # Existe uma aresta de u para v
            print(f"Adicionando aresta direta de {u} para {v} na matriz de fecho")

    print("Matriz após adicionar arestas diretas:")
    for linha in fecho:
        print(linha)
    
    # Adiciona laços (auto-laços) na diagonal
    for i in range(grafo.vertices):
        fecho[i][i] = 1
    
    print("Matriz após adicionar laços (auto-laços):")
    for linha in fecho:
        print(linha)
    
    # Aplica o algoritmo de Warshall para calcular o fecho transitivo
    print("Matriz de fecho transitivo antes do algoritmo de Warshall:")
    for linha in fecho:
        print(linha)
    
    for k in range(grafo.vertices):
        print(f"Processando vértice intermediário {k}:")
        for i in range(grafo.vertices):
            for j in range(grafo.vertices):
                if fecho[i][j] != (fecho[i][j] or (fecho[i][k] and fecho[k][j])):
                    print(f"Atualizando fecho[{i}][{j}] via {k}")
                fecho[i][j] = fecho[i][j] or (fecho[i][k] and fecho[k][j])
        
        print(f"Matriz após processar vértice intermediário {k}:")
        for linha in fecho:
            print(linha)
    
    return fecho



