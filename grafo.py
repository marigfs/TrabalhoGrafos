class Grafo:
    def __init__(self, vertices, arestas, direcionado=False):
        """
        Construtor da classe Grafo.
        
        Parâmetros:
        - vertices: Número de vértices no grafo.
        - arestas: Lista de arestas no formato (u, v, p), onde:
            u é o vértice de origem,
            v é o vértice de destino,
            p é o peso da aresta.
        - direcionado: Booleano que indica se o grafo é direcionado. Se False, o grafo é não direcionado.
        
        Este método inicializa a lista de adjacência do grafo e armazena os pesos das arestas.
        """
                        
        self.vertices = vertices  # Número de vértices no grafo
        self.direcionado = direcionado  # Indica se o grafo é direcionado ou não
        self.lista_adjacencia = {i: [] for i in range(vertices)}  # Inicializa a lista de adjacência para cada vértice
        self.lista_adjacencia_inversa = {i: [] for i in range(vertices)}  # Para armazenar a lista de adjacência do grafo transposto
        self.pesos_arestas = {}  # Dicionário para armazenar os pesos das arestas
        
        # Itera sobre cada aresta fornecida e preenche a lista de adjacência e os pesos das arestas
        for aresta in arestas:
            u, v, p = aresta
            if u >= vertices or v >= vertices:
                raise ValueError(f"Erro: Os vértices {u} ou {v} estão fora do intervalo permitido (0 a {vertices-1}).")
            
            # Adiciona a aresta à lista de adjacência
            self.lista_adjacencia[u].append(v)
            self.pesos_arestas[(u, v)] = p  # Adiciona o peso da aresta ao dicionário
            
            # Se o grafo for direcionado, preenche a lista de adjacência inversa
            if direcionado:
                self.lista_adjacencia_inversa[v].append(u)  # Para o grafo transposto
            else:
                self.lista_adjacencia[v].append(u)
                self.pesos_arestas[(v, u)] = p  # Aresta bidirecional (para grafos não direcionados)


    def __str__(self):
        """
        Método que retorna uma representação em string da lista de adjacência do grafo.
        Este método é útil para visualizar a estrutura do grafo.
        """
        return f"Adjacência: {self.lista_adjacencia}\nAdjacência Inversa: {self.lista_adjacencia_inversa if self.direcionado else 'N/A'}"
