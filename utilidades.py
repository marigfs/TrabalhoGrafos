from grafo import Grafo


def ler_grafo_do_exemplo():
    """
    Função que lê um exemplo de grafo a partir de uma entrada do usuário.
    A string de entrada deve seguir o formato:
    - A primeira linha contém o ID máximo do vértice e o número de arestas.
    - A segunda linha indica se o grafo é direcionado ou não.
    - As linhas subsequentes representam as arestas no formato: id_aresta u v p,
      onde u é o vértice de origem, v é o vértice de destino, e p é o peso da aresta.

    Retorna:
    - Um objeto da classe Grafo com os vértices, arestas e a informação se é direcionado.
    """
    
    while True:
        try:
            print("Digite os dados do grafo no formato correto ou 'exemplo' para usar um grafo predefinido:")
            print("Primeira linha: <ID máximo do vértice> <número de arestas>")
            print("Segunda linha: 'direcionado' ou 'nao_direcionado'")
            print("Linhas seguintes: <id_aresta> <u> <v> <p> (podem ser inseridos sem espaço)")
            print("Digite 'parar' em qualquer linha para finalizar a entrada de dados ou 'exemplo' para usar um grafo predefinido.")
            
            # Lendo a entrada do usuário
            dados_entrada = []
            print("\nDigite o grafo (pressione Enter duas vezes ou digite 'parar' para finalizar):")
            while True:
                linha = input().strip()
                if linha.lower() == "parar":
                    if len(dados_entrada) == 0:
                        print("Entrada de dados interrompida pelo usuário.")
                        return None
                    break
                if linha.lower() == "exemplo":
                    return usar_exemplo()
                if linha == "":
                    break
                dados_entrada.append(linha)

            # Verifica se há pelo menos 2 linhas (necessário para um grafo válido)
            if len(dados_entrada) < 2:
                raise ValueError("Formato inválido! O grafo deve ter pelo menos 2 linhas de entrada.")

            # Processa as linhas da entrada
            linhas = dados_entrada
            id_maximo_vertice, num_arestas = map(int, linhas[0].split())
            direcionado = linhas[1].strip().lower()

            if direcionado not in ['direcionado', 'nao_direcionado']:
                raise ValueError("Formato inválido! A segunda linha deve conter 'direcionado' ou 'nao_direcionado'.")

            direcionado = direcionado == 'direcionado'

            arestas = []

            # Processa as arestas a partir das linhas subsequentes
            for linha in linhas[2:]:
                partes = list(filter(None, linha.split()))  # Remove elementos vazios resultantes da divisão
                if len(partes) == 1:
                    partes = [partes[0][i:i+1] for i in range(4)]  # Divide os elementos se estiverem juntos
                elif len(partes) != 4:
                    raise ValueError("Formato inválido! Cada aresta deve ser representada por 4 valores: <id_aresta> <u> <v> <p>.")
                
                u, v, p = int(partes[1]), int(partes[2]), int(partes[3])
                arestas.append((u, v, p))

            num_vertices = id_maximo_vertice + 1  # Calcula o número total de vértices

            # Retorna o objeto Grafo criado com os dados processados
            return Grafo(num_vertices, arestas, direcionado)
        
        except ValueError as ve:
            print(f"\nErro: {ve}")
            print("Por favor, insira os dados novamente no formato correto.\n")


def usar_exemplo():
    """
    Função que retorna um grafo exemplo predefinido.
    O grafo é não direcionado e possui as seguintes arestas:
    3 4 
    nao_direcionado 
    0 0 1 1
    1 1 2 1 
    2 1 3 1
    3 2 3 1
    """

    id_maximo_vertice = 3
    direcionado = False
    arestas = [
        (0, 1, 1),
        (1, 2, 1),
        (1, 3, 1),
        (2, 3, 1)
    ]
    num_vertices = id_maximo_vertice + 1
    return Grafo(num_vertices, arestas, direcionado)





def imprimir_resultados(resultados):
    """
    Função que imprime os resultados obtidos a partir das funções executadas no grafo.
    
    Parâmetros:
    - resultados: Dicionário contendo os resultados das funções executadas no grafo.
    """
    for chave, valor in resultados.items():
        print(f"{chave}: {valor}")
