import numpy as np
from queue import *
from classAresta import *
    


    

class Graph_l:   
    def __init__(self, v, a, d): #(número de vértices, arestas, direcionado(bool))
        #self.arquivo_saida = [] #Lista que vai receber os resultados desejados
        self.v = v
        self.a = a
        self.d = d 
        self.num_arestas = len(self.a) 
        self.neg = False
        self.lista = [[] for i in range(self.v)] #Não sabemos quantos elementos tem em cada linha (quantos vizinhos cada vértice tem)
        self.lista_residual = [[] for i in range(self.v)]
        #self.pesos = [[] for i in range(self.v)]  #lista que contém os pesos 
        if self.d==False: #grafo não direcionado
            for i in range(self.num_arestas):
                u=self.a[i][0] #Pega o primeiro vértice do par de arestas
                v= self.a[i][1] #Pega o segundo vértice do par de arestas
                p= self.a[i][2] #Pega o peso da aresta
                if p < 0: self.neg = True
                self.lista[u-1]+= [[v ,p]] #Adiciona o vértice v na linha u (o índice é u-1, pois i se inicia no 0)
                self.lista[v-1]+= [[u, p]] #Adiciona o vértice u na linha v
                #self.pesos[u-1]+= [p]
                #self.pesos[v-1] += [p]


        else: #grafo direcionado
            for i in range(self.num_arestas):
                u=self.a[i][0] #Pega o primeiro vértice do par de arestas
                v= self.a[i][1] #Pega o segundo vértice do par de arestas
                c= self.a[i][2]
                original = Aresta(u, v, c, 0, False)
                reversa = Aresta(u, v, c, 0, True)
                self.lista[u-1].append(Aresta(u, v, c, 0, False))
                self.lista_residual[u-1].append(original)
                self.lista_residual[v-1].append(reversa)
                reversa.opointer= original
                original.rpointer = reversa



    def mostra_lista(self): #Retorna a lista de adjacência
        return self.lista 
    

    def BFS_ff(self, vi):
        marcados = np.zeros(self.v,dtype=int) #Lista com os nós que já foram explorados
        Q = Queue() #Cria uma fila vazia
        pai = np.array([-1] * self.v, dtype = int) #inicia vetor com os pais
        nivel = np.array([-1] * self.v, dtype = int) #inicia vetor dos níveis
        nivel[vi -1] = 0
        marcados[vi-1]=1#Marca vi, que é o nó em que começamos a busca
        Q.add(vi) #Adciona o nó na fila
        pai[vi-1] = 0 #Indica que é raiz 
        while Q.is_empty() == False : #A busca continua até a lista ficar vazia(Até todos os vértices serem explorados)
            v = Q.pop() #Remove o último vértice da fila, que corresponde ao vértice mais antigo
            for w in self.lista_residual[v-1] : #Visita os vizinhos de v
                #if w.cap_residual == 0.0 and w.residual == False: pass
                if marcados[w.v2-1]==0 and w.cap_residual>0: 
                    pai[w.v2-1]= v #Se w não tiver sido marcado, o vértice que o descobriu(seu pai) foi v
                    marcados[w.v2-1]=1 #Marca w se não foi descoberto ainda
                    Q.add(w.v2) #Adiciona w na primeira poição da fila
        return pai
    
    def get_gargalo(self, caminho):
        gargalo = np.inf
        t = len(caminho)
        for i in range (t-1):
            v_1= caminho[i]
            v_2 = caminho[i+1]
            for viz in self.lista_residual[v_2-1]: 
                if (viz.v2 ==v_1) and viz.residual is False:
                    #print(viz.v1,viz.v2, viz.cap_residual, viz.residual)
                    if viz.cap_residual < gargalo:
                         gargalo = viz.cap_residual
        #print(caminho)
        #print(gargalo)
        return gargalo    
    
    def get_caminho(self, s, t):
        pais = self.BFS_ff(s)
        v = pais[t-1]
        if v == -1: return 0.0
        caminho = [t]
        while v != s:
            caminho.append(v)
            v = pais[v-1]
        caminho.append(s)
        if -1 in caminho: return 0.0
        #print(caminho)
        gargalo = self.get_gargalo(caminho)
        #print(gargalo)
        for vertice in caminho:
            for aresta in self.lista_residual[vertice-1]:
                if (aresta.v1 and aresta.v2) in caminho:
                    aresta.atualiza_arestas(gargalo)
                    #print(f'capacidade {aresta.v1} e {aresta.v2} :{aresta.cap_residual}')
        return gargalo
    
    def guarda_resultado(self, grafo: str):
        arquivo_saida = open(f'resultado_{grafo}.txt', 'w')
        arquivo_saida.write(f'Fluxo máximo entre os vértices 1 e 2: {self.fluxoMax}\n')
        arquivo_saida.write('Alocação de fluxo:\n')
        for vertice in range(len(self.lista_residual)):
            for aresta in self.lista_residual[vertice]:
                arquivo_saida.write(f'{aresta.v1} {aresta.v2} {aresta.fluxo}\n' )
        arquivo_saida.close

    def FF(self, s, t, grafo: str, disco: bool):
        self.fluxoMax = 0.0
        gargalo = -1
        while gargalo != 0.0:
            gargalo = self.get_caminho(s,t)
            #print(gargalo)
            self.fluxoMax += gargalo
        for v in range(self.v):
            for aresta in self.lista_residual[v-1]:
                if aresta.residual == True:
                    self.lista_residual[v-1].remove(aresta)
        
               
        if disco:
            self.guarda_resultado(grafo)                    
        return self.fluxoMax
