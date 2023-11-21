import numpy as np
import heapq
from collections import defaultdict
from queue import *


class Graph_l:   
    def __init__(self, v, a, d): #(número de vértices, arestas, direcionado(bool))
        self.aqruivo_saida = [] #Lista que vai receber os resultados desejados
        self.v = v
        self.a = a
        self.d = d 
        self.num_arestas = len(self.a) 
        self.neg = False
        self.lista = [[] for i in range(self.v)] #Não sabemos quantos elementos tem em cada linha (quantos vizinhos cada vértice tem)
        self.residual = [[] for i in range(self.v)]
        self.residual_d = {i:{j: [] for j in range(self.v+1)} for i in range(self.v)}
        self.pesos = [[] for i in range(self.v)]  #lista que contém os pesos 
        if self.d==False:
            for i in range(self.num_arestas):
                u=self.a[i][0] #Pega o primeiro vértice do par de arestas
                v= self.a[i][1] #Pega o segundo vértice do par de arestas
                p= self.a[i][2] #Pega o peso da aresta
                if p < 0: self.neg = True
                self.lista[u-1]+= [[v ,p]] #Adiciona o vértice v na linha u (o índice é u-1, pois i se inicia no 0)
                self.lista[v-1]+= [[u, p]] #Adiciona o vértice u na linha v
                self.pesos[u-1]+= [p]
                self.pesos[v-1] += [p]
        else:
            for i in range(self.num_arestas):
                u=self.a[i][0] #Pega o primeiro vértice do par de arestas
                v= self.a[i][1] #Pega o segundo vértice do par de arestas
                p= self.a[i][2]
                if p < 0: self.neg = True
                self.lista[u-1]+= [[v, p]] #Adiciona o vértice v na linha u (o índice é u-1, pois i se inicia no 0)
                self.residual[u-1] += [[v, p, True]] #Aresta original
                self.residual_d[u-1][v] += [[p, True]]
                self.residual[v-1] += [[u, 0.0, False]] #Aresta residual
                self.residual_d[v-1][u] += [[0.0, False]]
                self.pesos[u-1]+= [p]   

    def mostra_lista(self): #Retorna a lista de adjacência
        return self.lista 
    
    def BFS_ff(self, vi):
        self.marcados = np.zeros(self.v,dtype=int) #Lista com os nós que já foram explorados
        self.Q = Queue() #Cria uma fila vazia
        pai = np.array([-1] * self.v, dtype = int) #inicia vetor com os pais
        nivel = np.array([-1] * self.v, dtype = int) #inicia vetor dos níveis
        nivel[vi -1] = 0
        self.marcados[vi-1]=1#Marca vi, que é o nó em que começamos a busca
        self.Q.add(vi) #Adciona o nó na fila
        pai[vi-1] = 0 #Indica que é raiz 
        while self.Q.is_empty() == False: #A busca continua até a lista ficar vazia(Até todos os vértices serem explorados)
            v = self.Q.pop() #Remove o último vértice da fila, que corresponde ao vértice mais antigo
            for w in self.residual[v-1]: #Visita os vizinhos de v
                #print(f'bfs:\n {w}\n{self.residual[v-1]}')
                #if w[2] == False: pass
                #else:
                if self.marcados[w[0]-1]==0: 
                    pai[w[0]-1]= v #Se w não tiver sido marcado, o vértice que o descobriu(seu pai) foi v
                    self.marcados[w[0]-1]=1 #Marca w se não foi descoberto ainda
                    self.Q.add(w[0]) #Adiciona w na primeira poição da fila
        return pai

    def BFS(self,vi,p): #Implementação da busca em largura
        self.marcados = np.zeros(self.v,dtype=int) #Lista com os nós que já foram explorados
        self.Q = Queue() #Cria uma fila vazia
        pai = np.array([-1] * self.v, dtype = int) #inicia vetor com os pais
        nivel = np.array([-1] * self.v, dtype = int) #inicia vetor dos níveis
        nivel[vi -1] = 0
        self.marcados[vi-1]=1#Marca vi, que é o nó em que começamos a busca
        self.Q.add(vi) #Adciona o nó na fila
        pai[vi-1] = 0 #Indica que é raiz 
        while self.Q.is_empty() == False: #A busca continua até a lista ficar vazia(Até todos os vértices serem explorados)
            v = self.Q.pop() #Remove o último vértice da fila, que corresponde ao vértice mais antigo
            for w in self.lista[v-1]: #Visita os vizinhos de v
                if self.marcados[w[0]-1]==0: 
                    pai[w[0]-1]= v #Se w não tiver sido marcado, o vértice que o descobriu(seu pai) foi v
                    nivel[w[0]-1] = nivel[v-1]+1 #Muda o nível atual para o nível de w
                    self.marcados[w[0]-1]=1 #Marca w se não foi descoberto ainda
                    self.Q.add(w[0]) #Adiciona w na primeira poição da fila

        self.maxlevel = np.max(nivel) #Pega o nível máximo

        self.BFStree = [pai, nivel, self.maxlevel] 
        if p==1:
            print(f'\nBusca BFS:\nPais: {pai}\nNíveis: {nivel}')
        return (self.BFStree)

    def get_gargalo(self, caminho):
        gargalo = np.inf
        for v in caminho:
            for viz in self.residual[v-1]:
                if viz[0] in caminho and viz[1] < gargalo and viz[2] == True:
                    gargalo = viz[1]
        return gargalo
    
    def get_caminho(self, s, t):
        pais = self.BFS_ff(s)
        print(pais)
        a = pais[t-1]
        if a == -1: return 0.0
        caminho = [t]
        while a != s:
            caminho.append(a)
            a = pais[a-1]
        caminho.append(s)
        print(caminho)
        gargalo = self.get_gargalo(caminho)
        print(gargalo)
        for v in caminho:
            for viz in self.residual[v-1]:
                if viz[0] in caminho:
                    if viz[2] == False: #aresta residual
                        viz[1] += gargalo
                    else: #aresta normal
                        viz[1] -= gargalo
                    if viz[1] == 0.0:
                        self.residual[v-1].remove(viz)
        return gargalo
    

    def aumentante(self, s, t):
        pais = self.BFS_ff(s)
        v = pais[t-1]
        caminho = [t]
        if v == -1: return 0.0 #se o pai de t for -1, não há caminho até ele
        while v != s:
            caminho.append(v)
            v = pais[v-1] #vai pegando o pai de v até chegar em s
        caminho.append(s)
        gargalo = self.get_gargalo(caminho)
        for u in caminho:
            for viz in self.residual[u-1]:
                if viz[0] in caminho and viz[2] == True: #aresta original no caminho
                    viz[1] -= gargalo #atualiza a capacidade
                    print(f'vizinhos do {viz[0]}: {self.residual[viz[0]-1]}')
                    for w in self.residual[viz[0]-1]:
                        print(w)
                        if w[0] == u and w[2] == False:
                            w[1] += gargalo #atualiza o fluxo na aresta residual
                            print(f'atualização: {w[1]}\nnova representação: {w}\nteste: {self.residual[viz[0]-1]}')
                            break
                elif viz[0] in caminho and viz[2] == False: #aresta residual no caminho
                    viz[1] += gargalo #atualiza o fluxo
                    print(f'vizinhos do {viz[0]}: {self.residual[viz[0]-1]}')
                    for z in self.residual[viz[0]-1]:
                        print(z)
                        if z[0] == u and z[2] == True:
                            z[1] -= gargalo #atualiza a capacidade da aresta original correspondente
                            print(f'atualização: {z[1]}\nnova representação: {z}\nteste: {self.residual[viz[0]-1]}')
                            break
        if viz[1] == 0.0 and viz[2] == True:
                    self.residual[u-1].remove(viz)
        return gargalo

    


    def FF(self, s, t):
        fluxoMax = 0.0
        gargalo = -1
        while gargalo != 0.0:
            #gargalo = self.get_caminho(s,t)
            gargalo = self.aumentante(s,t)
            fluxoMax += gargalo
        for v in range(self.v):
            for aresta in self.residual[v-1]:
                if aresta[2] == False:
                    self.residual[v-1].remove(aresta)
                    
        return fluxoMax, self.residual


