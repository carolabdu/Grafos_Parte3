class Aresta:
    def __init__(self, v1: int, v2: int, c, f, res: bool):
        self.v1 = v1
        self.v2 = v2
        self.cap = c
        self.fluxo = f
        self.residual = res
        if self.residual == True: #aresta residual
            self.cap_residual = self.fluxo
        else:
            self.cap_residual = self.cap - self.fluxo
          
        self.opointer = None
        self.rpointer = None
    def atualiza_arestas(self, gargalo):
        self.fluxo += gargalo
        if self.residual == True:
            self.cap_residual = self.fluxo
        else:
            self.cap_residual = self.cap - self.fluxo
            self.rpointer.atualiza_arestas(gargalo)
