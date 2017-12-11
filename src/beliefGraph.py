"""
6.01 State Estimation Graphs.
"""

from tkinter import Canvas, Entry, Label, LEFT, RIGHT, StringVar, \
                    Tk, Frame, Button, Toplevel


class Grapher(Toplevel):
    """
    Simulator for the estimation process in the lab.
    """

    def __init__(self, ideal):
        Toplevel.__init__(self)
        self.title('State Estimation Graphs')
        self._ugly_hidden_init(ideal)

    def _ugly_hidden_init(self, ideal):
        self.ideal = ideal
        self.numStates = len(ideal)
        self._CELL_WIDTH = 3
        self._CELL_HEIGHT = 50
        self._MARGIN = 5
        self._setupUI()

    def _setupUI(self):
        """
        Setup UI.
        """
        self.resizable(width=False, height=False)

        #obs model
        self.obsgraph = [Canvas(self, width=self._CELL_WIDTH, height=50) for i in range(self.numStates)]
        self.obsLabel = Label(self, text='P(O|S):')
        self.obsLabel.grid(row=19, column=0)
        for ix in range(len(self.obsgraph)):
            self.obsgraph[ix].grid(row=19, column=ix+2,ipady=5)
            self.obsgraph[ix]._old = None
            self.obsgraph[ix].create_rectangle(0,
                                               0,
                                               self._CELL_WIDTH,
                                               50, fill='white', outline='white') #clear
            self.obsgraph[ix].height = 50
            self.obsgraph[ix].color = 'blue'

        self.posteriorgraph = [Canvas(self, width=self._CELL_WIDTH, height=100) for i in range(self.numStates)]
        self.posteriorLabel = Label(self, text='Belief:')
        self.posteriorLabel.grid(row=24, column=0)
        for ix in range(len(self.posteriorgraph)):
            self.posteriorgraph[ix].grid(row=24, column=ix+2,ipady=5)
            self.posteriorgraph[ix]._old = None
            self.posteriorgraph[ix].create_rectangle(0,
                                                     0,
                                                     self._CELL_WIDTH,
                                                     100, fill='white', outline='white') #clear
            self.posteriorgraph[ix].height=100
            self.posteriorgraph[ix].color = 'blue'

        self.distgraph = [Canvas(self, width=self._CELL_WIDTH, height=50) for i in range(self.numStates)]
        self.distLabel = Label(self, text='Ideal\nDistance')
        self.distLabel.grid(row=7, column=0)
        for ix in range(len(self.distgraph)):
            self.distgraph[ix].grid(row=7, column=ix+2,ipady=5)
            self.distgraph[ix]._old = None
            self.distgraph[ix].create_rectangle(0,
                                               0,
                                               self._CELL_WIDTH,
                                               50, fill='red', outline='white') #clear
            self.distgraph[ix].height = 50
            self.distgraph[ix].color = 'white'
        self.maxDist = float(max(self.ideal)+3)
        self._drawGraph(self.distgraph, [i/self.maxDist for i in self.ideal])

        self.observation = StringVar()
        self.obsLabel = Label(self, textvariable=self.observation, height=1)
        self.obsLabel.grid(row=4,column=0)

        self.updateObsGraph([0 for i in range(self.numStates)])
        self.updateBeliefGraph([0 for i in range(self.numStates)])

    def updateObsLabel(self,obs):
        self.observation.set('Observed %d' % obs)

    def updateObsGraph(self,values):
        self._drawGraph(self.obsgraph,values)

    def updateBeliefGraph(self,values):
        self._drawGraph(self.posteriorgraph,values)

    def updateDist(self):
        self._drawGraph(self.distgraph,
                        [i/self.maxDist for i in self.ideal])

    def updateTrueRobot(self, s):
        box = self.distgraph[s]
        if box._old is not None:
            box.delete(box._old)
        box._old = box.create_rectangle(0, 0,
                                        self._CELL_WIDTH,
                                        box.height, fill='black', outline='black')

    def _drawGraph(self,graphvar,values):
        '''
        Make our rudimentary bar graph
        '''
        for (box,val) in zip(graphvar,values):
            if box._old is not None:
                box.delete(box._old)
            h = int(round(((1-val)*box.height)))
            if h != box.height:
                box._old = box.create_rectangle(0,
                                 (1-val)*box.height,
                                 self._CELL_WIDTH,
                                 box.height, fill=box.color, outline=box.color)
            else:
                box._old = None
