# Note: This world exists so that the brain has a world to generate readings for, even when interacting with a real
# robot. This world is never actually loaded by Soar; it serves as a template for gapWorld*.py
from soar.sim.world import *

world = World(dimensions=(8, 1.5), initial_position=(2.3, 0.5, 0.0), objects=[Wall((1.5,1), (2,1)),Wall((2.75, 1), (3.25, 1)),
                                                                              Wall((4, 1), (4.5, 1)),
                                                                              Wall((5.25, 1), (5.75, 1))])

