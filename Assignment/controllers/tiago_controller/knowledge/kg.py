from rdflib import Graph, Namespace, RDF, URIRef

HOME = Namespace("http://example.org/home#")

class KG:
    def __init__(self):
        self.g = Graph()
        self._waypoints = {}       # symbol -> DEF-name in .wbt
        self._path = []            # ordered list of symbols

    # symbolische plaats ↔ DEF-naam
    def add_waypoint(self, symbol: str, def_name: str):
        self._waypoints[symbol] = def_name
        s = URIRef(HOME[symbol])
        self.g.add((s, RDF.type, HOME.Place))

    def get_waypoint_def(self, symbol: str) -> str:
        return self._waypoints[symbol]

    def list_places(self):
        return list(self._waypoints.keys())

    # A -> B -> C
    def set_path(self, symbols: list[str]):
        self._path = symbols[:]
        for a, b in zip(symbols, symbols[1:]):
            sa = URIRef(HOME[a]); sb = URIRef(HOME[b])
            self.g.set((sa, HOME.next, sb))

    def get_path(self) -> list[str]:
        return self._path[:]

    # optioneel: gebruikt door executor; laat stilletjes toe
    def assert_robot_at(self, robot="tiago", place=None):
        if place is None: 
            return
        r = URIRef(HOME[robot]); p = URIRef(HOME[place])
        self.g.add((r, RDF.type, HOME.Robot))
        self.g.add((p, RDF.type, HOME.Place))
        self.g.set((r, HOME.at, p))