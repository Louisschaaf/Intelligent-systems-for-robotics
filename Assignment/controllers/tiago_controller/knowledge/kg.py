from rdflib import Graph, Namespace, Literal, RDF, URIRef

HOME = Namespace("http://example.org/home#")

class KG:
    def __init__(self):
        self.g = Graph()

    def assert_robot_at(self, robot="tiago", place="base"):
        r = URIRef(HOME[robot]); p = URIRef(HOME[place])
        self.g.add((r, RDF.type, HOME.Robot))
        self.g.add((p, RDF.type, HOME.Place))
        self.g.set((r, HOME.at, p))

    def assert_item_at(self, item, place):
        i = URIRef(HOME[item]); p = URIRef(HOME[place])
        self.g.add((i, RDF.type, HOME.Item))
        self.g.set((i, HOME.at, p))

    def to_pddl_init(self):
        # voorbeeldje hoe je later PDDL init kunt genereren
        inits = []
        for s, p, o in self.g:
            if p == HOME.at and (str(s).endswith("Robot")):
                inits.append(f"(at tiago base)")
        return inits