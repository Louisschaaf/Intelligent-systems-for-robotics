from rdflib import Graph, Namespace, Literal, RDF, URIRef

HOME = Namespace("http://example.org/home#")
# hardcoded room bounds
AREAS = {
    "room1": {"x": (-5, 0.0), "y": (0.0, 5)},    # NW
    "room2": {"x": (0.0, 5), "y": (0.0, 5)},     # NE
    "room3": {"x": (-5, 0.0), "y": (-5, 0.0)},   # SW
    "room4": {"x": (0.0, 5), "y": (-5, 0.0)},    # SE
}

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
    
    def assert_on(self, item, support):
        i = HOME[item]
        s = HOME[support]
        self.g.add((s, RDF.type, HOME.Support))
        self.g.set((i, HOME.on, s))
    
    def get_robot_location(self, robot="tiago"):
        q = f"""
        SELECT ?place WHERE {{
            home:{robot} home:at ?place .
        }}
        """
        res = self.g.query(q, initNs={"home": HOME})
        for row in res:
            return str(row.place).split("#")[-1]
        return None
      
    def map_position_to_area(self, pos):
        x, y, z = pos
        for area, bounds in AREAS.items():
            if bounds["x"][0] <= x <= bounds["x"][1] and bounds["y"][0] <= y <= bounds["y"][1]:
                return area
        return "unknown"


    def to_pddl_init(self):
        inits = []
        for s, p, o in self.g:
            s_name = str(s).split("#")[-1]
            p_name = str(p).split("#")[-1]
            o_name = str(o).split("#")[-1]
    
            if p_name == "at":
                inits.append(f"(at {s_name} {o_name})")
            elif p_name == "on":
                inits.append(f"(on {s_name} {o_name})")
    
        return inits