import heapq
import math
import time
from dataclasses import dataclass
from typing import Dict, FrozenSet, List, Optional, Tuple
import requests

OSRM_BASE = "https://router.project-osrm.org"


@dataclass
class StoreNode:
    store_id: str
    name: str
    city: str
    lat: float
    lon: float
    tire_order_qty: int
    is_depot: bool = False


@dataclass
class DispatchResult:
    strategy: str
    stop_sequence: List[StoreNode]
    total_miles: float
    total_minutes: float
    fuel_cost: float
    driver_cost: float
    total_cost: float
    co2_kg: float
    states_expanded: int
    compute_ms: float
    route_coords: List[List[float]]  # [[lon, lat], ...] for PyDeck PathLayer


REGIONAL_NETWORKS: Dict[str, List[StoreNode]] = {
    "Northern Virginia / DC Metro Hub": [
        StoreNode("DC-01", "Braddock Regional Tire DC", "Fairfax, VA", 38.8282, -77.3064, 0, is_depot=True),
        StoreNode("ST-101", "Tysons Corner Auto & Tire", "Tysons, VA", 38.9187, -77.2311, 42),
        StoreNode("ST-102", "Manassas Commercial Fleet Service", "Manassas, VA", 38.7509, -77.4753, 55),
        StoreNode("ST-103", "Leesburg Tire & Alignment", "Leesburg, VA", 39.1157, -77.5636, 30),
        StoreNode("ST-104", "Alexandria Potomac Tire Pros", "Alexandria, VA", 38.8048, -77.0469, 48),
        StoreNode("ST-105", "Woodbridge Expressway Tire Co.", "Woodbridge, VA", 38.6582, -77.2497, 36),
        StoreNode("ST-106", "Reston Dulles Tire Center", "Reston, VA", 38.9586, -77.3570, 28),
        StoreNode("ST-107", "Bethesda Beltway Tire & Wheel", "Bethesda, MD", 38.9847, -77.0947, 40),
        StoreNode("ST-108", "Gainesville Interstate Truck Tire", "Gainesville, VA", 38.7957, -77.6139, 50),
    ],
    "Chicagoland Distribution Hub": [
        StoreNode("DC-02", "Elk Grove O'Hare Tire DC", "Elk Grove Village, IL", 42.0039, -87.9703, 0, is_depot=True),
        StoreNode("ST-201", "Schaumburg Radial & Fleet", "Schaumburg, IL", 42.0334, -88.0834, 45),
        StoreNode("ST-202", "Naperville West Tire Depot", "Naperville, IL", 41.7508, -88.1535, 52),
        StoreNode("ST-203", "Evanston North Shore Tire", "Evanston, IL", 42.0451, -87.6877, 34),
        StoreNode("ST-204", "South Loop Commercial Tire", "Chicago, IL", 41.8576, -87.6270, 60),
        StoreNode("ST-205", "Oak Lawn Midway Tire Co.", "Oak Lawn, IL", 41.7199, -87.7479, 38),
        StoreNode("ST-206", "Joliet I-80 Truck Tire Center", "Joliet, IL", 41.5250, -88.0817, 48),
        StoreNode("ST-207", "Waukegan Lake County Tire", "Waukegan, IL", 42.3636, -87.8448, 32),
    ],
    "Dallas–Fort Worth Metroplex Hub": [
        StoreNode("DC-03", "Arlington DFW Central Tire DC", "Arlington, TX", 32.7357, -97.1081, 0, is_depot=True),
        StoreNode("ST-301", "Fort Worth Stockyards Tire", "Fort Worth, TX", 32.7887, -97.3486, 44),
        StoreNode("ST-302", "Plano North Tollway Tire", "Plano, TX", 33.0198, -96.6989, 50),
        StoreNode("ST-303", "Irving Airport Fleet Service", "Irving, TX", 32.8140, -96.9489, 35),
        StoreNode("ST-304", "Garland Eastside Tire Co.", "Garland, TX", 32.9126, -96.6389, 40),
        StoreNode("ST-305", "Denton I-35 Commercial Tire", "Denton, TX", 33.2148, -97.1331, 46),
        StoreNode("ST-306", "Mesquite Highway 80 Tire", "Mesquite, TX", 32.7668, -96.5992, 38),
    ],
}


def haversine_miles(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    r = 3958.8  # Earth radius in miles
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def build_road_matrices(nodes: List[StoreNode]) -> Tuple[List[List[float]], List[List[float]], bool]:
    """
    Queries OSRM Table API for real driving distance (miles) and duration (minutes).
    Falls back to road-circuity-adjusted Haversine if OSRM is unreachable.
    """
    n = len(nodes)
    coords_str = ";".join(f"{node.lon},{node.lat}" for node in nodes)
    url = f"{OSRM_BASE}/table/v1/driving/{coords_str}?annotations=distance,duration"

    try:
        resp = requests.get(url, timeout=8)
        resp.raise_for_status()
        data = resp.json()
        if data.get("code") == "Ok":
            # OSRM returns meters and seconds
            dist_miles = [[(d or 0.0) / 1609.344 for d in row] for row in data["distances"]]
            dur_mins = [[(t or 0.0) / 60.0 for t in row] for row in data["durations"]]
            return dist_miles, dur_mins, True
    except Exception:
        pass

    # Fallback: 1.28x circuity factor for urban/suburban road networks at ~38 mph avg
    dist_miles = [[0.0] * n for _ in range(n)]
    dur_mins = [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(n):
            if i != j:
                d = haversine_miles(nodes[i].lat, nodes[i].lon, nodes[j].lat, nodes[j].lon) * 1.28
                dist_miles[i][j] = d
                dur_mins[i][j] = (d / 38.0) * 60.0
    return dist_miles, dur_mins, False


def fetch_road_geometry(ordered_nodes: List[StoreNode]) -> List[List[float]]:
    """Fetches actual turn-by-turn road polyline coordinates from OSRM Route API."""
    coords_str = ";".join(f"{node.lon},{node.lat}" for node in ordered_nodes)
    url = f"{OSRM_BASE}/route/v1/driving/{coords_str}?overview=full&geometries=geojson"
    try:
        resp = requests.get(url, timeout=8)
        resp.raise_for_status()
        data = resp.json()
        if data.get("code") == "Ok":
            return data["routes"][0]["geometry"]["coordinates"]
    except Exception:
        pass
    return [[node.lon, node.lat] for node in ordered_nodes]


class B2BFleetRouter:
    """Optimizes multi-stop B2B delivery tours returning to the regional distribution center."""

    def __init__(
        self,
        nodes: List[StoreNode],
        dist_matrix: List[List[float]],
        time_matrix: List[List[float]],
        mpg: float = 9.5,
        fuel_price: float = 3.85,
        driver_hourly_rate: float = 32.0,
    ):
        self.nodes = nodes
        self.dist = dist_matrix
        self.time = time_matrix
        self.mpg = mpg
        self.fuel_price = fuel_price
        self.driver_hourly_rate = driver_hourly_rate
        self.mst_cache: Dict[FrozenSet[int], float] = {}

    def _prim_mst_lower_bound(self, curr_idx: int, remaining: FrozenSet[int], cost_mat: List[List[float]]) -> float:
        """
        Admissible lower bound for returning to depot (index 0) after visiting all `remaining` stops:
        min_edge(curr -> remaining) + MST(remaining) + min_edge(remaining -> depot 0).
        """
        if not remaining:
            return cost_mat[curr_idx][0]

        if remaining in self.mst_cache:
            mst_cost = self.mst_cache[remaining]
        else:
            items = list(remaining)
            if len(items) <= 1:
                mst_cost = 0.0
            else:
                tree = {items[0]}
                unvisited = set(items[1:])
                mst_cost = 0.0
                while unvisited:
                    best_w = float("inf")
                    best_v = -1
                    for v in unvisited:
                        w = min(cost_mat[u][v] for u in tree)
                        if w < best_w:
                            best_w, best_v = w, v
                    mst_cost += best_w
                    tree.add(best_v)
                    unvisited.remove(best_v)
            self.mst_cache[remaining] = mst_cost

        entry = min(cost_mat[curr_idx][v] for v in remaining)
        exit_to_depot = min(cost_mat[v][0] for v in remaining)
        return entry + mst_cost + exit_to_depot

    def solve_astar_mst(self, optimize_by: str = "time") -> DispatchResult:
        """Finds globally optimal round-trip delivery sequence using A* + Prim's MST heuristic."""
        t0 = time.perf_counter()
        self.mst_cache.clear()
        cost_mat = self.time if optimize_by == "time" else self.dist

        all_stops = frozenset(range(1, len(self.nodes)))
        start_state = (0, all_stops)

        frontier = [(self._prim_mst_lower_bound(0, all_stops, cost_mat), 0.0, start_state)]
        best_g = {start_state: 0.0}
        came_from: Dict[Tuple[int, FrozenSet[int]], Optional[Tuple[int, FrozenSet[int]]]] = {start_state: None}
        states_expanded = 0

        while frontier:
            _, g, state = heapq.heappop(frontier)
            curr_idx, remaining = state

            if not remaining:
                # Reconstruct stop indices and close loop back to Depot (0)
                seq_indices = []
                cur = state
                while cur is not None:
                    seq_indices.append(cur[0])
                    cur = came_from[cur]
                seq_indices.reverse()
                seq_indices.append(0)

                elapsed_ms = (time.perf_counter() - t0) * 1000
                return self._build_result(
                    f"Optimal A* (Prim's MST — {optimize_by.title()})",
                    seq_indices,
                    states_expanded,
                    elapsed_ms,
                )

            if g > best_g.get(state, float("inf")):
                continue

            states_expanded += 1
            for nxt_idx in remaining:
                nxt_remaining = remaining - {nxt_idx}
                nxt_state = (nxt_idx, nxt_remaining)
                new_g = g + cost_mat[curr_idx][nxt_idx]

                # Add return-to-depot cost on final step so A* compares complete tours
                step_extra = cost_mat[nxt_idx][0] if not nxt_remaining else 0.0
                total_path_g = new_g + step_extra

                if total_path_g < best_g.get(nxt_state, float("inf")):
                    best_g[nxt_state] = total_path_g
                    h = self._prim_mst_lower_bound(nxt_idx, nxt_remaining, cost_mat)
                    came_from[nxt_state] = state
                    heapq.heappush(frontier, (total_path_g + h, total_path_g, nxt_state))

        raise RuntimeError("No valid delivery tour found.")

    def solve_manifest_baseline(self) -> DispatchResult:
        """Simulates an unoptimized dispatch manifest (visiting stops in raw order-entry sequence)."""
        t0 = time.perf_counter()
        seq_indices = list(range(len(self.nodes))) + [0]
        elapsed_ms = (time.perf_counter() - t0) * 1000
        return self._build_result(
            "Unoptimized Order Manifest (FIFO)",
            seq_indices,
            len(self.nodes),
            elapsed_ms,
        )

    def _build_result(
        self, strategy: str, seq_indices: List[int], states_expanded: int, elapsed_ms: float
    ) -> DispatchResult:
        total_miles = sum(self.dist[seq_indices[i]][seq_indices[i + 1]] for i in range(len(seq_indices) - 1))
        total_mins = sum(self.time[seq_indices[i]][seq_indices[i + 1]] for i in range(len(seq_indices) - 1))
        ordered_nodes = [self.nodes[i] for i in seq_indices]

        gallons = total_miles / max(self.mpg, 1.0)
        fuel_cost = gallons * self.fuel_price
        driver_cost = (total_mins / 60.0) * self.driver_hourly_rate
        co2_kg = gallons * 10.18  # EPA diesel/commercial box-truck factor (~10.18 kg CO2/gal)

        return DispatchResult(
            strategy=strategy,
            stop_sequence=ordered_nodes,
            total_miles=round(total_miles, 1),
            total_minutes=round(total_mins, 1),
            fuel_cost=round(fuel_cost, 2),
            driver_cost=round(driver_cost, 2),
            total_cost=round(fuel_cost + driver_cost, 2),
            co2_kg=round(co2_kg, 1),
            states_expanded=states_expanded,
            compute_ms=round(elapsed_ms, 2),
            route_coords=fetch_road_geometry(ordered_nodes),
        )