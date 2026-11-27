# 🚚 B2B Regional Tire Distribution & Fleet Route Optimizer

A commercial multi-stop delivery route planner and fleet economics simulator built with **Python**, **Streamlit**, **PyDeck**, and the **OpenStreetMap Routing Machine (OSRM) API**.

The application optimizes regional B2B tire distribution routes across real highway networks—benchmarking **A* Search with a memoized Prim's Minimum Spanning Tree (MST) heuristic** against an unoptimized First-In, First-Out (FIFO) dispatch manifest across Northern Virginia / DC Metro, Chicagoland, and Dallas–Fort Worth hubs.

**🔗 Live Interactive App:** *[https://b2b-tire-fleet-optimizer-cmczggg9asrduxhtfajbyl.streamlit.app/]*

---

## Key Algorithmic & Engineering Highlights
- **Real Highway Network Matrices (`src/logistics.py`):** Queries the public OSRM Table & Route APIs (`router.project-osrm.org`) to compute real-world $N \times N$ driving-time and mileage matrices alongside turn-by-turn road polylines, with an automatic road-circuity-adjusted Haversine fallback.
- **Exact A* + Prim's MST Tour Sequencing:** Models round-trip delivery routing over `(current_stop, unvisited_stores)` and guides A* search using an admissible lower bound combining nearest-entry edge cost, memoized **Prim's Minimum Spanning Tree (MST)** cost across remaining stops, and minimum return-to-depot cost.
- **Commercial Fleet Economics & Payload Validation:** Validates single-truck box-truck tire capacity and quantifies trip-level savings across total mileage, driver labor hours, fuel burn (MPG), operating cost ($), and estimated $\text{CO}_2$ emissions.
- **Interactive Dispatch Map (`app.py`):** Renders real highway polylines and interactive store/depot pins with **PyDeck** (`PathLayer` + `ScatterplotLayer`) alongside a turn-by-turn driver manifest.

---

## Run Locally
```bash
git clone [https://github.com/Arrowhorse/b2b-tire-fleet-optimizer.git](https://github.com/Arrowhorse/b2b-tire-fleet-optimizer.git)
cd b2b-tire-fleet-optimizer
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\Activate.ps1
pip install -r requirements.txt
streamlit run app.py

Attribution & Academic Acknowledgment
The underlying multi-stop state-space search formulation and memoized Prim's Minimum Spanning Tree (MST) lower-bound heuristic were originally inspired by the UC Berkeley CS188 Intro to AI search curriculum (created by John DeNero and Dan Klein; http://ai.berkeley.edu). All road-network routing integration, fleet cost modeling, regional hub datasets, and PyDeck visualization components in this repository were independently engineered as an original applied portfolio project.
