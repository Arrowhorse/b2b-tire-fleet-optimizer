import pandas as pd
import pydeck as pdk
import streamlit as st
from src.logistics import B2BFleetRouter, REGIONAL_NETWORKS, build_road_matrices

st.set_page_config(
    page_title="B2B Tire Fleet & Route Optimizer",
    page_icon="🚚",
    layout="wide",
)


@st.cache_data(ttl=3600)
def get_cached_matrices(region_name: str, selected_ids: tuple):
    all_nodes = REGIONAL_NETWORKS[region_name]
    active_nodes = [all_nodes[0]] + [n for n in all_nodes[1:] if n.store_id in selected_ids]
    dist_mat, time_mat, is_live_osrm = build_road_matrices(active_nodes)
    return active_nodes, dist_mat, time_mat, is_live_osrm


# --- Header ---
st.title("🚚 B2B Regional Tire Distribution & Fleet Route Optimizer")
st.markdown(
    "Commercial multi-stop delivery route planner using **A* Search with a memoized Prim's Minimum Spanning Tree (MST) heuristic** "
    "over real highway networks to minimize fleet mileage, driver hours, and fuel burn."
)

# --- Sidebar Controls ---
st.sidebar.header("1. Distribution Network")
region = st.sidebar.selectbox("Regional Distribution Hub", list(REGIONAL_NETWORKS.keys()))
region_nodes = REGIONAL_NETWORKS[region]
depot = region_nodes[0]
available_stores = region_nodes[1:]

store_options = {f"{s.store_id}: {s.name} ({s.tire_order_qty} tires)": s.store_id for s in available_stores}
selected_labels = st.sidebar.multiselect(
    "Active B2B Customer Orders Today",
    options=list(store_options.keys()),
    default=list(store_options.keys()),
)
selected_ids = tuple(store_options[lbl] for lbl in selected_labels)

st.sidebar.header("2. Fleet & Optimization Settings")
optimize_metric = st.sidebar.radio("Primary Objective", ["time", "distance"], format_func=lambda x: "Fastest Drive Time" if x == "time" else "Shortest Road Distance")
truck_capacity = st.sidebar.number_input("Box Truck Capacity (Tires)", min_value=100, max_value=600, value=350, step=25)
mpg = st.sidebar.slider("Fleet Fuel Efficiency (MPG)", min_value=5.0, max_value=18.0, value=9.5, step=0.5)
fuel_price = st.sidebar.number_input("Fuel Cost ($/gal)", min_value=2.0, max_value=8.0, value=3.85, step=0.05)
driver_wage = st.sidebar.number_input("Driver Loaded Rate ($/hr)", min_value=18.0, max_value=75.0, value=34.0, step=1.0)

if len(selected_ids) < 2:
    st.warning("Select at least 2 customer tire stores in the sidebar to optimize a multi-stop route.")
    st.stop()

# --- Compute Matrices & Solve Routes ---
active_nodes, dist_mat, time_mat, is_live_osrm = get_cached_matrices(region, selected_ids)
total_tires = sum(n.tire_order_qty for n in active_nodes)

if total_tires > truck_capacity:
    st.error(
        f"⚠️ **Payload Warning:** Total order volume (**{total_tires} tires**) exceeds single-truck capacity (**{truck_capacity} tires**). "
        "Remove a stop or increase truck capacity in the sidebar."
    )
else:
    st.success(
        f"✅ **Load Verified:** {total_tires} / {truck_capacity} tires ({total_tires / truck_capacity * 100:.0f}% truck utilization) • "
        f"Routing Engine: {'Live OpenStreetMap Road Network (OSRM)' if is_live_osrm else 'Road-Adjusted Geodesic Model'}"
    )

router = B2BFleetRouter(
    nodes=active_nodes,
    dist_matrix=dist_mat,
    time_matrix=time_mat,
    mpg=mpg,
    fuel_price=fuel_price,
    driver_hourly_rate=driver_wage,
)

opt_res = router.solve_astar_mst(optimize_by=optimize_metric)
base_res = router.solve_manifest_baseline()

# --- KPI Metrics ---
miles_saved = base_res.total_miles - opt_res.total_miles
mins_saved = base_res.total_minutes - opt_res.total_minutes
cost_saved = base_res.total_cost - opt_res.total_cost

c1, c2, c3, c4 = st.columns(4)
c1.metric("Optimal Route Distance", f"{opt_res.total_miles} mi", delta=f"-{miles_saved:.1f} mi vs. FIFO", delta_color="inverse")
c2.metric("Total Drive Time", f"{opt_res.total_minutes:.0f} min", delta=f"-{mins_saved:.0f} min saved", delta_color="inverse")
c3.metric("Dispatch Operating Cost", f"${opt_res.total_cost:.2f}", delta=f"-${cost_saved:.2f} / trip", delta_color="inverse")
c4.metric("A* Solver Latency", f"{opt_res.compute_ms} ms", help=f"States expanded: {opt_res.states_expanded}")

st.divider()

# --- Map & Manifest ---
col_map, col_seq = st.columns([1.45, 1])

with col_map:
    st.subheader("🗺️ Optimal B2B Highway Delivery Loop")
    show_baseline = st.checkbox("Overlay Unoptimized FIFO Route (Dashed Red)", value=True)

    layers = []
    if show_baseline:
        layers.append(
            pdk.Layer(
                "PathLayer",
                data=[{"path": base_res.route_coords, "name": "Unoptimized FIFO Route"}],
                get_path="path",
                get_color=[239, 68, 68, 130],
                width_min_pixels=3,
            )
        )

    layers.append(
        pdk.Layer(
            "PathLayer",
            data=[{"path": opt_res.route_coords, "name": "Optimal A* Route"}],
            get_path="path",
            get_color=[37, 99, 235, 230],
            width_min_pixels=5,
        )
    )

    pin_rows = []
    for idx, node in enumerate(opt_res.stop_sequence[:-1]):
        pin_rows.append(
            {
                "lat": node.lat,
                "lon": node.lon,
                "label": f"DEPOT: {node.name}" if node.is_depot else f"Stop #{idx}: {node.name} ({node.tire_order_qty} tires)",
                "color": [16, 185, 129, 240] if node.is_depot else [245, 158, 11, 240],
                "radius": 1100 if node.is_depot else 800,
            }
        )

    layers.append(
        pdk.Layer(
            "ScatterplotLayer",
            data=pin_rows,
            get_position="[lon, lat]",
            get_fill_color="color",
            get_radius="radius",
            pickable=True,
        )
    )

    view_state = pdk.ViewState(latitude=depot.lat, longitude=depot.lon, zoom=9.2, pitch=0)
    st.pydeck_chart(
        pdk.Deck(
            layers=layers,
            initial_view_state=view_state,
            tooltip={"text": "{label}"},
        )
    )

with col_seq:
    st.subheader("📋 Driver Turn-by-Turn Stop Sequence")
    manifest_rows = []
    cum_tires = 0
    for idx, node in enumerate(opt_res.stop_sequence):
        if idx == 0:
            role = "Origin DC (Load Truck)"
        elif idx == len(opt_res.stop_sequence) - 1:
            role = "Return to DC (Empty)"
        else:
            role = f"Delivery Stop #{idx}"
            cum_tires += node.tire_order_qty
        manifest_rows.append(
            {
                "Leg": idx,
                "Role": role,
                "Location": node.name,
                "City": node.city,
                "Tires Dropped": node.tire_order_qty if not node.is_depot else 0,
            }
        )
    st.dataframe(pd.DataFrame(manifest_rows), use_container_width=True, hide_index=True)

# --- Economics Comparison Table ---
st.subheader("📊 Fleet Economics & Emissions Comparison")
comp_df = pd.DataFrame(
    [
        {
            "Dispatch Strategy": opt_res.strategy,
            "Total Miles": opt_res.total_miles,
            "Drive Time (min)": opt_res.total_minutes,
            "Fuel Cost ($)": opt_res.fuel_cost,
            "Driver Labor ($)": opt_res.driver_cost,
            "Total Trip Cost ($)": opt_res.total_cost,
            "Est. CO₂ (kg)": opt_res.co2_kg,
        },
        {
            "Dispatch Strategy": base_res.strategy,
            "Total Miles": base_res.total_miles,
            "Drive Time (min)": base_res.total_minutes,
            "Fuel Cost ($)": base_res.fuel_cost,
            "Driver Labor ($)": base_res.driver_cost,
            "Total Trip Cost ($)": base_res.total_cost,
            "Est. CO₂ (kg)": base_res.co2_kg,
        },
    ]
)
st.dataframe(comp_df, use_container_width=True, hide_index=True)