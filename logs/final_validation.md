# Final Validation — SafeTCPF SUMO Simulation

**Date:** 2026-06-25  
**SUMO Version:** 1.27.0  
**Status:** ✅ SIMULATION SUCCESSFULLY FIXED

---

## Vehicle Statistics

| Metric | Value |
|--------|-------|
| Total vehicles defined | 16 |
| Vehicles inserted | 16 |
| Vehicles completed | 16 |
| Vehicles stuck/waiting | 0 |
| Vehicles teleported | 0 |

## Route Statistics

| Route ID | Direction | Edges | Vehicles Using |
|----------|-----------|-------|---------------|
| r_NS | North → South | N_to_C → C_to_S | ns_0, ns_1, ns_2 |
| r_SN | South → North | S_to_C → C_to_N | sn_0, sn_1, sn_2 |
| r_EW | East → West | E_to_C → C_to_W | ew_0, ew_1, ew_2 |
| r_WE | West → East | W_to_C → C_to_E | we_0, we_1, we_2 |
| r_NW | North → West | N_to_C → C_to_W | nw_0, nw_1 |
| r_SE | South → East | S_to_C → C_to_E | se_0, se_1 |
| **Total unique routes** | **6** | | **16 vehicles** |

## Pedestrian Statistics

| Metric | Value |
|--------|-------|
| Pedestrian vType defined | ✅ (id="ped", vClass="pedestrian", width=0.5) |
| Total pedestrians defined | 6 |
| Pedestrians inserted | 6 |
| Pedestrians completed | 6 |
| Avg walk route length | 150.21 m |
| Avg walk duration | 116.33 s |
| Avg walk time loss | 13.21 s |
| Network sidewalks | ✅ (auto-generated via --sidewalks.guess) |
| Junction crossings | ✅ (auto-generated via --crossings.guess) |

### Individual Pedestrian Results

| Pedestrian | Direction | Depart | Arrival | Duration | Route Length | Time Loss |
|-----------|-----------|--------|---------|----------|-------------|-----------|
| ped_ns_0 | N→S | 2.00 | 141.00 | 139.00 s | 155.69 m | 14.10 s |
| ped_sn_0 | S→N | 4.00 | 126.00 | 122.00 s | 155.69 m | 11.75 s |
| ped_ew_0 | E→W | 6.00 | 131.00 | 125.00 s | 155.69 m | 14.14 s |
| ped_we_0 | W→E | 8.00 | 108.00 | 100.00 s | 155.69 m | 10.39 s |
| ped_nw_0 | N→W | 10.00 | 120.00 | 110.00 s | 139.26 m | 18.81 s |
| ped_se_0 | S→E | 12.00 | 114.00 | 102.00 s | 139.26 m | 10.10 s |

## Individual Vehicle Results

| Vehicle | Route | Depart | Arrival | Duration | Route Length | Wait Time | Time Loss |
|---------|-------|--------|---------|----------|-------------|-----------|-----------|
| ns_0 | N→S | 0.00 | 15.00 | 15.00 s | 194.90 m | 0.00 s | 0.65 s |
| sn_0 | S→N | 0.00 | 16.00 | 16.00 s | 194.90 m | 0.00 s | 0.75 s |
| ew_0 | E→W | 0.00 | 18.00 | 18.00 s | 194.90 m | 0.00 s | 3.18 s |
| we_0 | W→E | 0.00 | 19.00 | 19.00 s | 194.90 m | 0.00 s | 4.47 s |
| nw_0 | N→W | 2.00 | 22.00 | 20.00 s | 188.30 m | 0.00 s | 4.93 s |
| se_0 | S→E | 2.00 | 21.00 | 19.00 s | 188.30 m | 0.00 s | 2.00 s |
| ew_1 | E→W | 5.00 | 25.00 | 20.00 s | 194.90 m | 0.00 s | 4.48 s |
| ns_1 | N→S | 10.00 | 32.00 | 22.00 s | 194.90 m | 5.00 s | 7.49 s |
| sn_1 | S→N | 10.00 | 30.00 | 20.00 s | 194.90 m | 4.00 s | 4.63 s |
| ew_2 | E→W | 10.00 | 32.00 | 22.00 s | 194.90 m | 0.00 s | 8.06 s |
| we_1 | W→E | 10.00 | 38.00 | 28.00 s | 194.90 m | 11.00 s | 13.47 s |
| nw_1 | N→W | 15.00 | 39.00 | 24.00 s | 188.30 m | 6.00 s | 9.56 s |
| se_1 | S→E | 15.00 | 36.00 | 21.00 s | 188.30 m | 2.00 s | 5.18 s |
| ns_2 | N→S | 20.00 | 42.00 | 22.00 s | 194.90 m | 7.00 s | 6.39 s |
| sn_2 | S→N | 20.00 | 40.00 | 20.00 s | 194.90 m | 6.00 s | 5.76 s |
| we_2 | W→E | 20.00 | 52.00 | 32.00 s | 194.90 m | 11.00 s | 22.48 s |

## Simulation Duration

| Metric | Value |
|--------|-------|
| Simulation begin | 0.00 s |
| Simulation end | 1000.00 s |
| Last vehicle arrival | 52.00 s |
| Last pedestrian arrival | 141.00 s |
| All entities cleared by | 141.00 s |

## Validation Checklist

| Criterion | Status |
|-----------|--------|
| ✅ Vehicles appear | **PASS** — 16/16 inserted |
| ✅ Vehicles spawn on roads | **PASS** — all depart on valid lanes |
| ✅ Vehicles follow routes | **PASS** — all arrive at correct destination |
| ✅ Vehicles reach the intersection | **PASS** — all cross center junction |
| ✅ Vehicles leave the intersection | **PASS** — all have valid arrival times |
| ✅ No teleporting | **PASS** — 0 teleports |
| ✅ No invalid routes | **PASS** — 0 routing errors |
| ✅ No SUMO routing errors | **PASS** — 0 errors |
| ✅ No TraCI errors | **PASS** — 0 exceptions |
| ✅ No Python exceptions | **PASS** — clean execution |
| ✅ No SUMO warnings | **PASS** — 0 warnings |
| ✅ Pedestrians appear | **PASS** — 6/6 inserted |
| ✅ Pedestrians walk routes | **PASS** — all walked and arrived |
| ✅ Sidewalk infrastructure | **PASS** — sidewalks on all edges |
| ✅ Junction crossings | **PASS** — crossings at center junction |

---

## Result

**SIMULATION SUCCESSFULLY FIXED**
