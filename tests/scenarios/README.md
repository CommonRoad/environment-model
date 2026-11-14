# Test Scenarios

This directory contains [CommonRoad](https://commonroad.in.tum.de/) scenarios in XML and protobuf format.
They are used exclusively by the C++ and Python test suites.

Most scenarios were created by the Cyber-Physical Systems Group of the Technical University of Munich, for example with the
[CommonRoad Scenario Designer](https://github.com/CommonRoad/commonroad-scenario-designer).
Several scenarios are taken from the [CommonRoad scenario database](https://commonroad.in.tum.de/scenarios).
They are partially derived from the third-party data sources listed below.
Each scenario file names its origin in the `source`, `author`, and `affiliation` attributes of its root element.

## Data sources and attributions

| Source | Scenarios (examples) | Attribution / license |
|---|---|---|
| [OpenStreetMap](https://www.openstreetmap.org/) | ARG_Carcarana-6, BEL_Zwevegem-1, DEU_Guetersloh-25, ESP_Almansa-2, ITA_Foggia-7, USA_Lanker-1, USA_Peach-2, USA_Peach-4, ZAM_TestReadingIntersectionTrafficSign-1 | Map data © OpenStreetMap contributors, available under the [Open Database License (ODbL)](https://opendatacommons.org/licenses/odbl/) |
| [Next Generation Simulation (NGSIM)](https://ops.fhwa.dot.gov/trafficanalysistools/ngsim.htm) | USA_Lanker-1, USA_Peach-2, USA_Peach-4 | Trajectory data provided by the U.S. Department of Transportation, Federal Highway Administration |
| [SUMO](https://eclipse.dev/sumo/) traffic simulation | ARG_Carcarana-6, BEL_Zwevegem-1, DEU_Guetersloh-25, ESP_Almansa-2, ITA_Foggia-7 | Traffic generated with SUMO by the CommonRoad Scenario Factory |
| Research project Ko-HAF | DEU_Frankfurt-70 | Created within the Ko-HAF project (German Federal Ministry for Economic Affairs and Energy) and published in the CommonRoad scenario database |
| BMW Group (Car@TUM) | ZAM_Urban-2 | Created within the Car@TUM project and published in the CommonRoad scenario database |
| Aerial imagery (Bing Maps) | DEU_Muc-2 | Road network manually modeled from aerial imagery and published in the CommonRoad scenario database |
