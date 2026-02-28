# Drone Delivery System

A simulation of a drone delivery dispatch system that optimizes trip planning based on battery capacity, order priority, and safety constraints.

## Overview

The system manages drone deliveries across multiple zones. It packages orders into trips, simulates battery consumption based on payload weight and distance, and delivers orders while respecting customer preferences and safety rules.

## Features

- **Trip simulation** – Simulates drone trips with battery tracking; determines if orders can be delivered without running out of battery
- **Optimal positioning** – Finds the best position to add an order to minimize battery usage
- **Zone-based trips** – Packs orders by zone (oldest first), then fills remaining capacity by weight (heaviest to lightest)
- **Optimized delivery** – Addresses customer feedback:
  - Same-customer orders shipped together when possible
  - Subscriber orders prioritized for faster shipping
  - Fragile and hazardous orders never in the same trip
  - Perishable goods moved to the front of the queue

## Priority logic

`priority_score` is a single number; lower value means higher urgency:

| Score | Meaning                |
|------:|------------------------|
| 0     | perishable + subscriber |
| 1     | perishable only        |
| 2     | subscriber only        |
| 3     | neither                |

## Project Structure

```
drone_project/
├── drone.py           # Drone class: trip simulation, battery management, order positioning
├── dispatch_server.py # Dispatch logic: trip building, order packaging, delivery coordination
├── order.py           # Order model with zone, weight, flags (subscriber, fragile, hazardous, perishable)
├── delivery_zones.py  # Zone distance matrix and lookups
├── runner.py          # Main entry point: runs test drone, basic delivery, and optimized delivery
├── run_delivery.sh    # Shell script to run the simulation
├── distances.csv      # Distance matrix between zones (km)
└── deliveries.csv     # Order data (id, timestamp, zone, weight, user_id, flags)
```

## Requirements

- Python 3.6+

## Running the Project

```bash
python runner.py
```

Or use the shell script:

```bash
./run_delivery.sh
```

The runner executes:

1. **Test drone** – Verifies that the drone correctly accepts a small order and rejects an oversized one
2. **Basic delivery** – Packages trips by zone (oldest first) and delivers
3. **Optimized delivery** – Packages trips using customer-priority rules and delivers

## Data Format

**deliveries.csv** – Each row: `order_id, timestamp, zone, weight, user_id, subscriber, fragile, hazardous, perishable`

**distances.csv** – First row: number of zones; remaining rows: distance matrix (zone-to-zone distances in km)
