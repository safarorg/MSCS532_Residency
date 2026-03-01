"""
analysis.py — Performance comparison for Greedy vs Optimized drone delivery algorithms.

Runs both algorithms on small, medium, and large synthetic datasets and
produces a PNG chart and a CSV summary.

Usage:
    python analysis.py
"""

import csv
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from delivery_zones import DeliveryZones
from dispatch_server import DispatchServer


# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────

def _trip_distance(trip: list, delivery_zones: DeliveryZones) -> float:
    """Return the total km flown for a single trip (including return to warehouse)."""
    prev = 0  # start at warehouse (zone 0)
    total = 0.0
    for order in trip:
        zone = order.get_delivery_zone()
        total += delivery_zones.distance_between(prev, zone)
        prev = zone
    total += delivery_zones.distance_between(prev, 0)  # return leg
    return total


def _trip_battery_used(trip: list, delivery_zones: DeliveryZones) -> float:
    """
    Simulate a full trip from zone 0 at 100% charge and return how much
    battery was consumed (0.0 – 1.0).
    """
    empty_overhead = 512
    consumption_factor = 36739
    battery = 1.0
    prev = 0
    payload = sum(o.get_weight() for o in trip)
    for order in trip:
        dist = delivery_zones.distance_between(prev, order.get_delivery_zone())
        battery -= dist * (payload + empty_overhead) / consumption_factor
        payload -= order.get_weight()
        prev = order.get_delivery_zone()
    dist = delivery_zones.distance_between(prev, 0)
    battery -= dist * (0 + empty_overhead) / consumption_factor
    return 1.0 - battery  # fraction consumed


def _fragile_hazardous_violations(trips: list) -> int:
    """Count trips that contain both fragile and hazardous orders."""
    violations = 0
    for trip in trips:
        has_fragile = any(o.is_fragile() for o in trip)
        has_hazardous = any(o.is_hazardous() for o in trip)
        if has_fragile and has_hazardous:
            violations += 1
    return violations


# ──────────────────────────────────────────────────────────────────────────────
# Analyser
# ──────────────────────────────────────────────────────────────────────────────

class DeliveryAnalyzer:
    """Runs performance comparisons between the greedy and optimized algorithms."""

    def __init__(self):
        self.delivery_zones = DeliveryZones()
        self.delivery_zones.load_matrix('distances.csv')

    def run_comparison(self, csv_file: str, sort_keys: list, label: str) -> dict:
        """
        Run the algorithm with the given sort_keys on csv_file and collect metrics.

        Returns a dict with one key (label) mapping to a nested dict of metrics.
        """
        server = DispatchServer(self.delivery_zones)
        server.load_orders(csv_file)

        t_start = time.perf_counter()
        server.schedule_orders(sort_keys)
        t_end = time.perf_counter()
        runtime_ms = (t_end - t_start) * 1000

        trips = server.trips
        n_trips = len(trips)
        order_counts = [len(t) for t in trips]
        distances = [_trip_distance(t, self.delivery_zones) for t in trips]
        battery_used = [_trip_battery_used(t, self.delivery_zones) for t in trips]
        violations = _fragile_hazardous_violations(trips)

        metrics = {
            'total_trips': n_trips,
            'total_distance_km': sum(distances),
            'avg_orders_per_trip': sum(order_counts) / n_trips if n_trips else 0,
            'min_orders_per_trip': min(order_counts) if order_counts else 0,
            'max_orders_per_trip': max(order_counts) if order_counts else 0,
            'runtime_ms': runtime_ms,
            'avg_battery_used_pct': (sum(battery_used) / n_trips * 100
                                     if n_trips else 0),
            'frag_haz_violations': violations,
        }
        print(f'  [{label:>16}] trips={n_trips:4d}  '
              f'dist={sum(distances):7.1f} km  '
              f'runtime={runtime_ms:7.1f} ms  '
              f'violations={violations}')
        return {label: metrics}

    def plot_results(self, all_results: dict, output_path: str = 'analysis.png') -> None:
        """
        Plot a 2×3 grid of bar charts comparing both algorithms across dataset sizes.

        Parameters:
            all_results  Dict keyed by dataset label (e.g. 'Small (100)') ->
                         comparison dict from run_comparison().
            output_path  File path for the PNG output.
        """
        labels = list(all_results.keys())
        metrics = [
            ('total_trips',           'Total Trips',           'Trips'),
            ('total_distance_km',     'Total Distance (km)',   'Km'),
            ('avg_orders_per_trip',   'Avg Orders / Trip',     'Orders'),
            ('runtime_ms',            'Runtime (ms)',           'ms'),
            ('avg_battery_used_pct',  'Avg Battery Used (%)',   '%'),
            ('frag_haz_violations',   'Fragile+Hazardous Violations', 'Count'),
        ]

        fig, axes = plt.subplots(2, 3, figsize=(14, 8))
        fig.suptitle('Drone Delivery: Sort Key Ordering Comparison',
                     fontsize=14, fontweight='bold')
        axes = axes.flatten()

        x = range(len(labels))

        # Derive config labels from data; build a colour palette that scales with N
        config_labels = list(list(all_results.values())[0].keys())
        n_configs = len(config_labels)
        _palette = ['#4C72B0', '#DD8452', '#55A868', '#C44E52', '#8172B2']
        colors = {lbl: _palette[i % len(_palette)] for i, lbl in enumerate(config_labels)}

        # Bar width scales so all bars fit neatly inside each group
        bar_width = 0.8 / n_configs

        for ax, (metric_key, title, ylabel) in zip(axes, metrics):
            for i, config_lbl in enumerate(config_labels):
                offset = bar_width * (i - (n_configs - 1) / 2)
                vals = [all_results[lbl][config_lbl][metric_key] for lbl in labels]
                bars = ax.bar([xi + offset for xi in x], vals,
                              bar_width, label=config_lbl,
                              color=colors[config_lbl], alpha=0.85)
                for bar in bars:
                    h = bar.get_height()
                    ax.text(bar.get_x() + bar.get_width() / 2, h * 1.01,
                            f'{h:.1f}', ha='center', va='bottom', fontsize=7)

            ax.set_title(title, fontsize=10)
            ax.set_ylabel(ylabel, fontsize=9)
            ax.set_xticks(list(x))
            ax.set_xticklabels(labels, fontsize=8)
            ax.legend(fontsize=8)
            ax.grid(axis='y', linestyle='--', alpha=0.4)

        plt.tight_layout()
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f'Chart saved to {output_path}')

    def export_to_csv(self, all_results: dict,
                      output_path: str = 'analysis_results.csv') -> None:
        """
        Export comparison results to CSV.

        Each row is one (dataset_size, algorithm) combination.
        """
        fieldnames = [
            'dataset', 'algorithm',
            'total_trips', 'total_distance_km',
            'avg_orders_per_trip', 'min_orders_per_trip', 'max_orders_per_trip',
            'runtime_ms', 'avg_battery_used_pct', 'frag_haz_violations',
        ]
        with open(output_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for dataset_label, comparison in all_results.items():
                for algo_label, metrics in comparison.items():
                    row = {'dataset': dataset_label, 'algorithm': algo_label}
                    row.update(metrics)
                    writer.writerow(row)
        print(f'Results exported to {output_path}')


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    datasets = [
        ('Baseline (31)',  'deliveries.csv'),
        ('Small (100)',    'deliveries_small.csv'),
        ('Medium (500)',   'deliveries_medium.csv'),
        ('Large (2000)',   'deliveries_large.csv'),
    ]

    CONFIGS = [
        ('Zone-Only', ['delivery_zone']),
        ('Priority-First',  ['priority_score']),
        ('Real-world', ['priority_score', 'delivery_zone', 'timestamp']),
    ]

    analyzer = DeliveryAnalyzer()
    all_results = {}

    for dataset_label, csv_file in datasets:
        print(f'\n--- {dataset_label} ---')
        comparison = {}
        for config_label, sort_keys in CONFIGS:
            result = analyzer.run_comparison(csv_file, sort_keys, config_label)
            comparison.update(result)
        all_results[dataset_label] = comparison

    analyzer.plot_results(all_results, output_path='analysis_2.png')
    analyzer.export_to_csv(all_results, output_path='analysis_results_2.csv')
