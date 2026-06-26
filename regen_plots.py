"""
Standalone script to re-run only the visualization and PDF-report steps
using the already-generated raw_experiment_results.csv.
Run from the SafeTCPF workspace root.
"""
import os
import yaml
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from evaluation.experiment_runner import ExperimentRunner

# ── Config & paths ──────────────────────────────────────────────────────────
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

_cfg_path = os.path.join(BASE_DIR, 'config', 'config.yaml')
with open(_cfg_path, 'r') as _f:
    _CFG = yaml.safe_load(_f)

RESULTS_CSV  = os.path.join(BASE_DIR,
    _CFG.get('evaluation', {}).get('results_csv', 'evaluation/results/results.csv'))
runner = ExperimentRunner()
OUTPUT_DIR = runner.vis_dir
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Load pre-existing raw results
raw_csv = os.path.join(runner.results_dir, "raw_experiment_results.csv")
print(f"Loading {raw_csv} ...")
raw_df = pd.read_csv(raw_csv)
print(f"Loaded {len(raw_df)} rows.")

# Regenerate figures
print("Generating visualizations...")
runner.generate_visualizations(raw_df)

# Regenerate PDF reports
print("Generating PDF reports...")
runner.generate_pdf_reports(raw_df)

print(f"\nDone! Check {runner.vis_dir}/ and {runner.reports_dir}/ directories.")
